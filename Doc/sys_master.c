/*
 * EtherCAT 6轴驱动器控制程序
 * 基于原单轴版本扩展为6轴独立控制
 * 编译: gcc -o sys_master sys_master.c -I/path/to/ethercat/include -L/path/to/ethercat/lib -lethercat -lpthread -lrt -lm
 */

#include <signal.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>      // 提供 fcntl, F_GETFL, F_SETFL, O_NONBLOCK
#include <sys/mman.h>   // 提供 mlockall, MCL_CURRENT, MCL_FUTURE

#include <ecrt.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "sys_master.h"
#include "sys_leader.h"
#include "sys_share_messages.h"
#include "sys_share_conf.h"
#include "sys_share_ipc.h"
#include "sys_ctl.h"
#include "sys_commu_ipc.h"

// ===================== 配置参数 =====================
#define VENDOR_ID        0x00010203
#define PRODUCT_ID       0x00000402
#define FREQUENCY        500
#define PERIOD_NS        2000000

// 编码器参数
#define MOTOR_ENCODER_COUNTS_PER_REV   1048576
#define GEAR_RATIO                     101.0f
#define JOINT_ENCODER_COUNTS_PER_REV   (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO)

// DC时钟同步参数 
#define DC_SYNC0_CYCLE_TIME  2000000
#define DC_SYNC0_SHIFT       1000000
#define DC_ASSIGN_ACTIVATE   0x0300   

// 模式切换延时
#define MODE_SWITCH_DELAY_MS  500

// 软件位置环参数
#define POS_CONTROL_CYCLE_HZ      200
#define POS_CONTROL_PERIOD_MS     5
#define POS_CONTROL_PERIOD_S      0.005f

// 平滑参数
#define VEL_SMOOTH_WINDOW_MS      100
#define UPPER_POS_SMOOTH_CNT      (VEL_SMOOTH_WINDOW_MS / POS_CONTROL_PERIOD_MS)

// 控制参数
#define POS_KP_START              10.0f
#define POS_KP_MAX                80.0f
#define POS_KD                    3.0f
#define POS_KP                    15.0f

// 运动限制
#define MAX_VEL_DEG_PER_SEC       10.0f
#define MAX_ACC_DEG_PER_SEC2      30.0f

// 到位检测
#define POS_ARRIVAL_THRESHOLD     0.02f
#define VEL_ARRIVAL_THRESHOLD     0.5f
#define ARRIVAL_HOLD_MS           40

// 强制归零阈值
#define FORCE_ZERO_VEL_THRESHOLD  0.1f

// 模式切换阈值 
//#define POS_MODE_SWITCH_THRESHOLD 10.0f

// 状态上报端口
#define UPPER_STATUS_PORT 33334

// SDO操作超时
#define SDO_OPERATION_TIMEOUT_MS 100

// 限幅宏
#define LIMIT_MAX(x, max) ((x) > (max) ? (max) : (x))
#define LIMIT_MIN(x, min) ((x) < (min) ? (min) : (x))
#define LIMIT(x, min, max) LIMIT_MIN(LIMIT_MAX(x, max), min)


// ===================== 类型定义 =====================
typedef struct {
    unsigned int byte_offset;
    unsigned int bit_position;
    int size_bits;
    const char *name;
} pdo_entry_t;

typedef enum {
    MY_MASTER_STATE_IDLE = 0,
    MY_MASTER_STATE_INIT,
    MY_MASTER_STATE_CONFIG_PDO,
    MY_MASTER_STATE_CONFIG_DC,
    MY_MASTER_STATE_ACTIVE,
    MY_MASTER_STATE_ERROR
} MasterLifecycleState;

typedef enum {
    SLAVE_NOT_READY = 0,
    SLAVE_SWITCH_ON_DISABLED,
    SLAVE_READY_TO_SWITCH_ON,
    SLAVE_SWITCHED_ON,
    SLAVE_OPERATION_ENABLED,
    SLAVE_QUICK_STOP_ACTIVE,
    SLAVE_FAULT,
    SLAVE_UNKNOWN
} SlaveCiAState;

typedef enum {
    OP_MODE_NONE = 0,
    OP_MODE_CSP = 8,
    OP_MODE_CSV = 9,
} OpMode;

typedef enum {
    CTRL_MODE_IDLE = 0,
    CTRL_MODE_CSV_DIRECT_VEL,
    CTRL_MODE_CSV_SOFT_POS,
    CTRL_MODE_CSP_HARD_POS,
    CTRL_MODE_TRANSITION,
} ControlMode;

typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_ACCEL,
    MOTION_STATE_CRUISE,
    MOTION_STATE_DECEL,
    MOTION_STATE_HOLD,
} MotionState;

typedef struct {
    float pos_target;
    float pos_start;
    float pos_current;
    int active;
    int arrived;
    uint32_t arrival_hold_cnt;
    float vel_history[UPPER_POS_SMOOTH_CNT];
    int history_idx;
    MotionState state;
    float last_vel_cmd;
    float max_speed_reached;
    uint32_t cycle_cnt;
} CSVSoftPosState;

typedef struct {
    int32_t pos_start;
    int32_t pos_target;
    int step;
    int total_steps;
    int active;
} CSPHardPosState;

#pragma pack(1)
typedef struct {
    uint8_t   axisId;
    uint8_t   successCount;
    uint8_t   totalCount;
    uint8_t   reserved;
} KGU_PID_WRITE_RESP_4BYTE;

typedef struct {
    uint8_t   axisId;
    uint8_t   success;
    uint16_t  errorCode;
} KGU_PID_SAVE_RESP;
#pragma pack()

// ===================== 全局变量 - 6轴数组 =====================
static volatile int g_running = 1;
static volatile int g_stop_requested = 0;
static unsigned long g_cycle_counter = 0;

static const ssize_t kMsgDataOffset = 28;
static const ssize_t kMsgHeaderSize = (ssize_t)sizeof(MsgLongMsg);

static MasterLifecycleState g_master_state = MY_MASTER_STATE_IDLE;
static ec_master_t *g_master = NULL;
static ec_domain_t *g_domain = NULL;

// 6轴从站配置数组
static ec_slave_config_t *g_slave_configs[MASTER_ARMS_MAX_SIZE] = {NULL};

// 6轴PDO条目数组
static pdo_entry_t g_pdo_ctrl_word[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_target_pos[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_target_vel[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_target_tor[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_status_word[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_actual_pos[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_actual_vel[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_actual_tor[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_follow_err[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_err_code[MASTER_ARMS_MAX_SIZE];
static pdo_entry_t g_pdo_op_mode[MASTER_ARMS_MAX_SIZE];

static uint8_t *g_domain_data = NULL;

// 6轴从站状态数组
static SlaveCiAState g_slave_states[MASTER_ARMS_MAX_SIZE] = {
    SLAVE_NOT_READY, SLAVE_NOT_READY, SLAVE_NOT_READY,
    SLAVE_NOT_READY, SLAVE_NOT_READY, SLAVE_NOT_READY
};
static uint16_t g_current_ctrl_words[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int g_slaves_in_op[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int g_enable_op_requested[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int g_op_just_entered[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static uint32_t g_op_entered_time[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};

// 6轴模式管理数组
static OpMode g_current_op_modes[MASTER_ARMS_MAX_SIZE] = {
    OP_MODE_NONE, OP_MODE_NONE, OP_MODE_NONE,
    OP_MODE_NONE, OP_MODE_NONE, OP_MODE_NONE
};
static uint32_t g_mode_switch_times[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int g_mode_switchings[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int g_mode_switch_stables[MASTER_ARMS_MAX_SIZE] = {1, 1, 1, 1, 1, 1};

// 6轴控制模式数组
static ControlMode g_control_modes[MASTER_ARMS_MAX_SIZE] = {
    CTRL_MODE_IDLE, CTRL_MODE_IDLE, CTRL_MODE_IDLE,
    CTRL_MODE_IDLE, CTRL_MODE_IDLE, CTRL_MODE_IDLE
};

// 6轴运动控制数组
static float g_target_vel_degs[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int32_t g_target_vel_internals[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};
static int32_t g_current_pos_internals[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};

// 6轴位置环状态数组
static CSVSoftPosState g_csv_poss[MASTER_ARMS_MAX_SIZE];
static CSPHardPosState g_csp_poss[MASTER_ARMS_MAX_SIZE];

// 通信
static int32_t g_sock_upper = -1;
static struct sockaddr_in g_upper_addr;
static int g_upper_valid = 0;
static uint32_t g_frame_id = 0;
static uint32_t g_last_report = 0;

// 全局模块指针
static ARMS_MASTER_THREAD_INFO *g_pmodule = NULL;

// SDO操作保护
static volatile int g_sdo_operation_in_progress = 0;
static uint32_t g_sdo_start_time = 0;

// PID参数映射
typedef struct {
    uint16_t paramNo;
    uint16_t index;
    uint8_t  subindex;
    uint8_t  dataType;
    float    minVal;
    float    maxVal;
    float    defaultVal;
    const char* name;
    const char* desc;
} KguParamMapping;

static const KguParamMapping kguParamMap[] = {
    {31, 0x201F, 0, 4, 0, 9999, 2000, "Cur_Kp", "电流环比例增益"},
    {32, 0x2020, 0, 4, 0, 9999, 300,  "Cur_Ki", "电流环积分增益"},
    {33, 0x2021, 0, 4, 0, 9999, 1800, "Vel_Kp", "速度环比例增益"},
    {34, 0x2022, 0, 4, 0, 9999, 300,  "Vel_Ki", "速度环积分增益"},
    {65, 0x2041, 0, 4, 0, 24,   22,   "Pos_Kp1", "位置环比例增益1(粗调)"},
    {63, 0x203F, 0, 4, 1000, 5000, 2000, "Pos_Kp2", "位置环比例增益2(细调)"},
    {64, 0x2040, 0, 4, 0, 100,  0,    "Pos_FF", "位置前馈百分比"},
};
#define KGU_PARAM_COUNT (sizeof(kguParamMap) / sizeof(kguParamMap[0]))

// ===================== 6轴零位偏移配置 =====================
// 机械零位与实际零位的偏移量（单位：度）
// 实际角度 = 编码器角度 - 零位偏移
static const float AXIS_ZERO_OFFSETS[MASTER_ARMS_MAX_SIZE] = {
    17.03f,      // 轴1
    -6.587f,    // 轴2
    39.143f,     // 轴3
    47.096f,     // 轴4
    -24.272f,    // 轴5
    57.861f      // 轴6
};

// 零位校准后的当前实际位置（度），用于显示和正运动学
static float g_actual_positions_deg[MASTER_ARMS_MAX_SIZE] = {0, 0, 0, 0, 0, 0};

// 前向声明
static SlaveCiAState parse_slave_state(uint16_t status_word);
static const char* slave_state_str(SlaveCiAState state);
static void handle_slave_state_machine_axis(int axis);
static int switch_op_mode(int axis, OpMode new_mode);
static int master_init(void);
static int master_config_pdo(void);
static int master_config_dc(void);
static int master_activate(void);
static void master_cleanup(void);
static void send_pid_error_response(uint8_t axis, uint8_t error_code);

// ===================== 信号处理 =====================
static void signal_handler(int sig)
{
    printf("\n[master] 收到信号 %d, 正在停止...\n", sig);
    g_running = 0;
    g_stop_requested = 1;
}

// ===================== 单位转换 =====================
static inline float counts_to_degrees(int32_t counts)
{
    return (float)counts * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
}

static inline int32_t degrees_to_counts(float degrees)
{
    return (int32_t)(degrees * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

static inline int32_t deg_per_sec_to_csv(float deg_per_sec)
{
    return (int32_t)(deg_per_sec * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

static inline float rpm_to_joint_deg_per_sec(int32_t rpm)
{
    return (float)rpm * 6.0f / GEAR_RATIO;
}

// ===================== 零位校准转换函数 =====================

/**
 * @brief 编码器counts转换为实际角度（带零位偏移）
 * @param counts 编码器原始值
 * @param axis 轴号 0-5
 * @return 实际角度（度），已减去零位偏移
 */
static inline float counts_to_actual_degrees(int32_t counts, int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0.0f;
    float mechanical_deg = (float)counts * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
    return mechanical_deg + AXIS_ZERO_OFFSETS[axis];
}

/**
 * @brief 实际角度转换为编码器counts（带零位偏移）
 * @param actual_deg 实际角度（度），即期望的零位后角度
 * @param axis 轴号 0-5
 * @return 编码器counts值
 */
static inline int32_t actual_degrees_to_counts(float actual_deg, int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    float mechanical_deg = actual_deg + AXIS_ZERO_OFFSETS[axis];
    return (int32_t)(mechanical_deg * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
}

/**
 * @brief 获取轴的零位偏移量
 * @param axis 轴号 0-5
 * @return 零位偏移量（度）
 */
static inline float get_axis_zero_offset(int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0.0f;
    return AXIS_ZERO_OFFSETS[axis];
}

// ===================== 6轴PDO读写函数 =====================
static inline uint16_t read_status_word(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint16_t val;
    memcpy(&val, g_domain_data + g_pdo_status_word[axis].byte_offset, 2);
    return val;
}

static inline int32_t read_actual_position(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int32_t val;
    memcpy(&val, g_domain_data + g_pdo_actual_pos[axis].byte_offset, 4);

    // 同步更新实际角度缓存（用于显示和运动学）
    g_actual_positions_deg[axis] = counts_to_actual_degrees(val, axis);
    return val;
}

/**
 * @brief 获取指定轴的实际角度（已校准零位）
 * @param axis 轴号 0-5
 * @return 实际角度（度）
 */
static inline float get_actual_position_deg(int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0.0f;
    return g_actual_positions_deg[axis];
}

static inline int32_t read_actual_velocity(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int32_t val;
    memcpy(&val, g_domain_data + g_pdo_actual_vel[axis].byte_offset, 4);
    return val;
}

static inline int16_t read_actual_torque(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int16_t val;
    memcpy(&val, g_domain_data + g_pdo_actual_tor[axis].byte_offset, 2);
    return val;
}

static inline uint16_t read_error_code(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint16_t val;
    memcpy(&val, g_domain_data + g_pdo_err_code[axis].byte_offset, 2);
    return val;
}

static inline uint8_t read_op_mode_display(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    uint8_t val;
    memcpy(&val, g_domain_data + g_pdo_op_mode[axis].byte_offset, 1);
    return val;
}

static inline int32_t read_following_error(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return 0;
    int32_t val;
    memcpy(&val, g_domain_data + g_pdo_follow_err[axis].byte_offset, 4);
    return val;
}

static inline void write_control_word(int axis, uint16_t word)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_pdo_ctrl_word[axis].byte_offset, &word, 2);
    g_current_ctrl_words[axis] = word;
}

static inline void write_target_velocity(int axis, int32_t vel)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_pdo_target_vel[axis].byte_offset, &vel, 4);
}

static inline void write_target_position(int axis, int32_t pos)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_pdo_target_pos[axis].byte_offset, &pos, 4);
}

static inline void write_target_torque(int axis, int16_t tor)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    memcpy(g_domain_data + g_pdo_target_tor[axis].byte_offset, &tor, 2);
}

// ===================== CiA 402状态解析 =====================
static SlaveCiAState parse_slave_state(uint16_t status_word)
{
    if (status_word & 0x0008) return SLAVE_FAULT;
    if ((status_word & 0x006F) == 0x0007) return SLAVE_QUICK_STOP_ACTIVE;
    
    switch (status_word & 0x006F) {
        case 0x0000: return SLAVE_NOT_READY;
        case 0x0040: return SLAVE_SWITCH_ON_DISABLED;
        case 0x0021: return SLAVE_READY_TO_SWITCH_ON;
        case 0x0023: return SLAVE_SWITCHED_ON;
        case 0x0027: return SLAVE_OPERATION_ENABLED;
        default: break;
    }
    
    switch (status_word) {
        case 0x0250: return SLAVE_SWITCH_ON_DISABLED;
        case 0x0231: return SLAVE_READY_TO_SWITCH_ON;
        case 0x0233: return SLAVE_SWITCHED_ON;
        case 0x0237: return SLAVE_OPERATION_ENABLED;
        case 0x0210: return SLAVE_OPERATION_ENABLED;
        case 0x0218: return SLAVE_FAULT;
        default: break;
    }
    
    return SLAVE_UNKNOWN;
}

static const char* slave_state_str(SlaveCiAState state)
{
    switch (state) {
        case SLAVE_NOT_READY: return "Not Ready";
        case SLAVE_SWITCH_ON_DISABLED: return "Switch On Disabled";
        case SLAVE_READY_TO_SWITCH_ON: return "Ready to Switch On";
        case SLAVE_SWITCHED_ON: return "Switched On";
        case SLAVE_OPERATION_ENABLED: return "Operation Enabled";
        case SLAVE_QUICK_STOP_ACTIVE: return "Quick Stop";
        case SLAVE_FAULT: return "Fault";
        default: return "Unknown";
    }
}

static const char* op_mode_str(OpMode mode)
{
    switch (mode) {
        case OP_MODE_CSP: return "CSP";
        case OP_MODE_CSV: return "CSV";
        default: return "NONE";
    }
}


// static const char* control_mode_str(ControlMode mode)
// {
//     switch (mode) {
//         case CTRL_MODE_IDLE: return "空闲";
//         case CTRL_MODE_CSV_DIRECT_VEL: return "CSV直接速度";
//         case CTRL_MODE_CSV_SOFT_POS: return "CSV软件位置环";
//         case CTRL_MODE_CSP_HARD_POS: return "CSP硬件位置环";
//         case CTRL_MODE_TRANSITION: return "切换中";
//         default: return "未知";
//     }
// }

// ===================== SDO操作函数 =====================
static int safe_sdo_upload(int axis, uint16_t index, uint8_t subindex,
    uint8_t *target, size_t target_size, size_t *result_size, uint32_t timeout_ms)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return -1;
    if (g_sdo_operation_in_progress) return -10;
    if (g_master_state != MY_MASTER_STATE_ACTIVE || !g_master) return -1;
    
    ec_master_state_t ms;
    ecrt_master_state(g_master, &ms);
    if (ms.al_states != 0x08) return -2;
    
    uint16_t st = read_status_word(axis);
    SlaveCiAState slave_st = parse_slave_state(st);
    if (slave_st != SLAVE_OPERATION_ENABLED) return -3;
    
    g_sdo_operation_in_progress = 1;
    g_sdo_start_time = (uint32_t)g_cycle_counter;
    
    uint32_t abort_code = 0;
    int ret = ecrt_master_sdo_upload(g_master, axis, index, subindex,
                                     target, target_size, result_size, &abort_code);
    
    g_sdo_operation_in_progress = 0;
    return ret;
}

// static int safe_sdo_download(int axis, uint16_t index, uint8_t subindex,
//     uint8_t *data, size_t data_size, uint32_t timeout_ms)
// {
//     if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return -1;
//     if (g_master_state != MY_MASTER_STATE_ACTIVE || !g_master) return -1;
    
//     ec_master_state_t ms;
//     ecrt_master_state(g_master, &ms);
//     if (ms.al_states < 0x04) return -2;
    
//     uint32_t abort_code = 0;
//     int ret = ecrt_master_sdo_download(g_master, axis, index, subindex,
//                                        data, data_size, &abort_code);
//     return ret;
// }

// ===================== 6轴纯PD位置环 =====================
static void init_csv_soft_position(int axis, float target_actual_deg)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    
    // 读取当前编码器位置
    int32_t current_counts = read_actual_position(axis);
    float current_actual_deg = g_actual_positions_deg[axis];  // 使用缓存的实际角度
    
    memset(&g_csv_poss[axis], 0, sizeof(CSVSoftPosState));
    
    // 目标角度是实际角度，需要转换为机械角度存储
    g_csv_poss[axis].pos_target = target_actual_deg;
    g_csv_poss[axis].pos_start = current_actual_deg;
    g_csv_poss[axis].pos_current = current_actual_deg;
    g_csv_poss[axis].active = 1;
    g_csv_poss[axis].arrived = 0;
    g_csv_poss[axis].arrival_hold_cnt = 0;
    g_csv_poss[axis].cycle_cnt = 0;
    g_csv_poss[axis].max_speed_reached = 0;
    
    // 速度历史初始化使用当前实际速度
    float current_vel = rpm_to_joint_deg_per_sec(read_actual_velocity(axis));
    for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
        g_csv_poss[axis].vel_history[i] = current_vel;
    }
    g_csv_poss[axis].last_vel_cmd = current_vel;
    
    printf("[master] 轴%d启动PD位置环: 实际%.2f°->实际%.2f° (机械%.2f°->机械%.2f°)\n", 
           axis, 
           current_actual_deg, target_actual_deg,
           counts_to_degrees(current_counts), 
           counts_to_degrees(actual_degrees_to_counts(target_actual_deg, axis)));
}

static void update_csv_soft_position_loop(int axis)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    if (!g_csv_poss[axis].active) return;
    
    g_csv_poss[axis].cycle_cnt++;
    
    float pos_actual = g_actual_positions_deg[axis];  // 使用实际角度（已校准）
    float vel_actual = rpm_to_joint_deg_per_sec(read_actual_velocity(axis));
    g_csv_poss[axis].pos_current = pos_actual;
    
    // 目标也是实际角度，直接计算误差
    float pos_error = g_csv_poss[axis].pos_target - pos_actual;
    
    if (g_csv_poss[axis].arrived) {
        write_target_velocity(axis, 0);
        if (fabsf(pos_error) > 0.5f) {
            printf("[master] 轴%d位置偏离%.3f°，重新启动\n", axis, pos_error);
            init_csv_soft_position(axis, g_csv_poss[axis].pos_target);
        }
        return;
    }
    
    // PD控制器：V_cmd = Kp*e - Kd*vel_actual
    float vel_cmd = POS_KP * pos_error - POS_KD * vel_actual;
    
    // 近目标点限速（防超调）
    float abs_error = fabsf(pos_error);
    if (abs_error < 1.0f) {
        float max_vel_near_target = MAX_VEL_DEG_PER_SEC * abs_error;
        if (fabsf(vel_cmd) > max_vel_near_target) {
            vel_cmd = (vel_cmd > 0) ? max_vel_near_target : -max_vel_near_target;
        }
    }
    
    // 强制归零：误差和速度都很小时直接归零
    if (abs_error < FORCE_ZERO_VEL_THRESHOLD && fabsf(vel_actual) < 2.0f) {
        vel_cmd = 0;
    }
    
    // 加速度限制
    float acc = (vel_cmd - g_csv_poss[axis].last_vel_cmd) / POS_CONTROL_PERIOD_S;
    if (acc > MAX_ACC_DEG_PER_SEC2) {
        vel_cmd = g_csv_poss[axis].last_vel_cmd + MAX_ACC_DEG_PER_SEC2 * POS_CONTROL_PERIOD_S;
    } else if (acc < -MAX_ACC_DEG_PER_SEC2) {
        vel_cmd = g_csv_poss[axis].last_vel_cmd - MAX_ACC_DEG_PER_SEC2 * POS_CONTROL_PERIOD_S;
    }
    
    vel_cmd = LIMIT(vel_cmd, -MAX_VEL_DEG_PER_SEC, MAX_VEL_DEG_PER_SEC);
    g_csv_poss[axis].last_vel_cmd = vel_cmd;
    
    if (fabsf(vel_cmd) > g_csv_poss[axis].max_speed_reached) {
        g_csv_poss[axis].max_speed_reached = fabsf(vel_cmd);
    }
    
    // 速度平滑滤波
    g_csv_poss[axis].vel_history[g_csv_poss[axis].history_idx] = vel_cmd;
    g_csv_poss[axis].history_idx = (g_csv_poss[axis].history_idx + 1) % UPPER_POS_SMOOTH_CNT;
    
    float vel_smooth = 0;
    for (int i = 0; i < UPPER_POS_SMOOTH_CNT; i++) {
        vel_smooth += g_csv_poss[axis].vel_history[i];
    }
    vel_smooth /= UPPER_POS_SMOOTH_CNT;
    
    // 到位检测
    if (abs_error < POS_ARRIVAL_THRESHOLD && fabsf(vel_actual) < VEL_ARRIVAL_THRESHOLD) {
        g_csv_poss[axis].arrival_hold_cnt++;
        if (g_csv_poss[axis].arrival_hold_cnt >= (ARRIVAL_HOLD_MS / POS_CONTROL_PERIOD_MS)) {
            g_csv_poss[axis].arrived = 1;
            vel_smooth = 0;
            printf("[master] 轴%d到位稳定: 实际%.3f° (机械%.3f°)\n", 
                   axis, g_csv_poss[axis].pos_target,
                   counts_to_degrees(read_actual_position(axis)));
        }
    } else {
        g_csv_poss[axis].arrival_hold_cnt = 0;
    }
    
    // 速度命令转换为编码器单位下发（注意：速度是增量，不需要零位校准）
    g_target_vel_internals[axis] = deg_per_sec_to_csv(vel_smooth);
    write_target_velocity(axis, g_target_vel_internals[axis]);
}

// ===================== 6轴模式切换 =====================
static int switch_op_mode(int axis, OpMode new_mode)
{
    if (axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return -1;
    if (g_current_op_modes[axis] == new_mode) return 0;
    
    printf("[master] 轴%d模式切换: %s -> %s\n", axis,
           op_mode_str(g_current_op_modes[axis]), op_mode_str(new_mode));
    
    uint16_t current_status = read_status_word(axis);
    SlaveCiAState current_state = parse_slave_state(current_status);
    
    if (current_state == SLAVE_NOT_READY || current_state == SLAVE_FAULT) {
        printf("[master] 轴%d错误: 状态 %s 无法切换模式\n", axis, slave_state_str(current_state));
        return -1;
    }
    
    if (current_state == SLAVE_OPERATION_ENABLED) {
        write_target_velocity(axis, 0);
        write_target_position(axis, read_actual_position(axis));
        write_control_word(axis, 0x0007);
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        
        int wait = 0;
        SlaveCiAState new_state;
        do {
            usleep(10000);
            ecrt_master_receive(g_master);
            ecrt_domain_process(g_domain);
            new_state = parse_slave_state(read_status_word(axis));
            wait++;
        } while (new_state != SLAVE_SWITCHED_ON && wait < 100);
    }
    
    uint8_t mode_value = (uint8_t)new_mode;
    int sdo_retry = 0;
    int sdo_result = -1;
    
    while (sdo_retry < 3 && sdo_result != 0) {
        sdo_result = ecrt_slave_config_sdo8(g_slave_configs[axis], 0x6060, 0x00, mode_value);
        if (sdo_result != 0) {
            printf("[master] 轴%d SDO写入重试 %d...\n", axis, sdo_retry + 1);
            usleep(50000);
        }
        sdo_retry++;
    }
    
    if (sdo_result != 0) {
        fprintf(stderr, "[master] 轴%d错误: 无法设置操作模式 %d\n", axis, mode_value);
        return -1;
    }
    
    g_current_op_modes[axis] = new_mode;
    g_mode_switch_times[axis] = (uint32_t)g_cycle_counter;
    g_mode_switchings[axis] = 1;
    g_mode_switch_stables[axis] = 0;
    
    usleep(MODE_SWITCH_DELAY_MS * 1000);
    
    printf("[master] 轴%d模式切换完成\n", axis);
    return 0;
}

// ===================== 6轴状态机处理 =====================
static void handle_slave_state_machine_axis(int axis)
{
    if (!g_domain_data || axis < 0 || axis >= MASTER_ARMS_MAX_SIZE) return;
    
    uint16_t status = read_status_word(axis);
    SlaveCiAState new_state = parse_slave_state(status);
    uint16_t error_code = read_error_code(axis);
    
    if (new_state != g_slave_states[axis]) {
        printf("[master] 轴%d状态: %s -> %s (0x%04X, 错误码: 0x%04X)\n",
               axis, slave_state_str(g_slave_states[axis]), slave_state_str(new_state), 
               status, error_code);
        
        if (new_state == SLAVE_FAULT) {
            printf("[master] 轴%d故障: 0x%04X\n", axis, error_code);
            g_slaves_in_op[axis] = 0;
            g_enable_op_requested[axis] = 0;
            g_csv_poss[axis].active = 0;
            g_csp_poss[axis].active = 0;
            g_control_modes[axis] = CTRL_MODE_IDLE;
        }
        else if (new_state == SLAVE_OPERATION_ENABLED) {
            if (!g_slaves_in_op[axis]) {
                printf("[master] 轴%d进入Operation Enabled\n", axis);
                g_slaves_in_op[axis] = 1;
                g_enable_op_requested[axis] = 0;
                g_op_just_entered[axis] = 1;
                g_op_entered_time[axis] = (uint32_t)g_cycle_counter;
            }
        } else {
            g_slaves_in_op[axis] = 0;
        }
        
        g_slave_states[axis] = new_state;
    }
    
    static uint32_t last_fault_time[MASTER_ARMS_MAX_SIZE] = {0};
    if (new_state == SLAVE_FAULT && g_cycle_counter - last_fault_time[axis] > 500) {
        last_fault_time[axis] = g_cycle_counter;
        printf("[master] 轴%d自动尝试故障恢复...\n", axis);
        write_control_word(axis, 0x0080);
    }
}

// ===================== 6轴主站生命周期 =====================
static int master_init(void)
{
    printf("[master] 初始化EtherCAT主站 for %d axes...\n", MASTER_ARMS_MAX_SIZE);
    
    g_cycle_counter = 0;
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_slave_states[i] = SLAVE_NOT_READY;
        g_slaves_in_op[i] = 0;
        g_enable_op_requested[i] = 0;
        g_current_ctrl_words[i] = 0;
        g_current_op_modes[i] = OP_MODE_NONE;
        g_mode_switchings[i] = 0;
        g_mode_switch_stables[i] = 1;
        g_control_modes[i] = CTRL_MODE_IDLE;
    }

    g_master = ecrt_request_master(0);
    if (!g_master) {
        fprintf(stderr, "[master] 错误: 无法请求主站\n");
        return -1;
    }
    
    g_domain = ecrt_master_create_domain(g_master);
    if (!g_domain) {
        fprintf(stderr, "[master] 错误: 无法创建域\n");
        return -1;
    }
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_slave_configs[i] = ecrt_master_slave_config(g_master, 0, i, VENDOR_ID, PRODUCT_ID);
        if (!g_slave_configs[i]) {
            fprintf(stderr, "[master] 错误: 无法配置从站%d\n", i);
            return -1;
        }
    }
    
    printf("[master] 主站初始化完成: %d axes\n", MASTER_ARMS_MAX_SIZE);
    return 0;
}

static int master_config_pdo(void)
{
    printf("[master] 配置PDO for %d axes...\n", MASTER_ARMS_MAX_SIZE);
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        uint8_t op_mode = 9;
        if (ecrt_slave_config_sdo8(g_slave_configs[i], 0x6060, 0x00, op_mode)) {
            fprintf(stderr, "[master] 错误: 无法设置轴%d的默认CSV模式\n", i);
            return -1;
        }
        g_current_op_modes[i] = OP_MODE_CSV;

        uint32_t max_speed = 3000;
        ecrt_slave_config_sdo32(g_slave_configs[i], 0x607F, 0x00, max_speed);
        uint32_t max_acc = 10000;
        ecrt_slave_config_sdo32(g_slave_configs[i], 0x60C5, 0x00, max_acc);
        ecrt_slave_config_sdo32(g_slave_configs[i], 0x60C6, 0x00, max_acc);
        ecrt_slave_config_sdo32(g_slave_configs[i], 0x6083, 0x00, max_acc);
        ecrt_slave_config_sdo32(g_slave_configs[i], 0x6084, 0x00, max_acc);
        
        static ec_pdo_entry_info_t output_entries[] = {
            {0x6040, 0x00, 16},
            {0x607a, 0x00, 32},
            {0x60ff, 0x00, 32},
            {0x6071, 0x00, 16},
        };
        
        static ec_pdo_entry_info_t input_entries[] = {
            {0x6041, 0x00, 16},
            {0x6064, 0x00, 32},
            {0x606c, 0x00, 32},
            {0x6077, 0x00, 16},
            {0x60f4, 0x00, 32},
            {0x603f, 0x00, 16},
            {0x6061, 0x00, 8},
        };
        
        static ec_pdo_info_t pdos[] = {
            {0x1601, 4, output_entries},
            {0x1A01, 7, input_entries},
        };
        
        static ec_sync_info_t syncs[] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
            {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_DISABLE},
            {3, EC_DIR_INPUT, 1, pdos + 1, EC_WD_DISABLE},
            {0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DISABLE}
        };
        
        if (ecrt_slave_config_pdos(g_slave_configs[i], EC_END, syncs)) {
            fprintf(stderr, "[master] 错误: 配置轴%d的PDO失败\n", i);
            return -1;
        }
    }
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        unsigned int bit;
        g_pdo_ctrl_word[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6040, 0, g_domain, &bit);
        g_pdo_target_pos[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x607a, 0, g_domain, &bit);
        g_pdo_target_vel[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x60ff, 0, g_domain, &bit);
        g_pdo_target_tor[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6071, 0, g_domain, &bit);
        g_pdo_status_word[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6041, 0, g_domain, &bit);
        g_pdo_actual_pos[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6064, 0, g_domain, &bit);
        g_pdo_actual_vel[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x606c, 0, g_domain, &bit);
        g_pdo_actual_tor[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6077, 0, g_domain, &bit);
        g_pdo_follow_err[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x60f4, 0, g_domain, &bit);
        g_pdo_err_code[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x603f, 0, g_domain, &bit);
        g_pdo_op_mode[i].byte_offset = ecrt_slave_config_reg_pdo_entry(
            g_slave_configs[i], 0x6061, 0, g_domain, &bit);
    }
    
    printf("[master] PDO配置完成: %d axes\n", MASTER_ARMS_MAX_SIZE);
    return 0;
}

static int master_config_dc(void)
{
    printf("[master] 配置DC时钟同步 for %d axes...\n", MASTER_ARMS_MAX_SIZE);
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        ecrt_slave_config_dc(g_slave_configs[i], DC_ASSIGN_ACTIVATE,
                           DC_SYNC0_CYCLE_TIME, DC_SYNC0_SHIFT, 0, 0);
    }
    
    printf("[master] DC配置完成\n");
    return 0;
}

static int master_activate(void)
{
    printf("[master] 激活主站...\n");
    
    if (g_domain_data) {
        memset(g_domain_data, 0, ecrt_domain_size(g_domain));
    }
    
    if (ecrt_master_activate(g_master)) {
        fprintf(stderr, "[master] 错误: 激活主站失败\n");
        return -1;
    }
    
    g_domain_data = ecrt_domain_data(g_domain);
    if (!g_domain_data) {
        fprintf(stderr, "[master] 错误: 获取域数据失败\n");
        return -1;
    }

    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        write_control_word(i, 0x0000);
        write_target_velocity(i, 0);
        
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);
        int32_t current_pos = read_actual_position(i);
        write_target_position(i, current_pos);
        g_current_pos_internals[i] = current_pos;
        
        printf("[master] 轴%d初始位置: %d counts (%.2f°)\n", 
               i, current_pos, counts_to_degrees(current_pos));
    }
    
    ecrt_domain_queue(g_domain);
    ecrt_master_send(g_master);
    
    printf("[master] 等待%d个从站就绪...\n", MASTER_ARMS_MAX_SIZE);
    int retries = 0;
    int max_retries = 500;
    int all_ready = 0;
    
    while (retries < max_retries && !all_ready) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t dc_time = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        ecrt_master_application_time(g_master, dc_time);
        ecrt_master_sync_slave_clocks(g_master);
        
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);
        
        all_ready = 1;
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            uint16_t status = read_status_word(i);
            SlaveCiAState state = parse_slave_state(status);
            //uint16_t err_code = read_error_code(i);
            
            ec_master_state_t ms;
            ecrt_master_state(g_master, &ms);
            
            if (retries % 50 == 0) {
                printf("[master] 轴%d: AL=0x%02X, 状态=%s, 状态字=0x%04X\n",
                       i, ms.al_states, slave_state_str(state), status);
            }
            
            if (ms.al_states != 0x08 || state == SLAVE_FAULT) {
                all_ready = 0;
                if (state == SLAVE_FAULT) {
                    printf("[master] 轴%d故障，尝试清除...\n", i);
                    write_control_word(i, 0x0080);
                }
            }
            
            write_control_word(i, 0x0000);
            write_target_velocity(i, 0);
            write_target_position(i, g_current_pos_internals[i]);
        }
        
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        
        usleep(10000);
        retries++;
    }
    
    printf("[master] 主站激活完成\n");
    return 0;
}

static void master_cleanup(void)
{
    printf("[master] 清理主站资源...\n");
    
    if (g_master) {
        ecrt_release_master(g_master);
        g_master = NULL;
    }
    g_domain = NULL;
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        g_slave_configs[i] = NULL;
    }
    g_domain_data = NULL;
    g_master_state = MY_MASTER_STATE_IDLE;
    
    printf("[master] 清理完成\n");
}

// ===================== 6轴状态上报（零位校准后的角度） =====================
static void send_status_report(void)
{
    if (g_sock_upper < 0 || !g_upper_valid) return;

    MASTER_STATUS_REPORT report;
    memset(&report, 0, sizeof(report));
    
    report.mFrameId = g_frame_id++;
    report.mTimestamp = (uint32_t)(g_cycle_counter * 2);
    report.masterState = (uint8_t)g_master_state;
    report.slaveCount = MASTER_ARMS_MAX_SIZE;
    report.cycleCount = (uint32_t)g_cycle_counter;
    
    if (g_master) {
        ec_master_state_t ms;
        ecrt_master_state(g_master, &ms);
        report.masterALState = ms.al_states;
    }
    
    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
        SLAVE_PDO_DATA *s = &report.slaves[i];
        s->statusWord = read_status_word(i);
        
        // 上报实际角度（已校准零位），而不是机械角度
        s->actualPosition = (int32_t)(g_actual_positions_deg[i] * 1000.0f);  // 放大1000倍保持精度，或直接用float
        
        // 或者如果协议支持float，直接传：
        // s->actualPositionDeg = g_actual_positions_deg[i];
        
        s->actualVelocity = read_actual_velocity(i);
        s->actualTorque = read_actual_torque(i);
        s->followingError = read_following_error(i);
        s->errorCode = read_error_code(i);
        s->controlWord = g_current_ctrl_words[i];
        s->targetPosition = 0;
        s->targetVelocity = g_target_vel_internals[i];
        s->targetTorque = 0;
        
        // 可选：同时上报机械角度和零位偏移，方便上位机验证
        // s->mechanicalPosition = counts;
        // s->zeroOffset = (int32_t)(AXIS_ZERO_OFFSETS[i] * 1000.0f);
    }
    
    int flags = fcntl(g_sock_upper, F_GETFL, 0);
    fcntl(g_sock_upper, F_SETFL, flags | O_NONBLOCK);

    ssize_t sent = sendto(g_sock_upper, &report, sizeof(report), 0,
           (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
    
    if (sent != sizeof(report)) {
        printf("[master] 上报失败: %zd/%zu\n", sent, sizeof(report));
    }
}

static void check_send_report(void)
{
    if (g_cycle_counter - g_last_report >= 50) {
        g_last_report = g_cycle_counter;
        send_status_report();
    }
}

// ===================== 通信初始化 =====================
static void init_upper_socket(void)
{
    g_sock_upper = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock_upper < 0) {
        printf("[master] 创建状态上报socket失败\n");
        return;
    }
    
    memset(&g_upper_addr, 0, sizeof(g_upper_addr));
    g_upper_addr.sin_family = AF_INET;
    g_upper_addr.sin_port = htons(UPPER_STATUS_PORT);
    g_upper_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    g_upper_valid = 1;
    
    printf("[master] 状态上报socket已创建\n");
}

static void send_pid_error_response(uint8_t axis, uint8_t error_code)
{
    uint8_t resp[4];
    resp[0] = axis;
    resp[1] = 0;
    resp[2] = 0;
    resp[3] = error_code;
    
    if (g_sock_upper < 0) return;
    
    sendto(g_sock_upper, resp, 4, 0,
           (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
}

// ===================== 6轴命令处理 =====================
static void handle_upper_cmd(ARMS_MASTER_THREAD_INFO *pmodule, struct timespec *main_wake_time)
{
    (void)main_wake_time;
    
    if (!pmodule || pmodule->mSoket < 0) return;
    
    struct sockaddr_in peer;
    socklen_t peer_len = sizeof(peer);
    char buf[2048];
    
    ssize_t n = recvfrom(pmodule->mSoket, buf, sizeof(buf), MSG_DONTWAIT,
                         (struct sockaddr *)&peer, &peer_len);
    if (n <= 0) return;
    
    MsgLongMsg *msg = (MsgLongMsg *)buf;
    int32_t cmd = msg->mMsgId;
    
    if (cmd != CMD_HEARTBEAT) {
        printf("[master] 收到数据: cmd=%d, sender=%d\n", cmd, msg->mSender);
    }
    
    switch (cmd) {
        case CMD_LINK: {
            printf("[master] 收到LINK命令\n");
            if (n > kMsgHeaderSize) {
                uint16_t status_port = *(uint16_t*)msg->mData;
                printf("[master] 上位机状态接收端口: %d\n", status_port);
                g_upper_addr.sin_port = htons(status_port);
                g_upper_valid = 1;
            }
            int32_t resp = 1;
            sendto(g_sock_upper, &resp, sizeof(resp), 0,
                (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
            break;
        }
        
        case CMD_IGH_CREATE_MASTERS:
            printf("[master] 收到命令: 创建主站\n");
            if (g_master_state == MY_MASTER_STATE_IDLE) {
                if (master_init() == 0) {
                    g_master_state = MY_MASTER_STATE_INIT;
                }
            }
            break;
            
        case CMD_IGH_CONF_SERVO_PDOS:
            printf("[master] 收到命令: 配置PDO\n");
            if (g_master_state == MY_MASTER_STATE_INIT) {
                if (master_config_pdo() == 0) {
                    g_master_state = MY_MASTER_STATE_CONFIG_PDO;
                }
            }
            break;
            
        case CMD_IGH_START_OP:
            printf("[master] 收到命令: 激活主站\n");
            if (g_master_state == MY_MASTER_STATE_CONFIG_PDO || 
                g_master_state == MY_MASTER_STATE_INIT) {
                
                if (g_master_state == MY_MASTER_STATE_INIT) {
                    if (master_config_pdo() != 0) break;
                    g_master_state = MY_MASTER_STATE_CONFIG_PDO;
                }
                
                if (master_config_dc() != 0) break;
                if (master_activate() == 0) {
                    g_master_state = MY_MASTER_STATE_ACTIVE;
                    for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                        g_enable_op_requested[i] = 0;
                        g_slaves_in_op[i] = 0;
                        g_current_ctrl_words[i] = 0x0000;
                    }
                    send_status_report();
                    printf("[master] 主站激活完成\n");
                }
            }
            break;
            
        case CMD_DESTROY_MASTER:
            printf("[master] 收到命令: 销毁主站\n");
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                write_control_word(i, 0x0000);
            }
            if (g_domain_data) {
                ecrt_domain_queue(g_domain);
                ecrt_master_send(g_master);
                usleep(100000);
            }
            master_cleanup();
            break;
            
        case CMD_IGH_FSA_SHUTDOWN:
            printf("[master] 收到命令: FSA Shutdown\n");
            if (g_master_state == MY_MASTER_STATE_ACTIVE) {
                // 【修复】正确读取mData[0]
                uint8_t mask = 0x3F;  // 默认6轴
                if (n > kMsgDataOffset) {
                    // 强制转换为MsgLongMsg指针，然后访问mData
                    MsgLongMsg *msgLong = (MsgLongMsg *)msg;
                    mask = (uint8_t)msgLong->mData[0];
                }
                printf("[master] 消息长度: %zd, 数据偏移: %zu, 轴掩码: 0x%02X\n", 
                    n, kMsgDataOffset, mask);
                
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    if (mask & (1 << i)) {
                        printf("[master]  -> 控制轴%d Shutdown\n", i);
                        g_enable_op_requested[i] = 0;
                        g_slaves_in_op[i] = 0;
                        g_csv_poss[i].active = 0;
                        g_csp_poss[i].active = 0;
                        g_control_modes[i] = CTRL_MODE_IDLE;
                        write_control_word(i, 0x0006);
                    }
                }
            }
            break;

        case CMD_IGH_FSA_SWITCH_ON:
            printf("[master] 收到命令: FSA Switch On\n");
            if (g_master_state == MY_MASTER_STATE_ACTIVE) {
                uint8_t mask = 0x3F;
                if (n > kMsgDataOffset) {
                    MsgLongMsg *msgLong = (MsgLongMsg *)msg;
                    mask = (uint8_t)msgLong->mData[0];
                }
                printf("[master] 轴掩码: 0x%02X\n", mask);
                
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    if (mask & (1 << i)) {
                        write_control_word(i, 0x0007);
                    }
                }
            }
            break;

        case CMD_IGH_FSA_ENOP:
            printf("[master] 收到命令: FSA Enable Operation\n");
            if (g_master_state == MY_MASTER_STATE_ACTIVE) {
                uint8_t mask = 0x3F;
                if (n > kMsgDataOffset) {
                    MsgLongMsg *msgLong = (MsgLongMsg *)msg;
                    mask = (uint8_t)msgLong->mData[0];
                }
                printf("[master] 轴掩码: 0x%02X\n", mask);
                
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    if (mask & (1 << i)) {
                        uint16_t current_status = read_status_word(i);
                        SlaveCiAState current_state = parse_slave_state(current_status);
                        
                        if (current_state != SLAVE_SWITCHED_ON) {
                            printf("[master] 轴%d错误: 当前状态不是 Switched On\n", i);
                            continue;
                        }
                        
                        printf("[master] 轴%d准备进入OP状态...\n", i);
                        g_enable_op_requested[i] = 1;
                        g_op_just_entered[i] = 1;
                        g_op_entered_time[i] = (uint32_t)g_cycle_counter;
                    }
                }
            }
            break;

        case CMD_IGH_FSA_HALT:
            printf("[master] 收到命令: FSA Halt/Disable\n");
            if (g_master_state == MY_MASTER_STATE_ACTIVE) {
                uint8_t mask = 0x3F;
                if (n > kMsgDataOffset) {
                    MsgLongMsg *msgLong = (MsgLongMsg *)msg;
                    mask = (uint8_t)msgLong->mData[0];
                }
                printf("[master] 轴掩码: 0x%02X\n", mask);
                
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    if (mask & (1 << i)) {
                        g_enable_op_requested[i] = 0;
                        g_slaves_in_op[i] = 0;
                        g_csv_poss[i].active = 0;
                        g_csp_poss[i].active = 0;
                        g_control_modes[i] = CTRL_MODE_IDLE;
                        write_control_word(i, 0x0000);
                        g_target_vel_degs[i] = 0.0f;
                        g_target_vel_internals[i] = 0;
                    }
                }
            }
            break;

        case CMD_IGH_CLEAR_ERROR:
            printf("[master] 收到命令: FSA Fault Reset\n");
            if (g_master_state == MY_MASTER_STATE_ACTIVE) {
                uint8_t mask = 0x3F;
                if (n > kMsgDataOffset) {
                    MsgLongMsg *msgLong = (MsgLongMsg *)msg;
                    mask = (uint8_t)msgLong->mData[0];
                }
                printf("[master] 轴掩码: 0x%02X\n", mask);
                
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    if (mask & (1 << i)) {
                        g_enable_op_requested[i] = 0;
                        g_slaves_in_op[i] = 0;
                        g_csv_poss[i].active = 0;
                        g_csp_poss[i].active = 0;
                        g_control_modes[i] = CTRL_MODE_IDLE;
                        write_control_word(i, 0x0080);
                    }
                }
            }
            break;
            
        case CMD_HAND_VEL_MM: {
            if (g_master_state != MY_MASTER_STATE_ACTIVE) break;
            
            UPPER_VEL_MSG *vel = (UPPER_VEL_MSG *)msg->mData;
            
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                if (vel->mModuleFlag[i]) {
                    if (!g_slaves_in_op[i]) {
                        printf("[master] 轴%d错误: 从站未进入OP状态\n", i);
                        continue;
                    }
                    
                    if (g_current_op_modes[i] == OP_MODE_CSP || g_csv_poss[i].active) {
                        g_csv_poss[i].active = 0;
                        g_csp_poss[i].active = 0;
                        
                        if (g_current_op_modes[i] == OP_MODE_CSP) {
                            if (switch_op_mode(i, OP_MODE_CSV) != 0) continue;
                            usleep(100000);
                            write_control_word(i, 0x000F);
                            int wait_op = 0;
                            while (!g_slaves_in_op[i] && wait_op < 100) {
                                usleep(10000);
                                handle_slave_state_machine_axis(i);
                                wait_op++;
                            }
                        }
                    }
                    
                    float new_vel_deg = vel->mVel[i];
                    if (new_vel_deg > 90.0f) new_vel_deg = 90.0f;
                    if (new_vel_deg < -90.0f) new_vel_deg = -90.0f;
                    
                    g_target_vel_degs[i] = new_vel_deg;
                    g_target_vel_internals[i] = deg_per_sec_to_csv(g_target_vel_degs[i]);
                    g_control_modes[i] = CTRL_MODE_CSV_DIRECT_VEL;
                    
                    printf("[master] 轴%d CSV速度: %.3f deg/s\n", i, g_target_vel_degs[i]);
                }
            }
            break;
        }
            
        case CMD_HAND_RELA_POS_MM: {
            if (g_master_state != MY_MASTER_STATE_ACTIVE) break;
            
            uint8_t *module_flags = (uint8_t *)msg->mData;
            float *positions = (float *)(msg->mData + 8);  // 这里传入的是实际角度（度）
            
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                if (module_flags[i]) {
                    if (!g_slaves_in_op[i]) {
                        printf("[master] 轴%d错误: 从站未在OP状态\n", i);
                        continue;
                    }
                    
                    // positions[i] 是实际角度偏移量（度）
                    float rel_pos_deg = positions[i];
                    
                    // 获取当前实际角度
                    float current_actual_deg = get_actual_position_deg(i);
                    float target_actual_deg = current_actual_deg + rel_pos_deg;
                    
                    printf("[master] 轴%d相对位置命令: 当前实际%.3f° + %.3f° = 目标实际%.3f°\n", 
                        i, current_actual_deg, rel_pos_deg, target_actual_deg);
                    
                    // 切换到CSV模式并启动位置控制
                    if (g_current_op_modes[i] == OP_MODE_CSP) {
                        if (switch_op_mode(i, OP_MODE_CSV) != 0) continue;
                        usleep(100000);
                        write_control_word(i, 0x000F);
                        int wait_op = 0;
                        while (!g_slaves_in_op[i] && wait_op < 100) {
                            usleep(10000);
                            handle_slave_state_machine_axis(i);
                            wait_op++;
                        }
                    }
                    
                    g_csp_poss[i].active = 0;
                    g_csv_poss[i].active = 0;
                    
                    // 传入目标实际角度
                    init_csv_soft_position(i, target_actual_deg);
                    g_control_modes[i] = CTRL_MODE_CSV_SOFT_POS;
                }
            }
            break;
        }
                
        case CMD_HAND_ABS_POS_M: {
            if (g_master_state != MY_MASTER_STATE_ACTIVE) break;
            
            uint8_t *module_flags = (uint8_t *)msg->mData;
            float *positions = (float *)(msg->mData + 8);  // 这里传入的是目标实际角度（度）
            
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                if (module_flags[i]) {
                    if (!g_slaves_in_op[i]) continue;
                    
                    // positions[i] 直接就是目标实际角度
                    float target_actual_deg = positions[i];
                    float current_actual_deg = get_actual_position_deg(i);
                    
                    printf("[master] 轴%d绝对位置命令: 当前实际%.3f° -> 目标实际%.3f°\n", 
                        i, current_actual_deg, target_actual_deg);
                    
                    // 切换到CSV模式
                    if (g_current_op_modes[i] == OP_MODE_CSP) {
                        if (switch_op_mode(i, OP_MODE_CSV) != 0) continue;
                        usleep(100000);
                        write_control_word(i, 0x000F);
                        int wait_op = 0;
                        while (!g_slaves_in_op[i] && wait_op < 100) {
                            usleep(10000);
                            handle_slave_state_machine_axis(i);
                            wait_op++;
                        }
                    }
                    
                    g_csp_poss[i].active = 0;
                    g_csv_poss[i].active = 0;
                    
                    // 直接使用目标实际角度
                    init_csv_soft_position(i, target_actual_deg);
                    g_control_modes[i] = CTRL_MODE_CSV_SOFT_POS;
                }
            }
            break;
        }

        case CMD_PID_READ_PARAMS: {
            uint8_t axis = *(uint8_t *)msg->mData;
            if (axis >= MASTER_ARMS_MAX_SIZE) {
                send_pid_error_response(axis, 2);
                break;
            }
            
            if (g_master_state != MY_MASTER_STATE_ACTIVE) {
                send_pid_error_response(axis, 1);
                break;
            }
            
            ec_master_state_t ms_check;
            ecrt_master_state(g_master, &ms_check);
            if (ms_check.al_states != 0x08) {
                send_pid_error_response(axis, 3);
                break;
            }
            
            uint16_t st = read_status_word(axis);
            SlaveCiAState slave_st = parse_slave_state(st);
            if (slave_st != SLAVE_OPERATION_ENABLED) {
                send_pid_error_response(axis, 5);
                break;
            }
            
            printf("[master] 读取轴%d的PID参数\n", axis);
            
            KGU_PID_PARAMS_PACKET resp;
            memset(&resp, 0, sizeof(resp));
            resp.axisId = axis;
            resp.paramCount = 0;
            resp.resultCode = 0;
            
            for (int i = 0; i < (int)KGU_PARAM_COUNT && resp.paramCount < 8; i++) {
                resp.params[resp.paramCount].paramNo = kguParamMap[i].paramNo;
                resp.params[resp.paramCount].reserved = 0;
                resp.params[resp.paramCount].minValue = kguParamMap[i].minVal;
                resp.params[resp.paramCount].maxValue = kguParamMap[i].maxVal;
                resp.params[resp.paramCount].isValid = 0;
                
                strncpy(resp.params[resp.paramCount].name, kguParamMap[i].name, 15);
                resp.params[resp.paramCount].name[15] = '\0';
                strncpy(resp.params[resp.paramCount].description, kguParamMap[i].desc, 31);
                resp.params[resp.paramCount].description[31] = '\0';
                
                uint8_t sdoData[4] = {0};
                size_t actualSize = 0;
                
                int ret = safe_sdo_upload(axis, kguParamMap[i].index, 
                    kguParamMap[i].subindex, sdoData, kguParamMap[i].dataType, 
                    &actualSize, 50000);
                
                if (ret == 0 && actualSize == kguParamMap[i].dataType) {
                    uint32_t rawValue = 0;
                    memcpy(&rawValue, sdoData, kguParamMap[i].dataType);
                    float actualValue = (kguParamMap[i].dataType == 4) ? 
                                    (float)(int32_t)rawValue : (float)rawValue;
                    
                    resp.params[resp.paramCount].value = actualValue;
                    resp.params[resp.paramCount].isValid = 1;
                } else {
                    resp.params[resp.paramCount].value = kguParamMap[i].defaultVal;
                    resp.params[resp.paramCount].isValid = 0;
                }
                
                resp.paramCount++;
            }
            
            int packetSize = 4 + (resp.paramCount * 66);
            sendto(g_sock_upper, &resp, packetSize, 0,
                (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
            break;
        }

        case CMD_PID_WRITE_PARAMS: {
            if (g_master_state != MY_MASTER_STATE_ACTIVE) {
                KGU_PID_WRITE_RESP_4BYTE resp = {0xFF, 0, 0, 0};
                sendto(g_sock_upper, &resp, sizeof(resp), 0,
                    (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
                break;
            }
            
            const uint8_t* reqData = (const uint8_t*)msg->mData;
            uint8_t axis = reqData[0];
            uint8_t reqParamCount = reqData[1];
            
            if (axis >= MASTER_ARMS_MAX_SIZE) {
                KGU_PID_WRITE_RESP_4BYTE resp = {axis, 0, reqParamCount, 0};
                sendto(g_sock_upper, &resp, sizeof(resp), 0,
                    (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
                break;
            }
            
            int successCount = 0;
            
            for (int j = 0; j < reqParamCount && j < 8; j++) {
                int itemOffset = 4 + j * 6;
                uint16_t paramNo = *(uint16_t*)(reqData + itemOffset);
                float targetValue = *(float*)(reqData + itemOffset + 2);
                
                const KguParamMapping *map = NULL;
                for (int k = 0; k < (int)KGU_PARAM_COUNT; k++) {
                    if (kguParamMap[k].paramNo == paramNo) {
                        map = &kguParamMap[k];
                        break;
                    }
                }
                
                if (!map) continue;
                if (targetValue < map->minVal || targetValue > map->maxVal) continue;
                
                uint32_t rawValue = (uint32_t)targetValue;
                uint8_t sdoData[4];
                memcpy(sdoData, &rawValue, map->dataType);
                uint32_t abort_code = 0;
                
                int ret = ecrt_master_sdo_download(g_master, axis, map->index,
                    map->subindex, sdoData, map->dataType, &abort_code);
                
                if (ret == 0) successCount++;
            }
            
            KGU_PID_WRITE_RESP_4BYTE resp;
            resp.axisId = axis;
            resp.successCount = (uint8_t)successCount;
            resp.totalCount = reqParamCount;
            resp.reserved = 0;
            
            sendto(g_sock_upper, &resp, sizeof(resp), 0,
                (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
            break;
        }

        case CMD_PID_SAVE_TO_FLASH: {
            if (g_master_state != MY_MASTER_STATE_ACTIVE) {
                KGU_PID_SAVE_RESP resp = {0xFF, 0, 0xFFFF};
                sendto(g_sock_upper, &resp, sizeof(resp), 0,
                    (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
                break;
            }
            
            uint8_t axis = *(uint8_t *)msg->mData;
            if (axis >= MASTER_ARMS_MAX_SIZE) {
                KGU_PID_SAVE_RESP resp = {axis, 0, 1};
                sendto(g_sock_upper, &resp, sizeof(resp), 0,
                    (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
                break;
            }
            
            uint32_t saveSignature = 0x65766173;
            uint8_t saveData[4];
            memcpy(saveData, &saveSignature, 4);
            uint32_t abort_code = 0;
            
            int ret = ecrt_master_sdo_download(g_master, axis, 0x1010, 0x01,
                saveData, 4, &abort_code);
            
            if (ret != 0) {
                uint32_t resetCmd = 1111;
                uint8_t resetData[4];
                memcpy(resetData, &resetCmd, 4);
                uint32_t abort_code2 = 0;
                ret = ecrt_master_sdo_download(g_master, axis, 0x2000, 0,
                    resetData, 4, &abort_code2);
                if (ret == 0) abort_code = 0;
                else abort_code = abort_code2;
            }
            
            KGU_PID_SAVE_RESP resp;
            resp.axisId = axis;
            resp.success = (ret == 0) ? 1 : 0;
            resp.errorCode = (ret == 0) ? 0 : (uint16_t)(abort_code & 0xFFFF);
            
            sendto(g_sock_upper, &resp, sizeof(resp), 0,
                (struct sockaddr *)&g_upper_addr, sizeof(g_upper_addr));
            break;
        }
    }
}

// ===================== 6轴实时循环 =====================
static void run_master_cycle(ARMS_MASTER_THREAD_INFO *pmodule, struct timespec *wake_time)
{
    (void)pmodule;
    
    struct timespec rt_time;
    uint64_t dc_time;
    ec_master_state_t ms;
    
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wake_time, NULL);
    
    wake_time->tv_nsec += PERIOD_NS;
    while (wake_time->tv_nsec >= 1000000000) {
        wake_time->tv_nsec -= 1000000000;
        wake_time->tv_sec++;
    }

    if (g_sdo_operation_in_progress) {
        uint32_t elapsed_cycles = g_cycle_counter - g_sdo_start_time;
        if (elapsed_cycles > (SDO_OPERATION_TIMEOUT_MS / 2)) {
            printf("[master] SDO操作超时，强制恢复\n");
            g_sdo_operation_in_progress = 0;
            if (g_domain_data) {
                for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                    write_control_word(i, 0x0000);
                }
                ecrt_domain_queue(g_domain);
                ecrt_master_send(g_master);
            }
        }
    }
    
    if (g_master && g_master_state == MY_MASTER_STATE_ACTIVE) {
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_domain);
        
        if (g_domain_data) {
            for (int axis = 0; axis < MASTER_ARMS_MAX_SIZE; axis++) {
                handle_slave_state_machine_axis(axis);
                
                if (g_mode_switchings[axis]) {
                    uint32_t elapsed_ms = (g_cycle_counter - g_mode_switch_times[axis]) * 2;
                    if (elapsed_ms >= MODE_SWITCH_DELAY_MS) {
                        g_mode_switchings[axis] = 0;
                        g_mode_switch_stables[axis] = 1;
                        if (g_slave_states[axis] == SLAVE_SWITCHED_ON || 
                            g_slave_states[axis] == SLAVE_OPERATION_ENABLED) {
                            write_control_word(axis, 0x000F);
                        }
                    }
                }
                
                if (g_mode_switchings[axis]) {
                    write_target_velocity(axis, 0);
                    write_target_position(axis, g_current_pos_internals[axis]);
                    if (g_slave_states[axis] == SLAVE_OPERATION_ENABLED) {
                        write_control_word(axis, 0x000F);
                    } else if (g_slave_states[axis] == SLAVE_SWITCHED_ON) {
                        write_control_word(axis, 0x0007);
                    }
                    continue;
                }
                
                if (g_slaves_in_op[axis] && g_mode_switch_stables[axis]) {
                    switch (g_control_modes[axis]) {
                        case CTRL_MODE_CSV_DIRECT_VEL: {
                            write_target_velocity(axis, g_target_vel_internals[axis]);
                            write_target_position(axis, 0);
                            write_control_word(axis, 0x000F);
                            break;
                        }
                        
                        case CTRL_MODE_CSV_SOFT_POS: {
                            if (g_cycle_counter % 5 == 0) {
                                update_csv_soft_position_loop(axis);
                            }
                            write_target_velocity(axis, g_target_vel_internals[axis]);
                            write_target_position(axis, 0);
                            write_control_word(axis, 0x000F);
                            break;
                        }
                        
                        case CTRL_MODE_CSP_HARD_POS: {
                            if (g_csp_poss[axis].active && g_csp_poss[axis].step < g_csp_poss[axis].total_steps) {
                                float t = (float)g_csp_poss[axis].step / g_csp_poss[axis].total_steps;
                                float ratio = 10.0f * t * t * t - 15.0f * t * t * t * t + 6.0f * t * t * t * t * t;
                                
                                int32_t smooth_pos = g_csp_poss[axis].pos_start + 
                                    (int32_t)((g_csp_poss[axis].pos_target - g_csp_poss[axis].pos_start) * ratio);
                                
                                write_target_position(axis, smooth_pos);
                                g_csp_poss[axis].step++;
                            } else if (g_csp_poss[axis].active) {
                                write_target_position(axis, g_csp_poss[axis].pos_target);
                            } else {
                                write_target_position(axis, g_current_pos_internals[axis]);
                            }
                            
                            write_target_velocity(axis, 0);
                            write_control_word(axis, 0x000F);
                            break;
                        }
                        
                        default: {
                            if (g_current_op_modes[axis] == OP_MODE_CSP) {
                                write_target_position(axis, g_current_pos_internals[axis]);
                            } else {
                                write_target_velocity(axis, 0);
                            }
                            write_control_word(axis, 0x000F);
                            break;
                        }
                    }
                } else if (g_enable_op_requested[axis]) {
                    write_target_position(axis, g_current_pos_internals[axis]);
                    write_target_velocity(axis, 0);
                    write_control_word(axis, 0x000F);
                    
                    if (g_slave_states[axis] == SLAVE_OPERATION_ENABLED) {
                        g_slaves_in_op[axis] = 1;
                        g_enable_op_requested[axis] = 0;
                        g_op_just_entered[axis] = 1;
                        g_op_entered_time[axis] = (uint32_t)g_cycle_counter;
                    }
                } else if (g_slave_states[axis] == SLAVE_SWITCHED_ON) {
                    write_target_position(axis, g_current_pos_internals[axis]);
                    write_target_velocity(axis, 0);
                    write_control_word(axis, 0x0007);
                } else {
                    write_control_word(axis, g_current_ctrl_words[axis]);
                    write_target_position(axis, g_current_pos_internals[axis]);
                    write_target_velocity(axis, 0);
                }
                
                if (g_op_just_entered[axis] && g_slaves_in_op[axis]) {
                    if (g_cycle_counter - g_op_entered_time[axis] < 10) {
                        write_target_position(axis, g_current_pos_internals[axis]);
                        write_target_velocity(axis, 0);
                    }
                }

                // 在循环末尾添加这行
                (void)read_actual_position(axis);   // 仅用于更新缓存
            }
        }
        
        clock_gettime(CLOCK_MONOTONIC, &rt_time);
        dc_time = (uint64_t)rt_time.tv_sec * 1000000000ULL + rt_time.tv_nsec;
        ecrt_master_application_time(g_master, dc_time);
        ecrt_master_sync_reference_clock(g_master);
        ecrt_master_sync_slave_clocks(g_master);
        
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        
        if (g_cycle_counter % 25 == 0) {
            check_send_report();
        }

        if (g_cycle_counter % 5000 == 0) {
            ecrt_master_state(g_master, &ms);
            printf("[master] 状态: AL=0x%02X, 周期=%lu\n", ms.al_states, g_cycle_counter);
            for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
                // 读取并更新实际角度
                int32_t counts = read_actual_position(i);
                float actual_deg = g_actual_positions_deg[i];
                float mechanical_deg = counts_to_degrees(counts);
                float actual_vel_deg = rpm_to_joint_deg_per_sec(read_actual_velocity(i));
                
                printf("  轴%d: %s, 实际=%.2f°(机械=%.2f°,偏移=%.2f°), 速度=%.2f°/s\n",
                    i, slave_state_str(g_slave_states[i]),
                    actual_deg, mechanical_deg, AXIS_ZERO_OFFSETS[i],
                    actual_vel_deg);
            }
        }
    }
    
    g_cycle_counter++;
}

// ===================== 线程入口 =====================
void* master_arms_threadEntry(void* pModule)
{
    // 锁定内存，防止页面交换造成延迟
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // 设置实时调度策略（需要root权限）
    struct sched_param param = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("pthread_setschedparam");
        // 非致命，继续运行
    }

    ARMS_MASTER_THREAD_INFO *pmodule = (ARMS_MASTER_THREAD_INFO *)pModule;
    if (!pmodule) return NULL;
    
    g_pmodule = pmodule;
    
    printf("========================================\n");
    printf(" EtherCAT 6轴主站控制程序 v6.0\n");
    printf(" 支持轴数: %d\n", MASTER_ARMS_MAX_SIZE);
    printf("========================================\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    pmodule->mSoket = createSoket(get_mn_master_port(0), MN_LocalIpV4Str, 0, 0);
    if (pmodule->mSoket < 0) {
        printf("[master] 创建socket失败\n");
        return NULL;
    }
    
    init_upper_socket();
    
    printf("[master] 等待上位机命令...\n");
    
    struct timespec wake_time;
    clock_gettime(CLOCK_MONOTONIC, &wake_time);
    
    while (g_running) {
        handle_upper_cmd(pmodule, &wake_time);
        
        if (g_master_state == MY_MASTER_STATE_ACTIVE) {
            run_master_cycle(pmodule, &wake_time);
        } else {
            usleep(1000);
        }
    }
    
    printf("[master] 正在退出...\n");
    
    if (g_domain_data) {
        for (int i = 0; i < MASTER_ARMS_MAX_SIZE; i++) {
            write_control_word(i, 0x0000);
        }
        ecrt_domain_queue(g_domain);
        ecrt_master_send(g_master);
        usleep(100000);
    }
    
    master_cleanup();
    
    if (pmodule->mSoket >= 0) {
        close(pmodule->mSoket);
        pmodule->mSoket = -1;
    }
    if (g_sock_upper >= 0) {
        close(g_sock_upper);
        g_sock_upper = -1;
    }
    
    printf("[master] 已退出\n");
    return NULL;
}