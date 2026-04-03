/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : EtherCAT Master with CSV Motor Control for KaiserDrive
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Disable optimization for this file to prevent compiler reordering */
#pragma O0

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "osal.h"
#include "ethercattype.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
#include "ethercatbase.h"
#include "nicdrv.h"



// Define missing DC register (AssignActivate)
#ifndef ECT_REG_DCCUC
#define ECT_REG_DCCUC       0x0980  // DC Cyclic Unit Control (AssignActivate)
#endif

// STM32F4 Memory barrier operations for DMA coherence
// Note: STM32F407 Cortex-M4 does not have D-Cache, but we use memory barriers
// to ensure proper instruction and memory ordering
static inline void CleanDCache_by_Addr(uint32_t *addr, int32_t dsize)
{
    (void)addr;
    (void)dsize;
    // Cortex-M4 does not have D-Cache, but we use DSB to ensure memory writes complete
    __DSB();
}

static inline void InvalidateDCache_by_Addr(uint32_t *addr, int32_t dsize)
{
    (void)addr;
    (void)dsize;
    // Cortex-M4 does not have D-Cache, but we use DSB to ensure memory consistency
    __DSB();
}

// ============================================================================
// HIGH-PRECISION DC TIME IMPLEMENTATION
// Using STM32 DWT (Data Watchpoint and Trace) cycle counter
// Provides nanosecond-level precision for DC synchronization
// ============================================================================

// DWT registers
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)

// System clock frequency (168MHz)
#define SYSTEM_CLOCK_FREQ   168000000ULL
#define NS_PER_SECOND       1000000000ULL

// DC time offset (for synchronization)
static volatile int64_t dc_time_offset = 0;
static volatile uint32_t last_cyccnt = 0;

/**
 * @brief Initialize DWT cycle counter for high-precision timing
 */
static inline void DWT_Init(void)
{
    // Enable DWT in DEMCR
    DEMCR |= (1 << 24);  // Set TRCENA bit
    
    // Reset cycle counter
    DWT_CYCCNT = 0;
    
    // Enable cycle counter
    DWT_CTRL |= (1 << 0);  // Set CYCCNTENA bit
    
    last_cyccnt = 0;
    dc_time_offset = 0;
}

/**
 * @brief Get current time in nanoseconds (64-bit)
 * Uses DWT cycle counter for high precision
 */
static inline int64_t get_dc_time_ns(void)
{
    uint32_t cyccnt = DWT_CYCCNT;
    
    // Handle overflow (DWT_CYCCNT is 32-bit, overflows every ~25.5 seconds at 168MHz)
    if (cyccnt < last_cyccnt) {
        // Overflow occurred, add offset
        dc_time_offset += ((uint64_t)0x100000000ULL * NS_PER_SECOND) / SYSTEM_CLOCK_FREQ;
    }
    last_cyccnt = cyccnt;
    
    // Convert cycles to nanoseconds
    return dc_time_offset + ((uint64_t)cyccnt * NS_PER_SECOND) / SYSTEM_CLOCK_FREQ;
}

/**
 * @brief Get current time in microseconds (for SOEM compatibility)
 */
static inline uint32_t get_us_time(void)
{
    return (uint32_t)(get_dc_time_ns() / 1000ULL);
}

// ============================================================================
// ETH INTERRUPT MODE FOR DC SYNC PRECISION
// ============================================================================

// Flag to indicate packet received in interrupt
volatile uint8_t eth_packet_received = 0;

/**
 * @brief ETH RX Complete Callback - called from interrupt
 * This ensures immediate processing of received EtherCAT frames
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    (void)heth;
    eth_packet_received = 1;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// UART RX Buffer
#define RX_BUFFER_SIZE 64

// LED Definitions (Active Low)
#define LED1_PORT       GPIOE
#define LED1_PIN        GPIO_PIN_7    // Red LED - Error indicator
#define LED2_PORT       GPIOE
#define LED2_PIN        GPIO_PIN_8    // Blue LED - Status indicator
#define KEY_PORT        GPIOE
#define KEY_PIN         GPIO_PIN_0    // User key

// LED Blink Periods (ms)
#define BLINK_INIT          1000
#define BLINK_PRE_OP        500
#define BLINK_SAFE_OP       250
#define BLINK_OPERATIONAL   0
#define BLINK_ERROR         100

// EtherCAT Cycle Time (ms)
#define EC_CYCLE_TIME_MS    2

// Motor Parameters (from KaiserDrive ESI and sys_master.c)
#define VENDOR_ID           0x00010203
#define PRODUCT_ID          0x00000402
#define MOTOR_ENCODER_COUNTS_PER_REV    1048576     // 2^20
#define GEAR_RATIO                      101.0f      // Harmonic gear ratio
#define JOINT_ENCODER_COUNTS_PER_REV    (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO)

// Velocity conversion: degree/s to CSV counts/s
#define DEG_PER_SEC_TO_CSV_FACTOR       (JOINT_ENCODER_COUNTS_PER_REV / 360.0f)

// CSV Mode Parameters
#define TARGET_VELOCITY_DEFAULT 10.0f
#define MAX_VELOCITY            90.0f

// DC Sync Parameters
#define DC_SYNC0_CYCLE_TIME     2000000     // 2ms in ns
#define DC_SYNC0_SHIFT          1000000     // 1ms shift

// CiA 402 Operation Modes
#define OPMODE_CSP              8   // Cyclic Synchronous Position
#define OPMODE_CSV              9   // Cyclic Synchronous Velocity
#define OPMODE_CST              10  // Cyclic Synchronous Torque

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART RX Buffer
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

// EtherCAT Variables
volatile int ec_slave_count = 0;
volatile uint16_t ec_slave_state = 0;
volatile uint32_t led_blink_period = BLINK_INIT;
volatile uint32_t last_blink_time = 0;
volatile uint8_t led_state = 0;

// SOEM DC time variable (defined in ethercatmain.c)
extern int64 ec_DCtime;

// Motor Control Variables
volatile uint8_t motor_enabled = 0;
volatile uint8_t system_running = 0;
volatile uint8_t ec_initialized = 0;
volatile uint8_t ec_operational = 0;
volatile uint8_t manual_control = 0;  // Set when using manual commands
volatile uint16_t manual_controlword = 0;  // Manual control word value

// Target and actual values
volatile int32_t target_position = 0;      // CSP mode target position
volatile int32_t target_velocity_csv = 0;  // CSV mode target velocity (not used in CSP)
volatile int32_t actual_velocity_csv = 0;
volatile int32_t actual_position = 0;
volatile uint16_t statusword = 0;
volatile uint16_t controlword = 0;
volatile uint16_t error_code = 0;

// State machine variables
volatile uint8_t cia402_state = 0;
volatile uint32_t state_entry_time = 0;
volatile uint8_t init_phase = 1;       // Initialization phase flag
volatile uint32_t init_start_time = 0; // Initialization phase start time

// Operation mode
// CRITICAL: KaiserDrive ESI defines default mode as CSP (8) in InitCmd
// <InitCmd><Transition>PS</Transition><Index>#x6060</Index><Data>08</Data></InitCmd>
volatile int8_t operation_mode = OPMODE_CSP;  // Default to CSP mode (matches ESI)
volatile uint8_t mode_change_requested = 0;

// PDO Offsets (will be calculated during init)
// CRITICAL: Mark as volatile to prevent compiler optimization issues
volatile int ctrl_word_offset = 0;
volatile int target_pos_offset = 0;
volatile int target_vel_offset = 0;
volatile int target_tor_offset = 0;
volatile int status_word_offset = 0;
volatile int actual_pos_offset = 0;
volatile int actual_vel_offset = 0;
volatile int actual_tor_offset = 0;
volatile int error_code_offset = 0;

// Note: ec_IOmap is defined in SOEM library (ethercatmain.h)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// UART Functions
void UART_SendString(const char *str);
void UART_SendLine(const char *str);
void UART_SendHex(uint32_t val, uint8_t digits);
void UART_Poll_Receive(void);
void Process_Command(void);

// LED Functions
void Update_LED_Status(void);

// EtherCAT Functions
int EtherCAT_Init(void);
void EtherCAT_Cyclic_Task(void);
void Motor_State_Machine(void);
void Diagnose_EtherCAT_Frame(void);

// Helper Functions
uint8_t Parse_CiA402_State(uint16_t sw);
const char* Get_State_String(uint8_t state);
void Print_Status(void);
int Map_PDOs(int slave);

// Retarget printf
int _write(int file, char *ptr, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send string via UART
void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

// Send string with newline
void UART_SendLine(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
}

// Send hex value
void UART_SendHex(uint32_t val, uint8_t digits)
{
    char hex_str[9];
    const char hex_chars[] = "0123456789ABCDEF";
    int i;
    
    if (digits > 8) digits = 8;
    if (digits == 0) digits = 2;
    
    hex_str[digits] = '\0';
    for (i = digits - 1; i >= 0; i--) {
        hex_str[i] = hex_chars[val & 0x0F];
        val >>= 4;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)hex_str, digits, 100);
}

// Retarget printf to USART1
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// UART Polling Receive
void UART_Poll_Receive(void)
{
    uint8_t ch;
    static uint8_t last_was_cr = 0;
    static char debug_buf[16];
    static uint8_t debug_idx = 0;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        ch = (uint8_t)(huart1.Instance->DR & 0xFF);

        // Debug: store received char
        if (debug_idx < 15) {
            debug_buf[debug_idx++] = ch;
            debug_buf[debug_idx] = '\0';
        }

        // Echo character
        if (!(last_was_cr && ch == '\n'))
        {
            HAL_UART_Transmit(&huart1, &ch, 1, 10);
        }

        // Check for end of line
        if (ch == '\r' || ch == '\n')
        {
            if (!last_was_cr || ch != '\n')
            {
                rx_buffer[rx_index] = '\0';
                if (rx_index > 0)
                {
                    UART_SendLine("");
                    Process_Command();
                }
                rx_index = 0;
            }
            last_was_cr = (ch == '\r');
            // Reset debug buffer
            debug_idx = 0;
            debug_buf[0] = '\0';
        }
        else if (rx_index < RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_index++] = ch;
            last_was_cr = 0;
        }
    }
}

// parse CiA 402 state from statusword
uint8_t Parse_CiA402_State(uint16_t sw)
{
    // Handle special cases
    if (sw == 0x0000) return 0;  // Not Ready (initial state)
    if (sw == 0xFFFF) return 255; // Invalid/not connected

    // Check fault first (bit 3)
    if (sw & 0x0008) return 7;  // Fault

    // Check quick stop (bit 5)
    if (sw & 0x0020) {
        // Quick stop is active! Check other bits to determine base state
        uint8_t lower_bits = sw & 0x0007;
        if (lower_bits == 0x07) return 5;  // Quick Stop Active (bits 0,1,2 all set)
        // If in quick stop but not operation enabled, still return the lower bits state
        // But we need to handle this specially in the state machine
    }

    // Standard CiA 402 states (check bits 0-3 and 5-6)
    uint8_t state_bits = sw & 0x006F;

    switch (state_bits) {
        case 0x0000: return 0;  // Not Ready to Switch On
        case 0x0040: return 1;  // Switch On Disabled
        case 0x0021: return 2;  // Ready to Switch On
        case 0x0023: return 3;  // Switched On
        case 0x0027: return 4;  // Operation Enabled
        case 0x0007: return 2;  // Ready to Switch On (variant)
        case 0x0006: return 1;  // Switch On Disabled (KaiserDrive variant)
        default: break;
    }

    // Manufacturer-specific states for KaiserDrive
    switch (sw & 0x027F) {
        case 0x0250: return 1;  // Switch on disabled (with bit 9)
        case 0x0231: return 2;  // Ready to switch on (with bit 9)
        case 0x0233: return 3;  // Switched on (with bit 9)
        case 0x0237: return 4;  // Operation enabled (with bit 9)
        case 0x0210: return 4;  // Operation enabled variant
        case 0x0218: return 7;  // Fault (with bit 9)
    }

    // Handle Quick Stop cases: SW=0x0200, 0x0250, etc.
    if (sw & 0x0020) {
        // Quick stop active - return special state
        // For now, return based on lower bits
        uint8_t lower = sw & 0x000F;
        if (lower == 0x00) return 0;      // Quick stop + Not Ready
        if (lower == 0x01) return 2;     // Quick stop + Ready
        if (lower == 0x03) return 3;     // Quick stop + Switched On
        if (lower == 0x07) return 5;     // Quick stop + Op Enabled
        return 5;  // Default to Quick Stop state
    }

    // Fallback: check individual bits
    if (sw & 0x0040) return 1;  // Switch on disabled (bit 6)
    if ((sw & 0x0007) == 0x0007) return 4;  // All lower bits set = Op Enabled
    if ((sw & 0x0003) == 0x0003) return 3;  // Bits 0,1 set = Switched On
    if (sw & 0x0004) return 4;  // Operation enabled (bit 2)
    if (sw & 0x0002) return 3;  // Switched on (bit 1)
    if (sw & 0x0001) return 2;  // Ready to switch on (bit 0)

    return 255; // Unknown
}

// Get state string
const char* Get_State_String(uint8_t state)
{
    switch (state) {
        case 0: return "Not Ready";
        case 1: return "Switch On Disabled";
        case 2: return "Ready to Switch On";
        case 3: return "Switched On";
        case 4: return "Operation Enabled";
        case 5: return "Quick Stop";
        case 6: return "Fault Reaction";
        case 7: return "Fault";
        default: return "Unknown";
    }
}

// Update LED Status
void Update_LED_Status(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Check for error
    if (ec_slave_count > 0 && (ec_slave[1].state & EC_STATE_ERROR))
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);  // Red ON
        led_blink_period = BLINK_ERROR;
    }
    else
    {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);    // Red OFF
        
        // Set blink period based on state
        if (!ec_initialized) {
            led_blink_period = BLINK_INIT;
        } else if (!ec_operational) {
            led_blink_period = BLINK_PRE_OP;
        } else {
            led_blink_period = BLINK_OPERATIONAL;
        }
    }
    
    // Update Blue LED
    if (led_blink_period == 0) {
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);  // Constant ON
    } else {
        if ((current_time - last_blink_time) >= led_blink_period) {
            last_blink_time = current_time;
            led_state = !led_state;
            HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
}

// Map PDOs for slave
int Map_PDOs(int slave)
{
    // KaiserDrive PDO Configuration (from ESI):
    // RxPDO (SM2): 12 bytes - Controlword(2) + TargetPos(4) + TargetVel(4) + TargetTor(2)
    // TxPDO (SM3): 18 bytes - Statusword(2) + ActualPos(4) + ActualVel(4) + ActualTor(2) + FollowingErr(4) + ErrorCode(2)
    
    // Manually set SM configuration since auto-detection may fail
    ec_slave[slave].SM[2][0] = 0x00;  // StartAddr low
    ec_slave[slave].SM[2][1] = 0x11;  // StartAddr high (0x1100)
    ec_slave[slave].SM[2][2] = 0x0C;  // Length low (12 bytes)
    ec_slave[slave].SM[2][3] = 0x00;  // Length high
    /* SM2 Control = 0x64 as defined in ESI (Outputs, Buffered mode) */
    ec_slave[slave].SM[2][4] = 0x64;  // Control (0x64 = ESI default)
    ec_slave[slave].SM[2][5] = 0x00;  // Status
    ec_slave[slave].SM[2][6] = 0x01;  // Activate
    ec_slave[slave].SM[2][7] = 0x00;  // PDI Control
    
    ec_slave[slave].SM[3][0] = 0x00;  // StartAddr low
    ec_slave[slave].SM[3][1] = 0x1D;  // StartAddr high (0x1D00)
    ec_slave[slave].SM[3][2] = 0x12;  // Length low (18 bytes)
    ec_slave[slave].SM[3][3] = 0x00;  // Length high
    ec_slave[slave].SM[3][4] = 0x20;  // Control (0x20 = READ direction)
    ec_slave[slave].SM[3][5] = 0x00;  // Status
    ec_slave[slave].SM[3][6] = 0x01;  // Activate
    ec_slave[slave].SM[3][7] = 0x00;  // PDI Control
    
    // Mark SM types
    ec_slave[slave].SMtype[2] = EC_SMBUF_PDOUT;
    ec_slave[slave].SMtype[3] = EC_SMBUF_PDIN;
    
    // Manually set output/input bytes
    ec_slave[slave].Obytes = 12;
    ec_slave[slave].Ibytes = 18;
    
    // Set PDO offsets
    ctrl_word_offset = 0;
    target_pos_offset = 2;
    target_vel_offset = 6;
    target_tor_offset = 10;
    
    status_word_offset = 0;
    actual_pos_offset = 2;
    actual_vel_offset = 6;
    actual_tor_offset = 10;
    error_code_offset = 16;
    
    return 0;
}

// Initialize EtherCAT
int EtherCAT_Init(void)
{
    char buf[128];
    int i;
    
    UART_SendLine("");
    UART_SendLine("=== EtherCAT Initialization ===");
    
    // Check Ethernet link
    if (!ETH_GetLinkStatus()) {
        UART_SendLine("Warning: Ethernet link is DOWN!");
        UART_SendLine("Check:");
        UART_SendLine("  1. Use CROSSOVER cable (not straight)");
        UART_SendLine("  2. Motor is powered on");
        UART_SendLine("  3. Cable is firmly connected");
    } else {
        UART_SendLine("Ethernet link is UP");
    }
    
    // Read PHY registers for debugging
    UART_SendLine("Reading PHY registers...");
    extern uint32_t LAN8720A_PHY_ADDRESS;
    extern ETH_HandleTypeDef heth;
    uint32_t phy_id1, phy_id2, phy_bsr, phy_scsr;
    
    // Try to read from both PHY addresses
    for (i = 0; i <= 1; i++) {
        HAL_ETH_ReadPHYRegister(&heth, i, 2, &phy_id1);
        HAL_ETH_ReadPHYRegister(&heth, i, 3, &phy_id2);
        HAL_ETH_ReadPHYRegister(&heth, i, 1, &phy_bsr);
        HAL_ETH_ReadPHYRegister(&heth, i, 31, &phy_scsr);
        
        sprintf(buf, "PHY[%d]: ID1=0x%04X ID2=0x%04X BSR=0x%04X SCSR=0x%04X", 
                i, (uint16_t)phy_id1, (uint16_t)phy_id2, (uint16_t)phy_bsr, (uint16_t)phy_scsr);
        UART_SendLine(buf);
        
        if (phy_id1 != 0 && phy_id1 != 0xFFFF) {
            sprintf(buf, "  -> PHY detected at address %d", i);
            UART_SendLine(buf);
            
            // Parse speed from SCSR
            int speed = (phy_scsr >> 2) & 0x7;
            const char *speed_str;
            switch(speed) {
                case 1: speed_str = "10Mbps Half"; break;
                case 5: speed_str = "10Mbps Full"; break;
                case 2: speed_str = "100Mbps Half"; break;
                case 6: speed_str = "100Mbps Full"; break;
                default: speed_str = "Unknown"; break;
            }
            sprintf(buf, "  -> Link: %s", speed_str);
            UART_SendLine(buf);
        }
    }
    
    sprintf(buf, "Current PHY Address: %lu", LAN8720A_PHY_ADDRESS);
    UART_SendLine(buf);
    
    // Initialize SOEM
    UART_SendLine("Initializing SOEM...");
    
    // If already initialized, close first to ensure clean state
    if (ec_initialized) {
        UART_SendLine("  Closing previous SOEM instance...");
        ec_close();
        ec_initialized = 0;
        HAL_Delay(100);  // Wait for cleanup
    }
    
    if (!ec_init("eth0")) {
        UART_SendLine("ERROR: ec_init failed!");
        return -1;
    }
    UART_SendLine("SOEM initialized OK");

    // Scan for slaves
    UART_SendLine("Scanning for slaves...");
    UART_SendLine("  (Make sure to use CROSSOVER cable!)");

    UART_SendLine("  Calling ec_config_init...");
    
    // Debug: Check if TIM2 is running
    uint32_t tim2_before = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_Delay(10);
    uint32_t tim2_after = __HAL_TIM_GET_COUNTER(&htim2);
    sprintf(buf, "  TIM2 check: before=%lu, after=%lu", tim2_before, tim2_after);
    UART_SendLine(buf);
    
    // Debug: Test simple network communication before ec_config_init
    UART_SendLine("  Testing network communication...");
    uint16_t test_data = 0;
    int test_wkc = ecx_FPRD(&ecx_port, 0x1001, 0x0000, 2, &test_data, 2000);  // 2ms timeout
    sprintf(buf, "  Test read ESC type: wkc=%d, data=0x%04X", test_wkc, test_data);
    UART_SendLine(buf);
    
    // Debug: Add timeout check for ec_config_init
    UART_SendLine("  Starting ec_config_init with timeout protection...");
    ec_slave_count = ec_config_init(FALSE);
    UART_SendLine("  ec_config_init returned");
    sprintf(buf, "Found %d slave(s)", ec_slave_count);
    UART_SendLine(buf);

    // Wait for slave to stabilize after detection
    if (ec_slave_count > 0) {
        UART_SendLine("Waiting for slave to stabilize...");
        UART_SendLine("  Starting 1000ms delay...");
        HAL_Delay(1000);  // Wait 1s for slave to be ready
        UART_SendLine("  1000ms delay done");

        // Additional delay for ESC initialization
        UART_SendLine("Additional delay for ESC init...");
        UART_SendLine("  Starting 500ms delay...");
        HAL_Delay(500);
        UART_SendLine("  500ms delay done");
    }

    if (ec_slave_count < 1) {
        UART_SendLine("ERROR: No slaves found!");
        UART_SendLine("Troubleshooting:");
        UART_SendLine("  1. Use CROSSOVER ethernet cable");
        UART_SendLine("  2. Check motor is powered and in boot state");
        UART_SendLine("  3. Try resetting motor power");
        UART_SendLine("  4. Check cable connections");
        ec_close();
        return -1;
    }

    // Print slave info
    for (i = 1; i <= ec_slave_count; i++) {
        sprintf(buf, "Slave %d: %s (0x%08X:0x%08X)",
                i, ec_slave[i].name, ec_slave[i].eep_man, ec_slave[i].eep_id);
        UART_SendLine(buf);
        
        // CRITICAL: Initialize mailbox counter
        // SOEM doesn't initialize mbx_cnt, which causes random mailbox counter values
        // This can cause the slave to reject mailbox requests
        ec_slave[i].mbx_cnt = 0;
        sprintf(buf, "  Mailbox counter initialized to %d", ec_slave[i].mbx_cnt);
        UART_SendLine(buf);

        uint16_t configadr = ec_slave[i].configadr;

        // Quick test ESC communication
        uint16_t esc_type = 0;
        int wkc = ecx_FPRD(&ecx_port, configadr, 0x0000, 2, &esc_type, EC_TIMEOUTRET);

        if (wkc == 0) {
            UART_SendLine("  WARNING: ESC not responding!");
            UART_SendLine("  Use 'diag' command for detailed diagnosis.");
        } else {
            sprintf(buf, "  ESC Type=0x%04X (WKC=%d)", esc_type, wkc);
            UART_SendLine(buf);
        }
    }

    // ============================================================
    // CRITICAL FIX: Let ec_config_init handle SM0/SM1 configuration
    // Manual configuration causes AL Error 0x0016 on KaiserDrive
    // The slave will use EEPROM defaults for mailbox
    // ============================================================
    UART_SendLine("Using automatic mailbox configuration from EEPROM...");
    
    // Read and display current SM0/SM1 configuration (for debugging)
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;
        uint8_t sm0_cfg[8] = {0};
        uint8_t sm1_cfg[8] = {0};
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM0, 8, sm0_cfg, EC_TIMEOUTSAFE);
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM1, 8, sm1_cfg, EC_TIMEOUTSAFE);
        
        sprintf(buf, "  SM0 (after ec_config_init): %02X %02X %02X %02X %02X %02X %02X %02X",
                sm0_cfg[0], sm0_cfg[1], sm0_cfg[2], sm0_cfg[3],
                sm0_cfg[4], sm0_cfg[5], sm0_cfg[6], sm0_cfg[7]);
        UART_SendLine(buf);
        sprintf(buf, "  SM1 (after ec_config_init): %02X %02X %02X %02X %02X %02X %02X %02X",
                sm1_cfg[0], sm1_cfg[1], sm1_cfg[2], sm1_cfg[3],
                sm1_cfg[4], sm1_cfg[5], sm1_cfg[6], sm1_cfg[7]);
        UART_SendLine(buf);
        
        // Update SOEM internal structure from actual register values
        memcpy(ec_slave[i].SM[0], sm0_cfg, 8);
        memcpy(ec_slave[i].SM[1], sm1_cfg, 8);
        ec_slave[i].mbx_wo = (sm0_cfg[1] << 8) | sm0_cfg[0];
        ec_slave[i].mbx_l = (sm0_cfg[3] << 8) | sm0_cfg[2];
        ec_slave[i].mbx_ro = (sm1_cfg[1] << 8) | sm1_cfg[0];
        ec_slave[i].mbx_rl = (sm1_cfg[3] << 8) | sm1_cfg[2];
        
        sprintf(buf, "  Mailbox: WO=0x%04X, L=%d, RO=0x%04X, RL=%d",
                ec_slave[i].mbx_wo, ec_slave[i].mbx_l,
                ec_slave[i].mbx_ro, ec_slave[i].mbx_rl);
        UART_SendLine(buf);
    }
    
    // ============================================================
    // REQUEST PRE_OP STATE
    // Let the slave handle mailbox activation automatically
    // ============================================================
    UART_SendLine("Requesting PRE_OP state...");
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    
    // Wait for PRE_OP with timeout
    int preop_timeout = 500;  // Increased timeout
    int preop_ready = 0;
    while (preop_timeout-- && !preop_ready) {
        ec_readstate();
        if (ec_slave[1].state == EC_STATE_PRE_OP) {
            preop_ready = 1;
        }
        HAL_Delay(10);  // Check every 10ms
    }
    
    // Check AL status and error code
    ec_readstate();
    uint16_t al_status = ec_slave[1].state;
    uint16_t al_error = ec_slave[1].ALstatuscode;
    sprintf(buf, "  AL Status: 0x%04X, AL Code: 0x%04X", al_status, al_error);
    UART_SendLine(buf);
    
    if (preop_ready && (al_status & 0x0F) == EC_STATE_PRE_OP) {
        UART_SendLine("PRE_OP state reached successfully");
    } else {
        sprintf(buf, "Warning: PRE_OP failed, state=0x%04X", al_status);
        UART_SendLine(buf);
        if (al_error != 0) {
            sprintf(buf, "AL Error Code: 0x%04X", al_error);
            UART_SendLine(buf);
        }
    }

    // ============================================================
    // MAILBOX CONFIGURATION
    // SOEM's ec_config_init already reads mailbox config from EEPROM
    // We just need to verify and activate SM0/SM1
    // ============================================================

    UART_SendLine("Verifying Mailbox (SM0/SM1) configuration...");
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;

        // Read current SM0/SM1 configuration (set by ec_config_init from EEPROM)
        uint8_t sm0_read[8] = {0};
        uint8_t sm1_read[8] = {0};
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM0, 8, sm0_read, EC_TIMEOUTSAFE);
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM1, 8, sm1_read, EC_TIMEOUTSAFE);

        sprintf(buf, "  SM0 (MBoxOut) EEPROM: %02X %02X %02X %02X %02X %02X %02X %02X",
                sm0_read[0], sm0_read[1], sm0_read[2], sm0_read[3],
                sm0_read[4], sm0_read[5], sm0_read[6], sm0_read[7]);
        UART_SendLine(buf);
        sprintf(buf, "  SM1 (MBoxIn) EEPROM:  %02X %02X %02X %02X %02X %02X %02X %02X",
                sm1_read[0], sm1_read[1], sm1_read[2], sm1_read[3],
                sm1_read[4], sm1_read[5], sm1_read[6], sm1_read[7]);
        UART_SendLine(buf);

        // Extract mailbox addresses from SM registers
        uint16_t mbx_wo = sm0_read[0] | (sm0_read[1] << 8);
        uint16_t mbx_l = sm0_read[2] | (sm0_read[3] << 8);
        uint16_t mbx_ro = sm1_read[0] | (sm1_read[1] << 8);
        uint16_t mbx_rl = sm1_read[2] | (sm1_read[3] << 8);

        sprintf(buf, "  Mailbox from EEPROM: WO=0x%04X, L=%d, RO=0x%04X, RL=%d",
                mbx_wo, mbx_l, mbx_ro, mbx_rl);
        UART_SendLine(buf);

        // Verify SOEM's internal mailbox settings match EEPROM
        sprintf(buf, "  SOEM mbx_wo=0x%04X, mbx_l=%d, mbx_ro=0x%04X, mbx_rl=%d",
                ec_slave[i].mbx_wo, ec_slave[i].mbx_l, ec_slave[i].mbx_ro, ec_slave[i].mbx_rl);
        UART_SendLine(buf);

        // CRITICAL: SOEM may not have read EEPROM correctly, manually set mailbox addresses
        // Use default values from ESI file if EEPROM read failed
        if (ec_slave[i].mbx_l == 0) {
            UART_SendLine("  WARNING: SOEM mailbox not configured, using ESI defaults...");
            // Default mailbox configuration from ESI file
            ec_slave[i].mbx_wo = 0x1000;  // Mailbox Out start address
            ec_slave[i].mbx_l = 128;       // Mailbox length
            ec_slave[i].mbx_ro = 0x1080;  // Mailbox In start address
            ec_slave[i].mbx_rl = 128;      // Mailbox read length
            sprintf(buf, "  Using ESI defaults: mbx_wo=0x%04X, mbx_l=%d, mbx_ro=0x%04X, mbx_rl=%d",
                    ec_slave[i].mbx_wo, ec_slave[i].mbx_l, ec_slave[i].mbx_ro, ec_slave[i].mbx_rl);
            UART_SendLine(buf);
        }
        
        // CRITICAL: KaiserDrive EEPROM has SM0/SM1 address/length = 0
        // But SM is already activated (Activate=0x01), causing AL Error 0x0016
        // We MUST configure proper address/length in INIT state before going to PRE_OP
        
        // Update SOEM internal SM structure with current values
        memcpy(ec_slave[i].SM[0], sm0_read, 8);
        memcpy(ec_slave[i].SM[1], sm1_read, 8);
        ec_slave[i].SMtype[0] = 1;  // Mailbox Out
        ec_slave[i].SMtype[1] = 2;  // Mailbox In
        
        // CRITICAL: Use EEPROM configuration for SM0/SM1
        // KaiserDrive EEPROM has SM0=0x26, SM1=0x22 - use these values!
        // SM0 Control 0x26: Direction=0(READ), Mode=10(Mailbox), Int=01
        // SM1 Control 0x22: Direction=0(READ), Mode=10(Mailbox), Int=00
        UART_SendLine("  Using EEPROM SM0/SM1 configuration...");
        
        // Configure SM0 (Mailbox Out): Start=0x1000, Len=128, Control=0x26
        // CRITICAL: Use 0x26 (from EEPROM), NOT 0xA6!
        // Bit 7 = 0 (READ from ESC), Bit 5-6 = 01 (Interrupt), Bit 2-3 = 10 (Mailbox)
        // PDI Control = 0x00 (from EEPROM!)
        uint8_t sm0_cfg[8] = {0x00, 0x10, 0x80, 0x00, 0x26, 0x00, 0x01, 0x00};
        int wkc0 = ecx_FPWR(&ecx_port, configadr, ECT_REG_SM0, 8, sm0_cfg, EC_TIMEOUTSAFE);
        sprintf(buf, "  SM0 config (0x26): wkc=%d", wkc0);
        UART_SendLine(buf);
        
        // Configure SM1 (Mailbox In): Start=0x1080, Len=128, Control=0x22
        // Use 0x22 (from EEPROM) for Mailbox In
        // PDI Control = 0x00 (from EEPROM!)
        uint8_t sm1_cfg[8] = {0x80, 0x10, 0x80, 0x00, 0x22, 0x00, 0x01, 0x00};
        int wkc1 = ecx_FPWR(&ecx_port, configadr, ECT_REG_SM1, 8, sm1_cfg, EC_TIMEOUTSAFE);
        sprintf(buf, "  SM1 config (0x22): wkc=%d", wkc1);
        UART_SendLine(buf);
        
        // Verify configuration
        HAL_Delay(10);
        uint8_t sm0_verify[8], sm1_verify[8];
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM0, 8, sm0_verify, EC_TIMEOUTSAFE);
        ecx_FPRD(&ecx_port, configadr, ECT_REG_SM1, 8, sm1_verify, EC_TIMEOUTSAFE);
        
        uint16_t vstart = sm0_verify[0] | (sm0_verify[1] << 8);
        uint16_t vlen = sm0_verify[2] | (sm0_verify[3] << 8);
        uint8_t vctrl = sm0_verify[4];
        sprintf(buf, "  SM0 after config: Start=0x%04X, Len=%d, Ctrl=0x%02X, Act=0x%02X", 
                vstart, vlen, vctrl, sm0_verify[6]);
        UART_SendLine(buf);
        
        // Update SOEM internal variables
        memcpy(ec_slave[i].SM[0], sm0_verify, 8);
        memcpy(ec_slave[i].SM[1], sm1_verify, 8);
    }

    // ============================================================
    // CRITICAL: Enable DC sync mode (DC-Synchronous mode)
    // KaiserDrive requires DC sync mode to enter Operation Enabled
    // 
    // CRITICAL FIX: SYNC0 must be configured with correct DC start time
    // DCSTART0 = current_DC_time + shift + margin (not just shift)
    // ============================================================
    UART_SendLine("Enabling DC sync (DC-Synchronous mode)...");
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;

        // Set AssignActivate to 0x0300 (DC-Synchronous mode, like IGH)
        uint16_t assign_activate = 0x0300;
        int wkc_aa = ecx_FPWR(&ecx_port, configadr, ECT_REG_DCCUC, 2, &assign_activate, EC_TIMEOUTSAFE);
        sprintf(buf, "  AssignActivate set to 0x%04X (DC-Sync, wkc=%d)", assign_activate, wkc_aa);
        UART_SendLine(buf);
        
        // CRITICAL: Manually configure SYNC0 with correct DC timing
        // ecx_dcsync0 writes shift to DCSTART0, but it should be (current_DC_time + shift)
        UART_SendLine("  Configuring SYNC0 with correct DC timing...");
        
        // Step 1: Deactivate SYNC0 first
        uint8_t dc_act = 0;
        ecx_FPWR(&ecx_port, configadr, ECT_REG_DCSYNCACT, 1, &dc_act, EC_TIMEOUTSAFE);
        
        // Step 2: Read current DC system time from slave
        uint64_t dc_current_time = 0;
        int wkc_dc = ecx_FPRD(&ecx_port, configadr, ECT_REG_DCSYSTIME, 8, &dc_current_time, EC_TIMEOUTSAFE);
        if (wkc_dc > 0) {
            // Convert from little-endian if needed (ET1100 returns little-endian)
            dc_current_time = etohll(dc_current_time);
            sprintf(buf, "  Current DC time: %llu ns", dc_current_time);
            UART_SendLine(buf);
        } else {
            UART_SendLine("  Warning: Could not read DC time, using 0");
            dc_current_time = 0;
        }
        
        // Step 3: Set SYNC0 cycle time (2ms)
        uint32_t cycle_time = DC_SYNC0_CYCLE_TIME;
        ecx_FPWR(&ecx_port, configadr, ECT_REG_DCCYCLE0, 4, &cycle_time, EC_TIMEOUTSAFE);
        
        // Step 4: CRITICAL - Set SYNC0 start time to (current_DC_time + shift + margin)
        // Add 10ms margin to ensure SYNC0 starts in the future
        uint64_t start_time = dc_current_time + DC_SYNC0_SHIFT + 10000000; 
        ecx_FPWR(&ecx_port, configadr, ECT_REG_DCSTART0, 8, &start_time, EC_TIMEOUTSAFE);
        sprintf(buf, "  SYNC0 start time: %llu ns (DC+%llu ns)", start_time, DC_SYNC0_SHIFT + 10000000);
        UART_SendLine(buf);
        
        // Step 5: Activate SYNC0
        dc_act = 0x03;  // SYNC0 active, cyclic mode
        ecx_FPWR(&ecx_port, configadr, ECT_REG_DCSYNCACT, 1, &dc_act, EC_TIMEOUTSAFE);
        
        // Update slave DC state
        ec_slave[i].DCactive = 1;
        ec_slave[i].DCcycle = DC_SYNC0_CYCLE_TIME;
        ec_slave[i].DCshift = DC_SYNC0_SHIFT;
        
        sprintf(buf, "  SYNC0 activated: cycle=%d ms, shift=%d ms", 
                DC_SYNC0_CYCLE_TIME / 1000000, DC_SYNC0_SHIFT / 1000000);
        UART_SendLine(buf);
    }
    
    // Enable DC in group
    ec_group[0].hasdc = TRUE;
    ec_group[0].DCnext = 1;
    UART_SendLine("  DC sync enabled (DC-Synchronous mode)");
    
    // CRITICAL: Wait for SYNC0 to start (send some PDO frames)
    UART_SendLine("  Waiting for SYNC0 to stabilize...");
    for (int j = 0; j < 30; j++) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        HAL_Delay(2);
    }
    UART_SendLine("  DC/SYNC0 synchronized");
    HAL_Delay(50);

    // ============================================================
    // CRITICAL FIX: Skip all SDO communication
    // IGH configures SDO during slave configuration phase (before activation)
    // SOEM cannot use mailbox SDO in INIT/PRE_OP state with KaiserDrive
    // We will skip SDO and configure PDO manually like IGH does
    // ============================================================
    UART_SendLine("Skipping SDO communication (KaiserDrive compatibility)...");
    UART_SendLine("  Note: SDO will be configured manually like IGH");
    
    // Set default operation mode in software (actual mode set via PDO)
    UART_SendLine("  Setting default operation mode to CSV (9) in software");
    // The mode will be set via Control Word and PDO mapping

    // ============================================================
    // PDO CONFIGURATION (like IGH)
    // Skip PRE_OP, directly configure PDO and go to OPERATIONAL
    // ============================================================
    UART_SendLine("Configuring PDO mapping (IGH-style)...");
    
    // Manually set PDO assignment (from ESI file)
    // KaiserDrive default: 0x1601 (Rx), 0x1A01 (Tx)
    for (i = 1; i <= ec_slave_count; i++) {
        // Set PDO assignment in ec_slave structure
        // SM[2] is outputs: SM[i][2]=len_low, SM[i][3]=len_high
        ec_slave[i].SM[2][2] = 0x0C;  // Length low byte = 12
        ec_slave[i].SM[2][3] = 0x00;  // Length high byte = 0
        ec_slave[i].SM[3][2] = 0x12;  // Length low byte = 18
        ec_slave[i].SM[3][3] = 0x00;  // Length high byte = 0
        
        sprintf(buf, "  Slave %d: PDO Assignment 0x1601/0x1A01, SM2=%d bytes, SM3=%d bytes",
                i, ec_slave[i].SM[2][2] | (ec_slave[i].SM[2][3] << 8), 
                ec_slave[i].SM[3][2] | (ec_slave[i].SM[3][3] << 8));
        UART_SendLine(buf);
    }
    
    UART_SendLine("PDO configuration complete (SDO-less mode)");
    
    // Skip SDO communication - will be set via PDO
    UART_SendLine("Skipping SDO configuration (using PDO defaults)");
    
    UART_SendLine("Using default PDO Assignment from ESI: 0x1601 (Rx), 0x1A01 (Tx)");
    
    // Manually set PDO Assignment defaults based on ESI file
    UART_SendLine("Setting default PDO Assignment manually...");
    
    // RxPDO Assignment: 0x1C12[0] = 1, 0x1C12[1] = 0x1601
    // SM[2] is PDO Outputs - set length in bytes 2-3 (little endian)
    ec_slave[1].SM[2][2] = 0x0C;  // Length low byte = 12
    ec_slave[1].SM[2][3] = 0x00;  // Length high byte = 0
    ec_slave[1].SMtype[2] = 3;    // Outputs
    
    // TxPDO Assignment: 0x1C13[0] = 1, 0x1C13[1] = 0x1A01
    // SM[3] is PDO Inputs - set length in bytes 2-3 (little endian)
    ec_slave[1].SM[3][2] = 0x12;  // Length low byte = 18
    ec_slave[1].SM[3][3] = 0x00;  // Length high byte = 0
    ec_slave[1].SMtype[3] = 4;    // Inputs
    
    UART_SendLine("Default PDO Assignment set: 0x1601 (Rx), 0x1A01 (Tx)");

    // CRITICAL: Configure SM2/SM3 BEFORE calling ec_config_map_group
    // ec_config_map_group reads SM registers to determine PDO sizes
    // If SM2/SM3 are not configured, Obytes/Ibytes will be 0
    UART_SendLine("Configuring SM2/SM3 for PDO communication...");
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;
        
        // SM2 (PDO Outputs) at 0x1100, length=12 bytes
        // Control=0x64 per ESI (Outputs, Buffered mode)
        // PDI Control=0x01: Enable PDI access to this SM
        uint8_t sm2_cfg[8] = {0x00, 0x11, 0x0C, 0x00, 0x64, 0x00, 0x01, 0x01};
        int wkc_sm2 = ecx_FPWR(&ecx_port, configadr, ECT_REG_SM2, 8, sm2_cfg, EC_TIMEOUTSAFE);
        sprintf(buf, "  SM2 configured: Start=0x1100, Len=12, Ctrl=0x64, PDI=0x01, wkc=%d", wkc_sm2);
        UART_SendLine(buf);
        
        // SM3 (PDO Inputs) at 0x1D00, length=18 bytes
        // Control=0x20 per ESI (Inputs mode)
        // PDI Control=0x01: Enable PDI access to this SM
        uint8_t sm3_cfg[8] = {0x00, 0x1D, 0x12, 0x00, 0x20, 0x00, 0x01, 0x01};
        int wkc_sm3 = ecx_FPWR(&ecx_port, configadr, ECT_REG_SM3, 8, sm3_cfg, EC_TIMEOUTSAFE);
        sprintf(buf, "  SM3 configured: Start=0x1D00, Len=18, Ctrl=0x20, PDI=0x01, wkc=%d", wkc_sm3);
        UART_SendLine(buf);
        
        // CRITICAL: Also update the internal SM structure
        memcpy(ec_slave[i].SM[2], sm2_cfg, 8);
        memcpy(ec_slave[i].SM[3], sm3_cfg, 8);
    }

    // Now call ec_config_map_group - it will read SM2/SM3 registers
    // CRITICAL FIX: Modified SOEM source to use correct FMMU Type values
    // FMMU0 Type: 0x02 (outputs, master writes)
    // FMMU1 Type: 0x01 (inputs, master reads)
    UART_SendLine("Calling ec_config_map_group (with fixed FMMU Type)...");
    int pdo_configured = ec_config_map_group(&ec_IOmap, 0);
    sprintf(buf, "  ec_config_map_group returned: %d", pdo_configured);
    UART_SendLine(buf);
    sprintf(buf, "  Group Obytes=%d, Ibytes=%d", ec_group[0].Obytes, ec_group[0].Ibytes);
    UART_SendLine(buf);
    
    // DEBUG: Check outputs/inputs pointers
    sprintf(buf, "  DEBUG: outputs=%p, inputs=%p, IOmap=%p", 
            ec_group[0].outputs, ec_group[0].inputs, ec_IOmap);
    UART_SendLine(buf);
    sprintf(buf, "  DEBUG: outputs offset=%d, inputs offset=%d",
            (int)(ec_group[0].outputs - ec_IOmap), 
            (int)(ec_group[0].inputs - ec_IOmap));
    UART_SendLine(buf);

    if (ec_group[0].Obytes == 0 || ec_group[0].Ibytes == 0) {
        UART_SendLine("  ERROR: PDO mapping failed, manually setting Obytes/Ibytes!");
        ec_group[0].Obytes = 12;  // SM2 length
        ec_group[0].Ibytes = 18;  // SM3 length
        ec_slave[1].Obytes = 12;
        ec_slave[1].Ibytes = 18;
        sprintf(buf, "  Manually set: Obytes=%d, Ibytes=%d", ec_group[0].Obytes, ec_group[0].Ibytes);
        UART_SendLine(buf);
    }

    // SKIP SDO configuration - Mailbox not responding
    // Assume slave has default PDO Assignment from ESI (0x1601/0x1A01)
    UART_SendLine("Skipping SDO PDO Assignment (Mailbox not responding)");
    UART_SendLine("Assuming default PDO Assignment from ESI: 0x1601 (Rx), 0x1A01 (Tx)");
    
    // Manually configure PDO Assignment in ec_slave structure
    // This is needed because SDO communication is not working
    UART_SendLine("Manually configuring PDO Assignment in ec_slave structure...");
    for (i = 1; i <= ec_slave_count; i++) {
        // Set SM2 type to outputs (3)
        ec_slave[i].SMtype[2] = 3;  // EC_SMBUF_PDOUT
        // Set SM3 type to inputs (4)
        ec_slave[i].SMtype[3] = 4;  // EC_SMBUF_PDIN
        
        // Set SM2/SM3 lengths based on ESI file
        // SM2: 12 bytes (RxPDO 0x1601)
        ec_slave[i].SM[2][2] = 12;  // Length low byte
        ec_slave[i].SM[2][3] = 0;   // Length high byte
        // SM3: 18 bytes (TxPDO 0x1A01)
        ec_slave[i].SM[3][2] = 18;  // Length low byte
        ec_slave[i].SM[3][3] = 0;   // Length high byte
        
        sprintf(buf, "  Slave %d: SM2=outputs(%d bytes), SM3=inputs(%d bytes)", 
                i, 12, 18);
        UART_SendLine(buf);
    }
    
    // FMMU is configured by ec_config_map_group with CORRECT Type values (fixed in SOEM source)
    UART_SendLine("FMMU configured by ec_config_map_group (Type fixed in SOEM source)");
    
    // Clear AL Error Code register before state transition
    UART_SendLine("Clearing AL Error Code register...");
    uint16_t clear_error = 0;
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;
        int wkc_clr = ecx_FPWR(&ecx_port, configadr, ECT_REG_ALSTATCODE, 2, &clear_error, EC_TIMEOUTSAFE);
        sprintf(buf, "  AL Error Code cleared for slave %d (wkc=%d)", i, wkc_clr);
        UART_SendLine(buf);
    }

    // Wait for ESC to process configuration
    HAL_Delay(50);

    // SM2 uses ESI-defined value 0x64 (Outputs, Buffered mode)
    UART_SendLine("SM2 configuration: 0x64 (ESI default for Outputs)");

    // DC sync was already configured in INIT state
    // Just verify it's still enabled
    UART_SendLine("Verifying DC sync configuration...");
    sprintf(buf, "  DC hasdc=%d, DCnext=%d", ec_group[0].hasdc, ec_group[0].DCnext);
    UART_SendLine(buf);

    // Request SAFE_OP state
    UART_SendLine("Requesting SAFE_OP state...");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);

    int safeop_chk = 200;
    do {
        ec_readstate();
        HAL_Delay(1);
    } while (safeop_chk-- && (ec_slave[1].state != EC_STATE_SAFE_OP));

    if (ec_slave[1].state != EC_STATE_SAFE_OP) {
        sprintf(buf, "Warning: SAFE_OP failed, state=0x%04X", ec_slave[1].state);
        UART_SendLine(buf);
    } else {
        UART_SendLine("SAFE_OP state reached");
    }

    // Note: SM2 direction 0x64 (READ) is correct per ESI - no modification needed
    // Diagnostic: Check SM configuration
    UART_SendLine("Checking SM configuration...");
    uint8_t sm2[8], sm3[8];
    int wkc_sm2 = ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_SM2, 8, sm2, EC_TIMEOUTSAFE);
    int wkc_sm3 = ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_SM3, 8, sm3, EC_TIMEOUTSAFE);
    sprintf(buf, "SM2: %02X %02X %02X %02X %02X %02X %02X %02X (wkc=%d)",
            sm2[0], sm2[1], sm2[2], sm2[3], sm2[4], sm2[5], sm2[6], sm2[7], wkc_sm2);
    UART_SendLine(buf);
    sprintf(buf, "SM3: %02X %02X %02X %02X %02X %02X %02X %02X (wkc=%d)",
            sm3[0], sm3[1], sm3[2], sm3[3], sm3[4], sm3[5], sm3[6], sm3[7], wkc_sm3);
    UART_SendLine(buf);

    // If SM Act bit is disabled (byte 6, bit 0), re-enable it
    // SM Register layout: [0-1]=Addr, [2-3]=Len, [4]=Control, [5]=Status, [6]=Activate
    if ((sm2[6] & 0x01) == 0) {
        UART_SendLine("WARNING: SM2 Act bit is disabled, re-enabling...");
        sm2[6] |= 0x01;
        int wkc = ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_SM2, 8, sm2, EC_TIMEOUTSAFE);
        sprintf(buf, "SM2 re-enabled (wkc=%d)", wkc);
        UART_SendLine(buf);
    }
    if ((sm3[6] & 0x01) == 0) {
        UART_SendLine("WARNING: SM3 Act bit is disabled, re-enabling...");
        sm3[6] |= 0x01;
        int wkc = ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_SM3, 8, sm3, EC_TIMEOUTSAFE);
        sprintf(buf, "SM3 re-enabled (wkc=%d)", wkc);
        UART_SendLine(buf);
    }

    // Check AL status and error code
    uint16_t alstat = 0;
    int wkc_al = ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_ALSTAT, 2, &alstat, EC_TIMEOUTSAFE);
    sprintf(buf, "AL Status: 0x%04X (wkc=%d), ErrorCode: 0x%04X",
            etohs(alstat), wkc_al, ec_slave[1].ALstatuscode);
    UART_SendLine(buf);

    // SKIP SDO communication in SAFE_OP state - Mailbox not working
    // Use default PDO Assignment from ESI file
    UART_SendLine("Skipping SDO in SAFE_OP state (Mailbox not responding)");
    UART_SendLine("Using default PDO Assignment: 0x1601 (Rx), 0x1A01 (Tx)");

    // Initialize PDO offsets based on standard mapping
    // These offsets are relative to the process data buffer
    ctrl_word_offset = 0;      // Controlword at byte 0-1
    target_pos_offset = 2;     // Target position at byte 2-5
    target_vel_offset = 6;     // Target velocity at byte 6-9
    target_tor_offset = 10;    // Target torque at byte 10-11

    status_word_offset = 0;    // Statusword at byte 0-1
    actual_pos_offset = 2;     // Actual position at byte 2-5
    actual_vel_offset = 6;     // Actual velocity at byte 6-9
    actual_tor_offset = 10;    // Actual torque at byte 10-11
    error_code_offset = 16;    // Error code at byte 16-17

    sprintf(buf, "PDO offsets: ctrl=%d, pos=%d, vel=%d, tor=%d",
            ctrl_word_offset, target_pos_offset, target_vel_offset, target_tor_offset);
    UART_SendLine(buf);

    // CRITICAL: Before requesting OPERATIONAL, send some process data frames
    // Some slaves require valid PDO data before entering OPERATIONAL
    UART_SendLine("Preparing for OPERATIONAL transition...");
    UART_SendLine("  Sending initial process data frames...");
    
    // Initialize process data buffer with safe values
    memset(ec_IOmap, 0, sizeof(ec_IOmap));
    
    // Send a few process data cycles to establish communication
    for (int cycle = 0; cycle < 10; cycle++) {
        ec_send_processdata();
        HAL_Delay(1);
        ec_receive_processdata(EC_TIMEOUTRET);
        HAL_Delay(1);
    }
    UART_SendLine("  Initial process data sent");
    
    // TEST: Free Run mode - no DC sync maintenance needed
    
    // Request OPERATIONAL state
    UART_SendLine("Requesting OPERATIONAL state...");
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    int op_chk = 200;
    ec_operational = 0;
    do {
        ec_readstate();
        HAL_Delay(1);
        if (ec_slave[1].state == EC_STATE_OPERATIONAL) {
            ec_operational = 1;
        }
    } while (op_chk-- && !ec_operational);

    // CRITICAL: Re-enable SM Act bits after OPERATIONAL request
    // ESC may reset these during state transition
    UART_SendLine("Re-checking SM Act bits after OP transition...");
    uint8_t sm2_check[8], sm3_check[8];
    ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_SM2, 8, sm2_check, EC_TIMEOUTSAFE);
    ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_SM3, 8, sm3_check, EC_TIMEOUTSAFE);
    sprintf(buf, "  SM2: %02X %02X %02X %02X %02X %02X %02X %02X",
            sm2_check[0], sm2_check[1], sm2_check[2], sm2_check[3],
            sm2_check[4], sm2_check[5], sm2_check[6], sm2_check[7]);
    UART_SendLine(buf);
    sprintf(buf, "  SM3: %02X %02X %02X %02X %02X %02X %02X %02X",
            sm3_check[0], sm3_check[1], sm3_check[2], sm3_check[3],
            sm3_check[4], sm3_check[5], sm3_check[6], sm3_check[7]);
    UART_SendLine(buf);
    
    // CRITICAL: Check FMMU configuration
    UART_SendLine("Checking FMMU configuration...");
    uint8_t fmmu0[16], fmmu1[16], fmmu2[16];
    ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_FMMU0, 16, fmmu0, EC_TIMEOUTSAFE);
    ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_FMMU1, 16, fmmu1, EC_TIMEOUTSAFE);
    ecx_FPRD(&ecx_port, ec_slave[1].configadr, ECT_REG_FMMU2, 16, fmmu2, EC_TIMEOUTSAFE);
    sprintf(buf, "  FMMU0: LogAddr=%02X%02X%02X%02X Len=%02X%02X Type=%02X En=%02X",
            fmmu0[3], fmmu0[2], fmmu0[1], fmmu0[0], fmmu0[5], fmmu0[4], fmmu0[11], fmmu0[12]);
    UART_SendLine(buf);
    sprintf(buf, "  FMMU0: PhysAddr=%02X%02X StartBit=%02X EndBit=%02X",
            fmmu0[9], fmmu0[8], fmmu0[10], fmmu0[7]);
    UART_SendLine(buf);
    sprintf(buf, "  FMMU1: LogAddr=%02X%02X%02X%02X Len=%02X%02X Type=%02X En=%02X",
            fmmu1[3], fmmu1[2], fmmu1[1], fmmu1[0], fmmu1[5], fmmu1[4], fmmu1[11], fmmu1[12]);
    UART_SendLine(buf);
    sprintf(buf, "  FMMU1: PhysAddr=%02X%02X StartBit=%02X EndBit=%02X",
            fmmu1[9], fmmu1[8], fmmu1[10], fmmu1[7]);
    UART_SendLine(buf);
    sprintf(buf, "  FMMU2: LogAddr=%02X%02X%02X%02X Len=%02X%02X Type=%02X En=%02X",
            fmmu2[3], fmmu2[2], fmmu2[1], fmmu2[0], fmmu2[5], fmmu2[4], fmmu2[11], fmmu2[12]);
    UART_SendLine(buf);
    sprintf(buf, "  FMMU2: PhysAddr=%02X%02X StartBit=%02X EndBit=%02X",
            fmmu2[9], fmmu2[8], fmmu2[10], fmmu2[7]);
    UART_SendLine(buf);
    
    // FMMU is already configured by ec_config_map_group with CORRECT Type values
    // (Fixed in SOEM source: FMMU0 Type=0x02, FMMU1 Type=0x01)
    
    // CRITICAL: Configure FMMU2 for MBoxState if not already configured
    // ESI shows FMMU2 is used for mailbox state monitoring
    // FMMU2 maps SM0STAT bit3 (Mailbox Full) to logical address
    if (fmmu2[12] == 0) {
        UART_SendLine("  CRITICAL: FMMU2 not configured, configuring for MBoxState...");
        // FMMU2 per ESI: LogAddr=0x00000010, Len=1 bit, PhysAddr=0x0805 (SM0 status)
        uint8_t fmmu2_cfg[16] = {
            0x10, 0x00, 0x00, 0x00,  // Logical start address: 0x00000010
            0x01, 0x00,              // Length: 1 bit
            0x03,                    // Start bit: 3 (SM0stat bit3 = Mailbox Full)
            0x03,                    // End bit: 3
            0x05, 0x08,              // Physical address: 0x0805 (SM0 status register)
            0x00,                    // Physical start bit
            0x01,                    // Type: 0x01 = Inputs (Read from slave)
            0x01,                    // Enable
            0x00, 0x00, 0x00         // Reserved
        };
        int wkc_fmmu2 = ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_FMMU2, 16, fmmu2_cfg, EC_TIMEOUTSAFE);
        sprintf(buf, "  FMMU2 configured (MBoxState): wkc=%d", wkc_fmmu2);
        UART_SendLine(buf);
    }

    if ((sm2_check[6] & 0x01) == 0) {
        UART_SendLine("  WARNING: SM2 Act bit disabled, re-enabling...");
        sm2_check[6] |= 0x01;
        ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_SM2, 8, sm2_check, EC_TIMEOUTSAFE);
    }
    if ((sm3_check[6] & 0x01) == 0) {
        UART_SendLine("  WARNING: SM3 Act bit disabled, re-enabling...");
        sm3_check[6] |= 0x01;
        ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_SM3, 8, sm3_check, EC_TIMEOUTSAFE);
    }

    if (ec_operational) {
        UART_SendLine("OPERATIONAL state reached!");
        
        // CRITICAL: Try SDO communication in OPERATIONAL state
        // Some slaves only process mailbox in OPERATIONAL state
        UART_SendLine("Testing SDO communication in OPERATIONAL state...");
        HAL_Delay(100);  // Wait for slave to stabilize
        {
            uint8_t rxpdo_count = 0;
            int rxpdo_size = sizeof(rxpdo_count);
            int wkc_sdo = ecx_SDOread(&ecx_context, 1, 0x1C12, 0x00, FALSE, &rxpdo_size, &rxpdo_count, EC_TIMEOUTSAFE);
            sprintf(buf, "  SDO read 0x1C12:0 (RxPDO count): wkc=%d, value=%d", wkc_sdo, rxpdo_count);
            UART_SendLine(buf);
            
            if (wkc_sdo > 0) {
                UART_SendLine("  SDO in OPERATIONAL state WORKS!");
            } else {
                UART_SendLine("  SDO in OPERATIONAL state failed");
            }
        }
    } else {
        sprintf(buf, "Warning: OPERATIONAL timeout, slave state=0x%04X", ec_slave[1].state);
        UART_SendLine(buf);
        UART_SendLine("Using SAFE_OP for CiA402 state machine...");
    }

    // Note: SM2 direction 0x64 (READ) is correct per ESI - no modification needed

    // CRITICAL: PDO Assignment (0x1C12/0x1C13) is stored in CoE object dictionary
    // It can only be accessed via SDO (Mailbox), not directly via ESC registers
    // Since Mailbox is not responding, we assume slave uses default PDO Assignment:
    //   RxPDO: 0x1601 (Controlword, Target Position, Target Velocity, Target Torque)
    //   TxPDO: 0x1A01 (Statusword, Actual Position, Actual Velocity, Actual Torque, ...)
    UART_SendLine("PDO Assignment: Using ESI defaults (0x1601/0x1A01)");
    UART_SendLine("  Note: PDO Assignment is in CoE object dictionary (SDO only)");

    // Mark as initialized
    ec_initialized = 1;
    init_phase = 1;  // Start CiA402 initialization phase
    init_start_time = 0;

    UART_SendLine("EtherCAT initialization complete!");
    if (ec_operational || ec_slave[1].state == EC_STATE_SAFE_OP) {
        UART_SendLine("PDO communication ready (waiting for WKC > 0)");
    } else {
        UART_SendLine("WARNING: Slave not in OPERATIONAL or SAFE_OP state!");
    }

    return 0;
}


// Motor State Machine - CSV/CSP Mode Implementation
void Motor_State_Machine(void)
{
    static uint32_t last_cycle = 0;
    static uint8_t in_op = 0;           // Flag: motor in Operation Enabled
    static uint8_t op_requested = 0;    // Flag: enable operation requested
    uint32_t now = HAL_GetTick();

    // CRITICAL: For DC sync mode, PDO cycle must match SYNC0 cycle (2ms = 500Hz)
    // SM watchdog will trigger if PDO is not received within expected cycle time
    if ((now - last_cycle) < 2) return;  // 500Hz = 2ms period (matching SYNC0)
    last_cycle = now;

    if (!ec_initialized || ec_slave_count < 1) return;
    // Run state machine even in SAFE_OP (not just OPERATIONAL)
    // CiA402 state machine can work in SAFE_OP mode

    // Get process data pointers from ec_group[0] - SOEM standard way
    volatile uint8_t *outputs = ec_group[0].outputs;
    volatile uint8_t *inputs = ec_group[0].inputs;

    if (!outputs || !inputs) {
        UART_SendLine("[Motor SM] ERROR: outputs or inputs is NULL!");
        return;
    }

    // Check if group has valid PDO data
    if (ec_group[0].Obytes == 0 && ec_group[0].Ibytes == 0) {
        UART_SendLine("[Motor SM] ERROR: Obytes and Ibytes are 0!");
        return;
    }

    // Read inputs with bounds checking - use memcpy to avoid alignment issues
    uint16_t new_statusword = 0;
    int32_t new_actual_position = 0;
    int32_t new_actual_velocity = 0;
    uint16_t new_error_code = 0;

    // Check if we have enough input bytes before reading
    if (ec_group[0].Ibytes >= (status_word_offset + 2)) {
        memcpy(&new_statusword, (void *)(inputs + (int)status_word_offset), sizeof(uint16_t));
    }
    if (ec_group[0].Ibytes >= (actual_pos_offset + 4)) {
        memcpy(&new_actual_position, (void *)(inputs + (int)actual_pos_offset), sizeof(int32_t));
    }
    if (ec_group[0].Ibytes >= (actual_vel_offset + 4)) {
        memcpy(&new_actual_velocity, (void *)(inputs + (int)actual_vel_offset), sizeof(int32_t));
    }
    if (ec_group[0].Ibytes >= (error_code_offset + 2)) {
        memcpy(&new_error_code, (void *)(inputs + (int)error_code_offset), sizeof(uint16_t));
    }

    // Update global variables
    statusword = new_statusword;
    actual_position = new_actual_position;
    actual_velocity_csv = new_actual_velocity;
    error_code = new_error_code;

    // Parse state
    cia402_state = Parse_CiA402_State(statusword);

    // Handle state changes and fault recovery
    static uint32_t last_fault_time = 0;
    static uint8_t last_state = 255;
    if (cia402_state != last_state) {
        last_state = cia402_state;
        char buf[64];
        sprintf(buf, "State change: %s (SW:0x%04X)", Get_State_String(cia402_state), statusword);
        UART_SendLine(buf);
    }

    /* CiA402 Initialization Phase
     * Handles initial wake-up and Quick Stop exit
     * SW=0x0200 = Quick Stop active + Not Ready
     */
    if (init_phase) {
        uint32_t now = HAL_GetTick();

        if (init_start_time == 0) {
            init_start_time = now;
            UART_SendLine("[Init] CiA402 wake-up started");
        }

        uint16_t current_sw = statusword;
        uint8_t current_state = Parse_CiA402_State(current_sw);

        // Log state every 500ms
        static uint32_t last_init_log = 0;
        if ((now - last_init_log) > 500) {
            last_init_log = now;
            char buf[64];
            sprintf(buf, "[Init] State=%d SW=0x%04X", current_state, current_sw);
            UART_SendLine(buf);
        }

        // Handle state transitions
        // CRITICAL: If statusword is 0, we may have WKC=0 (no PDO data)
        // Still try to send controlword to wake up the drive
        if (current_sw == 0x0000) {
            // No status from drive - try sending wake-up sequence
            static uint8_t wake_up_step = 0;
            wake_up_step++;
            if (wake_up_step % 4 == 0) {
                controlword = 0x0000;  // Wake-up
            } else if (wake_up_step % 4 == 1) {
                controlword = 0x0006;  // Shutdown
            } else if (wake_up_step % 4 == 2) {
                controlword = 0x0007;  // Switch On
            } else {
                controlword = 0x000F;  // Enable Operation
            }
            if ((now - init_start_time) % 500 < 100) {
                char buf[64];
                sprintf(buf, "[Init] No response (SW=0), trying wake-up step %d, CW=0x%04X", wake_up_step % 4, controlword);
                UART_SendLine(buf);
            }
        }
        else if (current_state == 0) {  // Not Ready (including Quick Stop case)
            // Check if Quick Stop is active (bit 5)
            if (current_sw & 0x0020) {
                // Quick Stop active - need to enter Operation Enabled to clear
                controlword = 0x000F;  // Enable Operation
            } else {
                // Normal Not Ready - send Enable Voltage
                controlword = 0x0006;  // Shutdown (Enable voltage)
            }
        }
        else if (current_state == 7) {  // Fault
            controlword = 0x0080;  // Fault Reset
        }
        else if (current_state >= 1 && current_state <= 4) {
            // Good! We're in a valid operational state
            UART_SendLine("[Init] Drive ready!");
            init_phase = 0;  // Exit init phase
        }
        else if (current_state == 5) {  // Quick Stop
            // In Quick Stop state - need to go to Operation Enabled
            controlword = 0x000F;  // Enable Operation
            UART_SendLine("[Init] Quick Stop active, trying to exit...");
        }

        // Force exit after 5 seconds (increased from 2 seconds)
        if ((now - init_start_time) > 5000) {
            UART_SendLine("[Init] Timeout - exiting init phase");
            init_phase = 0;  // Exit init phase
        }

        // Write controlword and return
        uint16_t cw = controlword;
        memcpy((void *)(ec_group[0].outputs + (int)ctrl_word_offset), &cw, sizeof(uint16_t));
        return;
    }

    // Post-init phase: continue enabling until motor is ready
    // This runs after init_phase timeout but before normal state machine
    {
        static uint8_t post_init_phase = 1;
        uint16_t current_sw = statusword;
        uint8_t current_state = Parse_CiA402_State(current_sw);

        // If motor is in a good state, exit post-init
        if (current_state >= 1 && current_state <= 4) {
            post_init_phase = 1;
        }

        // During post-init phase, keep sending enable commands
        if (post_init_phase) {
            if (current_sw == 0x0000) {
                // Still no response from drive - continue wake-up sequence
                static uint8_t wake_step = 0;
                static uint32_t last_wake_time = 0;
                uint32_t now = HAL_GetTick();
                
                if ((now - last_wake_time) > 200) {  // Change step every 200ms
                    last_wake_time = now;
                    wake_step = (wake_step + 1) % 4;
                }
                
                if (wake_step == 0) controlword = 0x0000;
                else if (wake_step == 1) controlword = 0x0006;
                else if (wake_step == 2) controlword = 0x0007;
                else controlword = 0x000F;
                
                uint16_t cw = controlword;
                memcpy((void *)(ec_group[0].outputs + (int)ctrl_word_offset), &cw, sizeof(uint16_t));
                
                static uint32_t last_post_log = 0;
                if ((now - last_post_log) > 1000) {
                    last_post_log = now;
                    char buf[64];
                    sprintf(buf, "[PostInit] No response, wake step=%d CW=0x%04X", wake_step, cw);
                    UART_SendLine(buf);
                }
                return;  // Skip normal state machine during post-init
            }
            else if (current_state == 0 || current_state == 5 || current_state == 7) {
                // Still in problematic state - send appropriate command
                if (current_state == 7) {
                    controlword = 0x0080;  // Fault Reset
                } else if (current_sw & 0x0020) {
                    controlword = 0x000F;  // Quick Stop - need Enable
                } else {
                    controlword = 0x0006;  // Not Ready - Shutdown (Enable voltage)
                }

                uint16_t cw = controlword;
                memcpy((void *)(ec_group[0].outputs + (int)ctrl_word_offset), &cw, sizeof(uint16_t));

                static uint32_t last_post_log = 0;
                uint32_t now = HAL_GetTick();
                if ((now - last_post_log) > 500) {
                    last_post_log = now;
                    char buf[64];
                    sprintf(buf, "[PostInit] State=%d SW=0x%04X CW=0x%04X", current_state, current_sw, cw);
                    UART_SendLine(buf);
                }
                return;  // Skip normal state machine during post-init
            } else {
                // Motor reached good state
                static uint8_t ready_msg_printed = 0;
                if (!ready_msg_printed) {
                    UART_SendLine("[PostInit] Motor ready!");
                    ready_msg_printed = 1;
                }
                post_init_phase = 0;
            }
        }
    }

    // State machine - CiA 402 Power State Machine
    // Single place to set controlword based on current state and motor_enabled flag
    // Note: manual_control overrides motor_enabled flag
    if (manual_control) {
        // Manual control active - use manual_controlword
        controlword = manual_controlword;
    }
    else if (!motor_enabled) {
        // Disabled - clear flags and send shutdown
        in_op = 0;
        op_requested = 0;
        controlword = 0x0006;  // Shutdown
        target_velocity_csv = 0;
    }
    else if (cia402_state == 7) {  // Fault
        in_op = 0;
        op_requested = 0;
        if (now - last_fault_time > 500) {  // Try fault reset every 500ms
            last_fault_time = now;
            controlword = 0x0080;  // Fault Reset
        }
    }
    else if (in_op) {
        // Already in Operation Enabled - keep running
        controlword = 0x000F;  // Enable Operation
        
        if (operation_mode == OPMODE_CSV) {
            // CSV Mode: Write target velocity
            int32_t target_vel = target_velocity_csv;
            memcpy((void *)(ec_group[0].outputs + (int)target_vel_offset), &target_vel, sizeof(int32_t));
            
            // Write zero position and torque for safety
            int32_t zero32 = 0;
            int16_t zero16 = 0;
            memcpy((void *)(ec_group[0].outputs + (int)target_pos_offset), &zero32, sizeof(int32_t));
            memcpy((void *)(ec_group[0].outputs + (int)target_tor_offset), &zero16, sizeof(int16_t));
        } else {
            // CSP Mode (default): Write target position
            int32_t target_pos = target_position;
            memcpy((void *)(ec_group[0].outputs + (int)target_pos_offset), &target_pos, sizeof(int32_t));
            
            // Write zero velocity and torque for safety
            int32_t zero32 = 0;
            int16_t zero16 = 0;
            memcpy((void *)(ec_group[0].outputs + (int)target_vel_offset), &zero32, sizeof(int32_t));
            memcpy((void *)(ec_group[0].outputs + (int)target_tor_offset), &zero16, sizeof(int16_t));
        }
    }
    else if (op_requested) {
        // Waiting to enter Operation Enabled
        controlword = 0x000F;  // Enable Operation
        
        if (cia402_state == 4) {
            // Successfully entered Operation Enabled
            in_op = 1;
            op_requested = 0;
            UART_SendLine("Motor entered Operation Enabled!");
        }
    }
    else {
        // Auto state machine disabled - use manual commands
        // Commands: reset -> shutdown -> switchon -> enableop
        controlword = 0x0000;  // Default: wake up
    }
    
    // Write controlword
    uint16_t cw;
    static uint8_t last_manual = 0;
    if (manual_control) {
        cw = manual_controlword;
        // Debug: print manual control
        if (!last_manual) {
            last_manual = 1;
            char buf[64];
            sprintf(buf, "Manual control: CW=0x%04X", cw);
            UART_SendLine(buf);
        }
    } else {
        cw = controlword;
        last_manual = 0;
    }
    
    // Debug: print controlword being written
    static uint16_t last_written_cw = 0xFFFF;
    if (cw != last_written_cw) {
        last_written_cw = cw;
        char buf[64];
        sprintf(buf, "Writing CW=0x%04X to offset %d", cw, ctrl_word_offset);
        UART_SendLine(buf);
    }
    
    memcpy((void *)(ec_group[0].outputs + (int)ctrl_word_offset), &cw, sizeof(uint16_t));
    
    // Debug: verify data was written correctly
    uint16_t verify_cw = 0;
    memcpy(&verify_cw, (void *)(ec_group[0].outputs + (int)ctrl_word_offset), sizeof(uint16_t));
    if (verify_cw != cw) {
        char buf[64];
        sprintf(buf, "ERROR: CW write verification failed! Wrote 0x%04X, read back 0x%04X", cw, verify_cw);
        UART_SendLine(buf);
    }
    
    // Memory barrier to ensure data is written to memory before sending
    __DSB();
    
    // When not in operation, write safe values
    if (!in_op) {
        int32_t zero32 = 0;
        int16_t zero16 = 0;
        memcpy((void *)(ec_group[0].outputs + (int)target_pos_offset), &zero32, sizeof(int32_t));
        memcpy((void *)(ec_group[0].outputs + (int)target_vel_offset), &zero32, sizeof(int32_t));
        memcpy((void *)(ec_group[0].outputs + (int)target_tor_offset), &zero16, sizeof(int16_t));
        __DSB();
    }
}

// EtherCAT Cyclic Task
void EtherCAT_Cyclic_Task(void)
{
    static uint32_t last_cycle = 0;
    static uint32_t state_check_counter = 0;
    static uint32_t pdo_print_counter = 0;
    static int last_wkc = 0;
    static uint32_t cycle_counter = 0;
    uint32_t now = HAL_GetTick();
    
    if ((now - last_cycle) < EC_CYCLE_TIME_MS) return;
    last_cycle = now;
    cycle_counter++;
    
    if (!ec_initialized) return;
    
    // Check if group is properly configured
    if (ec_group[0].Obytes == 0 && ec_group[0].Ibytes == 0) return;
    
    // DC time update - Use system time for DC sync mode
    // CRITICAL: DC sync must be maintained for SYNC0 to work
    static uint32_t dc_time_ms = 0;
    static uint8_t dc_initialized = 0;
    if (ec_slave_count > 0) {
        // Always update DC time, regardless of hasdc flag
        // The hasdc flag might not be set correctly during initialization
        dc_time_ms += EC_CYCLE_TIME_MS;
        
        // CRITICAL: DC time should start from a reasonable value (not 0)
        // Some slaves require DC time to be > 0 for SYNC0 to work
        if (!dc_initialized) {
            dc_initialized = 1;
            // Start from 1 second (1000ms) to ensure DC time is valid
            dc_time_ms = 1000;
        }
        
        ec_DCtime = (int64)dc_time_ms * 1000000;  // Convert ms to ns
    }
    
    // Run motor state machine first to update controlword
    Motor_State_Machine();

    // CRITICAL: Ensure SM2/SM3 are activated before each PDO exchange
    // Some slaves disable SM during state transitions
    static uint32_t sm_check_counter = 0;
    sm_check_counter++;
    if (sm_check_counter >= 100) {  // Check every 100 cycles (200ms)
        sm_check_counter = 0;
        for (int slave_idx = 1; slave_idx <= ec_slave_count; slave_idx++) {
            uint16_t configadr = ec_slave[slave_idx].configadr;
            uint8_t sm2_stat[8] = {0};
            ecx_FPRD(&ecx_port, configadr, ECT_REG_SM2, 8, sm2_stat, EC_TIMEOUTSAFE);
            if ((sm2_stat[6] & 0x01) == 0) {
                // SM2 Act bit is disabled, re-enable
                sm2_stat[6] |= 0x01;
                ecx_FPWR(&ecx_port, configadr, ECT_REG_SM2, 8, sm2_stat, EC_TIMEOUTSAFE);
            }
            uint8_t sm3_stat[8] = {0};
            ecx_FPRD(&ecx_port, configadr, ECT_REG_SM3, 8, sm3_stat, EC_TIMEOUTSAFE);
            if ((sm3_stat[6] & 0x01) == 0) {
                // SM3 Act bit is disabled, re-enable
                sm3_stat[6] |= 0x01;
                ecx_FPWR(&ecx_port, configadr, ECT_REG_SM3, 8, sm3_stat, EC_TIMEOUTSAFE);
            }
        }
    }

    // Clean D-Cache before sending to ensure DMA sees the updated data
    if (ec_group[0].outputs != NULL && ec_group[0].Obytes > 0) {
        CleanDCache_by_Addr((uint32_t *)ec_group[0].outputs, ec_group[0].Obytes);
    }
    
    // Send process data (with updated controlword)
    int wkc_send = 0;
    if (ec_group[0].outputs != NULL) {
        wkc_send = ec_send_processdata();
    }
    
    // Receive process data
    int wkc = ec_receive_processdata(500);
    last_wkc = wkc;
    
    // Debug: print WKC periodically (every 5 seconds)
    static uint32_t wkc_print_counter = 0;
    wkc_print_counter++;
    if (wkc_print_counter >= 2500) {  // 2500 * 2ms = 5 seconds
        wkc_print_counter = 0;
        char buf[128];
        sprintf(buf, "[Debug] WKC=%d, send=%d, Obytes=%d, Ibytes=%d", 
                wkc, wkc_send, ec_group[0].Obytes, ec_group[0].Ibytes);
        UART_SendLine(buf);
        // Print DC time (for DC sync mode debugging)
        if (ec_group[0].hasdc) {
            sprintf(buf, "[Debug] DC time: %lld ns", (long long)ec_DCtime);
            UART_SendLine(buf);
        }
        // Print output data (what we're sending)
        if (ec_group[0].outputs != NULL && ec_group[0].Obytes > 0) {
            sprintf(buf, "[Debug] Output data: %02X %02X %02X %02X %02X %02X",
                    ec_group[0].outputs[0], ec_group[0].outputs[1], ec_group[0].outputs[2],
                    ec_group[0].outputs[3], ec_group[0].outputs[4], ec_group[0].outputs[5]);
            UART_SendLine(buf);
        }
        // Print input data (what we're receiving)
        if (ec_group[0].inputs != NULL && ec_group[0].Ibytes > 0) {
            sprintf(buf, "[Debug] Input data: %02X %02X %02X %02X %02X %02X",
                    ec_group[0].inputs[0], ec_group[0].inputs[1], ec_group[0].inputs[2],
                    ec_group[0].inputs[3], ec_group[0].inputs[4], ec_group[0].inputs[5]);
            UART_SendLine(buf);
        }
    }
    
    // Invalidate D-Cache after receiving to ensure CPU sees the updated data
    if (ec_group[0].inputs != NULL && ec_group[0].Ibytes > 0) {
        InvalidateDCache_by_Addr((uint32_t *)ec_group[0].inputs, ec_group[0].Ibytes);
    }
    
    // Check state periodically (every 100 cycles = 200ms @ 2ms cycle)
    state_check_counter++;
    if (state_check_counter >= 100) {
        state_check_counter = 0;
        ec_readstate();
        ec_operational = (ec_slave[1].state == EC_STATE_OPERATIONAL);
    }
    
    // Print status every 500 cycles = 1 second (1Hz)
    pdo_print_counter++;
    if (pdo_print_counter >= 500) {
        pdo_print_counter = 0;
        if (ec_slave_count > 0) {
            char buf[128];
            float pos_deg = actual_position * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
            float vel_deg = actual_velocity_csv * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
            
            // Read raw input data for debugging
            uint16_t raw_status = 0;
            if (ec_group[0].inputs != NULL && ec_group[0].Ibytes >= 2) {
                memcpy(&raw_status, (void*)ec_group[0].inputs, sizeof(uint16_t));
            }
            
            // Compact status output with controlword and motor_enabled flag
            // Also show raw status from input buffer to verify PDO mapping
            sprintf(buf, "[Status] Pos:%.2f Vel:%.2f %s SW:0x%04X CW:0x%04X WKC:%d", 
                    pos_deg, vel_deg, Get_State_String(cia402_state), raw_status, controlword, last_wkc);
            UART_SendLine(buf);
        }
    }
}

// Print System Status
void Print_Status(void)
{
    char buf[128];
    
    UART_SendLine("");
    UART_SendLine("=== Status ===");
    
    sprintf(buf, "ECAT: %s", ec_operational ? "OK" : "ERR");
    UART_SendLine(buf);
    
    // Read AL status and AL status code for diagnostics
    if (ec_slave_count > 0) {
        uint16_t al_status = 0;
        uint16_t al_status_code = 0;
        uint16_t configadr = ec_slave[1].configadr;
        ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &al_status, EC_TIMEOUTSAFE);
        ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTATCODE, 2, &al_status_code, EC_TIMEOUTSAFE);
        sprintf(buf, "AL Status: 0x%04X, AL Code: 0x%04X (%s)", 
                al_status, al_status_code, ec_ALstatuscode2string(al_status_code));
        UART_SendLine(buf);
    }
    
    if (ec_slave_count > 0) {
        sprintf(buf, "Motor: %s (0x%04X)", Get_State_String(cia402_state), statusword);
        UART_SendLine(buf);
        
        sprintf(buf, "Err: 0x%04X", error_code);
        UART_SendLine(buf);
        
        // Operation mode
        const char* mode_str;
        switch(operation_mode) {
            case OPMODE_CSP: mode_str = "CSP"; break;
            case OPMODE_CSV: mode_str = "CSV"; break;
            case OPMODE_CST: mode_str = "CST"; break;
            default: mode_str = "Unknown"; break;
        }
        sprintf(buf, "Mode: %s", mode_str);
        UART_SendLine(buf);
        
        // Position info
        float act_pos_deg = actual_position * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
        float tgt_pos_deg = target_position * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
        sprintf(buf, "Pos: %.2f deg (tgt: %.2f)", act_pos_deg, tgt_pos_deg);
        UART_SendLine(buf);
        
        // Velocity info
        float act_vel_deg = actual_velocity_csv * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
        float tgt_vel_deg = target_velocity_csv * 360.0f / JOINT_ENCODER_COUNTS_PER_REV;
        sprintf(buf, "Vel: %.2f deg/s (tgt: %.2f)", act_vel_deg, tgt_vel_deg);
        UART_SendLine(buf);
    }
    
    sprintf(buf, "En: %s", motor_enabled ? "ON" : "OFF");
    UART_SendLine(buf);
}

// Diagnose EtherCAT frame communication
void Diagnose_EtherCAT_Frame(void)
{
    char buf[128];
    uint16_t wkc;
    
    UART_SendLine("");
    UART_SendLine("=== EtherCAT Frame Diagnosis ===");
    
    // Test 1: Broadcast read of ESC type register (address 0x0000)
    UART_SendLine("Test 1: BRD ESC Type (0x0000)...");
    uint8_t brd_data[2] = {0, 0};
    wkc = ecx_BRD(&ecx_port, 0x0000, 0x0000, 2, &brd_data, EC_TIMEOUTSAFE);
    sprintf(buf, "  BRD WKC=%d, Data=0x%02X%02X", wkc, brd_data[1], brd_data[0]);
    UART_SendLine(buf);
    
    // Test 2: Try to read using different station addresses
    UART_SendLine("Test 2: FPRD with different addresses...");
    uint16_t test_addrs[] = {0x0000, 0x1001, 0x1000, 0x0001};
    for (int i = 0; i < 4; i++) {
        uint16_t esc_type = 0;
        wkc = ecx_FPRD(&ecx_port, test_addrs[i], 0x0000, 2, &esc_type, EC_TIMEOUTSAFE);
        sprintf(buf, "  FPRD addr=0x%04X: WKC=%d, ESC_Type=0x%04X", test_addrs[i], wkc, esc_type);
        UART_SendLine(buf);
        HAL_Delay(10);
    }
    
    // Test 3: Check if we can write station address
    UART_SendLine("Test 3: Configure station address...");
    uint16_t new_addr = 0x1001;
    wkc = ecx_APWR(&ecx_port, 0, 0x0010, 2, &new_addr, EC_TIMEOUTSAFE);
    sprintf(buf, "  APWR WKC=%d (should be 1 if successful)", wkc);
    UART_SendLine(buf);
    
    // Verify the address was set
    HAL_Delay(10);
    uint16_t read_addr = 0;
    wkc = ecx_FPRD(&ecx_port, 0x1001, 0x0010, 2, &read_addr, EC_TIMEOUTSAFE);
    sprintf(buf, "  FPRD verify: WKC=%d, Addr=0x%04X", wkc, read_addr);
    UART_SendLine(buf);
    
    // Test 4: Check port statistics
    UART_SendLine("Test 4: Port statistics...");
    sprintf(buf, "  TX errors: %d", ecx_port.txerror);
    UART_SendLine(buf);
    sprintf(buf, "  RX errors: %d", ecx_port.rxerror);
    UART_SendLine(buf);
    sprintf(buf, "  Packet count: %d", ecx_port.pktcnt);
    UART_SendLine(buf);
    
    UART_SendLine("=== Diagnosis Complete ===");
}

// Process UART Commands
void Process_Command(void)
{
    char *cmd = (char*)rx_buffer;
    char buf[128];
    
    // Debug: print received command
    sprintf(buf, "[Debug] Received command: '%s'", cmd);
    UART_SendLine(buf);
    
    // Convert to lowercase for comparison
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'A' && cmd[i] <= 'Z') {
            cmd[i] = cmd[i] - 'A' + 'a';
        }
    }
    
    if (strcmp(cmd, "init") == 0)
    {
        EtherCAT_Init();
    }
    else if (strcmp(cmd, "start") == 0)
    {
        system_running = 1;
        UART_SendLine("System started");
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        system_running = 0;
        target_velocity_csv = 0;
        UART_SendLine("System stopped");
    }
    else if (strncmp(cmd, "vel ", 4) == 0)
    {
        // CSV mode velocity command
        float vel_deg = atof(&cmd[4]);
        if (vel_deg >= -MAX_VELOCITY && vel_deg <= MAX_VELOCITY)
        {
            target_velocity_csv = (int32_t)(vel_deg * DEG_PER_SEC_TO_CSV_FACTOR);
            sprintf(buf, "Target velocity: %.2f deg/s (%ld counts/s)", 
                    vel_deg, target_velocity_csv);
            UART_SendLine(buf);
            
            if (operation_mode != OPMODE_CSV) {
                UART_SendLine("WARNING: Not in CSV mode. Use 'mode csv' to switch.");
            }
        }
        else
        {
            sprintf(buf, "Velocity out of range! (%.0f to %.0f deg/s)", 
                    -MAX_VELOCITY, MAX_VELOCITY);
            UART_SendLine(buf);
        }
    }
    else if (strncmp(cmd, "pos ", 4) == 0)
    {
        // CSP mode position command
        float pos_deg = atof(&cmd[4]);
        int32_t pos_counts = (int32_t)(pos_deg * JOINT_ENCODER_COUNTS_PER_REV / 360.0f);
        target_position = pos_counts;
        sprintf(buf, "Target position: %.4f deg (%ld counts)", pos_deg, pos_counts);
        UART_SendLine(buf);
        
        if (operation_mode != OPMODE_CSP) {
            UART_SendLine("WARNING: Not in CSP mode. Use 'mode csp' to switch.");
        }
    }
    else if (strncmp(cmd, "mode ", 5) == 0)
    {
        // Switch operation mode
        char *mode_str = &cmd[5];
        if (strcmp(mode_str, "csv") == 0) {
            operation_mode = OPMODE_CSV;
            UART_SendLine("Switched to CSV (Cyclic Synchronous Velocity) mode");
            UART_SendLine("Use 'vel <value>' to set velocity");
        } else if (strcmp(mode_str, "csp") == 0) {
            operation_mode = OPMODE_CSP;
            UART_SendLine("Switched to CSP (Cyclic Synchronous Position) mode");
            UART_SendLine("Use 'pos <value>' to set position");
        } else {
            UART_SendLine("Unknown mode. Use 'csv' or 'csp'");
        }
    }
    else if (strcmp(cmd, "enable") == 0)
    {
        motor_enabled = 1;
        UART_SendLine("Motor enabled");
    }
    else if (strcmp(cmd, "disable") == 0)
    {
        motor_enabled = 0;
        target_velocity_csv = 0;
        UART_SendLine("Motor disabled");
    }
    else if (strcmp(cmd, "status") == 0)
    {
        Print_Status();
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        // Fault reset
        manual_control = 1;
        manual_controlword = 0x0080;
        UART_SendLine("Fault reset (0x0080) - will apply on next cycle");
    }
    else if (strcmp(cmd, "wake") == 0)
    {
        // Wake up - send 0x0000 to initialize state machine
        manual_control = 1;
        manual_controlword = 0x0000;
        UART_SendLine("Wake up (0x0000) - initializing state machine");
    }
    else if (strcmp(cmd, "shutdown") == 0)
    {
        // CiA 402: Shutdown (transition to Ready to Switch On)
        manual_control = 1;
        manual_controlword = 0x0006;
        UART_SendLine("Shutdown (0x0006) -> Ready to Switch On");
    }
    else if (strcmp(cmd, "switchon") == 0)
    {
        // CiA 402: Switch On (transition to Switched On)
        UART_SendLine("[Debug] switchon command matched!");
        manual_control = 1;
        manual_controlword = 0x0007;
        UART_SendLine("Switch On (0x0007) -> Switched On");
    }
    else if (strcmp(cmd, "enableop") == 0)
    {
        // CiA 402: Enable Operation (transition to Operation Enabled)
        // CRITICAL: Before entering Operation Enabled, sync target position to actual position
        // to avoid following error fault (0x0131)
        UART_SendLine("Enable Operation: Syncing target position to actual position...");
        
        // Read current actual position from input data
        int32_t current_pos = 0;
        if (ec_group[0].inputs && ec_group[0].Ibytes >= (target_pos_offset + 4)) {
            memcpy(&current_pos, ec_group[0].inputs + (int)actual_pos_offset, sizeof(int32_t));
            // Convert from encoder counts to degrees for display
            float pos_deg = (float)current_pos / 10000.0f;
            sprintf(buf, "  Current position: %ld counts (%.2f deg)", current_pos, pos_deg);
            UART_SendLine(buf);
        }
        
        // Set target position to current position to avoid following error
        int32_t target_pos = current_pos;
        memcpy((void *)(ec_group[0].outputs + (int)target_pos_offset), &target_pos, sizeof(int32_t));
        sprintf(buf, "  Target position set to: %ld counts", target_pos);
        UART_SendLine(buf);
        
        // Also set target velocity and torque to 0 for safety
        int32_t zero32 = 0;
        int16_t zero16 = 0;
        memcpy((void *)(ec_group[0].outputs + (int)target_vel_offset), &zero32, sizeof(int32_t));
        memcpy((void *)(ec_group[0].outputs + (int)target_tor_offset), &zero16, sizeof(int16_t));
        UART_SendLine("  Target velocity and torque set to 0");
        
        // Send the updated PDO data before enabling operation
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        UART_SendLine("  PDO data updated");
        
        manual_control = 1;
        manual_controlword = 0x000F;
        UART_SendLine("Enable Operation (0x000F) -> Operation Enabled");
        
        // Enable SYNC0/SYNC1 for DC synchronous mode
        // This must be done BEFORE entering Operation Enabled to avoid DC sync clock lost fault
        // Only activate if DC sync mode is enabled (ec_group[0].hasdc == TRUE)
        if (ec_slave_count > 0 && ec_slave[1].hasdc && ec_group[0].hasdc) {
            // Always enable SYNC0/SYNC1 when entering Operation Enabled
            // This ensures DC sync is active even after fault reset
            UART_SendLine("Activating SYNC0/SYNC1 for DC synchronous mode...");
            
            // Re-set AssignActivate to ensure DC sync mode is enabled
            // CRITICAL: Use ECT_REG_DCCUC (0x0980), NOT ECT_REG_DCSYNCACT (0x0981)
            uint16_t assign_activate = 0x0300;
            int wkc_assign = ecx_FPWR(&ecx_port, ec_slave[1].configadr, ECT_REG_DCCUC, 
                                       sizeof(assign_activate), &assign_activate, EC_TIMEOUTSAFE);
            sprintf(buf, "  AssignActivate (0x0980) re-set to 0x%04X (wkc=%d)", assign_activate, wkc_assign);
            UART_SendLine(buf);
            
            // Use SOEM's built-in function to activate SYNC0 only (not SYNC0+SYNC1)
            // CRITICAL FIX: SYNC0周期必须与PDO周期匹配 (2ms), shift=1ms like IGH
            // Parameters: context, slave, activate, cycle_time, shift
            ecx_dcsync0(&ecx_context, 1, TRUE, DC_SYNC0_CYCLE_TIME, DC_SYNC0_SHIFT);
            UART_SendLine("  SYNC0 activated via ecx_dcsync0 (SYNC0 only, no SYNC1)");
            sprintf(buf, "  SYNC0 cycle: %ld ms, shift: %ld ms", 
                    DC_SYNC0_CYCLE_TIME / 1000000, DC_SYNC0_SHIFT / 1000000);
            UART_SendLine(buf);
            
            // Wait for DC time to stabilize - send multiple PDO cycles before enabling operation
            UART_SendLine("Waiting for DC time to stabilize...");
            for (int i = 0; i < 50; i++) {  // 50 * 2ms = 100ms
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                HAL_Delay(2);
            }
            UART_SendLine("DC time stabilized, proceeding to enable operation");
        } else {
            UART_SendLine("DC Sync mode - SYNC0 active");
        }
    }
    else if (strcmp(cmd, "disableop") == 0)
    {
        // CiA 402: Disable Operation (transition to Switched On)
        manual_control = 1;
        manual_controlword = 0x0007;
        UART_SendLine("Disable Operation (0x0007) -> Switched On");
    }
    else if (strcmp(cmd, "quickstop") == 0)
    {
        // CiA 402: Quick Stop
        manual_control = 1;
        manual_controlword = 0x0002;
        UART_SendLine("Quick Stop (0x0002)");
    }
    else if (strcmp(cmd, "auto") == 0)
    {
        // Return to automatic state machine control
        manual_control = 0;
        UART_SendLine("Automatic control enabled");
    }
    else if (strcmp(cmd, "diag") == 0)
    {
        // Diagnostic command to check PHY and link status
        UART_SendLine("");
        UART_SendLine("=== Diagnostic Info ===");
        
        extern uint32_t LAN8720A_PHY_ADDRESS;
        extern ETH_HandleTypeDef heth;
        uint32_t phy_id1, phy_id2, phy_bsr, phy_scsr, phy_bcr;
        
        sprintf(buf, "PHY Address: %lu", LAN8720A_PHY_ADDRESS);
        UART_SendLine(buf);
        
        // Read PHY registers
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, 2, &phy_id1);
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, 3, &phy_id2);
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, 0, &phy_bcr);
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, 1, &phy_bsr);
        HAL_ETH_ReadPHYRegister(&heth, LAN8720A_PHY_ADDRESS, 31, &phy_scsr);
        
        sprintf(buf, "PHY ID1: 0x%04X", (uint16_t)phy_id1);
        UART_SendLine(buf);
        sprintf(buf, "PHY ID2: 0x%04X", (uint16_t)phy_id2);
        UART_SendLine(buf);
        sprintf(buf, "PHY BCR: 0x%04X", (uint16_t)phy_bcr);
        UART_SendLine(buf);
        sprintf(buf, "PHY BSR: 0x%04X", (uint16_t)phy_bsr);
        UART_SendLine(buf);
        sprintf(buf, "PHY SCSR: 0x%04X", (uint16_t)phy_scsr);
        UART_SendLine(buf);
        
        // Parse link status
        if (phy_bsr & 0x04) {
            UART_SendLine("Link Status: UP");
        } else {
            UART_SendLine("Link Status: DOWN");
        }
        
        // Parse speed
        int speed = (phy_scsr >> 2) & 0x7;
        switch(speed) {
            case 1: UART_SendLine("Speed: 10Mbps Half Duplex"); break;
            case 5: UART_SendLine("Speed: 10Mbps Full Duplex"); break;
            case 2: UART_SendLine("Speed: 100Mbps Half Duplex"); break;
            case 6: UART_SendLine("Speed: 100Mbps Full Duplex"); break;
            default: UART_SendLine("Speed: Unknown"); break;
        }
        
        // Check ETH DMA status
        sprintf(buf, "ETH DMASR: 0x%08X", (unsigned int)heth.Instance->DMASR);
        UART_SendLine(buf);
        
        // Read ESC registers if EtherCAT is initialized
        if (ec_initialized && ec_slave_count > 0) {
            UART_SendLine("");
            UART_SendLine("=== ESC Registers ===");
            uint16_t configadr = ec_slave[1].configadr;
            uint16_t reg_val;
            
            // Read various ESC registers
            ecx_FPRD(&ecx_port, configadr, 0x0000, 2, &reg_val, EC_TIMEOUTSAFE);
            sprintf(buf, "Type: 0x%04X", reg_val);
            UART_SendLine(buf);
            
            ecx_FPRD(&ecx_port, configadr, 0x0008, 2, &reg_val, EC_TIMEOUTSAFE);
            sprintf(buf, "Features: 0x%04X", reg_val);
            UART_SendLine(buf);
            
            ecx_FPRD(&ecx_port, configadr, 0x0010, 2, &reg_val, EC_TIMEOUTSAFE);
            sprintf(buf, "Station Addr: 0x%04X", reg_val);
            UART_SendLine(buf);
            
            ecx_FPRD(&ecx_port, configadr, 0x0012, 2, &reg_val, EC_TIMEOUTSAFE);
            sprintf(buf, "Station Alias: 0x%04X", reg_val);
            UART_SendLine(buf);
            
            // Read AL status and code
            uint16_t al_status, al_code;
            ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &al_status, EC_TIMEOUTSAFE);
            ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTATCODE, 2, &al_code, EC_TIMEOUTSAFE);
            sprintf(buf, "AL Status: 0x%04X, Code: 0x%04X", al_status, al_code);
            UART_SendLine(buf);
            
            // Read SM0-SM3 status
            uint8_t sm[8];
            for (int sm_idx = 0; sm_idx < 4; sm_idx++) {
                ecx_FPRD(&ecx_port, configadr, ECT_REG_SM0 + (sm_idx * 8), 8, sm, EC_TIMEOUTSAFE);
                uint16_t start = sm[0] | (sm[1] << 8);
                uint16_t len = sm[2] | (sm[3] << 8);
                uint8_t ctrl = sm[4];
                uint8_t act = sm[6];
                sprintf(buf, "SM%d: Start=0x%04X Len=%d Ctrl=0x%02X Act=%d", 
                        sm_idx, start, len, ctrl, act);
                UART_SendLine(buf);
            }
        }
        
        UART_SendLine("");
        UART_SendLine("Troubleshooting tips:");
        UART_SendLine("1. Use CROSSOVER cable for direct connection");
        UART_SendLine("2. Motor must be in INIT state (power cycle if needed)");
        UART_SendLine("3. Check RJ45 connector LEDs");
    }
    else if (strcmp(cmd, "test") == 0)
    {
        // Test command: send a single EtherCAT BRD frame and show response
        UART_SendLine("");
        UART_SendLine("=== EtherCAT Test ===");
        
        // Initialize SOEM first if not already done
        if (!ec_initialized) {
            UART_SendLine("Initializing SOEM first...");
            if (!ec_init("eth0")) {
                UART_SendLine("ERROR: ec_init failed!");
                return;
            }
            ec_initialized = 1;
        }
        
        UART_SendLine("Port stats before test:");
        sprintf(buf, "  TX errors: %lu", ecx_port.txerror);
        UART_SendLine(buf);
        sprintf(buf, "  RX errors: %lu", ecx_port.rxerror);
        UART_SendLine(buf);
        sprintf(buf, "  Packets: %lu", ecx_port.pktcnt);
        UART_SendLine(buf);
        
        // Send a broadcast read to register 0x0000 (Type)
        uint16_t brd_data = 0;
        UART_SendLine("Sending BRD to register 0x0000 (Type)...");
        UART_SendLine("(Watch for TX and RX debug output)");
        
        int wkc = ecx_BRD(&ecx_port, 0x0000, 0x0000, 2, &brd_data, EC_TIMEOUTSAFE);
        
        sprintf(buf, "Working Counter: %d", wkc);
        UART_SendLine(buf);
        sprintf(buf, "Received Data: 0x%04X", brd_data);
        UART_SendLine(buf);
        
        UART_SendLine("Port stats after test:");
        sprintf(buf, "  TX errors: %lu", ecx_port.txerror);
        UART_SendLine(buf);
        sprintf(buf, "  RX errors: %lu", ecx_port.rxerror);
        UART_SendLine(buf);
        sprintf(buf, "  Packets: %lu", ecx_port.pktcnt);
        UART_SendLine(buf);
        
        if (wkc > 0) {
            UART_SendLine("SUCCESS: Slave responded!");
        } else {
            UART_SendLine("FAILED: No response from slave");
            UART_SendLine("");
            UART_SendLine("Check:");
            UART_SendLine("1. Use CROSSOVER ethernet cable");
            UART_SendLine("2. Motor is powered and in INIT state");
            UART_SendLine("3. Cable is firmly connected");
            UART_SendLine("");
            UART_SendLine("If no TX output seen, TX path has problem.");
            UART_SendLine("If TX seen but no RX, check cable/motor.");
        }
    }
    else if (strcmp(cmd, "sdo") == 0)
    {
        // SDO test command: try to read CiA402 objects via SDO
        UART_SendLine("");
        UART_SendLine("=== SDO Test ===");
        
        if (!ec_initialized) {
            UART_SendLine("ERROR: EtherCAT not initialized!");
            return;
        }
        
        int i;
        for (i = 1; i <= ec_slave_count; i++) {
            uint16_t configadr = ec_slave[i].configadr;
            char buf[128];
            
            sprintf(buf, "Slave %d SDO read test:", i);
            UART_SendLine(buf);
            
            // Try to read Statusword (0x6041)
            uint16_t statusword_sdo = 0;
            int size = sizeof(statusword_sdo);
            int wkc = ecx_SDOread(&ecx_context, i, 0x6041, 0x00, FALSE,
                                  &size, &statusword_sdo, EC_TIMEOUTSAFE);
            if (wkc > 0) {
                sprintf(buf, "  Statusword (0x6041): 0x%04X", statusword_sdo);
                UART_SendLine(buf);
            } else {
                sprintf(buf, "  Statusword read failed (wkc=%d)", wkc);
                UART_SendLine(buf);
            }
            
            // Try to read Controlword (0x6040)
            uint16_t controlword_sdo = 0;
            size = sizeof(controlword_sdo);
            wkc = ecx_SDOread(&ecx_context, i, 0x6040, 0x00, FALSE,
                              &size, &controlword_sdo, EC_TIMEOUTSAFE);
            if (wkc > 0) {
                sprintf(buf, "  Controlword (0x6040): 0x%04X", controlword_sdo);
                UART_SendLine(buf);
            } else {
                sprintf(buf, "  Controlword read failed (wkc=%d)", wkc);
                UART_SendLine(buf);
            }
            
            // Try to read Operation Mode (0x6060)
            uint8_t op_mode = 0;
            size = sizeof(op_mode);
            wkc = ecx_SDOread(&ecx_context, i, 0x6060, 0x00, FALSE,
                              &size, &op_mode, EC_TIMEOUTSAFE);
            if (wkc > 0) {
                sprintf(buf, "  Op Mode (0x6060): 0x%02X", op_mode);
                UART_SendLine(buf);
            } else {
                sprintf(buf, "  Op Mode read failed (wkc=%d)", wkc);
                UART_SendLine(buf);
            }
            
            // Try to read AL Status
            uint16_t al_status = 0;
            wkc = ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &al_status, EC_TIMEOUTSAFE);
            sprintf(buf, "  AL Status: 0x%04X (wkc=%d)", al_status, wkc);
            UART_SendLine(buf);
        }
    }
    else if (strcmp(cmd, "diag") == 0)
    {
        // Initialize SOEM first if not already done
        if (!ec_initialized) {
            UART_SendLine("Initializing SOEM first...");
            if (!ec_init("eth0")) {
                UART_SendLine("ERROR: ec_init failed!");
                return;
            }
            ec_initialized = 1;
        }
        Diagnose_EtherCAT_Frame();
    }
    else if (strcmp(cmd, "help") == 0)
    {
        UART_SendLine("");
        UART_SendLine("=== Command List ===");
        UART_SendLine("init       - Initialize EtherCAT");
        UART_SendLine("start      - Start cyclic task");
        UART_SendLine("stop       - Stop cyclic task");
        UART_SendLine("mode csv   - Switch to CSV (velocity) mode");
        UART_SendLine("mode csp   - Switch to CSP (position) mode");
        UART_SendLine("vel <deg/s>- Set target velocity (CSV mode)");
        UART_SendLine("pos <deg>  - Set target position (CSP mode)");
        UART_SendLine("enable     - Auto state machine (enable motor)");
        UART_SendLine("disable    - Disable motor");
        UART_SendLine("reset      - Fault reset (0x0080)");
        UART_SendLine("wake       - Wake up (0x0000) - initialize state machine");
        UART_SendLine("shutdown   - Shutdown (0x0006) -> Ready to Switch On");
        UART_SendLine("switchon   - Switch On (0x0007) -> Switched On");
        UART_SendLine("enableop   - Enable Operation (0x000F) -> Op Enabled");
        UART_SendLine("disableop  - Disable Operation (0x0007) -> Switched On");
        UART_SendLine("quickstop  - Quick Stop (0x0002)");
        UART_SendLine("auto       - Return to automatic control");
        UART_SendLine("status     - Show status");
        UART_SendLine("diag       - Show diagnostic info");
        UART_SendLine("test       - Test EtherCAT communication");
        UART_SendLine("sdo        - Test SDO communication");
        UART_SendLine("help       - Show this help");
        UART_SendLine("");
        UART_SendLine("=== CSV Mode Quick Start ===");
        UART_SendLine("1. init      - Initialize EtherCAT");
        UART_SendLine("2. enable    - Enable motor state machine");
        UART_SendLine("3. vel 10    - Set velocity to 10 deg/s");
        UART_SendLine("4. stop      - Stop motor");
        UART_SendLine("");
        UART_SendLine("=== CSP Mode Quick Start ===");
        UART_SendLine("1. init      - Initialize EtherCAT");
        UART_SendLine("2. mode csp  - Switch to CSP mode");
        UART_SendLine("3. enable    - Enable motor");
        UART_SendLine("4. pos 90    - Move to 90 degrees");
    }
    else
    {
        UART_SendLine("Unknown command. Type 'help' for list.");
    }
    
    UART_SendString("> ");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */

  // LED Test
  for (int i = 0; i < 3; i++)
  {
      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
      HAL_Delay(200);
  }
  
  // Start TIM2 for SOEM timing
  HAL_TIM_Base_Start(&htim2);
  
  // Turn off LEDs
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);

  // Welcome message
  UART_SendLine("");
  UART_SendLine("================================");
  UART_SendLine("  EtherCAT CSV Motor Control");
  UART_SendLine("  STM32F407 + SOEM + KaiserDrive");
  UART_SendLine("================================");
  UART_SendLine("");
  UART_SendLine("Type 'init' to initialize EtherCAT");
  UART_SendLine("Type 'help' for command list");
  UART_SendString("> ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // UART command processing
    UART_Poll_Receive();
    
    // LED status update
    Update_LED_Status();
    
    // Motor state machine
    Motor_State_Machine();

    /* CRITICAL FIX: AL Status Monitoring for Diagnostics
     * Monitor slave AL status every 5 seconds to help diagnose state transition issues
     * This helps identify why slaves get stuck in SAFE_OP (0x0004) or other states
     */
    {
        static uint32_t last_al_check = 0;
        static uint16_t last_al_status = 0xFFFF;
        uint32_t now = HAL_GetTick();

        if ((now - last_al_check) >= 5000 && ec_initialized && ec_slave_count > 0) {
            last_al_check = now;

            // Read AL status from all slaves
            for (int i = 1; i <= ec_slave_count; i++) {
                uint16_t configadr = ec_slave[i].configadr;
                uint16_t alstat = 0;
                int wkc = ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &alstat, EC_TIMEOUTSAFE);
                alstat = etohs(alstat);

                // Only print if changed or first time
                if (alstat != last_al_status || last_al_status == 0xFFFF) {
                    char buf[128];
                    const char *state_str = "Unknown";

                    switch (alstat & 0x0F) {
                        case 1:  state_str = "INIT";       break;
                        case 2:  state_str = "PREOP";      break;
                        case 3:  state_str = "BOOT";       break;
                        case 4:  state_str = "SAFE_OP";    break;
                        case 8:  state_str = "OPERATIONAL"; break;
                        default: state_str = "Unknown";     break;
                    }

                    sprintf(buf, "[AL Monitor] Slave %d: AL=0x%04X (%s), Requested=0x%02X, WKC=%d",
                            i, alstat, state_str, ec_slave[i].state, wkc);
                    UART_SendLine(buf);

                    // Print error code if in error state
                    if (alstat & 0x10) {  // Error bit set
                        uint16_t errcode = 0;
                        ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTATCODE, 2, &errcode, EC_TIMEOUTSAFE);
                        sprintf(buf, "[AL Monitor] ERROR! Slave %d: AL Error Code=0x%04X", i, etohs(errcode));
                        UART_SendLine(buf);
                    }
                }
                last_al_status = alstat;
            }
        }
    }

    // EtherCAT cyclic communication
    EtherCAT_Cyclic_Task();
    
    // Key press handling
    if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)
    {
        HAL_Delay(50);  // Debounce
        if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)
        {
            motor_enabled = !motor_enabled;
            if (!motor_enabled) target_velocity_csv = 0;
            
            UART_SendLine("");
            UART_SendString("Motor ");
            UART_SendLine(motor_enabled ? "ENABLED" : "DISABLED");
            UART_SendString("> ");
            
            // Wait for key release with timeout (max 1 second)
            uint32_t key_wait_start = HAL_GetTick();
            while (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)
            {
                if ((HAL_GetTick() - key_wait_start) > 1000)
                    break;
            }
        }
    }
    
    // Feed watchdog or toggle a debug pin to show main loop is running
    // This helps detect if the system is stuck
    static uint32_t loop_counter = 0;
    loop_counter++;
    
    // Toggle LED1 every 1000 loops to show main loop is running
    if ((loop_counter % 1000) == 0) {
        HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
    }
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
