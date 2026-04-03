#ifndef _ETHERCATTYPE_H_
#define _ETHERCATTYPE_H_

#include "osal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* EtherCAT Ethernet type */
#define ETH_P_ECAT          0x88A4

/* Maximum EtherCAT frame data size */
#define EC_MAXECATFRAME     1518
#define EC_MAXDATA          1486
#define EC_HEADERSIZE       14
#define EC_ECATheadersize   2
#define EC_ECATHEADERSIZE   2
#define EC_ELENGTHSIZE      2
#define EC_CMDOFFSET        (EC_HEADERSIZE + EC_ECATHEADERSIZE)
#define EC_WKCOFFSET(len)   (EC_CMDOFFSET + EC_CMDHEADERSIZE + (len))

/* Maximum number of slaves */
#define EC_MAXSLAVE         200
/* Maximum number of groups */
#define EC_MAXGROUP         2
/* Maximum IOmap size */
#define EC_MAXIOMAP         4096
/* Maximum mailbox size */
#define EC_MAXMBX           1486
/* Maximum EtherCAT command header size */
#define EC_CMDHEADERSIZE    10
/* Working counter size */
#define EC_WKCSIZE          2
/* Datagram follows flag in dlength field */
#define EC_DATAGRAMFOLLOWS  (1 << 15)

/* Default timeout values in microseconds */
#define EC_TIMEOUTMON       500
#define EC_TIMEOUTRET       2000
#define EC_TIMEOUTSAFE      20000
#define EC_TIMEOUTSTATE     2000000
#define EC_TIMEOUTEEP       20000
#define EC_TIMEOUTRET3      2000
#define EC_TIMEOUTRET2      500
#define EC_TIMEOUTRXM       700000

/* Local delay for mailbox polling */
#define EC_LOCALDELAY       100

/* CoE mailbox types */
typedef enum {
    ECT_COES_EMERGENCY  = 0x01,
    ECT_COES_SDOREQ,
    ECT_COES_SDORES,
    ECT_COES_TXPDO,
    ECT_COES_RXPDO,
    ECT_COES_TXPDO_RR,
    ECT_COES_RXPDO_RR,
    ECT_COES_SDOINFO
} ec_coesdo;

/* SDO command codes */
#define ECT_SDO_DOWN_INIT    0x21
#define ECT_SDO_DOWN_EXP     0x23
#define ECT_SDO_DOWN_INIT_CA 0x31
#define ECT_SDO_UP_REQ       0x40
#define ECT_SDO_UP_REQ_CA    0x50

/* Max buffer count */
#define EC_MAXBUF           16
/* Buffer size */
#define EC_BUFSZ            EC_MAXECATFRAME

/* Buffer type */
typedef uint8_t ec_bufT[EC_BUFSZ];

/* EtherCAT commands */
typedef enum {
    EC_CMD_NOP  = 0x00,
    EC_CMD_APRD = 0x01,
    EC_CMD_APWR = 0x02,
    EC_CMD_APRW = 0x03,
    EC_CMD_FPRD = 0x04,
    EC_CMD_FPWR = 0x05,
    EC_CMD_FPRW = 0x06,
    EC_CMD_BRD  = 0x07,
    EC_CMD_BWR  = 0x08,
    EC_CMD_BRW  = 0x09,
    EC_CMD_LRD  = 0x0A,
    EC_CMD_LWR  = 0x0B,
    EC_CMD_LRW  = 0x0C,
    EC_CMD_ARMW = 0x0D,
    EC_CMD_FRMW = 0x0E
} ec_cmdtype;

/* EtherCAT slave states */
typedef enum {
    EC_STATE_NONE       = 0x00,
    EC_STATE_INIT       = 0x01,
    EC_STATE_PRE_OP     = 0x02,
    EC_STATE_BOOT       = 0x03,
    EC_STATE_SAFE_OP    = 0x04,
    EC_STATE_OPERATIONAL= 0x08,
    EC_STATE_ACK        = 0x10,
    EC_STATE_ERROR      = 0x10
} ec_state;

/* EtherCAT frame header */
typedef struct __attribute__((packed)) {
    uint8_t  da[6];     /* Destination MAC */
    uint8_t  sa[6];     /* Source MAC */
    uint16_t etype;     /* EtherType 0x88A4 */
    uint16_t elength;   /* EtherCAT length + type */
} ec_etherheadert;

/* EtherCAT datagram header */
typedef struct __attribute__((packed)) {
    uint8_t  command;    /* EtherCAT command */
    uint8_t  index;      /* Index */
    uint16_t ADP;        /* Auto-increment address */
    uint16_t ADO;        /* Physical memory address */
    uint16_t dlength;    /* Data length + next + circulating */
} ec_comt;

/* EtherCAT datagram with data and working counter */
#define ec_headersize   sizeof(ec_comt)

/* Slave info structure */
typedef struct {
    uint16  state;
    uint16  ALstatuscode;
    uint16  configadr;
    uint16  aliasadr;
    uint32  eep_man;        /* Manufacturer ID from EEPROM */
    uint32  eep_id;         /* Product ID from EEPROM */
    uint32  eep_rev;        /* Revision from EEPROM */
    uint16  Itype;
    uint16  Dtype;
    uint16  Obits;          /* Output bits */
    uint32  Obytes;         /* Output bytes */
    uint8  *outputs;        /* Pointer to output data in IOmap */
    uint8   Ostartbit;
    uint16  Ibits;          /* Input bits */
    uint32  Ibytes;         /* Input bytes */
    uint8  *inputs;         /* Pointer to input data in IOmap */
    uint8   Istartbit;
    /* SM (SyncManager) configuration */
    uint8   SMtype[8];
    uint8   SM[8][8];       /* SM configuration data */
    uint8   FMMU[4][16];   /* FMMU configuration data */
    uint16  mbx_l;          /* Mailbox write length */
    uint16  mbx_wo;         /* Mailbox write offset */
    uint16  mbx_rl;         /* Mailbox read length */
    uint16  mbx_ro;         /* Mailbox read offset */
    uint16  mbx_proto;      /* Mailbox protocol support flags */
    uint8   mbx_cnt;        /* Mailbox counter */
    boolean hasdc;          /* Slave has DC (Distributed Clock) */
    uint8   ptype;          /* Port type */
    uint8   topology;       /* Topology */
    uint8   activeports;    /* Active ports */
    uint8   consumedports;  /* Consumed ports */
    int16   parent;         /* Parent slave */
    uint8   parentport;     /* Parent port */
    uint32  DCrtA;          /* DC receive time port A */
    uint32  DCrtB;          /* DC receive time port B */
    uint32  DCrtC;          /* DC receive time port C */
    uint32  DCrtD;          /* DC receive time port D */
    int32   pdelay;         /* Propagation delay */
    uint16  DCnext;         /* DC next */
    uint16  DCprevious;     /* DC previous */
    int32   DCcycle;        /* DC cycle */
    int32   DCshift;        /* DC shift */
    uint8   DCactive;       /* DC active */
    char    name[40];       /* Slave name from EEPROM */
    uint8   islost;         /* Slave lost flag */
} ec_slavet;

/* Slave group */
typedef struct {
    uint32  logstartaddr;
    uint32  Obytes;
    uint8  *outputs;
    uint32  Ibytes;
    uint8  *inputs;
    boolean hasdc;
    uint16  DCnext;
    int16   Ebuscurrent;
    uint8   blockLRW;
    uint16  nsegments;
    uint16  Isegment;
    uint16  Ioffset;
    uint16  outputsWKC;
    uint16  inputsWKC;
    boolean docheckstate;
    uint32  IOsegment[EC_MAXIOMAP / 64];
    uint16  IOsegmentcount;
} ec_groupt;

/* Adapter */
typedef struct ec_adapter {
    char name[128];
    char desc[128];
    struct ec_adapter *next;
} ec_adaptert;

/* Redundant port structure */
typedef struct {
    int     sock;
    ec_bufT rxbuf[EC_MAXBUF];
    int     rxbufstat[EC_MAXBUF];
    int     rxsa[EC_MAXBUF];
} ecx_redportt;

/* PDO assign structure for Complete Access mode */
typedef struct __attribute__((packed)) {
    uint8   n;
    uint8   nu1;
    uint16  index[256];
} ec_PDOassignt;

/* PDO description structure for Complete Access mode */
typedef struct __attribute__((packed)) {
    uint8   n;
    uint8   nu1;
    uint32  PDO[256];
} ec_PDOdesct;

/* SyncManager Communication Type structure for Complete Access mode */
typedef struct __attribute__((packed)) {
    uint8   n;
    uint8   nu1;
    uint8   SMtype[8];
} ec_SMcommtypet;

/* SDO object dictionary list structure */
#define EC_MAXODLIST    1024
#define EC_MAXNAME      40
typedef struct {
    uint16  Slave;
    uint16  Entries;
    uint16  Index[EC_MAXODLIST];
    uint16  DataType[EC_MAXODLIST];
    uint8   ObjectCode[EC_MAXODLIST];
    uint8   MaxSub[EC_MAXODLIST];
    char    Name[EC_MAXODLIST][EC_MAXNAME];
} ec_ODlistt;

/* SDO object entry list structure */
typedef struct {
    uint16  Entries;
    uint8   ValueInfo[256];
    uint16  DataType[256];
    uint16  BitLength[256];
    uint16  ObjAccess[256];
    char    Name[256][EC_MAXNAME];
} ec_OElistt;

/* Error structure */
#define EC_MAXELIST     64
typedef struct {
    osal_timert Time;
    uint16      Slave;
    uint16      Index;
    uint8       SubIdx;
    uint32      Etype;
    int32       AbortCode;
} ec_errort;

/* Error ring buffer */
typedef struct {
    int16     head;
    int16     tail;
    ec_errort Error[EC_MAXELIST + 1];
} ec_eringt;

/* Index stack structure */
typedef struct {
    uint8   pushed;
    uint8   pulled;
    uint8   idx[EC_MAXBUF];
    void    *data[EC_MAXBUF];
    uint16  length[EC_MAXBUF];
} ec_idxstackT;

/* SDO constants */
#define ECT_SDO_SMCOMMTYPE      0x1c00
#define ECT_SDO_PDOASSIGN       0x1c10

/* CoE SDO Info service types */
#define ECT_COES_SDOINFO        0x08
#define ECT_GET_ODLIST_REQ      0x01
#define ECT_GET_ODLIST_RES      0x02
#define ECT_GET_OD_REQ          0x03
#define ECT_GET_OD_RES          0x04
#define ECT_GET_OE_REQ          0x05
#define ECT_GET_OE_RES          0x06
#define ECT_SDOINFO_ERROR       0x07

/* Error types */
#define EC_ERR_TYPE_SDO_ERROR       0x01
#define EC_ERR_TYPE_SDOINFO_ERROR   0x02

/* Helper macros */
#define LO_BYTE(x)  ((uint8_t)((x) & 0xFF))
#define HI_BYTE(x)  ((uint8_t)(((x) >> 8) & 0xFF))

#ifdef __cplusplus
}
#endif

#endif /* _ETHERCATTYPE_H_ */
