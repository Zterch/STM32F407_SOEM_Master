#ifndef _ETHERCATMAIN_H_
#define _ETHERCATMAIN_H_

#include "ethercattype.h"
#include "nicdrv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EtherCAT register addresses */
#define ECT_REG_TYPE        0x0000
#define ECT_REG_PORTDES     0x0007
#define ECT_REG_ESCSUP      0x0008
#define ECT_REG_STAESSION   0x0010
#define ECT_REG_ALIAS       0x0012
#define ECT_REG_DLCTL       0x0100
#define ECT_REG_DLPORT      0x0101
#define ECT_REG_DLALIAS     0x0103
#define ECT_REG_DLSTAT      0x0110
#define ECT_REG_ALCTL       0x0120
#define ECT_REG_ALSTAT      0x0130
#define ECT_REG_ALSTATCODE  0x0134
#define ECT_REG_PDICTL      0x0140
#define ECT_REG_IRQMASK     0x0200
#define ECT_REG_RXERR       0x0300
#define ECT_REG_FRXERR      0x0308
#define ECT_REG_EPUECNT     0x030C
#define ECT_REG_PECNT       0x030D
#define ECT_REG_PEERR       0x030E
#define ECT_REG_LLCNT       0x0310
#define ECT_REG_WDCNT       0x0442
#define ECT_REG_EEPCFG      0x0500
#define ECT_REG_EEPCTL      0x0502
#define ECT_REG_EEPSTAT     0x0502
#define ECT_REG_EEPADR      0x0504
#define ECT_REG_EEPDAT      0x0508
#define ECT_REG_FMMU0       0x0600
#define ECT_REG_FMMU1       0x0610
#define ECT_REG_FMMU2       0x0620
#define ECT_REG_SM0         0x0800
#define ECT_REG_SM0STAT     0x0805
#define ECT_REG_SM1         0x0808
#define ECT_REG_SM1STAT     0x080D
#define ECT_REG_SM1ACT      0x080E
#define ECT_REG_SM2         0x0810
#define ECT_REG_SM3         0x0818
#define ECT_REG_DCSYSTIME   0x0910
#define ECT_REG_DCSPEEDCNT  0x0930
#define ECT_REG_DCTIMEFILT  0x0934
#define ECT_REG_DCCUC       0x0980
#define ECT_REG_DCSYNCACT   0x0981
#define ECT_REG_DCSTART0    0x0990
#define ECT_REG_DCCYCLE0    0x09A0
#define ECT_REG_DCCYCLE1    0x09A4

/* SyncManager types */
#define EC_SMBUF_MAILBOX_W  1
#define EC_SMBUF_MAILBOX_R  2
#define EC_SMBUF_PDOUT      3
#define EC_SMBUF_PDIN       4

/* Main SOEM context structure */
typedef struct ecx_context {
    ecx_portt      *port;
    ec_slavet      *slavelist;
    int            *slavecount;
    int             maxslave;
    volatile ec_groupt *grouplist;
    int             maxgroup;
    uint8          *esibuf;
    uint32         *esimap;
    uint16          esislave;
    boolean        *ecaterror;
    uint8          *IOmap;
    uint32          IOmapSize;
    int64          *DCtime;
    ec_slavet      *dc_slave;       /* DC reference slave */
    /* Distributed clock */
    boolean         manualstatechange;
} ecx_contextt;

/* Global/default context (simple API) */
extern ecx_contextt  ecx_context;
extern ec_slavet     ec_slave[EC_MAXSLAVE];
extern int           ec_slavecount;
extern volatile ec_groupt ec_group[EC_MAXGROUP];
extern boolean       EcatError;
extern int64         ec_DCtime;
extern uint8         ec_IOmap[EC_MAXIOMAP];

/* Simple API (uses global context) */
int ec_init(const char *ifname);
void ec_close(void);
int ec_config_init(uint8 usetable);
int ec_config_map(void *pIOmap);
int ec_config_map_group(void *pIOmap, uint8 group);

int ec_statecheck(uint16 slave, uint16 reqstate, int timeout);
int ec_writestate(uint16 slave);
uint16 ec_readstate(void);
int ec_recover_slave(uint16 slave, int timeout);
int ec_reconfig_slave(uint16 slave, int timeout);

/* Process data exchange */
int ec_send_processdata(void);
int ec_receive_processdata(int timeout);
int ec_send_processdata_group(uint8 group);
int ec_receive_processdata_group(uint8 group, int timeout);

/* Context API */
int ecx_init(ecx_contextt *context, const char *ifname);
void ecx_close(ecx_contextt *context);
int ecx_config_init(ecx_contextt *context, uint8 usetable);
int ecx_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group);
int ecx_statecheck(ecx_contextt *context, uint16 slave, uint16 reqstate, int timeout);
int ecx_writestate(ecx_contextt *context, uint16 slave);
uint16 ecx_readstate(ecx_contextt *context);
int ecx_send_processdata_group(ecx_contextt *context, uint8 group);
int ecx_receive_processdata_group(ecx_contextt *context, uint8 group, int timeout);

/* EEPROM access */
uint32 ecx_readeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, int timeout);

/* SII (Slave Information Interface) categories */
#define ECT_SII_STRING    10
#define ECT_SII_GENERAL   30
#define ECT_SII_FMMU      40
#define ECT_SII_SM        41
#define ECT_SII_PDO       50

/* Mailbox types */
#define ECT_MBXT_ERR     0x00
#define ECT_MBXT_AOE     0x01
#define ECT_MBXT_EOE     0x02
#define ECT_MBXT_COE     0x03
#define ECT_MBXT_FOE     0x04
#define ECT_MBXT_SOE     0x05
#define ECT_MBXT_VOE     0x0F

#ifdef __cplusplus
}
#endif

#endif
