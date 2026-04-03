#include "ethercatcoe.h"
#include "ethercatbase.h"
#include "osal.h"
#include <string.h>
#include <stdio.h>

/* SDO structure, compatible with SOEM 1.3.1 */
typedef struct __attribute__((packed)) {
    uint16 length;
    uint16 address;
    uint8  priority;
    uint8  mbxtype;
} ec_mbxheadert;

/* SDO data union - must be packed for use in packed struct */
typedef union __attribute__((packed)) {
    uint8   bdata[0x200];
    uint16  wdata[0x100];
    uint32  ldata[0x80];
} ec_SDODatat;

/* SDO request/response structure - matches original SOEM 1.3.1 */
typedef struct __attribute__((packed)) {
    ec_mbxheadert   MbxHeader;   /* 6 bytes */
    uint16          CANOpen;     /* 2 bytes - CoE header */
    uint8           Command;     /* 1 byte - SDO command */
    uint16          Index;       /* 2 bytes */
    uint8           SubIndex;    /* 1 byte */
    ec_SDODatat     data;        /* Data starts here (offset 12) */
} ec_SDOt;

/* SDO command codes */
#define ECT_SDO_DOWN_EXP        0x23  /* Download expedited */
#define ECT_SDO_DOWN_INIT       0x21  /* Download normal init */
#define ECT_SDO_DOWN_INIT_CA    0x31  /* Download CA init */
#define ECT_SDO_UP_REQ          0x40  /* Upload request */
#define ECT_SDO_UP_REQ_CA       0x50  /* Upload CA request */
#define ECT_SDO_SEG_UP_REQ      0x60  /* Segment upload request */
#define ECT_SDO_SEG_DOWN        0x00  /* Segment download */
#define ECT_SDO_ABORT           0x80  /* Abort */

#define ECT_SDO_UP_EXP          0x43  /* Expedited upload response */
#define ECT_SDO_UP_SEG          0x00  /* Segment upload response */

/* CoE service types */
#define ECT_COES_SDOREQ         0x01  /* SDO request */
#define ECT_COES_SDORES         0x02  /* SDO response */

/* Mailbox types */
#define ECT_MBXT_COE            0x03  /* CoE */

/* Maximum SDO size */
#define EC_MAXSDO               0x200

/* Timeout definitions */
#define EC_TIMEOUTTXM           50000   /* Timeout for mailbox transmit (increased from 20ms to 50ms) */
#define EC_TIMEOUTRXM           1000000 /* Timeout for mailbox receive (increased from 700ms to 1s) */

/* Debug printf function */
extern void UART_SendString(char *str);
extern void UART_SendHex(uint32_t val, uint8_t digits);
extern void UART_SendLine(char *str);

/* Mailbox send - based on original SOEM 1.3.1 ecx_mbxsend */
/* CRITICAL: mbx already contains mailbox header from ec_SDOt structure */
/* DO NOT add another mailbox header! */
static int ecx_mbxsend_local(ecx_contextt *context, uint16 slave, uint8 *mbx, int timeout)
{
    uint16 configadr = context->slavelist[slave].configadr;
    uint16 mbx_wo = context->slavelist[slave].mbx_wo;
    uint16 mbx_l = context->slavelist[slave].mbx_l;
    osal_timert timer;
    int wkc = 0;
    uint8 SMstat = 0;
    
    /* Get actual send length from mailbox header (mbx[0:1]) */
    /* Mailbox header format (6 bytes):
     * Byte 0-1: Mailbox data length (SDO data length, not including header)
     * Byte 2-3: Station address
     * Byte 4: Channel (0) & Priority (0)
     * Byte 5: Protocol type (0x03 for CoE)
     */
    int sendlen = mbx[0] | (mbx[1] << 8);
    sendlen += 6;  /* Add mailbox header size */
    if (sendlen < 16) sendlen = 16;  /* Minimum mailbox size */
    if (sendlen > mbx_l) sendlen = mbx_l;  /* Maximum mailbox size */

    UART_SendString("[SDO Debug] mbxsend: slave=");
    UART_SendHex(slave, 2);
    UART_SendString(", configadr=0x");
    UART_SendHex(configadr, 4);
    UART_SendString(", mbx_wo=0x");
    UART_SendHex(mbx_wo, 4);
    UART_SendString(", mbx_l=");
    UART_SendHex(mbx_l, 2);
    UART_SendLine("");

    if (mbx_wo == 0 || mbx_l == 0) {
        UART_SendLine("[SDO Debug] ERROR: mbx_wo or mbx_l is 0!");
        return 0;
    }

    /* Debug: print first 16 bytes of mailbox (header + CoE + SDO) */
    UART_SendString("[SDO Debug] Mailbox data: ");
    for (int i = 0; i < 16 && i < sendlen; i++) {
        UART_SendHex(mbx[i], 2);
        UART_SendString(" ");
    }
    UART_SendLine("");

    /* Wait for SM0 not busy and not full */
    /* SM0 is the output mailbox (master->slave) at address mbx_wo */
    /* Bit7 = Busy, Bit3 = Full */
    osal_timer_start(&timer, timeout);
    int poll_cnt = 0;
    int reset_done = 0;
    do {
        SMstat = 0;
        wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM0STAT, 1, &SMstat, EC_TIMEOUTRET);
        if (wkc > 0) {
            /* Check if SM0 is ready: not busy (bit7=0) and not full (bit3=0) */
            if ((SMstat & 0x88) == 0) { 
                break;
            }
            /* CRITICAL: If SM0 is busy (0x80), try to reset it */
            if ((SMstat & 0x80) && !reset_done) {
                UART_SendString("[SDO Debug] SM0 busy (0x80), resetting...");
                /* Disable SM0 - write Activate=0, PDI Control=0 */
                uint8_t sm0_disable[2] = {0x00, 0x00};  /* Activate=0, PDI Control=0 */
                ecx_FPWR(context->port, configadr, ECT_REG_SM0 + 6, 2, sm0_disable, EC_TIMEOUTRET);
                osal_usleep(2000);
                /* Re-enable SM0 - write Activate=1, PDI Control=0 (EEPROM value) */
                uint8_t sm0_enable[2] = {0x01, 0x00};  /* Activate=1, PDI Control=0 */
                ecx_FPWR(context->port, configadr, ECT_REG_SM0 + 6, 2, sm0_enable, EC_TIMEOUTRET);
                osal_usleep(2000);
                reset_done = 1;
                UART_SendLine(" done");
                continue;  /* Check status again */
            }
            if (poll_cnt < 3) {
                UART_SendString("[SDO Debug] SM0stat=0x");
                UART_SendHex(SMstat, 2);
                UART_SendLine(" (waiting for not busy/full)");
            }
        }
        poll_cnt++;
        if (timeout > EC_LOCALDELAY) {
            osal_usleep(EC_LOCALDELAY);
        }
    } while (!osal_timer_is_expired(&timer));

    UART_SendString("[SDO Debug] SM0stat=0x");
    UART_SendHex(SMstat, 2);
    UART_SendString(", polls=");
    UART_SendHex(poll_cnt, 2);
    UART_SendLine("");

    if (osal_timer_is_expired(&timer)) {
        UART_SendLine("[SDO Debug] ERROR: SM0 timeout waiting for ready!");
        return 0;
    }
    
    /* Debug: Check AL status before sending mailbox */
    uint16 alstat = 0;
    int al_wkc = ecx_FPRD(context->port, configadr, ECT_REG_ALSTAT, 2, &alstat, EC_TIMEOUTRET);
    UART_SendString("[SDO Debug] AL Status: 0x");
    UART_SendHex(etohs(alstat), 4);
    UART_SendString(" wkc=");
    UART_SendHex(al_wkc, 1);
    UART_SendLine("");
    
    /* Debug: Check SM0 configuration */
    uint8 sm0_cfg[8];
    int sm0_wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM0, 8, sm0_cfg, EC_TIMEOUTRET);
    UART_SendString("[SDO Debug] SM0 config: ");
    for (int i = 0; i < 8; i++) {
        UART_SendHex(sm0_cfg[i], 2);
        UART_SendString(" ");
    }
    UART_SendString("wkc=");
    UART_SendHex(sm0_wkc, 1);
    UART_SendLine("");
    
    /* Debug: Check SM0 activation (byte 6 of SM0 config) */
    uint8 sm0_act = sm0_cfg[6];
    UART_SendString("[SDO Debug] SM0 activation: 0x");
    UART_SendHex(sm0_act, 2);
    UART_SendLine("");

    /* Debug: print first 32 bytes of mailbox (header + CoE + SDO) */
    UART_SendString("[SDO Debug] Mailbox data (with header): ");
    for (int i = 0; i < 32 && i < sendlen; i++) {
        UART_SendHex(mbx[i], 2);
        UART_SendString(" ");
    }
    UART_SendLine("");

    /* CRITICAL FIX: Use actual data length instead of full mailbox size */
    /* Send mailbox - use actual data length, NOT full mailbox buffer size */
    UART_SendString("[SDO Debug] Sending mailbox len=");
    UART_SendHex(sendlen, 2);
    UART_SendLine("");
    
    wkc = ecx_FPWR(context->port, configadr, mbx_wo, sendlen, mbx, EC_TIMEOUTRET);

    UART_SendString("[SDO Debug] FPWR: addr=0x");
    UART_SendHex(mbx_wo, 4);
    UART_SendString(", len=");
    UART_SendHex(sendlen, 2);
    UART_SendString(", result wkc=");
    UART_SendHex(wkc, 2);
    UART_SendLine("");
    
    /* Debug: Check SM0 status after FPWR */
    uint8 sm0stat_after = 0;
    ecx_FPRD(context->port, configadr, ECT_REG_SM0STAT, 1, &sm0stat_after, EC_TIMEOUTRET);
    UART_SendString("[SDO Debug] SM0stat after: 0x");
    UART_SendHex(sm0stat_after, 2);
    UART_SendLine("");
    
    /* CRITICAL FIX: Wait longer for slave to process the mailbox */
    /* Some slaves (like KaiserDrive) need more time to process mailbox */
    if (wkc > 0) {
        /* Wait for slave to process the mailbox */
        osal_usleep(5000); /* 5ms delay - increased from 1ms */
        
        /* Debug: Check SM0 status after delay */
        uint8 sm0stat_delayed = 0;
        ecx_FPRD(context->port, configadr, ECT_REG_SM0STAT, 1, &sm0stat_delayed, EC_TIMEOUTRET);
        UART_SendString("[SDO Debug] SM0stat after 5ms delay: 0x");
        UART_SendHex(sm0stat_delayed, 2);
        UART_SendLine("");
    }

    return wkc;
}

/* Mailbox receive - based on standard SOEM 1.3.1 ecx_mbxreceive */
static int ecx_mbxreceive_local(ecx_contextt *context, uint16 slave, uint8 *mbx, int timeout)
{
    uint16 configadr = context->slavelist[slave].configadr;
    uint16 mbx_ro = context->slavelist[slave].mbx_ro;
    uint16 mbx_rl = context->slavelist[slave].mbx_rl;
    osal_timert timer;
    int wkc = 0;
    uint8 SMstat = 0;  /* Use 8-bit like original SOEM */

    UART_SendString("[SDO Debug] mbxreceive: slave=");
    UART_SendHex(slave, 2);
    UART_SendString(", configadr=0x");
    UART_SendHex(configadr, 4);
    UART_SendString(", mbx_ro=0x");
    UART_SendHex(mbx_ro, 4);
    UART_SendString(", mbx_rl=");
    UART_SendHex(mbx_rl, 2);
    UART_SendLine("");

    if (mbx_ro == 0 || mbx_rl == 0) {
        UART_SendLine("[SDO Debug] ERROR: mbx_ro or mbx_rl is 0!");
        return 0;
    }

    /* CRITICAL FIX: Check SM1 configuration before polling */
    uint8 sm1_cfg[8];
    int sm1_wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM1, 8, sm1_cfg, EC_TIMEOUTRET);
    if (sm1_wkc > 0) {
        UART_SendString("[SDO Debug] SM1 config: ");
        for (int i = 0; i < 8; i++) {
            UART_SendHex(sm1_cfg[i], 2);
            UART_SendString(" ");
        }
        UART_SendLine("");
        /* Check if SM1 is activated */
        if ((sm1_cfg[6] & 0x01) == 0) {
            UART_SendLine("[SDO Debug] WARNING: SM1 not activated!");
        }
    }

    /* Poll SM1 status for mailbox full flag - use standard SOEM approach */
    /* CRITICAL: Read 8 bits and check bit 3 (like original SOEM) */
    osal_timer_start(&timer, timeout);
    int poll_count = 0;
    do {
        /* Read 8-bit SM1STAT (original SOEM way) */
        wkc = ecx_FPRD(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
        
        if (poll_count < 3) {  // Only print first few polls to avoid spam
            UART_SendString("[SDO Debug] SM1stat poll[");
            UART_SendHex(poll_count, 2);
            UART_SendString("]: wkc=");
            UART_SendHex(wkc, 2);
            UART_SendString(", SMstat=0x");
            UART_SendHex(SMstat, 2);
            UART_SendLine("");
        }
        poll_count++;
        
        /* Check bit 3 (mailbox full) like original SOEM */
        if ((wkc > 0) && ((SMstat & 0x08) > 0)) { /* Mailbox full - bit 3 */
            UART_SendString("[SDO Debug] SM1 full (bit3), reading mailbox...");
            wkc = ecx_FPRD(context->port, configadr, mbx_ro, mbx_rl, mbx, EC_TIMEOUTRET);
            UART_SendString(" wkc=");
            UART_SendHex(wkc, 2);
            UART_SendLine("");
            return wkc;
        }
        
        if ((wkc <= 0) || ((SMstat & 0x08) == 0)) {
            osal_usleep(EC_LOCALDELAY);
        }
    } while (!osal_timer_is_expired(&timer));

    UART_SendString("[SDO Debug] ERROR: SM1 timeout waiting for full! polls=");
    UART_SendHex(poll_count, 4);
    UART_SendString(", final SMstat=0x");
    UART_SendHex(SMstat, 2);
    UART_SendLine("");
    return 0;
}

/* Get next mailbox counter value */
static uint8 ec_nextmbxcnt(uint8 cnt)
{
    cnt++;
    if (cnt > 7) {
        cnt = 1;
    }
    return cnt;
}

/* Clear mailbox buffer */
static void ec_clearmbx(uint8 *mbx)
{
    memset(mbx, 0, EC_MAXMBX);
}

/* SDO Read - based on original SOEM 1.3.1 */
int ecx_SDOread(ecx_contextt *context, uint16 slave, uint16 index, uint8 subindex,
                boolean CA, int *psize, void *p, int timeout)
{
    ec_SDOt *SDOp, *aSDOp;
    uint16 bytesize, Framedatasize;
    int wkc;
    int32 SDOlen;
    uint8 *bp;
    uint8 *hp;
    uint8 mbx[EC_MAXMBX];
    uint8 cnt, toggle;
    boolean NotLast;

    UART_SendString("[SDO Debug] SDOread: slave=");
    UART_SendHex(slave, 2);
    UART_SendString(", index=0x");
    UART_SendHex(index, 4);
    UART_SendString(", subindex=");
    UART_SendHex(subindex, 2);
    UART_SendLine("");

    ec_clearmbx(mbx);
    /* Empty slave out mailbox if something is in. Timeout set to 0 */
    wkc = ecx_mbxreceive_local(context, slave, mbx, 0);

    ec_clearmbx(mbx);
    aSDOp = (ec_SDOt *)mbx;
    SDOp = (ec_SDOt *)mbx;
    SDOp->MbxHeader.length = htoes(0x000a);
    SDOp->MbxHeader.address = htoes(0x0000);
    SDOp->MbxHeader.priority = 0x00;
    /* Get new mailbox count value, used as session handle */
    cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
    context->slavelist[slave].mbx_cnt = cnt;
    SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
    SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* SDO request */

    if (CA) {
        SDOp->Command = ECT_SDO_UP_REQ_CA; /* Upload request complete access */
    } else {
        SDOp->Command = ECT_SDO_UP_REQ; /* Upload request normal */
    }

    SDOp->Index = htoes(index);
    if (CA && (subindex > 1)) {
        subindex = 1;
    }
    SDOp->SubIndex = subindex;
    SDOp->data.ldata[0] = 0;

    UART_SendString("[SDO Debug] SDO request: Command=0x");
    UART_SendHex(SDOp->Command, 2);
    UART_SendString(", CANOpen=0x");
    UART_SendHex(etohs(SDOp->CANOpen), 4);
    UART_SendString(", cnt=");
    UART_SendHex(cnt, 1);
    UART_SendLine("");

    /* Send CoE request to slave */
    wkc = ecx_mbxsend_local(context, slave, mbx, EC_TIMEOUTTXM);
    if (wkc > 0) /* Succeeded to place mailbox in slave ? */
    {
        UART_SendLine("[SDO Debug] mbxsend OK, waiting for response...");
        /* Clean mailbox buffer */
        ec_clearmbx(mbx);
        /* Read slave response */
        wkc = ecx_mbxreceive_local(context, slave, mbx, timeout);
        if (wkc > 0) /* Succeeded to read slave response ? */
        {
            UART_SendString("[SDO Debug] mbxreceive OK, checking response...");
            UART_SendString(" mbxtype=0x");
            UART_SendHex(aSDOp->MbxHeader.mbxtype & 0x0f, 1);
            UART_SendString(", CANOpen type=0x");
            UART_SendHex(etohs(aSDOp->CANOpen) >> 12, 1);
            UART_SendString(", Index=0x");
            UART_SendHex(etohs(aSDOp->Index), 4);
            UART_SendString(", Command=0x");
            UART_SendHex(aSDOp->Command, 2);
            UART_SendLine("");
            /* Slave response should be CoE, SDO response and the correct index */
            if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
                ((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
                 (aSDOp->Index == SDOp->Index))
            {
                UART_SendLine("[SDO Debug] Response OK!");
                if ((aSDOp->Command & 0x02) > 0)
                {
                    /* Expedited frame response */
                    bytesize = 4 - ((aSDOp->Command >> 2) & 0x03);
                    UART_SendString("[SDO Debug] Expedited response, bytesize=");
                    UART_SendHex(bytesize, 1);
                    UART_SendLine("");
                    if (*psize >= bytesize) /* Parameter buffer big enough ? */
                    {
                        /* Copy parameter in parameter buffer */
                        memcpy(p, (void*)&aSDOp->data.ldata[0], bytesize);
                        /* Return the real parameter size */
                        *psize = bytesize;
                    }
                    else
                    {
                        UART_SendLine("[SDO Debug] ERROR: Buffer too small!");
                        wkc = 0;
                    }
                }
                else
                { /* Normal frame response */
                    SDOlen = etohl(aSDOp->data.ldata[0]);
                    /* Does parameter fit in parameter buffer ? */
                    if (SDOlen <= *psize)
                    {
                        bp = p;
                        hp = p;
                        /* Calculate mailbox transfer size */
                        Framedatasize = (etohs(aSDOp->MbxHeader.length) - 10);
                        if (Framedatasize < SDOlen) /* Transfer in segments? */
                        {
                            /* Copy parameter data in parameter buffer */
                            memcpy(hp, (void*)&aSDOp->data.ldata[1], Framedatasize);
                            /* Increment buffer pointer */
                            hp += Framedatasize;
                            *psize = Framedatasize;
                            NotLast = TRUE;
                            toggle = 0x00;
                            while (NotLast) /* Segmented transfer */
                            {
                                SDOp = (ec_SDOt *)mbx;
                                SDOp->MbxHeader.length = htoes(0x000a);
                                SDOp->MbxHeader.address = htoes(0x0000);
                                SDOp->MbxHeader.priority = 0x00;
                                cnt = ec_nextmbxcnt(context->slavelist[slave].mbx_cnt);
                                context->slavelist[slave].mbx_cnt = cnt;
                                SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
                                SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* SDO request */
                                SDOp->Command = ECT_SDO_SEG_UP_REQ + toggle; /* Segment upload request */
                                SDOp->Index = htoes(index);
                                SDOp->SubIndex = subindex;
                                SDOp->data.ldata[0] = 0;
                                /* Send segmented upload request to slave */
                                wkc = ecx_mbxsend_local(context, slave, mbx, EC_TIMEOUTTXM);
                                /* Is mailbox transfered to slave ? */
                                if (wkc > 0)
                                {
                                    ec_clearmbx(mbx);
                                    /* Read slave response */
                                    wkc = ecx_mbxreceive_local(context, slave, mbx, timeout);
                                    /* Has slave responded ? */
                                    if (wkc > 0)
                                    {
                                        /* Slave response should be CoE, SDO response */
                                        if ((((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
                                             ((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
                                             ((aSDOp->Command & 0xe0) == 0x00)))
                                        {
                                            /* Calculate mailbox transfer size */
                                            Framedatasize = etohs(aSDOp->MbxHeader.length) - 3;
                                            if ((aSDOp->Command & 0x01) > 0)
                                            { /* Last segment */
                                                NotLast = FALSE;
                                                if (Framedatasize == 7)
                                                    /* Substract unused bytes from frame */
                                                    Framedatasize = Framedatasize - ((aSDOp->Command & 0x0e) >> 1);
                                                /* Copy to parameter buffer - data starts at Index field in segment response */
                                                memcpy(hp, (uint8*)&(aSDOp->Index), Framedatasize);
                                            }
                                            else /* Segments follow */
                                            {
                                                /* Copy to parameter buffer - data starts at Index field in segment response */
                                                memcpy(hp, (uint8*)&(aSDOp->Index), Framedatasize);
                                                /* Increment buffer pointer */
                                                hp += Framedatasize;
                                            }
                                            /* Update parametersize */
                                            *psize += Framedatasize;
                                        }
                                        /* Unexpected frame returned from slave */
                                        else
                                        {
                                            NotLast = FALSE;
                                            if ((aSDOp->Command) == ECT_SDO_ABORT) /* SDO abort frame received */
                                                wkc = 0;
                                            else
                                                wkc = 0;
                                        }
                                    }
                                }
                                toggle = toggle ^ 0x10; /* Toggle bit for segment request */
                            }
                        }
                        /* Non segmented transfer */
                        else
                        {
                            /* Copy to parameter buffer */
                            memcpy(bp, (void*)&aSDOp->data.ldata[1], SDOlen);
                            *psize = SDOlen;
                        }
                    }
                    /* Parameter buffer too small */
                    else
                    {
                        wkc = 0;
                    }
                }
            }
            /* Other slave response */
            else
            {
                if ((aSDOp->Command) == ECT_SDO_ABORT) /* SDO abort frame received */
                {
                    wkc = 0;
                }
                else
                {
                    wkc = 0;
                }
            }
        }
    }
    return wkc;
}

/* SDO Write - based on original SOEM 1.3.1 */
int ecx_SDOwrite(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIndex,
                 boolean CA, int psize, void *p, int Timeout)
{
    ec_SDOt *SDOp, *aSDOp;
    int wkc, maxdata;
    uint8 mbx[EC_MAXMBX];
    uint8 cnt, toggle;
    uint16 framedatasize;
    boolean NotLast;
    uint8 *hp;

    UART_SendString("[SDO Debug] SDOwrite: slave=");
    UART_SendHex(Slave, 2);
    UART_SendString(", Index=0x");
    UART_SendHex(Index, 4);
    UART_SendString(", SubIndex=");
    UART_SendHex(SubIndex, 2);
    UART_SendString(", psize=");
    UART_SendHex(psize, 2);
    UART_SendLine("");

    ec_clearmbx(mbx);
    /* Empty slave out mailbox if something is in. Timeout set to 0 */
    wkc = ecx_mbxreceive_local(context, Slave, mbx, 0);
    ec_clearmbx(mbx);
    aSDOp = (ec_SDOt *)mbx;
    SDOp = (ec_SDOt *)mbx;
    maxdata = context->slavelist[Slave].mbx_l - 0x10; /* Data section = mailbox size - 6 mbx - 2 CoE - 8 sdo req */

    UART_SendString("[SDO Debug] maxdata=");
    UART_SendHex(maxdata, 2);
    UART_SendLine("");

    /* If small data use expedited transfer */
    if ((psize <= 4) && !CA)
    {
        SDOp->MbxHeader.length = htoes(0x000a);
        SDOp->MbxHeader.address = htoes(0x0000);
        SDOp->MbxHeader.priority = 0x00;
        /* Get new mailbox counter, used for session handle */
        cnt = ec_nextmbxcnt(context->slavelist[Slave].mbx_cnt);
        context->slavelist[Slave].mbx_cnt = cnt;
        SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
        SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* SDO request */
        SDOp->Command = ECT_SDO_DOWN_EXP | (((4 - psize) << 2) & 0x0c); /* Expedited SDO download transfer */
        SDOp->Index = htoes(Index);
        SDOp->SubIndex = SubIndex;
        hp = p;
        /* Copy parameter data to mailbox */
        memcpy((void*)&SDOp->data.ldata[0], hp, psize);

        UART_SendString("[SDO Debug] Expedited write: Command=0x");
        UART_SendHex(SDOp->Command, 2);
        UART_SendString(", cnt=");
        UART_SendHex(cnt, 1);
        UART_SendLine("");

        /* Send mailbox SDO download request to slave */
        wkc = ecx_mbxsend_local(context, Slave, mbx, EC_TIMEOUTTXM);
        if (wkc > 0)
        {
            ec_clearmbx(mbx);
            /* Read slave response */
            wkc = ecx_mbxreceive_local(context, Slave, mbx, Timeout);
            if (wkc > 0)
            {
                /* Response should be CoE, SDO response, correct index and subindex */
                if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
                    ((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
                     (aSDOp->Index == SDOp->Index) &&
                     (aSDOp->SubIndex == SDOp->SubIndex))
                {
                     /* All OK */
                }
                /* Unexpected response from slave */
                else
                {
                    if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
                    {
                        wkc = 0;
                    }
                    else
                    {
                        wkc = 0;
                    }
                }
            }
        }
    }
    else
    {
        framedatasize = psize;
        NotLast = FALSE;
        if (framedatasize > maxdata)
        {
            framedatasize = maxdata;  /* Segmented transfer needed */
            NotLast = TRUE;
        }
        SDOp->MbxHeader.length = htoes(0x0a + framedatasize);
        SDOp->MbxHeader.address = htoes(0x0000);
        SDOp->MbxHeader.priority = 0x00;
        /* Get new mailbox counter, used for session handle */
        cnt = ec_nextmbxcnt(context->slavelist[Slave].mbx_cnt);
        context->slavelist[Slave].mbx_cnt = cnt;
        SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
        SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* SDO request */
        if (CA)
        {
            SDOp->Command = ECT_SDO_DOWN_INIT_CA; /* Complete Access, normal SDO init download transfer */
        }
        else
        {
            SDOp->Command = ECT_SDO_DOWN_INIT; /* Normal SDO init download transfer */
        }
        SDOp->Index = htoes(Index);
        SDOp->SubIndex = SubIndex;
        if (CA && (SubIndex > 1))
        {
            SDOp->SubIndex = 1;
        }
        SDOp->data.ldata[0] = htoel(psize);
        hp = p;
        /* Copy parameter data to mailbox */
        memcpy((void*)&SDOp->data.ldata[1], hp, framedatasize);
        hp += framedatasize;
        psize -= framedatasize;
        /* Send mailbox SDO download request to slave */
        wkc = ecx_mbxsend_local(context, Slave, mbx, EC_TIMEOUTTXM);
        if (wkc > 0)
        {
            ec_clearmbx(mbx);
            /* Read slave response */
            wkc = ecx_mbxreceive_local(context, Slave, mbx, Timeout);
            if (wkc > 0)
            {
                /* Response should be CoE, SDO response, correct index and subindex */
                if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
                    ((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
                     (aSDOp->Index == SDOp->Index) &&
                     (aSDOp->SubIndex == SDOp->SubIndex))
                {
                    /* All ok */
                    maxdata += 7;
                    toggle = 0;
                    /* Repeat while segments left */
                    while (NotLast)
                    {
                        SDOp = (ec_SDOt *)mbx;
                        framedatasize = psize;
                        NotLast = FALSE;
                        SDOp->Command = 0x01; /* Last segment */
                        if (framedatasize > maxdata)
                        {
                            framedatasize = maxdata;  /* More segments needed */
                            NotLast = TRUE;
                            SDOp->Command = 0x00; /* Segments follow */
                        }
                        if (!NotLast && (framedatasize < 7))
                        {
                            SDOp->MbxHeader.length = htoes(0x0a); /* Minimum size */
                            SDOp->Command = 0x01 + ((7 - framedatasize) << 1); /* Last segment reduced octets */
                        }
                        else
                        {
                            SDOp->MbxHeader.length = htoes(framedatasize + 3); /* Data + 2 CoE + 1 SDO */
                        }
                        SDOp->MbxHeader.address = htoes(0x0000);
                        SDOp->MbxHeader.priority = 0x00;
                        /* Get new mailbox counter value */
                        cnt = ec_nextmbxcnt(context->slavelist[Slave].mbx_cnt);
                        context->slavelist[Slave].mbx_cnt = cnt;
                        SDOp->MbxHeader.mbxtype = ECT_MBXT_COE + (cnt << 4); /* CoE */
                        SDOp->CANOpen = htoes(0x000 + (ECT_COES_SDOREQ << 12)); /* SDO request */
                        SDOp->Command = SDOp->Command + toggle; /* Add toggle bit to command byte */
                                /* Copy segment data to mailbox - data starts at Index field in segment request */
                                memcpy((uint8*)&SDOp->Index, hp, framedatasize);
                        /* Update parameter buffer pointer */
                        hp += framedatasize;
                        psize -= framedatasize;
                        /* Send SDO download request */
                        wkc = ecx_mbxsend_local(context, Slave, mbx, EC_TIMEOUTTXM);
                        if (wkc > 0)
                        {
                            ec_clearmbx(mbx);
                            /* Read slave response */
                            wkc = ecx_mbxreceive_local(context, Slave, mbx, Timeout);
                            if (wkc > 0)
                            {
                                if (((aSDOp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_COE) &&
                                    ((etohs(aSDOp->CANOpen) >> 12) == ECT_COES_SDORES) &&
                                    ((aSDOp->Command & 0xe0) == 0x20))
                                {
                                    /* All OK, nothing to do */
                                }
                                else
                                {
                                    if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
                                    {
                                        wkc = 0;
                                    }
                                    else
                                    {
                                        wkc = 0;
                                    }
                                    wkc = 0;
                                    NotLast = FALSE;
                                }
                            }
                        }
                        toggle = toggle ^ 0x10; /* Toggle bit for segment request */
                    }
                }
                /* Unexpected response from slave */
                else
                {
                    if (aSDOp->Command == ECT_SDO_ABORT) /* SDO abort frame received */
                    {
                        wkc = 0;
                    }
                    else
                    {
                        wkc = 0;
                    }
                    wkc = 0;
                }
            }
        }
    }

    return wkc;
}

/* Simple API wrappers */
int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout)
{
    return ecx_SDOread(&ecx_context, slave, index, subindex, CA, psize, p, timeout);
}

int ec_SDOwrite(uint16 slave, uint16 index, uint8 subindex, boolean CA, int psize, void *p, int timeout)
{
    return ecx_SDOwrite(&ecx_context, slave, index, subindex, CA, psize, p, timeout);
}

/* Report SDO error */
static void ecx_SDOerror(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIdx, int32 AbortCode)
{
    (void)context; (void)Slave; (void)Index; (void)SubIdx; (void)AbortCode;
    // Error handling - can be expanded later
}

/* Report packet error */
static void ecx_packeterror(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
    (void)context; (void)Slave; (void)Index; (void)SubIdx; (void)ErrorCode;
    // Error handling - can be expanded later
}

/** Read PDO assign structure
 * @param[in]  context        = context struct
 * @param[in]  Slave         = Slave number
 * @param[in]  PDOassign     = PDO assign object
 * @return total bitlength of PDO assign
 */
int ecx_readPDOassign(ecx_contextt *context, uint16 Slave, uint16 PDOassign)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ecx_SDOread(context, Slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            rdl = sizeof(rdat); rdat = 0;
            /* read PDO assign */
            wkc = ecx_SDOread(context, Slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            idx = etohs(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt); subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ecx_SDOread(context, Slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2); rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ecx_SDOread(context, Slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    if (LO_BYTE(rdat2) < 0xff)
                    {
                        bsize += LO_BYTE(rdat2);
                    }
                    else
                    {
                        rdl = sizeof(rdat); rdat = htoes(0xff);
                        bsize += etohs(rdat);
                    }
                }
            }
        }
    }
    /* return total found bitlength (PDO) */
    return bsize;
}

/** Read PDO assign structure in Complete Access mode
 * @param[in]  context        = context struct
 * @param[in]  Slave         = Slave number
 * @param[in]  PDOassign     = PDO assign object
 * @return total bitlength of PDO assign
 */
int ecx_readPDOassignCA(ecx_contextt *context, uint16 Slave, uint16 PDOassign)
{
    uint16 idxloop, nidx, subidxloop, idx, subidx;
    int wkc, bsize = 0, rdl;
    ec_PDOassignt PDOassign_buf;
    ec_PDOdesct PDOdesc_buf;

    /* find maximum size of PDOassign buffer */
    rdl = sizeof(ec_PDOassignt);
    PDOassign_buf.n = 0;
    /* read rxPDOassign in CA mode, all subindexes are read in one struct */
    wkc = ecx_SDOread(context, Slave, PDOassign, 0x00, TRUE, &rdl, &PDOassign_buf, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (PDOassign_buf.n > 0))
    {
        nidx = PDOassign_buf.n;
        bsize = 0;
        /* for each PDO do */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            /* get index from PDOassign struct */
            idx = etohs(PDOassign_buf.index[idxloop - 1]);
            if (idx > 0)
            {
                rdl = sizeof(ec_PDOdesct);
                PDOdesc_buf.n = 0;
                /* read SDO's that are mapped in PDO, CA mode */
                wkc = ecx_SDOread(context, Slave, idx, 0x00, TRUE, &rdl, &PDOdesc_buf, EC_TIMEOUTRXM);
                subidx = PDOdesc_buf.n;
                /* extract all bitlengths of SDO's */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    bsize += LO_BYTE(etohl(PDOdesc_buf.PDO[subidxloop - 1]));
                }
            }
        }
    }

    /* return total found bitlength (PDO) */
    return bsize;
}

/** CoE read PDO mapping.
 *
 * CANopen has standard indexes defined for PDO mapping. This function
 * tries to read them and collect a full input and output mapping size
 * of designated slave.
 *
 * Principal structure in slave:
 * 1C00:00 is number of SM defined
 * 1C00:01 SM0 type -> 1C10
 * 1C00:02 SM1 type -> 1C11
 * 1C00:03 SM2 type -> 1C12
 * 1C00:04 SM3 type -> 1C13
 * Type 0 = unused, 1 = mailbox in, 2 = mailbox out,
 * 3 = outputs (RxPDO), 4 = inputs (TxPDO).
 *
 * 1C12:00 is number of PDO's defined for SM2
 * 1C12:01 PDO assign SDO #1 -> f.e. 1A00
 * 1C12:02 PDO assign SDO #2 -> f.e. 1A04
 *
 * 1A00:00 is number of object defined for this PDO
 * 1A00:01 object mapping #1, f.e. 60100710 (SDO 6010 SI 07 bitlength 0x10)
 *
 * @param[in]  context        = context struct
 * @param[in] Slave    = Slave number
 * @param[out] Osize   = Size in bits of output mapping (rxPDO) found
 * @param[out] Isize   = Size in bits of input mapping (txPDO) found
 * @return >0 if mapping succesful.
 */
int ecx_readPDOmap(ecx_contextt *context, uint16 Slave, int *Osize, int *Isize)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize;
    uint8 SMt_bug_add;

    *Isize = 0;
    *Osize = 0;
    SMt_bug_add = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ecx_SDOread(context, Slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > 8)
            nSM = 8;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ecx_SDOread(context, Slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                // start slave bug prevention code, remove if possible
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                }
                if(tSM)
                {
                    tSM += SMt_bug_add; // only add if SMt > 0
                }
                if((iSM == 2) && (tSM == 0)) // SM2 has type 0, this is a bug in the slave!
                {
                    tSM = 3;
                }
                if((iSM == 3) && (tSM == 0)) // SM3 has type 0, this is a bug in the slave!
                {
                    tSM = 4;
                }
                // end slave bug prevention code

                context->slavelist[Slave].SMtype[iSM] = tSM;
                /* check if SM is unused -> clear enable flag */
                if (tSM == 0)
                {
                    context->slavelist[Slave].SM[iSM][6] = 0;
                }
                if ((tSM == 3) || (tSM == 4))
                {
                    /* read the assign PDO */
                    Tsize = ecx_readPDOassign(context, Slave, ECT_SDO_PDOASSIGN + iSM );
                    /* if a mapping is found */
                    if (Tsize)
                    {
                        context->slavelist[Slave].SM[iSM][2] = (Tsize + 7) / 8;
                        context->slavelist[Slave].SM[iSM][3] = 0;
                        if (tSM == 3)
                        {
                            /* we are doing outputs */
                            *Osize += Tsize;
                        }
                        else
                        {
                            /* we are doing inputs */
                            *Isize += Tsize;
                        }
                    }
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((*Isize > 0) || (*Osize > 0))
    {
        retVal = 1;
    }

    return retVal;
}

/** CoE read PDO mapping in Complete Access mode (CA).
 *
 * CANopen has standard indexes defined for PDO mapping. This function
 * tries to read them and collect a full input and output mapping size
 * of designated slave. Slave has to support CA, otherwise use ec_readPDOmap().
 *
 * @param[in]  context        = context struct
 * @param[in] Slave      = Slave number
 * @param[out] Osize   = Size in bits of output mapping (rxPDO) found
 * @param[out] Isize   = Size in bits of input mapping (txPDO) found
 * @return >0 if mapping succesful.
 */
int ecx_readPDOmapCA(ecx_contextt *context, uint16 Slave, int *Osize, int *Isize)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize;
    uint8 SMt_bug_add;
    ec_SMcommtypet SMcommtype_buf;

    *Isize = 0;
    *Osize = 0;
    SMt_bug_add = 0;
    rdl = sizeof(ec_SMcommtypet);
    SMcommtype_buf.n = 0;
    /* read SyncManager Communication Type object count Complete Access*/
    wkc = ecx_SDOread(context, Slave, ECT_SDO_SMCOMMTYPE, 0x00, TRUE, &rdl, &SMcommtype_buf, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (SMcommtype_buf.n > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM = SMcommtype_buf.n - 1;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > 8)
        {
            nSM = 8;
        }
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            tSM = SMcommtype_buf.SMtype[iSM];

            // start slave bug prevention code, remove if possible
            if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
            {
                SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
            }
            if(tSM)
            {
                tSM += SMt_bug_add; // only add if SMt > 0
            }
            // end slave bug prevention code

            context->slavelist[Slave].SMtype[iSM] = tSM;
            /* check if SM is unused -> clear enable flag */
            if (tSM == 0)
            {
                context->slavelist[Slave].SM[iSM][6] = 0;
            }
            if ((tSM == 3) || (tSM == 4))
            {
                /* read the assign PDO */
                Tsize = ecx_readPDOassignCA(context, Slave, ECT_SDO_PDOASSIGN + iSM );
                /* if a mapping is found */
                if (Tsize)
                {
                    context->slavelist[Slave].SM[iSM][2] = (Tsize + 7) / 8;
                    context->slavelist[Slave].SM[iSM][3] = 0;
                    if (tSM == 3)
                    {
                        /* we are doing outputs */
                        *Osize += Tsize;
                    }
                    else
                    {
                        /* we are doing inputs */
                        *Isize += Tsize;
                    }
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((*Isize > 0) || (*Osize > 0))
    {
        retVal = 1;
    }
    return retVal;
}

/* Simple wrapper functions */
int ec_readPDOassign(uint16 Slave, uint16 PDOassign)
{
    return ecx_readPDOassign(&ecx_context, Slave, PDOassign);
}

int ec_readPDOassignCA(uint16 Slave, uint16 PDOassign)
{
    return ecx_readPDOassignCA(&ecx_context, Slave, PDOassign);
}

int ec_readPDOmap(uint16 Slave, int *Osize, int *Isize)
{
    return ecx_readPDOmap(&ecx_context, Slave, Osize, Isize);
}

int ec_readPDOmapCA(uint16 Slave, int *Osize, int *Isize)
{
    return ecx_readPDOmapCA(&ecx_context, Slave, Osize, Isize);
}
