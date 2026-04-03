#include "ethercatmain.h"
#include "ethercatbase.h"
#include "osal.h"
#include <string.h>
#include <stdio.h>

/* For __DSB() intrinsic on ARM Cortex-M */
#if defined(__ARMCC_VERSION) || defined(__CC_ARM)
#include <cmsis_armcc.h>
#elif defined(__GNUC__)
#define __DSB() __asm volatile ("dsb" ::: "memory")
#else
#define __DSB() ((void)0)
#endif

/* EtherCAT broadcast MAC */
static const uint8 eth_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/* Source MAC - will be set from NIC */
static const uint8 eth_srcmac[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

/* Global instances */
ec_slavet    ec_slave[EC_MAXSLAVE];
int          ec_slavecount = 0;
volatile ec_groupt ec_group[EC_MAXGROUP];
boolean      EcatError = FALSE;
int64        ec_DCtime = 0;
uint8        ec_IOmap[EC_MAXIOMAP];

/* Global context */
ecx_contextt ecx_context;

/* EEPROM buffer */
static uint8  esibuf[4096];
static uint32 esimap[128];

static void ecx_init_context(ecx_contextt *context)
{
    context->port = &ecx_port;
    context->slavelist = ec_slave;
    context->slavecount = &ec_slavecount;
    context->maxslave = EC_MAXSLAVE;
    context->grouplist = (volatile ec_groupt *)ec_group;
    context->maxgroup = EC_MAXGROUP;
    context->esibuf = esibuf;
    context->esimap = esimap;
    context->ecaterror = &EcatError;
    context->IOmap = ec_IOmap;
    context->IOmapSize = EC_MAXIOMAP;
    context->DCtime = &ec_DCtime;
    context->manualstatechange = FALSE;
}

/* Initialize SOEM, open Ethernet port */
int ecx_init(ecx_contextt *context, const char *ifname)
{
    ecx_init_context(context);
    return ecx_setupnic(context->port, ifname, 0);
}

int ec_init(const char *ifname)
{
    ecx_init_context(&ecx_context);
    return ecx_setupnic(ecx_context.port, ifname, 0);
}

void ecx_close(ecx_contextt *context)
{
    ecx_closenic(context->port);
}

void ec_close(void)
{
    ecx_close(&ecx_context);
}

/* Read EEPROM from slave (word address) */
uint32 ecx_readeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, int timeout)
{
    uint16 configadr = context->slavelist[slave].configadr;
    uint32 edat = 0;
    uint16 estat;
    osal_timert timer;

    /* Write EEPROM address */
    uint16 addr = eeproma;
    ecx_FPWR(context->port, configadr, ECT_REG_EEPADR, sizeof(addr), &addr, timeout);

    /* Command: read */
    uint16 cmd = 0x0100; /* read command */
    ecx_FPWR(context->port, configadr, ECT_REG_EEPCTL, sizeof(cmd), &cmd, timeout);

    /* Wait for completion with timeout check */
    osal_timer_start(&timer, timeout);
    int loop_count = 0;
    do {
        estat = 0;
        ecx_FPRD(context->port, configadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET);
        loop_count++;
        if (loop_count > 1000) {
            /* Timeout - break loop */
            break;
        }
    } while ((estat & 0x8000) && !osal_timer_is_expired(&timer));

    /* Read data */
    ecx_FPRD(context->port, configadr, ECT_REG_EEPDAT, sizeof(edat), &edat, timeout);

    return edat;
}

/* Read EEPROM slave name string */
static void ecx_read_slavename(ecx_contextt *context, uint16 slave, int timeout)
{
    /* CRITICAL: Read mailbox configuration from EEPROM */
    /* Mailbox addresses are stored at EEPROM addresses 0x0018 and 0x001A */

    /* Read RX mailbox address (0x0018) - contains: write mailbox addr + length */
    uint32 rxmbx = ecx_readeeprom(context, slave, 0x0018, timeout);
    context->slavelist[slave].mbx_wo = (uint16)(rxmbx & 0xFFFF);
    context->slavelist[slave].mbx_l = (uint16)(rxmbx >> 16);

    /* Read TX mailbox address (0x001A) - contains: read mailbox addr + length */
    uint32 txmbx = ecx_readeeprom(context, slave, 0x001A, timeout);
    context->slavelist[slave].mbx_ro = (uint16)(txmbx & 0xFFFF);
    context->slavelist[slave].mbx_rl = (uint16)(txmbx >> 16);
    if (context->slavelist[slave].mbx_rl == 0) {
        context->slavelist[slave].mbx_rl = context->slavelist[slave].mbx_l;
    }

    /* Read vendor ID, product code, revision */
    context->slavelist[slave].eep_man = (uint16)ecx_readeeprom(context, slave, 0x0008, timeout);
    context->slavelist[slave].eep_id = (uint16)ecx_readeeprom(context, slave, 0x000A, timeout);
    context->slavelist[slave].eep_rev = (uint16)ecx_readeeprom(context, slave, 0x000C, timeout);

    snprintf(context->slavelist[slave].name, 40, "Slave%d",
             slave);
}

/* Scan and count slaves on the network */
int ecx_config_init(ecx_contextt *context, uint8 usetable)
{
    (void)usetable;
    uint16 wkc;
    uint16 slavecnt = 0;

    /* Reset slave count */
    *context->slavecount = 0;
    memset(context->slavelist, 0, sizeof(ec_slavet) * context->maxslave);
    memset((void *)context->grouplist, 0, sizeof(ec_groupt) * context->maxgroup);

    /* Detect number of slaves by broadcast read of DL status */
    /* Use BRD (Broadcast Read) which doesn't require configured addresses */
    uint8 brd_data[2] = {0, 0};
    
    /* Debug: Print before BRD */
    printf("[ecx_config_init] Before ecx_BRD\r\n");
    
    wkc = ecx_BRD(context->port, 0x0000, ECT_REG_TYPE, 2, &brd_data, EC_TIMEOUTSAFE);
    
    /* Debug: Print after BRD */
    printf("[ecx_config_init] After ecx_BRD, wkc=%d\r\n", wkc);
    
    slavecnt = wkc;

    if (slavecnt == 0)
        return 0;

    /* Found slaves - will be reported by application */

    /* Limit to max */
    if (slavecnt > (context->maxslave - 1))
        slavecnt = context->maxslave - 1;

    *context->slavecount = slavecnt;

    /* Reset all slaves to INIT */
    uint16 init_state = EC_STATE_INIT;
    ecx_BWR(context->port, 0x0000, ECT_REG_ALCTL, sizeof(init_state), &init_state, EC_TIMEOUTSAFE);
    osal_usleep(10000);

    /* Configure station addresses */
    for (uint16 i = 1; i <= slavecnt; i++) {
        uint16 configadr = 0x1000 + i;
        context->slavelist[i].configadr = configadr;
        context->slavelist[i].state = EC_STATE_INIT;

        /* Write station address */
        ecx_APWR(context->port, (uint16)(1 - i), ECT_REG_STAESSION, sizeof(configadr), &configadr, EC_TIMEOUTSAFE);

        /* Read alias address */
        ecx_FPRD(context->port, configadr, ECT_REG_ALIAS, sizeof(context->slavelist[i].aliasadr),
                 &context->slavelist[i].aliasadr, EC_TIMEOUTSAFE);

        /* Read EEPROM info */
        ecx_read_slavename(context, i, EC_TIMEOUTEEP);

        /* Read AL status */
        ecx_FPRD(context->port, configadr, ECT_REG_ALSTAT, sizeof(context->slavelist[i].state),
                 &context->slavelist[i].state, EC_TIMEOUTSAFE);

        /* Slave info will be reported by application */
    }

    return slavecnt;
}

int ec_config_init(uint8 usetable)
{
    return ecx_config_init(&ecx_context, usetable);
}

/* Configure SyncManagers and FMMU, map PDOs to IOmap */
int ecx_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
    uint32 offset_out = 0;
    uint32 offset_in = 0;
    uint16 slavecnt = *context->slavecount;

    /* For each slave, configure SM and FMMU based on EEPROM/SII data */
    for (uint16 i = 1; i <= slavecnt; i++) {
        uint16 configadr = context->slavelist[i].configadr;
        uint16 SM2_start = 0, SM2_len = 0;
        uint16 SM3_start = 0, SM3_len = 0;

        /* Read SyncManager info from EEPROM */
        /* SM0/SM1 are for mailbox, SM2 for outputs, SM3 for inputs */

        /* Try to read SM configuration from SII */
        /* For now, use a simplified approach: read SM registers */
        uint8 smdata[8];

        /* Read SM2 (Process Data Output) */
        if (ecx_FPRD(context->port, configadr, ECT_REG_SM2, 8, smdata, EC_TIMEOUTSAFE) > 0) {
            SM2_start = smdata[0] | (smdata[1] << 8);
            SM2_len = smdata[2] | (smdata[3] << 8);
            if (SM2_len > 0 && SM2_start > 0) {
                context->slavelist[i].Obytes = SM2_len;
                context->slavelist[i].Obits = SM2_len * 8;
                context->slavelist[i].outputs = (uint8 *)pIOmap + offset_out;
                context->slavelist[i].SMtype[2] = EC_SMBUF_PDOUT;

                /* Configure FMMU0 for outputs */
                uint8 fmmu[16] = {0};
                fmmu[0] = (uint8)(offset_out & 0xFF);        /* Logical start addr */
                fmmu[1] = (uint8)((offset_out >> 8) & 0xFF);
                fmmu[2] = (uint8)((offset_out >> 16) & 0xFF);
                fmmu[3] = (uint8)((offset_out >> 24) & 0xFF);
                fmmu[4] = (uint8)(SM2_len & 0xFF);           /* Length */
                fmmu[5] = (uint8)((SM2_len >> 8) & 0xFF);
                fmmu[6] = 0;                                  /* Start bit */
                fmmu[7] = 7;                                  /* End bit */
                fmmu[8] = (uint8)(SM2_start & 0xFF);          /* Physical start */
                fmmu[9] = (uint8)((SM2_start >> 8) & 0xFF);
                fmmu[10] = 0;                                 /* Physical start bit */
                fmmu[11] = 0x02;                              /* Write (output from master, master writes to slave) */
                fmmu[12] = 0x01;                              /* Enable */

                ecx_FPWR(context->port, configadr, ECT_REG_FMMU0, 16, fmmu, EC_TIMEOUTSAFE);
                memcpy(context->slavelist[i].FMMU[0], fmmu, 16);

                offset_out += SM2_len;
            }
        }

        /* Read SM3 (Process Data Input) */
        if (ecx_FPRD(context->port, configadr, ECT_REG_SM3, 8, smdata, EC_TIMEOUTSAFE) > 0) {
            SM3_start = smdata[0] | (smdata[1] << 8);
            SM3_len = smdata[2] | (smdata[3] << 8);
            if (SM3_len > 0 && SM3_start > 0) {
                context->slavelist[i].Ibytes = SM3_len;
                context->slavelist[i].Ibits = SM3_len * 8;
                /* For LRW command, logical addresses should be contiguous
                 * FMMU0: 0x0000 to offset_out-1 (outputs)
                 * FMMU1: offset_out to offset_out+offset_in-1 (inputs)
                 */
                context->slavelist[i].inputs = (uint8 *)pIOmap + offset_out + offset_in;
                context->slavelist[i].SMtype[3] = EC_SMBUF_PDIN;

                /* Configure FMMU1 for inputs */
                uint8 fmmu[16] = {0};
                uint32 logaddr = offset_out + offset_in;  /* Contiguous with outputs */
                fmmu[0] = (uint8)(logaddr & 0xFF);
                fmmu[1] = (uint8)((logaddr >> 8) & 0xFF);
                fmmu[2] = (uint8)((logaddr >> 16) & 0xFF);
                fmmu[3] = (uint8)((logaddr >> 24) & 0xFF);
                fmmu[4] = (uint8)(SM3_len & 0xFF);
                fmmu[5] = (uint8)((SM3_len >> 8) & 0xFF);
                fmmu[6] = 0;
                fmmu[7] = 7;
                fmmu[8] = (uint8)(SM3_start & 0xFF);
                fmmu[9] = (uint8)((SM3_start >> 8) & 0xFF);
                fmmu[10] = 0;
                fmmu[11] = 0x01;  /* Read (input to master, master reads from slave) */
                fmmu[12] = 0x01;  /* Enable */

                ecx_FPWR(context->port, configadr, ECT_REG_FMMU1, 16, fmmu, EC_TIMEOUTSAFE);
                memcpy(context->slavelist[i].FMMU[1], fmmu, 16);

                offset_in += SM3_len;
            }
        }
    }

    /* Store group info */
    context->grouplist[group].Obytes = offset_out;
    context->grouplist[group].outputs = (uint8 *)pIOmap;
    context->grouplist[group].Ibytes = offset_in;
    /* For LRW command, inputs are contiguous with outputs */
    context->grouplist[group].inputs = (uint8 *)pIOmap + offset_out;
    context->grouplist[group].logstartaddr = 0;

    /* IOmap configured - will be reported by application */

    return 1;
}

int ec_config_map(void *pIOmap)
{
    return ecx_config_map_group(&ecx_context, pIOmap, 0);
}

int ec_config_map_group(void *pIOmap, uint8 group)
{
    return ecx_config_map_group(&ecx_context, pIOmap, group);
}

/* Write requested state to slave */
int ecx_writestate(ecx_contextt *context, uint16 slave)
{
    uint16 state = context->slavelist[slave].state;

    if (slave == 0) {
        /* Write to all slaves via broadcast */
        return ecx_BWR(context->port, 0x0000, ECT_REG_ALCTL, sizeof(state), &state, EC_TIMEOUTSAFE);
    } else {
        uint16 configadr = context->slavelist[slave].configadr;
        return ecx_FPWR(context->port, configadr, ECT_REG_ALCTL, sizeof(state), &state, EC_TIMEOUTSAFE);
    }
}

int ec_writestate(uint16 slave)
{
    return ecx_writestate(&ecx_context, slave);
}

/* Read state of all slaves */
uint16 ecx_readstate(ecx_contextt *context)
{
    uint16 lowest = 0xFF;
    uint16 slavecnt = *context->slavecount;

    for (uint16 i = 1; i <= slavecnt; i++) {
        uint16 configadr = context->slavelist[i].configadr;
        uint16 state = 0;
        ecx_FPRD(context->port, configadr, ECT_REG_ALSTAT, sizeof(state), &state, EC_TIMEOUTSAFE);
        context->slavelist[i].state = state;
        if ((state & 0x0F) < lowest)
            lowest = state & 0x0F;
    }
    return lowest;
}

uint16 ec_readstate(void)
{
    return ecx_readstate(&ecx_context);
}

/* Check slave state with timeout */
int ecx_statecheck(ecx_contextt *context, uint16 slave, uint16 reqstate, int timeout)
{
    osal_timert timer;
    uint16 state;

    osal_timer_start(&timer, timeout);
    do {
        if (slave == 0) {
            state = ecx_readstate(context);
        } else {
            uint16 configadr = context->slavelist[slave].configadr;
            state = 0;
            ecx_FPRD(context->port, configadr, ECT_REG_ALSTAT, sizeof(state), &state, EC_TIMEOUTRET);
            context->slavelist[slave].state = state;
        }
        if ((state & 0x0F) == (reqstate & 0x0F))
            return state;
        osal_usleep(1000);
    } while (!osal_timer_is_expired(&timer));

    return state;
}

int ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
{
    return ecx_statecheck(&ecx_context, slave, reqstate, timeout);
}

/* Send process data using LRW with optional DC synchronization */
/* If DC is enabled (grp->hasdc), adds FRMW datagram to read DC time */
int ecx_send_processdata_group(ecx_contextt *context, uint8 group)
{
    volatile ec_groupt *grp = &context->grouplist[group];
    uint32 logaddr = grp->logstartaddr;
    uint16 length;
    int wkc;

    /* If we have both inputs and outputs, use LRW */
    if (grp->Obytes > 0 && grp->Ibytes > 0) {
        /* CRITICAL: When FMMUs use separate addresses, LRW length = Obytes + Ibytes */
        /* Current config: FMMU0 at 0x0000, FMMU1 at 0x000C */
        length = grp->Obytes + grp->Ibytes;  // 12 + 18 = 30 bytes
        /* For LRW, we send output data and expect to get input data back */
        /* Use static buffer to avoid stack overflow (EC_MAXIOMAP=4096 is too large for stack) */
        static uint8 lrwdata[64];  // Max 64 bytes for our application
        if (length > sizeof(lrwdata)) length = sizeof(lrwdata);
        
        /* Clear buffer first, then copy output data */
        memset(lrwdata, 0, length);
        /* Memory barrier to ensure output data is flushed from cache/registers */
        __DSB();
        /* Use volatile pointer to ensure compiler reads from memory */
        memcpy(lrwdata, (const void *)(volatile uint8_t *)grp->outputs, grp->Obytes);
        
        /* Debug: print first few bytes being sent */
        static uint32_t debug_counter = 0;
        debug_counter++;
        if (debug_counter <= 3) {
            printf("[LRW Send] logaddr=0x%08X, len=%d, data=%02X %02X %02X %02X\r\n",
                   logaddr, length, lrwdata[0], lrwdata[1], lrwdata[2], lrwdata[3]);
        }

        int idx = ecx_getindex(context->port);
        uint8 *frame = (uint8 *)&context->port->rxbuf[idx];
        
        /* Setup LRW datagram (first datagram) */
        ecx_setupdatagram(frame, EC_CMD_LRW, (uint8)idx,
                         (uint16)(logaddr & 0xFFFF), (uint16)(logaddr >> 16),
                         length, lrwdata);
        
        /* If DC is enabled, add FPWR datagram to write DC time to reference slave */
        /* This synchronizes the slave's DC clock with the master's DC time */
        if (grp->hasdc && grp->DCnext > 0) {
            uint16 dc_adr = ec_slave[grp->DCnext].configadr;
            uint64 DCtE = htoell(*context->DCtime);
            /* Add FPWR datagram after LRW - writes DC system time to reference slave */
            /* This is similar to IGH's ecrt_master_sync_reference_clock() */
            ecx_adddatagram(frame, EC_CMD_FPWR, (uint8)idx, FALSE,
                           dc_adr, ECT_REG_DCSYSTIME, sizeof(uint64), &DCtE);
        }
        
        ecx_outframe(context->port, idx, 0);

        /* Store idx for receive */
        grp->IOsegment[0] = idx;
        return 1;
    }
    /* Output only: use LWR */
    else if (grp->Obytes > 0) {
        wkc = ecx_LWR(context->port, logaddr, grp->Obytes, grp->outputs, EC_TIMEOUTRET);
        return wkc;
    }
    /* Input only: use LRD */
    else if (grp->Ibytes > 0) {
        wkc = ecx_LRD(context->port, logaddr + EC_MAXIOMAP / 2, grp->Ibytes, grp->inputs, EC_TIMEOUTRET);
        return wkc;
    }

    return 0;
}

int ec_send_processdata_group(uint8 group)
{
    return ecx_send_processdata_group(&ecx_context, group);
}

int ec_send_processdata(void)
{
    return ec_send_processdata_group(0);
}

/**
 * Receive process data from slaves with DC synchronization support
 * Based on original SOEM 1.3.1 implementation
 * @param[in] context   = context struct
 * @param[in] group     = group index
 * @param[in] timeout   = timeout in us
 * @return Workcounter
 */
int ecx_receive_processdata_group(ecx_contextt *context, uint8 group, int timeout)
{
    volatile ec_groupt *grp = &context->grouplist[group];
    int wkc = 0;
    uint64 DCtE;

    if (grp->Obytes > 0 && grp->Ibytes > 0) {
        /* LRW was sent, wait for response */
        int idx = grp->IOsegment[0];
        int framelen = ecx_waitinframe(context->port, idx, timeout);
        if (framelen > 0) {
            uint8 *frame = (uint8 *)&context->port->rxbuf[idx];
            ec_comt *comt = (ec_comt *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE);
            uint16 dlen = etohs(comt->dlength) & 0x07FF;
            uint16 *wkcp = (uint16 *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + dlen);
            wkc = etohs(*wkcp);

            /* Copy input data from response */
            /* For separate logical addresses: response contains [Output echo][Input data] */
            /* FMMU1 inputs are at offset Obytes in the response */
            uint8 *datap = frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE;
            
            /* Debug: print received data */
            static uint32_t debug_counter = 0;
            debug_counter++;
            if (debug_counter <= 3) {
                printf("[LRW Recv] framelen=%d, dlen=%d, wkc=%d, data=%02X %02X %02X %02X\r\n",
                       framelen, dlen, wkc, datap[0], datap[1], datap[2], datap[3]);
            }
            
            /* Copy to inputs buffer - input data is AFTER output echo */
            /* FMMU1 is at logical address 0x000C, which is at offset Obytes (12) */
            if (grp->Ibytes > 0 && grp->inputs) {
                memcpy(grp->inputs, datap + grp->Obytes, grp->Ibytes);
            }
        }
        ecx_setbufstat(context->port, idx, EC_BUF_EMPTY);
    }

    return wkc;
}

int ec_receive_processdata_group(uint8 group, int timeout)
{
    return ecx_receive_processdata_group(&ecx_context, group, timeout);
}

int ec_receive_processdata(int timeout)
{
    return ec_receive_processdata_group(0, timeout);
}

/* Recover slave */
int ec_recover_slave(uint16 slave, int timeout)
{
    ec_slave[slave].state = EC_STATE_INIT;
    ec_writestate(slave);
    return ec_statecheck(slave, EC_STATE_INIT, timeout);
}

int ec_reconfig_slave(uint16 slave, int timeout)
{
    (void)timeout;
    return ec_recover_slave(slave, EC_TIMEOUTSTATE);
}
