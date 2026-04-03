#include "ethercatbase.h"
#include "ethercatmain.h"
#include "nicdrv.h"
#include <string.h>
#include <stdio.h>

/* EtherCAT broadcast MAC */
static const uint8 eth_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/* Source MAC - will be set from NIC */
static const uint8 eth_srcmac[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

/* Build Ethernet + EtherCAT header and first datagram in buffer */
int ecx_setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
    ec_etherheadert *ehp;
    ec_comt *comt;
    uint8 *datap;
    uint16 *wkc;

    /* Ethernet header */
    ehp = (ec_etherheadert *)frame;
    memcpy(ehp->da, eth_broadcast, 6);
    memcpy(ehp->sa, eth_srcmac, 6);
    ehp->etype = oshw_htons(ETH_P_ECAT);

    /* EtherCAT header: length (11 bits) | reserved (1 bit) | type (4 bits = 0x1) */
    uint16 ecatlen = (uint16)(length + EC_CMDHEADERSIZE + EC_WKCSIZE);
    ehp->elength = htoes((ecatlen & 0x07FF) | 0x1000);

    /* EtherCAT datagram */
    comt = (ec_comt *)((uint8 *)frame + EC_HEADERSIZE + EC_ECATheadersize);
    comt->command = com;
    comt->index = idx;
    comt->ADP = htoes(ADP);
    comt->ADO = htoes(ADO);
    comt->dlength = htoes(length & 0x07FF); /* no more datagrams */

    /* Copy data */
    datap = (uint8 *)frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE;
    if (data && length > 0)
        memcpy(datap, data, length);
    else if (length > 0)
        memset(datap, 0, length);

    /* Working counter = 0 */
    wkc = (uint16 *)(datap + length);
    *wkc = 0;

    return (int)(EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + length + EC_WKCSIZE);
}

/* Add EtherCAT datagram to an existing frame with previous datagram(s)
 * This allows multiple datagrams in a single EtherCAT frame
 * @param[in] frame     = existing frame buffer with first datagram
 * @param[in] com       = command
 * @param[in] idx       = index
 * @param[in] more      = TRUE if more datagrams will follow this one
 * @param[in] ADP       = Address Position
 * @param[in] ADO       = Address Offset
 * @param[in] length    = length of datagram data
 * @param[in] data      = data buffer
 * @return Offset to data in frame (for retrieving response data)
 */
int ecx_adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data)
{
    ec_etherheadert *ehp;
    ec_comt *comt;
    uint8 *datap;
    uint16 *wkc;
    uint16 prev_dlength;
    uint16 new_offset;

    ehp = (ec_etherheadert *)frame;
    
    /* Get current EtherCAT frame length (excluding Ethernet header) */
    prev_dlength = etohs(ehp->elength) & 0x07FF;
    
    /* Update EtherCAT header with new total length */
    uint16 new_ecatlen = prev_dlength + EC_CMDHEADERSIZE + length + EC_WKCSIZE;
    ehp->elength = htoes((new_ecatlen & 0x07FF) | 0x1000);
    
    /* Find the previous datagram's dlength field and set "more datagrams" flag */
    /* First datagram starts at EC_HEADERSIZE + EC_ECATHEADERSIZE */
    uint16 offset = EC_HEADERSIZE + EC_ECATHEADERSIZE;
    ec_comt *prev_comt = (ec_comt *)((uint8 *)frame + offset);
    
    /* Set the "datagram follows" flag in previous datagram(s) */
    uint16 curr_dlen = etohs(prev_comt->dlength) & 0x07FF;
    prev_comt->dlength = htoes(curr_dlen | EC_DATAGRAMFOLLOWS);
    
    /* Calculate offset for new datagram */
    new_offset = EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + curr_dlen + EC_WKCSIZE;
    
    /* Setup new datagram header */
    comt = (ec_comt *)((uint8 *)frame + new_offset);
    comt->command = com;
    comt->index = idx;
    comt->ADP = htoes(ADP);
    comt->ADO = htoes(ADO);
    
    if (more) {
        comt->dlength = htoes(length | EC_DATAGRAMFOLLOWS);
    } else {
        comt->dlength = htoes(length & 0x07FF);
    }
    
    /* Copy data */
    datap = (uint8 *)frame + new_offset + EC_CMDHEADERSIZE;
    if (data && length > 0)
        memcpy(datap, data, length);
    else if (length > 0)
        memset(datap, 0, length);
    
    /* Working counter = 0 */
    wkc = (uint16 *)(datap + length);
    *wkc = 0;
    
    /* Return offset to data (relative to frame start) */
    return (int)(new_offset + EC_CMDHEADERSIZE);
}

/* Generic EtherCAT transaction: build frame, send, wait for response */
static int ecx_command(ecx_portt *port, uint8 com, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{
    int idx;
    int wkc = 0;
    int framelen = 0;
    osal_timert timer1;
    uint8 txframe[EC_BUFSZ];  /* Temporary buffer to protect sent data */

    idx = ecx_getindex(port);
    ecx_setupdatagram(&port->rxbuf[idx], com, (uint8)idx, ADP, ADO, length, data);
    
    /* Save the frame to temporary buffer before sending */
    memcpy(txframe, &port->rxbuf[idx], EC_BUFSZ);
    
    /* Retry loop - send and wait for response until timeout */
    osal_timer_start(&timer1, timeout);
    int retry_count = 0;
    do {
        retry_count++;
        if (retry_count > 1000) {
            break;
        }
        
        /* Restore frame from temporary buffer (in case it was overwritten) */
        memcpy(&port->rxbuf[idx], txframe, EC_BUFSZ);
        
        /* Send frame */
        ecx_outframe(port, idx, 0);
        
        /* Wait for response with fixed timeout per retry */
        framelen = ecx_waitinframe(port, idx, EC_TIMEOUTRET);
        
        if (framelen > 0) {
            break; /* Got response, exit retry loop */
        }
    } while (!osal_timer_is_expired(&timer1));
    
    if (framelen > 0) {
        /* Extract working counter */
        uint8 *frame = (uint8 *)&port->rxbuf[idx];
        
        ec_comt *comt = (ec_comt *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE);
        uint16 dlen = etohs(comt->dlength) & 0x07FF;
        uint16 *wkcp = (uint16 *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + dlen);
        wkc = etohs(*wkcp);

        /* Copy received data back */
        if (data && length > 0) {
            memcpy(data, frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE, length);
        }
    }

    ecx_setbufstat(port, idx, EC_BUF_EMPTY);
    return wkc;
}

/* Byte order conversion helpers
 * EtherCAT protocol is LITTLE-ENDIAN, ARM Cortex-M4 is LITTLE-ENDIAN
 * => No byte swapping needed for EtherCAT fields */
uint16 htoes(uint16 val) { return val; }
uint16 etohs(uint16 val) { return val; }
uint32 htoel(uint32 val) { return val; }
uint32 etohl(uint32 val) { return val; }
uint64 htoell(uint64 val) { return val; }
uint64 etohll(uint64 val) { return val; }

/* EtherCAT command implementations */
int ecx_BWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_BWR, ADP, ADO, length, data, timeout); }

int ecx_BRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_BRD, ADP, ADO, length, data, timeout); }

int ecx_APRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_APRD, ADP, ADO, length, data, timeout); }

int ecx_APWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_APWR, ADP, ADO, length, data, timeout); }

int ecx_APRW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_APRW, ADP, ADO, length, data, timeout); }

int ecx_FPRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_FPRD, ADP, ADO, length, data, timeout); }

int ecx_FPWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_FPWR, ADP, ADO, length, data, timeout); }

int ecx_FPRW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_FPRW, ADP, ADO, length, data, timeout); }

int ecx_LRD(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_LRD, (uint16)(LogAdr & 0xFFFF), (uint16)(LogAdr >> 16), length, data, timeout); }

int ecx_LWR(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_LWR, (uint16)(LogAdr & 0xFFFF), (uint16)(LogAdr >> 16), length, data, timeout); }

int ecx_LRW(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_LRW, (uint16)(LogAdr & 0xFFFF), (uint16)(LogAdr >> 16), length, data, timeout); }

/**
 * LRW with Distributed Clock
 * Logical Read Write with DC time synchronization
 * Simplified version for current SOEM architecture
 * @param[in] port      = port context struct
 * @param[in] LogAdr    = Logical memory address
 * @param[in] length    = length of databuffer
 * @param[in,out] data  = databuffer to write to and read from slave
 * @param[in] DCrs      = Distributed Clock reference slave address
 * @param[out] DCtime   = DC time read from reference slave
 * @param[in] timeout   = timeout in us
 * @return Workcounter or EC_NOFRAME
 */
int ecx_LRWDC(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout)
{
    uint8 idx;
    int wkc;
    uint8 *frame;
    ec_comt *comt;
    uint16 total_len;
    uint64 DCtE;

    idx = ecx_getindex(port);
    frame = (uint8 *)&port->rxbuf[idx];

    /* Build Ethernet header */
    memcpy(frame, eth_broadcast, 6);
    memcpy(frame + 6, eth_srcmac, 6);
    *(uint16 *)(frame + 12) = oshw_htons(ETH_P_ECAT);

    /* EtherCAT header: 2 datagrams (LRW + FRMW) */
    total_len = EC_CMDHEADERSIZE + length + EC_WKCSIZE + EC_CMDHEADERSIZE + sizeof(uint64) + EC_WKCSIZE;
    *(uint16 *)(frame + EC_HEADERSIZE) = htoes((total_len & 0x07FF) | 0x1000);

    /* First datagram: LRW */
    comt = (ec_comt *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE);
    comt->command = EC_CMD_LRW;
    comt->index = idx;
    comt->ADP = htoes((uint16)(LogAdr & 0xFFFF));
    comt->ADO = htoes((uint16)(LogAdr >> 16));
    comt->dlength = htoes((length + EC_CMDHEADERSIZE + sizeof(uint64) + EC_WKCSIZE) | 0x8000); /* more datagrams follow */

    /* Copy LRW data */
    if (data && length > 0)
        memcpy(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE, data, length);

    /* WKC for LRW */
    *(uint16 *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + length) = 0;

    /* Second datagram: FRMW for DC time */
    comt = (ec_comt *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + length + EC_WKCSIZE);
    comt->command = EC_CMD_FRMW;
    comt->index = idx;
    comt->ADP = htoes(DCrs);
    comt->ADO = htoes(ECT_REG_DCSYSTIME);
    comt->dlength = htoes(sizeof(uint64)); /* last datagram */

    /* Copy DC time */
    DCtE = htoell(*DCtime);
    memcpy(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + length + EC_WKCSIZE + EC_CMDHEADERSIZE, &DCtE, sizeof(uint64));

    /* WKC for FRMW */
    *(uint16 *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + length + EC_WKCSIZE + EC_CMDHEADERSIZE + sizeof(uint64)) = 0;

    /* Send frame */
    ecx_outframe(port, idx, 0);

    /* Wait for response */
    int framelen = ecx_waitinframe(port, idx, timeout);

    if (framelen > 0) {
        /* Extract working counter from first datagram */
        comt = (ec_comt *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE);
        uint16 dlen = etohs(comt->dlength) & 0x07FF;
        uint16 *wkcp = (uint16 *)(frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + dlen);
        wkc = etohs(*wkcp);

        /* Copy received data back */
        if (data && length > 0) {
            memcpy(data, frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE, length);
        }

        /* Extract DC time from second datagram response */
        uint16 dlen2 = etohs(comt->dlength) & 0x07FF;
        memcpy(&DCtE, frame + EC_HEADERSIZE + EC_ECATHEADERSIZE + EC_CMDHEADERSIZE + dlen2 + EC_WKCSIZE + EC_CMDHEADERSIZE, sizeof(uint64));
        *DCtime = etohll(DCtE);
    }

    ecx_setbufstat(port, idx, EC_BUF_EMPTY);

    return wkc;
}

int ecx_FRMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout)
{ return ecx_command(port, EC_CMD_FRMW, ADP, ADO, length, data, timeout); }
