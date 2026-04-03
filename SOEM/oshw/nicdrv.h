#ifndef _NICDRV_H_
#define _NICDRV_H_

#include "ethercattype.h"

/* Maximum number of ports (1 for STM32) */
#define EC_MAXPORT 1

/* Buffer state definitions */
#define EC_BUF_EMPTY    0x00
#define EC_BUF_ALLOC    0x01
#define EC_BUF_TX       0x02
#define EC_BUF_RCVD     0x03
#define EC_BUF_COMPLETE 0x04

typedef struct {
    /* Ethernet handle */
    void          *eth_handle;
    /* Stack and temporary receive buffers */
    ec_bufT        rxbuf[EC_MAXBUF];
    volatile int   rxbufstat[EC_MAXBUF];
    int            rxsa[EC_MAXBUF];
    /* Temporary receive buffer */
    ec_bufT        tempbuf;
    /* Receive counters */
    int            tempinbuft;
    /* Last received index */
    int            lastidx;
    /* Redundant port flag */
    int            redstate;
    /* Statistics */
    int            txerror;
    int            rxerror;
    /* RX missed counter */
    unsigned int   pktcnt;
} ecx_portt;

extern ecx_portt     ecx_port;
extern ecx_redportt  ecx_redport;

int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary);
int ecx_closenic(ecx_portt *port);
void ecx_setbufstat(ecx_portt *port, int idx, int bufstat);
int ecx_getindex(ecx_portt *port);
int ecx_outframe(ecx_portt *port, int idx, int stacknumber);
int ecx_outframe_red(ecx_portt *port, int idx);
int ecx_waitinframe(ecx_portt *port, int idx, int timeout);
int ecx_srconfirm(ecx_portt *port, int idx, int timeout);

/* Receive raw frame (called from ETH IRQ or polling) */
void ecx_receive_packet(ecx_portt *port, uint8_t *data, uint16_t len);

#endif
