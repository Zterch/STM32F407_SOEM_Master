#include "oshw.h"
#include "nicdrv.h"
#include "osal.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// EtherCAT Ethernet type
#define ETH_P_ECAT  0x88A4

// Frame length shift for RX descriptor STATUS field (FL is bits [29:16])
#define ETH_DMARXDESC_FRAMELENGTHSHIFT  16U

extern ETH_HandleTypeDef heth;

// RX/TX buffers (defined in main.c)
extern uint8_t RxBuff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE];
extern uint8_t TxBuff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE];

// Port instances
ecx_portt    ecx_port;
ecx_redportt ecx_redport;

// The source MAC address
// static uint8_t priMAC[6];  // Not used currently

uint16 oshw_htons(uint16 host)
{
    return ((host & 0xFF00) >> 8) | ((host & 0x00FF) << 8);
}

uint16 oshw_ntohs(uint16 network)
{
    return oshw_htons(network);
}

ec_adaptert *oshw_find_adapters(void)
{
    static ec_adaptert adapter;
    memset(&adapter, 0, sizeof(adapter));
    strncpy(adapter.name, "eth0", sizeof(adapter.name));
    strncpy(adapter.desc, "STM32 ETH", sizeof(adapter.desc));
    adapter.next = NULL;
    return &adapter;
}

void oshw_free_adapters(ec_adaptert *adapter)
{
    // Static adapter, nothing to free
}

int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary)
{
    (void)ifname;
    (void)secondary;

    // Clear port structure
    memset(port, 0, sizeof(ecx_portt));

    // Store ETH handle
    port->eth_handle = &heth;

    // Initialize buffer states
    for (int i = 0; i < EC_MAXBUF; i++) {
        port->rxbufstat[i] = EC_BUF_EMPTY;
    }

    // Store MAC address (if needed for future use)
    // MAC address is available in heth.Init.MACAddr

    // CRITICAL: Stop ETH and disable interrupts to avoid race conditions
    // EtherCAT requires polling mode for precise timing control
    HAL_ETH_Stop(&heth);
    HAL_NVIC_DisableIRQ(ETH_IRQn);
    
    // Clear any pending ETH interrupts in NVIC
    NVIC_ClearPendingIRQ(ETH_IRQn);
    
    // Restart ETH in polling mode (no interrupts)
    HAL_ETH_Start(&heth);

    // Enable promiscuous mode for EtherCAT
    // EtherCAT frames come back with modified data, we need to receive all
    ETH_MACFilterConfigTypeDef filterConfig;
    HAL_ETH_GetMACFilterConfig(&heth, &filterConfig);
    filterConfig.PromiscuousMode = ENABLE;
    HAL_ETH_SetMACFilterConfig(&heth, &filterConfig);

    // Assign TX buffer addresses to TX descriptors
    for (uint32_t i = 0; i < ETH_TX_DESC_CNT; i++) {
        ETH_DMADescTypeDef *txdesc = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[i];
        txdesc->DESC2 = (uint32_t)TxBuff[i];
    }

    // CRITICAL: Clear any pending RX frames by polling and discarding
    // This prevents stale frames from interfering with new communication
    uint32_t descidx = heth.RxDescList.RxDescIdx;
    for (int i = 0; i < ETH_RX_DESC_CNT; i++) {
        ETH_DMADescTypeDef *dmarxdesc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[descidx];
        // If descriptor is owned by CPU (OWN=0), return it to DMA
        if ((dmarxdesc->DESC0 & ETH_DMARXDESC_OWN) == 0) {
            dmarxdesc->DESC1 = heth.Init.RxBuffLen | ETH_DMARXDESC_DIC | ETH_DMARXDESC_RCH;
            __DSB();
            dmarxdesc->DESC0 = ETH_DMARXDESC_OWN;
        }
        descidx = (descidx + 1) % ETH_RX_DESC_CNT;
    }
    heth.RxDescList.RxDescIdx = descidx;

    return 1;
}

int ecx_closenic(ecx_portt *port)
{
    HAL_ETH_Stop((ETH_HandleTypeDef *)port->eth_handle);
    return 0;
}

int ecx_getindex(ecx_portt *port)
{
    int idx;
    int cnt = 0;

    do {
        idx = port->lastidx + 1;
        if (idx >= EC_MAXBUF)
            idx = 0;
        port->lastidx = idx;
        cnt++;
        // Memory barrier to ensure consistent read of rxbufstat
        __DSB();
    } while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF));

    port->rxbufstat[idx] = EC_BUF_ALLOC;
    __DSB();
    return idx;
}

void ecx_setbufstat(ecx_portt *port, int idx, int bufstat)
{
    port->rxbufstat[idx] = bufstat;
}

// Send raw Ethernet frame using DMA TX descriptors directly
static int ecx_send_frame(ecx_portt *port, void *frame, int len)
{
    ETH_HandleTypeDef *ethh = (ETH_HandleTypeDef *)port->eth_handle;

    // Find next available TX descriptor
    uint32_t descidx = ethh->TxDescList.CurTxDesc;
    ETH_DMADescTypeDef *dmatxdesc = (ETH_DMADescTypeDef *)ethh->TxDescList.TxDesc[descidx];

    // Check if descriptor is available (OWN bit = 0 means CPU owns it)
    if (dmatxdesc->DESC0 & ETH_DMATXDESC_OWN) {
        port->txerror++;
        return 0;
    }

    // Copy frame data to buffer pointed by DESC2
    uint8_t *buffer = (uint8_t *)dmatxdesc->DESC2;
    memcpy(buffer, frame, len);

    // Pad to minimum frame size
    if (len < 60)
        len = 60;

    // Set frame length in DESC1
    dmatxdesc->DESC1 = (len & ETH_DMATXDESC_TBS1);

    // Set FS, LS, TCH (chained), checksum, and OWN bits
    dmatxdesc->DESC0 = ETH_DMATXDESC_OWN | ETH_DMATXDESC_LS | ETH_DMATXDESC_FS
                      | ETH_DMATXDESC_TCH | ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;

    // Data synchronization barrier to ensure descriptor is written before DMA reads it
    __DSB();

    // Resume DMA transmission if suspended
    if (ethh->Instance->DMASR & ETH_DMASR_TBUS) {
        ethh->Instance->DMASR = ETH_DMASR_TBUS;
        ethh->Instance->DMATPDR = 0;
    }

    // Move to next descriptor
    ethh->TxDescList.CurTxDesc = (descidx + 1) % ETH_TX_DESC_CNT;

    return len;
}

int ecx_outframe(ecx_portt *port, int idx, int stacknumber)
{
    (void)stacknumber;
    ec_bufT *buf = &port->rxbuf[idx];

    // Frame layout:
    // [0-5]   = destination MAC
    // [6-11]  = source MAC
    // [12-13] = EtherType (0x88A4, big-endian)
    // [14-15] = EtherCAT header (little-endian): length[10:0] | reserved | type[3:0]
    // [16+]   = EtherCAT datagrams

    // EtherCAT header is at byte 14-15, little-endian on ARM
    uint16_t ecathdr = (*buf)[14] | ((uint16_t)(*buf)[15] << 8);
    int ecatlen = ecathdr & 0x07FF; // lower 11 bits = payload length
    int framelen = 14 + 2 + ecatlen;    // Ethernet header (14) + EtherCAT header (2) + payload
    if (framelen < 60) framelen = 60;

    port->rxbufstat[idx] = EC_BUF_TX;
    
    // Memory barrier to ensure rxbufstat is written before sending
    __DSB();

    return ecx_send_frame(port, buf, framelen);
}

int ecx_outframe_red(ecx_portt *port, int idx)
{
    return ecx_outframe(port, idx, 0);
}

// Poll for received Ethernet frames using DMA RX descriptors directly
static int ecx_receive_poll(ecx_portt *port)
{
    ETH_HandleTypeDef *ethh = (ETH_HandleTypeDef *)port->eth_handle;
    int received = 0;

    uint32_t descidx = ethh->RxDescList.RxDescIdx;
    ETH_DMADescTypeDef *dmarxdesc = (ETH_DMADescTypeDef *)ethh->RxDescList.RxDesc[descidx];

    while ((dmarxdesc->DESC0 & ETH_DMARXDESC_OWN) == 0)
    {
        if ((dmarxdesc->DESC0 & ETH_DMARXDESC_ES) == 0)
        {
            uint16_t len = (dmarxdesc->DESC0 & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT;
            uint8_t *buffer = (uint8_t *)dmarxdesc->DESC2;

            // Check if this is an EtherCAT frame (EtherType 0x88A4)
            uint16_t ethtype = (buffer[12] << 8) | buffer[13];
            if (ethtype == ETH_P_ECAT) {
                ecx_receive_packet(port, buffer, len);
                received++;
            }
        } else {
            port->rxerror++;
        }

        // Return descriptor to DMA: set buffer size, chained mode, disable interrupt on completion
        dmarxdesc->DESC1 = ethh->Init.RxBuffLen | ETH_DMARXDESC_DIC | ETH_DMARXDESC_RCH;
        // Set OWN bit last (gives descriptor back to DMA)
        __DSB();
        dmarxdesc->DESC0 = ETH_DMARXDESC_OWN;

        // Move to next descriptor
        descidx = (descidx + 1) % ETH_RX_DESC_CNT;
        ethh->RxDescList.RxDescIdx = descidx;
        dmarxdesc = (ETH_DMADescTypeDef *)ethh->RxDescList.RxDesc[descidx];

        // Resume DMA reception if suspended
        if (ethh->Instance->DMASR & ETH_DMASR_RBUS) {
            ethh->Instance->DMASR = ETH_DMASR_RBUS;
            ethh->Instance->DMARPDR = 0;
        }
    }
    return received;
}

// Process received EtherCAT packet - match to waiting buffer
void ecx_receive_packet(ecx_portt *port, uint8_t *data, uint16_t len)
{
    // EtherCAT frame: after Ethernet header (14 bytes)
    // [12-13] = EtherType (should be 0x88A4)
    // [14-15] = EtherCAT header (2 bytes): length | type
    // [16] = first datagram header: command(1), index(1), ...
    // The index byte at offset 17 identifies which buffer this reply belongs to

    if (len < 18)
        return;

    // Debug: print received packet info (disabled for performance)
    // printf("RX: len=%d idx=%d stat=%d type=%04X cmd=%02X\r\n",
    //        len, data[17], port->rxbufstat[data[17]],
    //        (uint16_t)((data)[12]<<8 | (data)[13]), data[16]);

    int idx = data[17]; // EtherCAT datagram index
    if (idx >= EC_MAXBUF) {
        return;
    }

    // Direct access to rxbufstat
    int val = port->rxbufstat[idx];

    if (val == EC_BUF_TX) {
        memcpy(&port->rxbuf[idx], data, len);
        port->rxsa[idx] = len;
        port->rxbufstat[idx] = EC_BUF_RCVD;
        __DSB();
        port->pktcnt++;
    } else {
        // Store in temp buffer if no matching TX buffer
        memcpy(&port->tempbuf, data, len);
        port->tempinbuft = len;
    }
}

int ecx_waitinframe(ecx_portt *port, int idx, int timeout)
{
    osal_timert timer;
    osal_timer_start(&timer, timeout);
    int poll_count = 0;

    do {
        // Poll for received frames
        ecx_receive_poll(port);
        poll_count++;
        
        // Memory barrier before reading rxbufstat
        __DSB();
        
        // Read rxbufstat directly
        int stat_val = port->rxbufstat[idx];
        
        // Safety check: prevent infinite loop
        if (poll_count > 100000) {
            return 0;
        }

        if (stat_val == EC_BUF_RCVD) {
            port->rxbufstat[idx] = EC_BUF_COMPLETE;
            return port->rxsa[idx]; // Return frame length
        }
    } while (!osal_timer_is_expired(&timer));

    return 0; // Timeout
}

int ecx_srconfirm(ecx_portt *port, int idx, int timeout)
{
    int result = ecx_waitinframe(port, idx, timeout);
    if (result > 0) {
        return EC_BUF_COMPLETE;
    }
    return EC_BUF_EMPTY;
}
