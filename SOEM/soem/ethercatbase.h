#ifndef _ETHERCATBASE_H_
#define _ETHERCATBASE_H_

#include "ethercattype.h"
#include "nicdrv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Byte order helpers (EtherCAT = little-endian = ARM native) */
uint16 htoes(uint16 val);
uint16 etohs(uint16 val);
uint32 htoel(uint32 val);
uint32 etohl(uint32 val);
uint64 htoell(uint64 val);
uint64 etohll(uint64 val);

/* Network byte order helpers (for Ethernet header) */
uint16 oshw_htons(uint16 host);
uint16 oshw_ntohs(uint16 network);

/* Setup datagram in buffer */
int ecx_setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);

/* Add datagram to existing frame (for multiple datagrams per frame) */
int ecx_adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);

/* Build and send EtherCAT frame, wait for response */
int ecx_BWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_BRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_APRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_APWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_APRW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_FPRD(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_FPWR(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_FPRW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ecx_LRD(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
int ecx_LWR(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
int ecx_LRW(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, int timeout);
int ecx_LRWDC(ecx_portt *port, uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout);
int ecx_FRMW(ecx_portt *port, uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);

#ifdef __cplusplus
}
#endif

#endif
