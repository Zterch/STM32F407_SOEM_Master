#ifndef _ETHERCATCOE_H_
#define _ETHERCATCOE_H_

#include "ethercatmain.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SDO read/write via CoE (CANopen over EtherCAT) */
int ecx_SDOread(ecx_contextt *context, uint16 slave, uint16 index, uint8 subindex,
                boolean CA, int *psize, void *p, int timeout);
int ecx_SDOwrite(ecx_contextt *context, uint16 slave, uint16 index, uint8 subindex,
                 boolean CA, int psize, void *p, int timeout);

/* Simple API */
int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
int ec_SDOwrite(uint16 slave, uint16 index, uint8 subindex, boolean CA, int psize, void *p, int timeout);

/* Read PDO assign and mapping */
int ecx_readPDOassign(ecx_contextt *context, uint16 Slave, uint16 PDOassign);
int ecx_readPDOassignCA(ecx_contextt *context, uint16 Slave, uint16 PDOassign);
int ecx_readPDOmap(ecx_contextt *context, uint16 Slave, int *Osize, int *Isize);
int ecx_readPDOmapCA(ecx_contextt *context, uint16 Slave, int *Osize, int *Isize);

/* Simple API for PDO */
int ec_readPDOassign(uint16 Slave, uint16 PDOassign);
int ec_readPDOassignCA(uint16 Slave, uint16 PDOassign);
int ec_readPDOmap(uint16 Slave, int *Osize, int *Isize);
int ec_readPDOmapCA(uint16 Slave, int *Osize, int *Isize);

#ifdef __cplusplus
}
#endif

#endif
