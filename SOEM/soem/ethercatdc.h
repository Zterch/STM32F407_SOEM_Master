#ifndef _ETHERCATDC_H_
#define _ETHERCATDC_H_

#include "ethercatmain.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean ecx_configdc(ecx_contextt *context);
void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift);
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift);

boolean ec_configdc(void);
void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift);
void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift);

#ifdef __cplusplus
}
#endif

#endif
