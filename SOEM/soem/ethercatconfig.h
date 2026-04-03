#ifndef _ETHERCATCONFIG_H_
#define _ETHERCATCONFIG_H_

#include "ethercatmain.h"

#ifdef __cplusplus
extern "C" {
#endif

int ecx_config(ecx_contextt *context, uint8 usetable, void *pIOmap);
int ec_config(uint8 usetable, void *pIOmap);

#ifdef __cplusplus
}
#endif

#endif
