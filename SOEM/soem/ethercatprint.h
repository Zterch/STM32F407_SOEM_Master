#ifndef _ETHERCATPRINT_H_
#define _ETHERCATPRINT_H_

#include "ethercatmain.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *ec_ALstatuscode2string(uint16 ALstatuscode);
char *ec_elist2string(void);
const char *ec_sdoerror2string(uint32 sdoerrorcode);

#ifdef __cplusplus
}
#endif

#endif
