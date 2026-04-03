#ifndef _OSHW_H_
#define _OSHW_H_

#include "ethercattype.h"
#include <stdint.h>

uint16 oshw_htons(uint16 host);
uint16 oshw_ntohs(uint16 network);

ec_adaptert *oshw_find_adapters(void);
void oshw_free_adapters(ec_adaptert *adapter);

#endif
