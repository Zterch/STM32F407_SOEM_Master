#ifndef _OSAL_DEFS_H_
#define _OSAL_DEFS_H_

#include <stdint.h>

// No RTOS - bare metal
typedef int32_t  int32;
typedef uint32_t uint32;
typedef int16_t  int16;
typedef uint16_t uint16;
typedef int8_t   int8;
typedef uint8_t  uint8;
typedef uint8_t  boolean;

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#endif
