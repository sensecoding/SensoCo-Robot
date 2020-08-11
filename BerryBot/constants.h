#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "simpletools.h"

#define WHEEL_D_MM     66.2
#define MM_PER_REV     (WHEEL_D_MM * PI)
#define TICKS_PER_REV  64
#define MM_PER_TICK    (MM_PER_REV / TICKS_PER_REV)

#define SYSCLK_FREQ      CLKFREQ
#define SYSCLK           CNT

#define MM_TO_TICKS(_x) (int)(_x / MM_PER_TICK + 0.5)

#define FALSE 0
#define TRUE (!FALSE)

#define ERR_ARRAY_SIZE 2

#endif
