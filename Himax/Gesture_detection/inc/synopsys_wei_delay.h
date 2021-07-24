#ifndef ARC_DELAY_H
#define ARC_DELAY_H

#include "hx_drv_tflm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CPU_CLK 400000000

void hal_delay_ms(unsigned int input_ms);

#ifdef __cplusplus
}
#endif

#endif /* ARC_DELAY_H */

