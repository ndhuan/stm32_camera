#ifndef __SYSTICK_H
#define __SYSTICK_H

/*******************************************************************************
 * COMMON DEFINITIONS
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "def.h"

void SYSTIM_Init(void);
void SYSTIM_Callback(VOID_PFUNC_VOID f_timing);
void SYSTIM_DelayTms(uint32_t T);
bool SYSTIM_Timeout(uint32_t tim_tick, uint32_t time_out);
uint32_t SYSTIM_Tick(void);

#endif /* __SYSTICK_H */
