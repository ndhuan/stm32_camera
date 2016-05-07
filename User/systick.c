#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "systick.h"
#include "def.h"
VOID_PFUNC_VOID SYSTIM_TimPFunc = NULL;
uint32_t tick1ms=0;
/******************************************************************************
 * @fn     SYSTIM_Init     
 * @brief  Initialise the system tick timer, interrupt every 1ms
 * @param  None
 * @retval None
 ******************************************************************************/
void SYSTIM_Init(void)
{
    SysTick_Config(SystemCoreClock/1000);
}

/******************************************************************************
 * @fn     SYSTIM_Callback     
 * @brief  
 * @param  None
 * @retval None
 ******************************************************************************/
void SYSTIM_Callback(VOID_PFUNC_VOID f_timing)
{
    SYSTIM_TimPFunc = f_timing;
}

/******************************************************************************
 * @fn     SYSTIM_DelayTms     
 * @brief  Delay T(ms)
 * @param  T: delay time (ms)
 * @retval None
 ******************************************************************************/
void SYSTIM_DelayTms(uint32_t T)
{
    uint32_t t = SYSTIM_Tick();
    while(SYSTIM_Timeout(t, T) == false);
}


/******************************************************************************
 * @fn     SYSTIM_DelayTms     
 * @brief  Delay T(ms)
 * @param  T: delay time (ms)
 * @retval None
 ******************************************************************************/
bool SYSTIM_Timeout(uint32_t tim_tick, uint32_t time_out)
{
    uint32_t dt;
    uint32_t t = SYSTIM_Tick();
    if (tim_tick > t)
        dt = 0xFFFFFFFF - (tim_tick - t);
    else
        dt = t - tim_tick;
    
    if (dt >= time_out)
    {
        return true;
    }
    return false;
}

/******************************************************************************
 * @fn     SYSTIM_Tick     
 * @brief  Get the system tick (ms)
 *          This interface makes sure other functions can not change the system tick 
 * @param  
 * @retval None
 ******************************************************************************/
uint32_t SYSTIM_Tick(void)
{
    return tick1ms;
}

/******************************************************************************
 * @fn     SYSTIM_Interrupt     
 * @brief  Delay T(ms)
 * @param  T: delay time (ms)
 * @retval None
 ******************************************************************************/
void SysTick_Handler(void)
{
    tick1ms++;
    //LED_Set(LED_RED, TOGGLE);
    
    if (SYSTIM_TimPFunc != NULL)
    {
        SYSTIM_TimPFunc();
    }
}
