/*
 * us_delay.c
 *
 *  Created on: May 4, 2025
 *      Author: Lenovo
 */

#include "us_delay.h"

void HAL_Delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    while ((DWT->CYCCNT - start) < cycles);
}



