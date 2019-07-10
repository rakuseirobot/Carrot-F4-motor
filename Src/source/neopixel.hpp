/*
 * neopixel.hpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */

#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void  NeoPixInit (TIM_HandleTypeDef* _htim, uint32_t channel);
void  NeoPixStart (uint8_t* data, int len, bool wait);


#ifdef __cplusplus
}
#endif
