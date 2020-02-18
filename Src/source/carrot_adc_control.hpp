/*
 * carrot_adc_control.hpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */

#ifndef SOURCE_CARROT_ADC_CONTROL_HPP_
#define SOURCE_CARROT_ADC_CONTROL_HPP_

#include <stdint.h>
void init_adc(void);
enum{ ADC_BUFFER_LENGTH = 1024 };
extern uint16_t adc1_Buffer[1024];

#ifdef __cplusplus
extern "C" {
#endif

void check_adc1_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CARROT_ADC_CONTROL_HPP_ */
