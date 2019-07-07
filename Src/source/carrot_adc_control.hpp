/*
 * carrot_adc_control.hpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */

#ifndef SOURCE_CARROT_ADC_CONTROL_HPP_
#define SOURCE_CARROT_ADC_CONTROL_HPP_


void init_adc(void);


#ifdef __cplusplus
extern "C" {
#endif

void check_adc1_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CARROT_ADC_CONTROL_HPP_ */
