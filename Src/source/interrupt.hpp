/*
 * interrupt.hpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */

#ifndef SOURCE_INTERRUPT_HPP_
#define SOURCE_INTERRUPT_HPP_


#ifdef __cplusplus
extern "C" {
#endif
void EMERGENCY_notification(void *argument);
void _HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void _EXTI0_IRQHandler(void);
#ifdef __cplusplus
};
#endif



#endif /* SOURCE_INTERRUPT_HPP_ */
