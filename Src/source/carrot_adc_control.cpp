#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "peripheral.hpp"
#include "carrot_adc_control.hpp"
#include "uart_control.hpp"


enum{ ADC_BUFFER_LENGTH = 1024 };
uint32_t adc1_Buffer[ADC_BUFFER_LENGTH];

void init_adc(void){

	memset(adc1_Buffer, 0, sizeof(adc1_Buffer));
	HAL_ADC_Start_DMA(&hadc1, adc1_Buffer, (uint32_t)ADC_BUFFER_LENGTH);
}
void check_adc1_task(void *argument){
	while(1){
		serial.putint((int64_t)adc1_Buffer[1]);
		serial.string("\n\r");
		HAL_GPIO_TogglePin(FET8_GPIO_Port,FET8_Pin);
		osDelay(500);
	}
	return;
}
