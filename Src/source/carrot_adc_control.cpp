#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "peripheral.hpp"
#include "carrot_adc_control.hpp"
#include "uart_control.hpp"
#include "motor_control.hpp"

extern int16_t ff;

enum{ ADC_BUFFER_LENGTH = 1024 };
uint16_t adc1_Buffer[ADC_BUFFER_LENGTH];

bool LiPo_warning = false;
float LiPo_boltage = 0.0;
void init_adc(void){
	memset(adc1_Buffer, 0, sizeof(adc1_Buffer));
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *) adc1_Buffer, (uint32_t)ADC_BUFFER_LENGTH);
}
void check_adc1_task(void *argument){
	while(1){
		/*
		serial.string("[LiPo_Status] Val:");
		serial.putint(adc1_Buffer[2]);
		serial.string("/");
		serial.putint(4095);
		serial.string(" ,Vol:");
		serial.putfloat(3.3*(float)adc1_Buffer[2]/4095*(2200+330)/330);
		serial.string("\n\r");
		*/
		LiPo_boltage=3.3*(float)adc1_Buffer[2]/4095*(2200+330)/330;
		if((3.3*(float)adc1_Buffer[2]/4095*(2200+330)/330)<=22.2){//1cell under 3.7V
			serial.string("\e[41m\e[33mLiPo is under Voltage !!! \e[0m\n\r");
			LiPo_warning=true;
		}
		osDelay(1000);
	}
	return;
}
