#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "peripheral.hpp"
#include "carrot_adc_control.hpp"
#include "uart_control.hpp"
#include "motor_control.hpp"

extern uart serial;

extern int16_t ff;

enum{ ADC_BUFFER_LENGTH = 2 };
uint16_t adc1_Buffer[ADC_BUFFER_LENGTH];

bool LiPo_warning = false;
bool Logic_warning = false;
float LiPo_boltage = 0.0;
float Logic_boltage = 0.0;
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
		LiPo_boltage=3.3*(float)adc1_Buffer[0]/4095*(2200+330)/330;
		Logic_boltage=3.3*(float)adc1_Buffer[1]/4095*(2200+330)/330;
		if(LiPo_boltage<=LIPO_WARNING_VOLTAGE){//1cell under 3.5V
			serial.string("\e[41m\e[33mMOTOR_SUPPLY is under Voltage !!! \e[0m\n\r");
			LiPo_warning=true;
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,5);
			if(EMERGENCY==false){
				HAL_GPIO_WritePin(FET_RIGHT_GPIO_Port,FET_RIGHT_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(FET_LEFT_GPIO_Port,FET_LEFT_Pin,GPIO_PIN_RESET);
			}
		}
		else{
			LiPo_warning=false;
			if(EMERGENCY==false){
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
			}
			#ifdef LOW_POWER
			#else
			HAL_GPIO_TogglePin(FET_RIGHT_GPIO_Port,FET_RIGHT_Pin);
			HAL_GPIO_TogglePin(FET_LEFT_GPIO_Port,FET_LEFT_Pin);
			#endif
		}
		if(Logic_boltage<=LOGIC_WARNING_VOLTAGE){
			serial.string("\e[41m\e[33mLOGIC is under Voltage !!! \e[0m\n\r");
			Logic_warning=true;
		}
		else{
			Logic_warning=false;
		}
		osDelay(802);
	}
	return;
}
