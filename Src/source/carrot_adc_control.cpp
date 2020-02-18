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

uint16_t adc1_Buffer[ADC_BUFFER_LENGTH];

bool LiPo_warning = false;
bool Logic_warning = false;
float LiPo_boltage = 0.0;
float Logic_boltage = 0.0;

float LIPO_WARNING_VOLTAGE=0;


char MOTOR_BATTERY_TYPE[4][8]={"LiPo","LiFe","PbSO4","UNKNOWN"};
uint8_t MOTOR_BATTERY_TYPE_NUM=3;


void init_adc(void){
	memset(adc1_Buffer, 0, sizeof(adc1_Buffer));
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *) adc1_Buffer, (uint32_t)ADC_BUFFER_LENGTH);

	while(LiPo_boltage==0);
	if(LiPo_boltage>21){
		/*
		 * LiPo 6cell 3.7V/cell
		 * Don't use under 3.4V/cell
		 */
		LIPO_WARNING_VOLTAGE=21;
		MOTOR_BATTERY_TYPE_NUM=0;//LiPo
	}
	else if(LiPo_boltage>18){
		/*
		 * LiFe 6cell under 3.3V/cell
		 * Don't use under 3V/cell
		 */
		LIPO_WARNING_VOLTAGE=18;
		MOTOR_BATTERY_TYPE_NUM=1;//LiFe
	}
	else if(LiPo_boltage>10){
		/*
		 * Pb under 11V ---too low?
		 * Default 12V
		 */
		LIPO_WARNING_VOLTAGE=11;
		MOTOR_BATTERY_TYPE_NUM=2;//PbS4
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,5);
		for(uint8_t i;i<=10;i++){
			HAL_GPIO_TogglePin(FET_RED_GPIO_Port, FET_RED_Pin);
			HAL_Delay(500);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);	}

	return;
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
		serial.string("[UART_LOSS] Count:");
		serial.putint(UART_LOSS);
		serial.string("\n\r");


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
