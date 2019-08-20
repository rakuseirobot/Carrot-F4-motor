/*
 * motor_control.hpp
 *
 *  Created on: 2019/01/14
 *      Author: shun2
 */

#ifndef SOURCE_MOTOR_CONTROL_HPP_
#define SOURCE_MOTOR_CONTROL_HPP_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define RAD2DEG(x)		(float)((x) * 180 / M_PI)
#define DEG2DEG(x)		(float)((x) * M_PI / 180)


#define X_Encoder_COUNT TIM2->CNT
#define Y_Encoder_COUNT TIM3->CNT




#ifdef __cplusplus

typedef enum{
	X_axis,
	Y_axis
}ch_t;

enum{
	motor_x_data=1,
	motor_y_data=2,
	motor_turnspeed_data=3,
	motor_bb_data=4
};

typedef struct{
	float vx;
	float vy;
	float theta;
}motor_target_t;

typedef struct{
	float X;
	float Y;
	/*float xdiff[2]={};
	float ydiff[2]={};
	float xsum=0;
	float ysum=0;*/
}motor_var_t;

typedef struct{
	float now=0;
	float omega;
	float odiff[2]={};
	float osum=0;
}angle_var_t;


typedef enum{
	ROLL_RIGHT,
	ROLL_LEFT
}roll_angle_t;

typedef enum{
	MOTORCH_1 = TIM_CHANNEL_1,
	MOTORCH_2 = TIM_CHANNEL_2,
	MOTORCH_3 = TIM_CHANNEL_3,
	MOTORCH_4 = TIM_CHANNEL_4,
}motorch_t;

namespace motor{
	void init(void);
	void update_pwm(void);
	void update_target(void);
	void move_angle(float,int16_t);
	void brake(void);
}
int32_t Get_Encoder(ch_t x);
void start_encoder();
void stop_encoder();

#ifdef __cplusplus
extern "C" {
#endif

void motor_task(void *argument);

void raspi_uart_func(void);

void update_encoder(void);

void encoder_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif

#endif /* SOURCE_MOTOR_CONTROL_HPP_ */
