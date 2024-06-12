/*
 * BL_motor.c
 *
 *  Created on: Jan 14, 2024
 *      Author: angel
 */


#include <BL_motor.h>
#include "Configuration.h"

float BL_DegreeSec2RPM(float speed_degsec){
	float speed_rpm = speed_degsec * 60/360;
	return speed_rpm;
}

void BL_set_PWM(float duty){

	TIM10->CCR1 = duty*TIM10->ARR;
}
