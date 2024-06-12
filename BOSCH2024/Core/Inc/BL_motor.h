/*
 * BL_motor.h
 *
 *  Created on: Jan 14, 2024
 *      Author: angel
 */

#ifndef INC_BL_MOTOR_H_
#define INC_BL_MOTOR_H_


#include <PID.h>
#include <main.h>
#include "math.h"


#define GIRI_MIN_CONV 0.0006283185// old0.000503271

//ENCODER
#define ENCODER_PPR 2048
#define GEARBOX_RATIO 1
#define ENCODER_COUNTING_MODE 4

float BL_DegreeSec2RPM(float);
void BL_set_PWM(float);


#endif /* INC_BL_MOTOR_H_ */
