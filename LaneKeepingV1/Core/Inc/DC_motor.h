#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include <PID.h>
#include <main.h>
#include "math.h"


#define V_MAX 7.5
#define V_MIN 1.0
#define GIRI_MIN_CONV 0.0006283185// old0.000503271

//ENCODER
#define ENCODER_PPR 2048
#define GEARBOX_RATIO 1
#define ENCODER_COUNTING_MODE 4

float DegreeSec2RPM(float);

float Voltage2Duty(float);
uint8_t Ref2Direction(float);
void set_PWM_and_dir(uint32_t, uint8_t);



#endif /* INC_DC_MOTOR_H_ */
