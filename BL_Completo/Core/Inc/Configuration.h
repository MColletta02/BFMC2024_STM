/*
 * Configuration.h
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "math.h"

//PRESS USER BUTTON
#define SHORT_PRESS_THRESHOLD  100 //Sono multipli di 10ms
#define LONG_PRESS_THRESHOLD  200 //Sono multipli di 10ms

//PWM
#define MIN_PWM 0.01
#define MAX_PWM 0.99
#define NEUTRAL_PWM 0.074568

//TRACTION PID
#define TRACTION_SAMPLING_TIME 0.01 //[s]
#define MAX_U_TRACTION MAX_PWM // Valore di duty [% del periodo]
#define MIN_U_TRACTION MIN_PWM // Valore di duty [% del periodo]

#define KP_TRACTION 0.0000011//0.00006 //0.0001
#define KI_TRACTION 0.001 //0.001 //0.003
#define TEMPO_SALITA_PID_TRAZIONE 0.2 //0.3 //[s]

#define KP_TRACTION_RWD 0.00000007 //0.00000007
#define KI_TRACTION_RWD 0.00025 //0.00025
#define TEMPO_SALITA_PID_TRAZIONE_RWD 0.2 //[s]

#define KP_TRACTION_DESC 0.00000004
#define KI_TRACTION_DESC 0.0019 //0.0008

//STEERING PID
#define STEERING_SAMPLING_TIME 0.01 //[s]
#define MAX_U_STEERING 36 // [°]
#define MIN_U_STEERING -36 //[°]
#define KP_STEERING 20 //20 //250
#define KI_STEERING 250 //250 //55

//#define KP_STEERING_STR 20 //20
//#define KI_STEERING_STR 250 //250

//ENCODER SAMPLING TIME
#define ENCODER_SAMPLING_TIME 0.01 //[s]

//VEHICLE PARAMETERS for RPM<->m/s
#define WHEEL_RADIUS 0.0325 //m
#define MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION 2.58
#define DIFFERENTIAL_GEARBOX_RATIO 3.0909090909
//float RPM_2_m_s = ((2 * M_PI / 60) * WHEEL_RADIUS / MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION) / DIFFERENTIAL_GEARBOX_RATIO;

#define MAX_CURVATURE_RADIUS_FOR_STRAIGHT 10

//CORREZIONI MANOVRE
#define CORREZIONE_LAMBDA 1.08 //Correzione applicata sulla lambda nell'intervallo che la richiede


#endif /* INC_CONFIGURATION_H_ */
