#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>
#include <stdio.h>


typedef struct PID{
	float Kp; //guadagno proporzionale
	float Ki; //guadagno integrale
	float Kd; //guadagno derivativo

	float Tc; //periodo
	float u_max; //limite superiore
	float u_min; //limite inferiore

	float e_old;
	float Iterm;
}PID;

void init_PID(PID*, float, float, float);
void tune_PID(PID*,float,float,float);
float PID_controller(PID*,float,float, float);



#endif /* INC_PID_H_ */
