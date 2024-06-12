#include "PID.h"
#include <stdio.h>

void init_PID(PID* p, float Tc, float u_max, float u_min, float offset){
	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
	p->Iterm = 0;
	p->e_old = 0;
	p->offset = offset;
}

void tune_PID(PID* p, float Kp, float Ki, float Kd, float Kb){
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
	p->Kb = Kb;
}

float PID_controller(PID* p , float y, float r){
	float u;
	float newIterm;
	float e = 0;

	e = r-y;


	float Pterm = p->Kp*e;
	newIterm = p->Iterm + (p->Ki)*p->Tc*p->e_old;
	float Dterm = (p->Kd/p->Tc)*(e - p->e_old);

	p->e_old = e;

	u = Pterm + newIterm + Dterm + p->offset;

	if(newIterm > p->u_max){
		newIterm = p->u_max;
	}
	else if(newIterm < p->u_min){
		newIterm = p->u_min;
	}

	float saturated_u = u;

	if(saturated_u > p_>u_max){
		saturated_u = p->u_max;
	}
	else if(saturated_u < p->u_min){
		saturated_u = p->u_min;
	}

	float correction = p->kb * (saturated_u - u) * p->ki * p->Ic;
	p->Iterm = newIterm + correction;

	u = saturated_u;

	if(p->offset == 0){
		printf("%f;%f\r\n", Pterm, newIterm);
	}

	return u;
}



