#include "PID.h"
#include "Configuration.h"

#include <stdio.h>


void init_PID(PID* p, float Tc, float u_max, float u_min){
	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
	p->Iterm = 0;
	p->e_old = 0;
}

void tune_PID(PID*p, float Kp, float Ki, float Kd){
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
}

float PID_controller(PID* p , float y, float r, float zero){
	float u;
	float newIterm;
	float e = 0;

	e = r-y;


	float Pterm = p->Kp*e;
	newIterm = p->Iterm + (p->Ki)*p->Tc*p->e_old;
	float Dterm = (p->Kd/p->Tc)*(e - p->e_old);

	p->e_old = e;

	u = Pterm + newIterm + Dterm + zero;
	if(u > p->u_max){
		u = p->u_max;
	} else if(u < p->u_min){
		u = p->u_min;
	} else {
		p->Iterm = newIterm;
	}

	//Entra qua dentro solo se è il PID di sterzo
	if(zero == 0){
		//printf("errore: %.2f, y: %.2f, r: %.2f, u: %.2f \r\n", e, y, r, u);
	}

	return u;
}



