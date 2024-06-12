#include <DC_motor.h>

float DegreeSec2RPM(float speed_degsec){
	float speed_rpm = speed_degsec * 60/360;
	return speed_rpm;
}

float Voltage2Duty(float u){

	float duty = 100*u/V_MAX;

	if(duty>100){
		duty=100;
	} else if(duty<0){
		duty = 0;
	}

	return duty;
}

uint8_t Ref2Direction(float y_ref){
	uint8_t dir;

	if(y_ref>=0){
		dir = 0;
	} else {
		dir = 1;
	}
	return dir;
}

void set_PWM_and_dir(uint32_t duty, uint8_t dir){

	TIM10->CCR1 = ((float)duty/100)*TIM10->ARR;

	if( dir == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}else if ( dir == 1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	}
}
