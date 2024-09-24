#include "motor.h"

void set_pwm(TIM_TypeDef* timer, double duty)
{
	timer->CCR1 = (uint32_t)((timer->ARR + 1) * duty);
	//TIM14->EGR = TIM_EGR_UG;		
}

void run_motor_pwm(DC_MOTOR* motor, double duty)
{
	double base_duty = 0;
	
	set_pwm(motor->timer, duty + base_duty);
}

void init_dc_motor(DC_MOTOR* motor)
{
	switch(motor->motor_index)
	{
		case MOTOR_1:
			HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
			break;
		default:
			break;
	}
}