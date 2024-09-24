#include "main.h"

void set_pwm(TIM_TypeDef* timer, double duty);
void run_motor_pwm(DC_MOTOR* motor, double duty);
void init_dc_motor(DC_MOTOR* motor);