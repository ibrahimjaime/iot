#include "driver/mcpwm.h"

void pwm_setup(mcpwm_unit_t new_mcpwm_num,  mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int gpio_num);
static float convert_pwm_duty(float pwm_duty);
void change_pwm_duty(float duty_cycle);