#include "driver/mcpwm.h"

void pwm_setup(mcpwm_unit_t new_mcpwm_num,  mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int freq, int gpio_num, const char * storage_name, const char * storage_key);
static int8_t convert_pwm_duty(int8_t pwm_duty);
void change_pwm_duty(int8_t duty_cycle);