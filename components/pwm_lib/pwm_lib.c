/* pwm_lib.c */
#include <stdio.h>
#include "driver/mcpwm.h"
#include "pwm_lib.h"

mcpwm_unit_t mcpwm_num;
mcpwm_timer_t timer_num;
bool pwm_init = false;

/**
 * @brief Configura el puerto y la frecuencia de la señal PWM.
 * 
 * @param mcpwm_num new_mcpwm_num : Unidad MCPWM empleada. 
 * @param timer_num new_timer_num : Timer usado como referencia.
 * @param mcpwm_io_signals_t io_signal : Señal de salida de unidad MCPWM.
 * @param int gpio_num: Puerto de salida de la señal PWM.
 * 
 * @par Returns
 *    Nothing.
 */
void pwm_setup(mcpwm_unit_t new_mcpwm_num,  mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int gpio_num)
{   
    mcpwm_num = new_mcpwm_num;
    timer_num = new_timer_num;
    mcpwm_gpio_init(mcpwm_num, io_signal, gpio_num);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50000;
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_init = (mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config) == 0) ? true : false;
    change_pwm_duty(0);
}

/**
 * @brief Convertidor de ancho de pulso de PWM, 
 * invierte el valor ingresado debido al funcionamiento del circuito de potencia.
 * 
 * @param float pwm_duty : ancho de pulso deseado.
 * 
 * @return 
 *  - new_pwm_duty : ancho de pulso convertido.
 */
float convert_pwm_duty(float pwm_duty)
{
    float new_pwm_duty = 0;
    new_pwm_duty = 100 - pwm_duty;
    if(new_pwm_duty < 0){
        new_pwm_duty = 0;
    }else if (new_pwm_duty > 100){
        new_pwm_duty = 100;
    }
    return new_pwm_duty;
}

/**
 * @brief Cambia el ancho de pulso de la señal PWM.
 * 
 * @param float duty_cycle : Ancho de pulso.
 * 
 * @par Returns
 *    Nothing.
 */
void change_pwm_duty(float duty_cycle)
{
    if(pwm_init){
        duty_cycle = convert_pwm_duty(duty_cycle);
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else{
        printf("ERROR: Configure PWM before setting duty cycle\n");
    }
}
