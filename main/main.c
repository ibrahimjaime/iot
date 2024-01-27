#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "math.h"
#include "driver/mcpwm.h"
#include "pwm_lib.h"
#include "mqtt_lib.h"

#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define LIGHT_GPIO 19
#define LIGHT_GPIO_1 18
#define LIGHT_GPIO_2 5
#define LIGHT_GPIO_3 17
#define GPIO_PWM0A_OUT 15

static QueueHandle_t uart0_queue;
float LM35_temp = 0;
float lux = 0;
float pwm_duty = 0;
xSemaphoreHandle temp_key = NULL;
xSemaphoreHandle light_key = NULL;

/**
 * @brief Inicializa puertos digitales utilizados.
 * 
 * @par Parameters
 *    None.
 * 
 * @par Returns
 *    Nothing.
 */
void iot_gpio_init(void)
{
    gpio_reset_pin(LIGHT_GPIO);
    gpio_set_direction(LIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LIGHT_GPIO, 0);
    gpio_reset_pin(LIGHT_GPIO_1);
    gpio_set_direction(LIGHT_GPIO_1, GPIO_MODE_OUTPUT);
    gpio_set_level(LIGHT_GPIO_1, 0);
    gpio_reset_pin(LIGHT_GPIO_2);
    gpio_set_direction(LIGHT_GPIO_2, GPIO_MODE_OUTPUT);
    gpio_set_level(LIGHT_GPIO_2, 0);
    gpio_reset_pin(LIGHT_GPIO_3);
    gpio_set_direction(LIGHT_GPIO_3, GPIO_MODE_OUTPUT);
    gpio_set_level(LIGHT_GPIO_3, 0);
}

/**
 * @brief Tarea encargada de publicar los datos por MQTT.
 * 
 * @par Returns
 *    Nothing.
 */
void publisher_task(void *pvParameter)
{
    char pub_temp_lux[100];
    float read_temp = 0;
    float read_light = 0;
    bool read_new_temp = false;
    bool read_new_light = false;

	while(1) {
        if(light_key != NULL){
            if(xSemaphoreTake(light_key, pdMS_TO_TICKS(100))){
                read_light = lux;
                xSemaphoreGive(light_key);
                read_new_light = true;
            }
        }
        if(temp_key != NULL){
            if(xSemaphoreTake(temp_key, pdMS_TO_TICKS(100))){
                read_temp = LM35_temp;
                xSemaphoreGive(temp_key);
                read_new_temp = true;
            }
        }

        if((read_new_temp == true) && (read_new_light == true)){
            sprintf(pub_temp_lux, "{\"temp\": %.2f, \"lux\": %.2f}\n ", read_temp, read_light);
            mqtt_publish(pub_temp_lux);
            read_new_temp = false;
            read_new_light = false;
        }
        vTaskDelay(2000 / portTICK_RATE_MS);
	}

}

/**
 * @brief Tarea encargada de leer el puerto analógico 
 * y procesar los datos del LDR para obtener un valor en lux.
 * 
 * @par Returns
 *    Nothing.
 */
static void LDR_reader(void *arg)
{
    float LDR = 0; //LDR Resistencia
    const int Rref = 9750; //Resistencia de referencia 9.75KΩ
    float Vref = 0;
    const float GAMMA  = 0.7;

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    while (1) 
    {
        if(light_key != NULL){
            if(xSemaphoreTake(light_key, pdMS_TO_TICKS(100))){
                Vref = ((adc1_get_raw(ADC1_CHANNEL_4)*3.3)/4095);
                LDR = (3.3-Vref)/(Vref/Rref);
                lux = pow(50000*pow(10,GAMMA)/LDR,(1/GAMMA));
                xSemaphoreGive(light_key);
            }
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarea encargada de leer el puerto analógico 
 * y procesar los datos del sensor LM35 para obtener un valor de temperatura.
 * 
 * @par Returns
 *    Nothing.
 */
static void LM35_reader(void *arg)
{   
    float LM35_volt;
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    while (1) 
    {
        if(temp_key != NULL){
            if(xSemaphoreTake(temp_key, pdMS_TO_TICKS(100))){
                LM35_volt = ((adc1_get_raw(ADC1_CHANNEL_5)*3.3)/4095)+0.05;
                LM35_temp = (LM35_volt/0.01)+4.7;
                xSemaphoreGive(temp_key);
            }
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

/**
 * @brief Tarea encargada establecer el valor 
 * de ancho de pulso de PWM recibido por MQTT en la salida del microcontrolador.
 * 
 * @par Returns
 *    Nothing.
 */
static void set_pwm(void *arg)
{
    pwm_setup(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, GPIO_PWM0A_OUT);
    while(1){
        change_pwm_duty(pwm_duty);
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

}

/**
 * @brief Main
 * 
 * @par Returns
 *    Nothing.
 */
void app_main()
{   
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("*", "UART test");
    iot_gpio_init();
	nvs_flash_init();
    wifi_init();
    temp_key = xSemaphoreCreateMutex();
    light_key = xSemaphoreCreateMutex();
    xTaskCreate(LDR_reader, "LDR_reader", 4096, NULL, 5, NULL);
    xTaskCreate(LM35_reader, "LM35_reader", 4096, NULL, 5, NULL);
    xTaskCreate(set_pwm, "set_pwm", 4096, NULL, 5, NULL);
	xTaskCreate(&publisher_task, "publisher_task", 2048, NULL, 5, NULL );
}