#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
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
#include "math.h"
#include "driver/mcpwm.h"
#include "pwm_lib.h"
#include "iot_lib.h"
#include "filters.h"

#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define GPIO_PWM0A_OUT 15
#define MQTT_PUB_TEMP_LUX "iot/temp_lux"

static QueueHandle_t uart0_queue;
float LM35_temp = 0;
float lux = 0;
xSemaphoreHandle temp_key = NULL;
xSemaphoreHandle light_key = NULL;

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

	while(1) {
        if(light_key != NULL){
            if(xSemaphoreTake(light_key, pdMS_TO_TICKS(100))){
                read_light = lux;
                xSemaphoreGive(light_key);
            }
        }
        if(temp_key != NULL){
            if(xSemaphoreTake(temp_key, pdMS_TO_TICKS(100))){
                read_temp = LM35_temp;
                xSemaphoreGive(temp_key);
            }
        }
        sprintf(pub_temp_lux, "{\"temp\": %.2f, \"lux\": %.2f}\n ", read_temp, read_light);
        mqtt_publish(pub_temp_lux, MQTT_PUB_TEMP_LUX);
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
    struct MovAvrS luxes; //Crea la estructura para el calculo del filtro
    struct MovAvrS *luxespoint = &luxes; //Crea un puntero a la estructura
    MovingAvaregeInit(luxespoint,10); //Inicializa la estructura
    float luxesdata[luxes.N]; //Crea el arreglo de datos con el que se va a trabajar en el filtro
    for(int j=0;j<luxes.N;j++){ //Inicializa el arreglo de datos en 0
        luxesdata[j]=0;
    }
    float LDR = 0; //LDR Resistencia
    float lux_raw = 0; //Valor de luminocidad antes del filtro
    const int Rref = 9750; //Resistencia de referencia 9.75KΩ
    float Vref = 0;
    const float GAMMA  = 0.7;
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    TickType_t xLastWakeTime1 = xTaskGetTickCount();
    while (1) 
    {
        Vref = ((adc1_get_raw(ADC1_CHANNEL_4)*3.3)/4095);
        LDR = (3.3-Vref)/(Vref/Rref);
        lux_raw = pow(50000*pow(10,GAMMA)/LDR,(1/GAMMA));
        MovingAvarageFilter(luxespoint,luxesdata,lux_raw); //Aplicacion del Filtro de media movil
        if(light_key != NULL){
            if(xSemaphoreTake(light_key, pdMS_TO_TICKS(100))){
                lux = luxes.MovAvrResult;
                xSemaphoreGive(light_key);
            }
        }
        vTaskDelayUntil(&xLastWakeTime1, 1000/ portTICK_PERIOD_MS); //Para forzar un tiempo absoluto entre mediciones
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
    struct MovAvrS temp; //Crea la estructura para el calculo del filtro
    struct MovAvrS *temppoint = &temp; //Crea un puntero a la estructura
    MovingAvaregeInit(temppoint,10); //Inicializa la estructura
    float tempdata[temp.N]; //Crea el arreglo de datos con el que se va a trabajar en el filtro
    for(int i=0;i<temp.N;i++){ //Inicializa el arreglo de datos en 0
        tempdata[i]=0;
    }
    float LM35_volt = 0; //Lectura del ADC en volts
    float LM35_raw = 0; //Temperatura del sensor antes del filtro
    float LM35_Offset = 10; //Off set
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    TickType_t xLastWakeTime2 = xTaskGetTickCount();
    while (1) 
    {
        LM35_volt = ((adc1_get_raw(ADC1_CHANNEL_5)*3.3)/4095);
        LM35_raw = (LM35_volt/0.01)+LM35_Offset;
        MovingAvarageFilter(temppoint,tempdata,LM35_raw); //Aplicacion del Filtro de media movil
        if(temp_key != NULL){
            if(xSemaphoreTake(temp_key, pdMS_TO_TICKS(100))){ //Si no está tomado el semaforo cambia la variable
                LM35_temp = temp.MovAvrResult;
                xSemaphoreGive(temp_key);
            }
        }
        vTaskDelayUntil(&xLastWakeTime2, 1000/ portTICK_PERIOD_MS); //Para forzar un tiempo absoluto entre mediciones
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
    char *topics[4]={'\0'};
    topics[0]="iot/light0";
    topics[1]="iot/light1";
    topics[2]="iot/light2";
    topics[3]="iot/light3";
    char storage_nsp[] = "storage";
    char pwm_topic[] = "iot/pwm0";
    int pwm_freq = 50000;
    int ports[] = {19, 18, 5, 17};
    int out_num = 4;
    const char *ssid = "brinet715";//"millokira"//"fabriwifi"//"Utn_WifiPass"
    const char *pass = "03829f50";//"rocki2021"//"iotproject"//"WifiPass**"
    const char *uri = "mqtt://192.168.120.32:1883";//"mqtt://192.168.1.6:1883"

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    
    nvs_flash_init();
    
    iot_dgt_setup(topics, storage_nsp, ports, out_num);
    iot_pwm_setup(pwm_topic, storage_nsp, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM_TIMER_0, MCPWM0A, GPIO_PWM0A_OUT, pwm_freq);
    iot_init(ssid, pass, uri);

    temp_key = xSemaphoreCreateMutex();
    light_key = xSemaphoreCreateMutex();
    xTaskCreate(LDR_reader, "LDR_reader", 4096, NULL, 5, NULL);
    xTaskCreate(LM35_reader, "LM35_reader", 4096, NULL, 5, NULL);
	xTaskCreate(&publisher_task, "publisher_task", 2048, NULL, 5, NULL );
}