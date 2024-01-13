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

#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static const char *TAG = "MQTT_EXAMPLE";
static QueueHandle_t uart0_queue;

#define EXAMPLE_ESP_WIFI_SSID "millokira"//"fabriwifi"//"Utn_WifiPass"
#define EXAMPLE_ESP_WIFI_PASS "rocki2021"//"iotproject"//"WifiPass**"
#define MAX_RETRY 10
static int retry_cnt = 0;

float LM35_temp = 0;
float lux = 0;
#define MQTT_PUB_TEMP_LUX "iot/temp_lux"
#define MQTT_SUB_LIGHT_0 "iot/light0"
#define MQTT_SUB_LIGHT_1 "iot/light1"
#define MQTT_SUB_LIGHT_2 "iot/light2"
#define MQTT_SUB_LIGHT_3 "iot/light3"
#define MQTT_TOPYC_LEN 3
#define MQTT_SUB_TOPYC_LEN 6
#define LIGHT_GPIO 19
#define LIGHT_GPIO_1 18
#define LIGHT_GPIO_2 5
#define LIGHT_GPIO_3 17

uint32_t MQTT_CONNECTED = 0;

xSemaphoreHandle temp_key = NULL;
xSemaphoreHandle light_key = NULL;

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


static void mqtt_app_start(void);

static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
            ESP_LOGI(TAG, "Max Retry Failed: Wi-Fi Connection\n");
        break;

    default:
        break;
    }
    return ESP_OK;
}

void wifi_init(void)
{
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    //new
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    char topic_received[6];

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNECTED = 1;

        msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_0, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_1, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_2, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_3, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED = 0;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        strncpy(topic_received, event->topic+(MQTT_TOPYC_LEN + 1), MQTT_SUB_TOPYC_LEN);

        if(0 == strcmp("light0", topic_received)){
            if(0 == strcmp("true", event->data)){
                gpio_set_level(LIGHT_GPIO, 1);
            }else{
                gpio_set_level(LIGHT_GPIO, 0);
            }
        }
        else if(0 == strcmp("light1", topic_received)){
            if(0 == strcmp("true", event->data)){
                gpio_set_level(LIGHT_GPIO_1, 1);
            }else{
                gpio_set_level(LIGHT_GPIO_1, 0);
            }
        }
        else if(0 == strcmp("light2", topic_received)){
            if(0 == strcmp("true", event->data)){
                gpio_set_level(LIGHT_GPIO_2, 1);
            }else{
                gpio_set_level(LIGHT_GPIO_2, 0);
            }
        }
        else if(0 == strcmp("light3", topic_received)){
            if(0 == strcmp("true", event->data)){
                gpio_set_level(LIGHT_GPIO_3, 1);
            }else{
                gpio_set_level(LIGHT_GPIO_3, 0);
            }
        }
        else{
            printf("Topic not mached!\n");           
        }
        memset(event->data,0, event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_mqtt_client_handle_t client = NULL;
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://192.168.1.5:1883"};////192.168.137.66:1883

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

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
        sprintf(pub_temp_lux, "{\"temp\": %.2f, \"lux\": %.2f}\n ", LM35_temp, lux);

        if (MQTT_CONNECTED){
            printf("MQTT_PUB:%s\n", pub_temp_lux);
			esp_mqtt_client_publish(client, MQTT_PUB_TEMP_LUX, pub_temp_lux, 0, 0, 0);
        }
        vTaskDelay(2000 / portTICK_RATE_MS);
	}

}

/**
 * @brief ADC
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
 * @brief ADC
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
    printf("Testing mqtt...\n");
    iot_gpio_init();
	nvs_flash_init();
    wifi_init();
    temp_key = xSemaphoreCreateMutex();
    light_key = xSemaphoreCreateMutex();
    xTaskCreate(LDR_reader, "LDR_reader", 4096, NULL, 5, NULL);
    xTaskCreate(LM35_reader, "LM35_reader", 4096, NULL, 5, NULL);
	xTaskCreate(&publisher_task, "publisher_task", 2048, NULL, 5, NULL );
}