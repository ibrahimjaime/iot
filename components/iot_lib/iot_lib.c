/* iot_lib.c */
#include <stdio.h>
#include <string.h>
#include "iot_lib.h"
#include "driver/mcpwm.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "pwm_lib.h"
#include "nvs_flash.h"
#include "nvs.h"

#define MAX_RETRY 10
#define MQTT_BROKER_URI "mqtt://192.168.1.6:1883"
#define EXAMPLE_ESP_WIFI_SSID "millokira"//"fabriwifi"//"Utn_WifiPass"
#define EXAMPLE_ESP_WIFI_PASS "rocki2021"//"iotproject"//"WifiPass**"
#define MQTT_PUB_TEMP_LUX "iot/temp_lux"
#define MQTT_SUB_LIGHT_0 "iot/light0"
#define MQTT_SUB_LIGHT_1 "iot/light1"
#define MQTT_SUB_LIGHT_2 "iot/light2"
#define MQTT_SUB_LIGHT_3 "iot/light3"
#define MQTT_SUB_PWM_0 "iot/pwm0"
#define MQTT_TOPYC_LEN 3
#define LIGHT_GPIO 19
#define LIGHT_GPIO_1 18
#define LIGHT_GPIO_2 5
#define LIGHT_GPIO_3 17

static int retry_cnt = 0;
float pwm_duty = 0;
uint32_t MQTT_CONNECTED = 0;
int8_t light0_status;
int8_t light1_status;
int8_t light2_status;
int8_t light3_status;
esp_mqtt_client_handle_t client = NULL;

/**
 * @brief Rutinas para responder ante eventos de WiFi.
 * 
 * @return 
 *  - Código de error.
 */
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        printf("Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        printf("Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        printf("got ip: starting MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        printf("disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
            printf("Max Retry Failed: Wi-Fi Connection\n");
        break;

    default:
        break;
    }
    return;
}

/**
 * @brief Inicialización de WiFi
 *
 * @par Returns
 *    Nothing.
 */
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

/**
 * @brief Rutinas para responder ante eventos de MQTT.
 *
 *  Esta función es llamanda por los eventos que produce el cliente MQTT.
 *
 * @param handler_args : Datos del usuario registrados al evento.
 * @param base : Base de eventos que identifica el evento.
 * @param event_id : id del evento recivido.
 * @param event_data : Datos del evento.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    printf("Event dispatched from event loop base=%s, event_id=%d\n", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    int sub_topic_len = 0;
    float new_pwm_duty;
    char topic_received[6];
    esp_err_t nvs_err;
    nvs_handle_t storage_handler;

    switch ((esp_mqtt_event_id_t)event_id){
        case MQTT_EVENT_CONNECTED:
            printf("MQTT_EVENT_CONNECTED");
            MQTT_CONNECTED = 1;

            msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_0, 0);
            printf("sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_1, 0);
            printf("sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_2, 0);
            printf("sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_LIGHT_3, 0);
            printf("sent subscribe successful, msg_id=%d", msg_id);
            
            msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_PWM_0, 0);
            printf("sent subscribe successful, msg_id=%d", msg_id);
            break;

        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT_EVENT_DISCONNECTED");
            MQTT_CONNECTED = 0;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            printf("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            printf("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            printf("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            printf("MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            printf("TOPIC LEN=%d\r\n", event->topic_len);

            if(event->topic_len > strlen(MQTT_SUB_LIGHT_0)){
                printf("ERROR unexpected topic_len: %d", event->topic_len);
                break;
            }
            sub_topic_len = event->topic_len - (MQTT_TOPYC_LEN + 1);
            strncpy(topic_received, event->topic+(MQTT_TOPYC_LEN + 1), sub_topic_len);
            printf("topic_received %s\n", topic_received);
            printf("topic_received len %d\n", strlen(topic_received));
            printf("sub_topic_len len %d\n", sub_topic_len);
            if(0 == strcmp("light0", topic_received)){
                nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
                if (nvs_err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                }
                if(0 == strcmp("true", event->data)){
                    gpio_set_level(LIGHT_GPIO, 1);
                    light0_status = 1;
                }else{
                    gpio_set_level(LIGHT_GPIO, 0);
                    light0_status = 0;
                }
                nvs_err = nvs_set_i8(storage_handler, "light0", light0_status);
                printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                nvs_err = nvs_commit(storage_handler);
                printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
            }
            else if(0 == strcmp("light1", topic_received)){
                nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
                if (nvs_err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                }
                if(0 == strcmp("true", event->data)){
                    gpio_set_level(LIGHT_GPIO_1, 1);
                    light1_status = 1;
                }else{
                    gpio_set_level(LIGHT_GPIO_1, 0);
                    light1_status = 0;
                }
                nvs_err = nvs_set_i8(storage_handler, "light1", light1_status);
                printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                nvs_err = nvs_commit(storage_handler);
                printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
            }
            else if(0 == strcmp("light2", topic_received)){
                nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
                if (nvs_err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                }
                if(0 == strcmp("true", event->data)){
                    gpio_set_level(LIGHT_GPIO_2, 1);
                    light2_status = 1;
                }else{
                    gpio_set_level(LIGHT_GPIO_2, 0);
                    light2_status = 0;              
                }
                nvs_err = nvs_set_i8(storage_handler, "light1", light1_status);
                printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                nvs_err = nvs_commit(storage_handler);
                printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
            }
            else if(0 == strcmp("light3", topic_received)){
                nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
                if (nvs_err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                }
                if(0 == strcmp("true", event->data)){
                    gpio_set_level(LIGHT_GPIO_3, 1);
                    light3_status = 1;                 
                }else{
                    gpio_set_level(LIGHT_GPIO_3, 0);
                    light3_status = 0;
                }
                nvs_err = nvs_set_i8(storage_handler, "light3", light1_status);
                printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                nvs_err = nvs_commit(storage_handler);
                printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
            }
            else if(0 == strcmp("pwm0", topic_received)){
                nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
                if (nvs_err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                }
                new_pwm_duty = atof(event->data);
                if(new_pwm_duty != pwm_duty)
                {
                    change_pwm_duty(new_pwm_duty);
                    pwm_duty = new_pwm_duty;
                    nvs_err = nvs_set_i8(storage_handler, "pwm0", (int8_t) new_pwm_duty);
                    printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                    nvs_err = nvs_commit(storage_handler);
                    printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
                }
            }
            else{
                printf("Topic not mached!\n");           
            }
            nvs_close(storage_handler);
            memset(event->data,0, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            printf("MQTT_EVENT_ERROR");
            break;
        default:
            printf("Other event id:%d", event->event_id);
            break;
    }
}

/**
 * @brief Inizialización de MQTT.
 *  Configura e inicializa el cliente y define la función de respuestas a eventos.
 * @par Returns
 *    Nothing.
 */
static void mqtt_app_start(void)
{
    printf("STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .uri = MQTT_BROKER_URI};

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void mqtt_publish(const char *data)
{
    if (MQTT_CONNECTED){
        printf("MQTT_PUB:%s\n", data);
        esp_mqtt_client_publish(client, MQTT_PUB_TEMP_LUX, data, 0, 0, 0);
    }
}

/**
 * @brief Inicializa puertos digitales.
 * 
 * @par Parameters
 *    None.
 * 
 * @par Returns
 *    Nothing.
 */
void iot_gpio_init(void)
{
    esp_err_t nvs_err;
    nvs_handle_t storage_handler;
    nvs_err = nvs_open("storage", NVS_READWRITE, &storage_handler);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    nvs_err = nvs_get_i8(storage_handler, "light0", &light0_status);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    nvs_err = nvs_get_i8(storage_handler, "light1", &light1_status);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    nvs_err = nvs_get_i8(storage_handler, "light2", &light2_status);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    nvs_err = nvs_get_i8(storage_handler, "light3", &light3_status);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    nvs_close(storage_handler);

    gpio_reset_pin(LIGHT_GPIO);
    gpio_set_direction(LIGHT_GPIO, GPIO_MODE_OUTPUT);
    if((light0_status == 1) || (light0_status == 0)){
        gpio_set_level(LIGHT_GPIO, light0_status);
    }
    else {
        gpio_set_level(LIGHT_GPIO, 0);
    }
    gpio_reset_pin(LIGHT_GPIO_1);
    gpio_set_direction(LIGHT_GPIO_1, GPIO_MODE_OUTPUT);
    if((light1_status == 1) || (light1_status == 0)){
        gpio_set_level(LIGHT_GPIO_1, light1_status);
    }
    else {
        gpio_set_level(LIGHT_GPIO_1, 0);
    }
    gpio_reset_pin(LIGHT_GPIO_2);
    gpio_set_direction(LIGHT_GPIO_2, GPIO_MODE_OUTPUT);
    if((light2_status == 1) || (light2_status == 0)){
        gpio_set_level(LIGHT_GPIO_2, light2_status);
    }
    else {
        gpio_set_level(LIGHT_GPIO_2, 0);
    }
    gpio_reset_pin(LIGHT_GPIO_3);
    gpio_set_direction(LIGHT_GPIO_3, GPIO_MODE_OUTPUT);
    if((light3_status == 1) || (light3_status == 0)){
        gpio_set_level(LIGHT_GPIO_3, light3_status);
    }
    else {
        gpio_set_level(LIGHT_GPIO_3, 0);
    }
}