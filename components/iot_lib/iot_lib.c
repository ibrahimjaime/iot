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
#define MQTT_BROKER_URI "mqtt://192.168.120.32:1883"//"mqtt://192.168.1.6:1883"
#define EXAMPLE_ESP_WIFI_SSID "brinet715"//"millokira"//"fabriwifi"//"Utn_WifiPass"
#define EXAMPLE_ESP_WIFI_PASS "03829f50"//"rocki2021"//"iotproject"//"WifiPass**"
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

#define MAX_TOPICS 10
#define MAX_TOPICS_LEN 15

char topics[MAX_TOPICS][MAX_TOPICS_LEN] = {{0}};
char pwm_topics[MAX_TOPICS][MAX_TOPICS_LEN] = {{0}};
int out_ports[MAX_TOPICS];
int topics_num = 0;
int pwm_topics_num = 0;
int8_t ports_status[MAX_TOPICS];
char nvs_namespace[MAX_TOPICS][MAX_TOPICS_LEN];
char pwm_nvs_namespace[MAX_TOPICS][MAX_TOPICS_LEN];

static int retry_cnt = 0;
uint32_t MQTT_CONNECTED = 0;
int8_t light0_status;
int8_t light1_status;
int8_t light2_status;
int8_t light3_status;
int8_t pwm_duty = 0;
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
    char topic_received[strlen(MQTT_SUB_LIGHT_0)];
    esp_err_t nvs_err;
    nvs_handle_t storage_handler;
    int8_t new_pwm_duty;

    switch ((esp_mqtt_event_id_t)event_id){
        case MQTT_EVENT_CONNECTED:
            printf("MQTT_EVENT_CONNECTED\n");
            MQTT_CONNECTED = 1;
            for (int i = 0; i < topics_num; i++){
                msg_id = esp_mqtt_client_subscribe(client, topics[i], 0);
                printf("topic: %s\n", topics[i]);
                printf("sent subscribe successful, msg_id=%d\n", msg_id);
            }
            for (int i = 0; i < pwm_topics_num; i++){
                msg_id = esp_mqtt_client_subscribe(client, pwm_topics[i], 0);
                printf("topic: %s\n", pwm_topics[i]);
                printf("sent subscribe successful, msg_id=%d\n", msg_id);
            }
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

            for (int i = 0; i < topics_num; i++){
                if(strstr(event->topic, topics[i]) != NULL){
                    if(0 == strcmp("true", event->data)){
                        gpio_set_level(out_ports[i], 1);
                        ports_status[i] = 1;
                    }else{
                        gpio_set_level(out_ports[i], 0);
                        ports_status[i] = 0;
                    }
                    nvs_err = nvs_open(nvs_namespace[i], NVS_READWRITE, &storage_handler);
                    if (nvs_err != ESP_OK) {
                        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                    }
                    nvs_err = nvs_set_i8(storage_handler, topics[i], ports_status[i]);
                    printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                    nvs_err = nvs_commit(storage_handler);
                    printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
                    nvs_close(storage_handler);
                    memset(event->data,0, event->data_len);
                    return;
                }
            }
            for (int i = 0; i < pwm_topics_num; i++){
                if(strstr(event->topic, pwm_topics[i]) != NULL){
                    new_pwm_duty = atoi(event->data);
                    if(new_pwm_duty != pwm_duty)
                    {
                        change_pwm_duty(new_pwm_duty);
                        pwm_duty = new_pwm_duty;
                        nvs_err = nvs_open(pwm_nvs_namespace[i], NVS_READWRITE, &storage_handler);
                        if (nvs_err != ESP_OK) {
                            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
                        }  
                        nvs_err = nvs_set_i8(storage_handler, pwm_topics[i], new_pwm_duty);
                        printf((nvs_err != ESP_OK) ? "NVS Set Failed!\n" : "NVS Set Done\n");
                        nvs_err = nvs_commit(storage_handler);
                        printf((nvs_err != ESP_OK) ? "NVS Commit Failed!\n" : "NVS Commit Done\n");
                        nvs_close(storage_handler);
                    }
                    memset(event->data,0, event->data_len);
                    return;
                }
            }
            memset(event->data,0, event->data_len);
            printf("Topic not mached!\n");
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
    printf("STARTING MQTT\n");
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

void iot_dgt_setup(char **new_topics, char * storage_name, int ports[],int out_num){
    topics_num += out_num;
    for (int i = 0; i < out_num; i++){
        strncpy(nvs_namespace[i], storage_name, strlen(storage_name)+1);
        strncpy(topics[i],new_topics[i], strlen(new_topics[i])+1);
        out_ports[i] = ports[i];
    }
    esp_err_t nvs_err;
    nvs_handle_t storage_handler;
    nvs_err = nvs_open(storage_name, NVS_READWRITE, &storage_handler);
    if (nvs_err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
    }
    
    for (int i = 0; i < (out_num); i++){
        nvs_err = nvs_get_i8(storage_handler, new_topics[i], &ports_status[i]);
        if (nvs_err != ESP_OK) {
            printf("Error (%s) getting NVS value!\n", esp_err_to_name(nvs_err));
        }
        gpio_reset_pin(out_ports[i]);
        gpio_set_direction(out_ports[i], GPIO_MODE_OUTPUT);
        if((ports_status[i] == 1) || (ports_status[i] == 0)){
            gpio_set_level(out_ports[i], (uint32_t) ports_status[i]);
        }
        else {
            gpio_set_level(out_ports[i], 0);
        }
    }
    nvs_close(storage_handler);
}

void iot_pwm_setup(char * new_pwm_topic, char * storage_name, mcpwm_unit_t new_mcpwm_num,  mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int gpio_num){
    pwm_topics_num += 1;
    strncpy(pwm_nvs_namespace[pwm_topics_num-1], storage_name, strlen(storage_name)+1);
    strncpy(pwm_topics[pwm_topics_num-1], new_pwm_topic, strlen(new_pwm_topic)+1);
    pwm_setup(new_mcpwm_num, new_timer_num, io_signal, gpio_num, storage_name, new_pwm_topic);
}