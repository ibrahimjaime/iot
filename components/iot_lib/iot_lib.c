/* iot_lib.c */
#include <stdio.h>
#include "iot_lib.h"
#include "driver/mcpwm.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"

#define MAX_RETRY 10
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
uint32_t MQTT_CONNECTED = 0;
esp_mqtt_client_handle_t client = NULL;

extern xSemaphoreHandle pwm_key;
extern float pwm_duty;

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
    printf("Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    int sub_topic_len = 0;
    char topic_received[6];

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

            sub_topic_len = event->topic_len - (MQTT_TOPYC_LEN + 1);
            strncpy(topic_received, event->topic+(MQTT_TOPYC_LEN + 1), sub_topic_len);

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
            else if(0 == strcmp("pwm0", topic_received)){
                if(pwm_key != NULL){
                    if(xSemaphoreTake(pwm_key, pdMS_TO_TICKS(100))){
                        pwm_duty = atof(event->data);
                        xSemaphoreGive(pwm_key);
                    }
                }
            }
            else{
                printf("Topic not mached!\n");           
            }
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
        .uri = "mqtt://192.168.1.6:1883"};

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