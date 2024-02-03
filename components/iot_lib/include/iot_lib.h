#include "esp_err.h"
#include "esp_event.h"
#include "driver/mcpwm.h"

void iot_dgt_setup(char **subtopics, char * storage_name, int ports[],int out_num);
void iot_pwm_setup(char * new_pwm_topic, char * storage_name, mcpwm_unit_t new_mcpwm_num,  mcpwm_generator_t new_pwm_operator, mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int gpio_num, int freq);
void iot_init(const char * conf_wifi_ssid, const char * conf_wifi_pass, const char *conf_uri);
void mqtt_publish(const char * data, const char * pub_topic);
static void mqtt_app_start(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);