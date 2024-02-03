#include "esp_err.h"
#include "esp_event.h"

void iot_dgt_setup(char **subtopics, char * storage_name, int ports[],int out_num);
//void iot_pwm_setup(mcpwm_unit_t new_mcpwm_num,  mcpwm_timer_t new_timer_num, mcpwm_io_signals_t io_signal, int gpio_num, const char * storage_name, const char * storage_key);
//void iot_init();
void wifi_init(void);
void mqtt_publish(const char *data);
static void mqtt_app_start(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_app_start(void);