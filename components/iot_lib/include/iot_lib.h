#include "esp_err.h"
#include "esp_event.h"

void wifi_init(void);
void mqtt_publish(const char *data);
void iot_gpio_init(void);
static void mqtt_app_start(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_app_start(void);