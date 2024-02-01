idf_component_register(SRCS "iot_lib.c"
                  INCLUDE_DIRS "include"
                  REQUIRES mqtt pwm_lib nvs_flash)