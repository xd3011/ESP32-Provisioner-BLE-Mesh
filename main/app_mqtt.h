#ifndef __APP_MQTT_H
#define __APP_MQTT_H
#include <stdint.h>

#include "mqtt_client.h"

typedef void (*mqtt_data_pt_t)(uint8_t *data, uint16_t lenght);
esp_mqtt_client_handle_t get_mqtt_client();

void mqtt_app_start(void);
void mqtt_data_pt_set_callback(void *cb);

#endif