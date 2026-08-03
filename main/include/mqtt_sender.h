#ifndef MQTT_SENDER_H
#define MQTT_SENDER_H

#include "common_variables.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

TickType_t xMQTTSenderFrequency = pdMS_TO_TICKS(100);

void setup_mqtt();
void mqtt_publisher_task(void* arg);

#endif  // MQTT_SENDER_H
