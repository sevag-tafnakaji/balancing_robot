#include "mqtt_sender.h"

#include <stdio.h>
#include <string.h>

static const char* mqtt_tag = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(mqtt_tag, "MQTT_EVENT_CONNECTED");
      mqtt_initialised = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(mqtt_tag, "MQTT_EVENT_DISCONNECTED");
      mqtt_initialised = false;
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGE(mqtt_tag, "MQTT_EVENT_ERROR");
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                int32_t event_id, void* event_data) {
  mqtt_event_handler_cb(event_data);
}

void setup_mqtt() {
  ESP_LOGI(mqtt_tag, "Initialising NVS, netif, and WiFi");

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = CONFIG_BROKER_URL,
  };

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, mqtt_client);
  esp_mqtt_client_start(mqtt_client);
}

void mqtt_publisher_task(void* arg) {
  char buf[256];

  while (1) {
    if (!mqtt_initialised) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    sensorData_t sensor;
    state_t state;
    motor_torque_t motor;

    xSemaphoreTake(mqtt_sensor_sem, portMAX_DELAY);
    sensor = mqtt_sensor;
    xSemaphoreGive(mqtt_sensor_sem);

    xSemaphoreTake(mqtt_estimated_state_sem, portMAX_DELAY);
    state = mqtt_state;
    xSemaphoreGive(mqtt_estimated_state_sem);

    xSemaphoreTake(mqtt_motor_torque_sem, portMAX_DELAY);
    motor = mqtt_controller;
    xSemaphoreGive(mqtt_motor_torque_sem);

    snprintf(buf, sizeof(buf),
             "{\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
             "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f}",
             sensor.accel.x, sensor.accel.y, sensor.accel.z, sensor.gyro.x,
             sensor.gyro.y, sensor.gyro.z);
    esp_mqtt_client_publish(mqtt_client, "robot/sensor", buf, 0, 0, 0);

    snprintf(buf, sizeof(buf),
             "{\"x\":%.4f,\"v\":%.4f,\"pitch\":%.4f,\"omega\":%.4f}",
             state.x, state.v, state.pitch, state.omega);
    esp_mqtt_client_publish(mqtt_client, "robot/state", buf, 0, 0, 0);

    snprintf(buf, sizeof(buf), "{\"T_left\":%.4f,\"T_right\":%.4f}",
             motor.T_left, motor.T_right);
    esp_mqtt_client_publish(mqtt_client, "robot/motor", buf, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
