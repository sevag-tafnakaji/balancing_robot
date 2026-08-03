#include "common_variables.h"
#include "controller.c"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "estimator.c"
#include "mpu6050.c"
#include "mqtt_sender.c"
#include "tb6612fng.c"

static const char* main_tag = "main";

void app_main(void) {
  ESP_LOGI(main_tag, "Startup..");
  ESP_LOGI(main_tag, "Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(main_tag, "IDF version: %s", esp_get_idf_version());

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  ESP_LOGI(main_tag, "This is ESP8266 chip with %d CPU core(s), and WiFi",
           chip_info.cores);

  ESP_LOGI(main_tag, "Initialising queues");

  raw_sensor_queue = xQueueCreate(5, sizeof(sensorData_t));

  if (raw_sensor_queue == NULL) {
    ESP_LOGE(main_tag,
             "Error when building queue for raw sensor data. Stopping.");
    return;
  }

  estimated_state_queue = xQueueCreate(5, sizeof(state_t));

  if (estimated_state_queue == NULL) {
    ESP_LOGE(main_tag,
             "Error when building queue for state estimation. Stopping.");
    return;
  }

  motor_torque_queue = xQueueCreate(5, sizeof(motor_torque_t));

  if (motor_torque_queue == NULL) {
    ESP_LOGE(main_tag,
             "Error when building queue for motor torques. Stopping.");
    return;
  }

  ESP_LOGI(main_tag, "Initialising semaphores");

  mqtt_sensor_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(mqtt_sensor_sem);

  mqtt_estimated_state_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(mqtt_estimated_state_sem);

  mqtt_motor_torque_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(mqtt_motor_torque_sem);

  ESP_LOGI(main_tag, "Beginning tasks");

  // start i2c task
  xTaskCreate(mpu6050_task, "mpu6050 reading task", 2048, NULL, 20, NULL);

  xTaskCreate(estimate_task, "Angle estimator task", 2048, NULL, 10, NULL);

  xTaskCreate(controller_task, "Controller task", 2048, NULL, 3, NULL);

  xTaskCreate(driver_task, "DC Motor driver task", 2048, NULL, 3, NULL);

  // setup_mqtt();

  // xTaskCreate(mqtt_publisher_task, "MQTT publisher task", 2048, NULL, 1,
  // NULL);
}
