#include "common_variables.h"
#include "controller.c"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "estimator.c"
#include "mpu6050.c"
#include "tb6612fng.c"

static const char* main_tag = "main";

void app_main(void) {
  ESP_LOGI(main_tag, "Beginning tasks");

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

  // start i2c task
  xTaskCreate(mpu6050_task, "mpu6050 reading task", 2048, NULL, 20, NULL);

  xTaskCreate(estimate_task, "Angle estimator task", 2048, NULL, 10, NULL);

  xTaskCreate(controller_task, "Controller task", 2048, NULL, 5, NULL);

  xTaskCreate(driver_task, "DC Motor driver task", 2048, NULL, 5, NULL);
}
