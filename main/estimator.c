#include "estimator.h"

#include "esp_log.h"
#include "freertos/task.h"

static const char* estimator_tag = "Estimator";

esp_err_t read_from_queue(sensorData_t* dest) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response =
      xQueueReceive(raw_sensor_queue, dest, xQueueRecieveBlockTime);

  if (response == pdTRUE)
    return ESP_OK;
  else
    return ESP_ERR_NO_MEM;
}

void initialise_estimates() {
  ESP_LOGI(estimator_tag, "Initialising estimator");
  acc_est.roll = 0;
  acc_est.pitch = 0;
  acc_est.yaw = 0;
  gyro_est.roll = 0;
  gyro_est.pitch = 0;
  gyro_est.yaw = 0;
  fusion_est.roll = 0;
  fusion_est.pitch = 0;
  fusion_est.yaw = 0;

  dt = 0.0f;
}

void estimate_angles(float dt, float tau) {
  // blocking action:
  if (read_from_queue(&raw_sensor_values) != ESP_OK) {
    ESP_LOGE(estimator_tag,
             "Failed when attempting to read raw values from queue");
  }

  float g = powf(raw_sensor_values.accel.x * raw_sensor_values.accel.x +
                     raw_sensor_values.accel.y * raw_sensor_values.accel.y +
                     raw_sensor_values.accel.z * raw_sensor_values.accel.z,
                 0.5);

  acc_est.roll = atan2f(raw_sensor_values.accel.y, raw_sensor_values.accel.z);
  acc_est.pitch = asinf(-raw_sensor_values.accel.x / g);

  float p = raw_sensor_values.gyro.x;
  float q = raw_sensor_values.gyro.y;
  float r = raw_sensor_values.gyro.z;

  gyro_est.roll = (p + tanf(fusion_est.pitch) * sinf(fusion_est.roll) * q +
                   cosf(fusion_est.roll) * r) *
                  dt;
  gyro_est.pitch = (q * cosf(fusion_est.roll) - r * sinf(fusion_est.roll)) * dt;

  fusion_est.roll =
      (tau) * (fusion_est.roll + gyro_est.roll) + (1 - tau) * acc_est.roll;
  fusion_est.pitch =
      (tau) * (fusion_est.pitch + gyro_est.pitch) + (1 - tau) * acc_est.pitch;
  fusion_est.yaw = 0;  // requires magnetometer for accurate yaw estimate

  ESP_LOGI(estimator_tag, "Euler Angle Estimates: (%d.%d, %d.%d, %d.%d)",
           (int)(fusion_est.roll), (int)(fabs(fusion_est.roll) * 100) % 100,
           (int)(fusion_est.pitch), (int)(fabs(fusion_est.pitch) * 100) % 100,
           (int)(fusion_est.yaw), (int)(fabs(fusion_est.yaw) * 100) % 100);
}

void estimate_task(void* arg) {
  portTickType xLastWakeTime;
  initialise_estimates();

  xLastWakeTime = xTaskGetTickCount();
  float tau = 0.80f;
  dt = 0.02f;

  while (1) {
    if (calibration_finished) {
      // ESP_LOGI(estimator_tag, "dt: %d.%d", (int)dt,
      //          (int)(fabs(dt) * 100000) % 100000);
      estimate_angles(dt, tau);
      vTaskDelayUntil(&xLastWakeTime, xEstimatorFrequency);
    }
  }
}
