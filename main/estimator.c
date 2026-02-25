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
  float g = powf(raw_sensor_values.accel.x * raw_sensor_values.accel.x +
                     raw_sensor_values.accel.y * raw_sensor_values.accel.y +
                     raw_sensor_values.accel.z * raw_sensor_values.accel.z,
                 0.5);

  acc_est.roll = atan2f(raw_sensor_values.accel.y, raw_sensor_values.accel.z);
  acc_est.pitch = asinf(-raw_sensor_values.accel.x / g);

  float p = raw_sensor_values.gyro.x;
  float q = raw_sensor_values.gyro.y;
  float r = raw_sensor_values.gyro.z;

  // Singularity when pitch close to ±pi/2, roll unexpected values due to
  // division by 0
  gyro_est.roll = (p + tanf(fusion_est.pitch) * sinf(fusion_est.roll) * q +
                   tanf(fusion_est.pitch) * cosf(fusion_est.roll) * r) *
                  dt;
  gyro_est.pitch = (q * cosf(fusion_est.roll) - r * sinf(fusion_est.roll)) * dt;
  gyro_est.yaw = (q * sinf(fusion_est.roll) / cosf(fusion_est.pitch) +
                  r * cosf(fusion_est.roll) / cosf(fusion_est.pitch)) *
                 dt;

  fusion_est.roll =
      (1 - tau) * (fusion_est.roll + gyro_est.roll) + (tau)*acc_est.roll;
  fusion_est.pitch =
      (1 - tau) * (fusion_est.pitch + gyro_est.pitch) + (tau)*acc_est.pitch;
  // requires magnetometer for accurate yaw estimate
  fusion_est.yaw = tau * gyro_est.yaw;
}

void estimate_task(void* arg) {
  portTickType xLastWakeTime;
  initialise_estimates();

  xLastWakeTime = xTaskGetTickCount();
  float tau = 0.95f;
  dt = 0.01f;

  int counter = 0;

  while (1) {
    if (!calibration_finished) {
      continue;
    }

    // blocking action:
    if (read_from_queue(&raw_sensor_values) != ESP_OK) {
      ESP_LOGE(estimator_tag,
               "Failed when attempting to read raw values from queue");
    }

    estimate_angles(dt, tau);

    if (counter % 5 == 0) {
      ESP_LOGI(estimator_tag,
               "Euler Angle Estimates: (%d.%d, %d.%d, %d.%d), counter: %d",
               (int)(fusion_est.roll * 180 / M_PI),
               (int)(fabs(fusion_est.roll * 180 / M_PI) * 100) % 100,
               (int)(fusion_est.pitch * 180 / M_PI),
               (int)(fabs(fusion_est.pitch * 180 / M_PI) * 100) % 100,
               (int)(fusion_est.yaw * 180 / M_PI),
               (int)(fabs(fusion_est.yaw * 180 / M_PI) * 100) % 100, counter);
    }

    counter++;
    vTaskDelayUntil(&xLastWakeTime, xEstimatorFrequency);
  }
}
