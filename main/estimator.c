#include "estimator.h"

#include "esp_log.h"
#include "freertos/task.h"

static const char* estimator_tag = "Estimator";

esp_err_t read_from_sensor_queue(sensorData_t* dest) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response =
      xQueueReceive(raw_sensor_queue, dest, xSensorQueueRecieveBlockTime);

  if (response == pdTRUE)
    return ESP_OK;
  else
    return ESP_ERR_NO_MEM;
}

esp_err_t write_to_estimate_queue(state_t* data) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response = xQueueSendToBack(estimated_state_queue, data,
                                         xEstimateQueueWriteBlockTime);
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

  state_est.x = 0.0f;
  state_est.v = 0.0f;
  state_est.pitch = 0.0f;
  state_est.omega = 0.0f;

  dt = 0.0f;
}

void estimate_state(float dt, float tau) {
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

  state_est.pitch = fusion_est.pitch;
  state_est.omega = gyro_est.pitch / dt;
  // accel + friction term
  state_est.v += (raw_sensor_values.accel.x - 0.9f * state_est.v) * dt;
  state_est.x += state_est.v * dt;
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
    if (read_from_sensor_queue(&raw_sensor_values) != ESP_OK) {
      ESP_LOGE(estimator_tag,
               "Failed when attempting to read raw values from queue");
      continue;
    }

    estimate_state(dt, tau);

    if (counter % 5 == 0) {
      ESP_LOGD(estimator_tag,
               "State Estimates: (%d.%d, %d.%d, %d.%d, %d.%d), counter: %d",
               (int)(state_est.x), (int)(fabs(state_est.x) * 100) % 100,
               (int)(state_est.v), (int)(fabs(state_est.v) * 100) % 100,
               (int)(state_est.pitch * 180 / M_PI),
               (int)(fabs(state_est.pitch * 180 / M_PI) * 100) % 100,
               (int)(state_est.omega), (int)(fabs(state_est.omega) * 100) % 100,
               counter);
    }

    counter++;

    // blocking action:
    if (write_to_estimate_queue(&state_est) != ESP_OK) {
      ESP_LOGE(estimator_tag,
               "Failed when attempting to send estimated state to queue");
      continue;
    }
    vTaskDelayUntil(&xLastWakeTime, xEstimatorFrequency);
  }
}
