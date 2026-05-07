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
    return ESP_FAIL;
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
    return ESP_FAIL;
}

void ekf_init(ekf_t* ekf) {
  ekf->x[0] = 0.0f;  // pitch
  ekf->x[1] = 0.0f;  // gyro bias

  ekf->P[0][0] = 1.0f;
  ekf->P[0][1] = 0.0f;
  ekf->P[1][0] = 0.0f;
  ekf->P[1][1] = 1.0f;

  ekf->Q[0][0] = 0.001f;  // pitch process noise
  ekf->Q[0][1] = 0.0f;
  ekf->Q[1][0] = 0.0f;
  ekf->Q[1][1] = 0.003f;  // bias random walk

  ekf->R = 0.03f;  // accelerometer measurement noise
}

void estimate_state(float dt) {
  float gyro_y = raw_sensor_values.gyro.y;
  float ax = raw_sensor_values.accel.x;
  float ay = raw_sensor_values.accel.y;
  float az = raw_sensor_values.accel.z;

  // --- EKF Predict ---
  float pitch_pred = ekf.x[0] + (gyro_y - ekf.x[1]) * dt;
  float bias_pred = ekf.x[1];

  // P_pred = F * P * F^T + Q, where F = [[1, -dt], [0, 1]]
  float P00 = ekf.P[0][0] - dt * ekf.P[1][0] -
              dt * (ekf.P[0][1] - dt * ekf.P[1][1]) + ekf.Q[0][0];
  float P01 = ekf.P[0][1] - dt * ekf.P[1][1] + ekf.Q[0][1];
  float P10 = ekf.P[1][0] - dt * ekf.P[1][1] + ekf.Q[1][0];
  float P11 = ekf.P[1][1] + ekf.Q[1][1];

  // --- EKF Update ---
  float z = atan2f(-ax, sqrtf(ay * ay + az * az));
  float y = z - pitch_pred;
  float S = P00 + ekf.R;
  float K0 = P00 / S;
  float K1 = P10 / S;

  ekf.x[0] = pitch_pred + K0 * y;
  ekf.x[1] = bias_pred + K1 * y;

  // P = (I - K*H) * P_pred, H = [1, 0]
  ekf.P[0][0] = (1.0f - K0) * P00;
  ekf.P[0][1] = (1.0f - K0) * P01;
  ekf.P[1][0] = P10 - K1 * P00;
  ekf.P[1][1] = P11 - K1 * P01;

  float pitch = ekf.x[0];
  float omega = gyro_y - ekf.x[1];

  // gravity-compensated forward acceleration
  float a_forward = ax * cosf(pitch) + az * sinf(pitch);

  state_est.pitch = pitch;
  state_est.omega = omega;
  state_est.v += a_forward * dt;
  state_est.x += state_est.v * dt;
}

void estimate_task(void* arg) {
  portTickType xLastWakeTime;

  ekf_init(&ekf);
  state_est.x = 0.0f;
  state_est.v = 0.0f;
  state_est.pitch = 0.0f;
  state_est.omega = 0.0f;

  xLastWakeTime = xTaskGetTickCount();
  dt = 0.01f;

  int counter = 0;

  while (1) {
    if (!calibration_finished) {
      continue;
    }

    if (read_from_sensor_queue(&raw_sensor_values) != ESP_OK) {
      ESP_LOGE(estimator_tag,
               "Failed when attempting to read raw values from queue");
      continue;
    }

    estimate_state(dt);

    if (mqtt_initialised && counter % 300 == 0) {
      xSemaphoreTake(mqtt_estimated_state_sem, pdMS_TO_TICKS(10));
      mqtt_state = state_est;
      xSemaphoreGive(mqtt_estimated_state_sem);
    }

    if (counter % 5 == 0) {
      ESP_LOGD(estimator_tag,
               "State Estimates: (%lf, %lf, %lf, %lf), counter: %d",
               state_est.x, state_est.v, state_est.pitch * 180 / M_PI,
               state_est.omega, counter);
    }

    counter++;

    if (write_to_estimate_queue(&state_est) != ESP_OK) {
      ESP_LOGE(estimator_tag,
               "Failed when attempting to send estimated state to queue");
      continue;
    }
    vTaskDelayUntil(&xLastWakeTime, xEstimatorFrequency);
  }
}
