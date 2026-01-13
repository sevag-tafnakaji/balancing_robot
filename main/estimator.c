#include "estimator.h"

#include "esp_log.h"
#include "freertos/task.h"

static const char* estimator_tag = "Estimator";

void initialise_estimates() {
  ESP_LOGI(estimator_tag, "Initialising estimator");
  xSemaphoreTake(estimator_sem, portMAX_DELAY);
  acc_est.roll = 0;
  acc_est.pitch = 0;
  acc_est.yaw = 0;
  gyro_est.roll = 0;
  gyro_est.pitch = 0;
  gyro_est.yaw = 0;
  fusion_est.roll = 0;
  fusion_est.pitch = 0;
  fusion_est.yaw = 0;
  xSemaphoreGive(estimator_sem);

  dt = 0.0f;
}

void estimate_angles(float dt, float tau) {
  xSemaphoreTake(sensors_sem, portMAX_DELAY);
  acc_est.roll = atan2(raw_sensor_values.accel.y, raw_sensor_values.accel.z);
  acc_est.pitch =
      atan2(-raw_sensor_values.accel.x,
            pow(raw_sensor_values.accel.y * raw_sensor_values.accel.y +
                    raw_sensor_values.accel.z * raw_sensor_values.accel.z,
                0.5));

  gyro_est.roll += raw_sensor_values.gyro.x * dt;
  gyro_est.pitch += raw_sensor_values.gyro.y * dt;
  gyro_est.yaw += raw_sensor_values.gyro.z * dt;
  xSemaphoreGive(sensors_sem);

  xSemaphoreTake(estimator_sem, portMAX_DELAY);
  fusion_est.roll = (tau) * (fusion_est.roll + gyro_est.roll * dt) +
                    (1 - tau) * acc_est.roll * (180 / M_PI);
  fusion_est.pitch = (tau) * (fusion_est.pitch + gyro_est.pitch * dt) +
                     (1 - tau) * acc_est.pitch * (180 / M_PI);
  fusion_est.yaw = 0;  // requires magnetometer for accurate yaw estimate
  ESP_LOGI(estimator_tag, "Euler Angle Estimates: (%d.%d, %d.%d, %d.%d)",
           (int)(fusion_est.roll), (int)(fabs(fusion_est.roll) * 100) % 100,
           (int)(fusion_est.pitch), (int)(fabs(fusion_est.pitch) * 100) % 100,
           (int)(fusion_est.yaw), (int)(fabs(fusion_est.yaw) * 100) % 100);
  xSemaphoreGive(estimator_sem);
}

void estimate_task(void* arg) {
  portTickType xLastWakeTime;
  initialise_estimates();

  xLastWakeTime = xTaskGetTickCount();
  float tau = 0.9f;
  dt = 0.02;

  while (1) {
    if (calibration_finished) {
      // ESP_LOGI(estimator_tag, "dt: %d.%d", (int)dt,
      //          (int)(fabs(dt) * 100000) % 100000);
      estimate_angles(dt, tau);
      vTaskDelayUntil(&xLastWakeTime, xEstimatorFrequency);
    }
  }
}
