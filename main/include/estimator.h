#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <math.h>

#include "common_variables.h"
#include "data_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#define EKF_N 2

typedef struct {
  float x[EKF_N];
  float P[EKF_N][EKF_N];
  float Q[EKF_N][EKF_N];
  float R;
} ekf_t;

ekf_t ekf;
state_t state_est;

TickType_t xEstimatorFrequency = pdMS_TO_TICKS(10);
TickType_t xSensorQueueRecieveBlockTime = pdMS_TO_TICKS(10);
TickType_t xEstimateQueueWriteBlockTime = portMAX_DELAY;

float dt;
sensorData_t raw_sensor_values;

esp_err_t read_from_sensor_queue(sensorData_t*);
esp_err_t write_to_estimate_queue(state_t*);

void ekf_init(ekf_t* ekf);
void estimate_state(float dt);
void estimate_task(void* arg);

#endif  // ESTIMATOR_H
