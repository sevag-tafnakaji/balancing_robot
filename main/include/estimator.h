#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <math.h>

#include "common_variables.h"
#include "data_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// angle estimates from gyroscope
eulerAngles_t gyro_est;

// angle estimates from accelerometer
eulerAngles_t acc_est;

// angle estimate from sensor fusion
eulerAngles_t fusion_est;

state_t state_est;

TickType_t xEstimatorFrequency = pdMS_TO_TICKS(10);
TickType_t xSensorQueueRecieveBlockTime = pdMS_TO_TICKS(10);
TickType_t xEstimateQueueWriteBlockTime = portMAX_DELAY;

float dt;
sensorData_t raw_sensor_values;

esp_err_t read_from_sensor_queue(sensorData_t*);
esp_err_t write_to_estimate_queue(state_t*);

// Higher tau -> trust gyroscope results more
void estimate_state(float dt, float tau);
void initialise_estimates();

void estimate_task(void* arg);

#endif  // ESTIMATOR_H
