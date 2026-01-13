#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <math.h>

#include "common_variables.h"
#include "data_types.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "system_semaphores.h"

// angle estimates from gyroscope
eulerAngles_t gyro_est;

// angle estimates from accelerometer
eulerAngles_t acc_est;

// angle estimate from sensor fusion
eulerAngles_t fusion_est;

TickType_t xEstimatorFrequency = 20 / portTICK_RATE_MS;

float dt;

// Higher tau -> trust gyroscope results more
void estimate_angles(float dt, float tau);
void initialise_estimates();

void estimate_task(void* arg);

#endif  // ESTIMATOR_H
