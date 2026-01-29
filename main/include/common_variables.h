#ifndef COMMON_VARIABLES_H
#define COMMON_VARIABLES_H

#include "data_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "stdbool.h"

#define MAX_SENSOR_QUEUE_SIZE 150

bool calibration_finished = false;
// sensorData_t raw_sensor_values;
xQueueHandle raw_sensor_queue;

#endif  // COMMON_VARIABLES_H
