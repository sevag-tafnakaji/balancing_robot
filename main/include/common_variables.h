#ifndef COMMON_VARIABLES_H
#define COMMON_VARIABLES_H

#include "data_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "stdbool.h"

#define MAX_SENSOR_QUEUE_SIZE 150

bool calibration_finished = false;
bool mqtt_initialised = false;

xQueueHandle raw_sensor_queue;
xQueueHandle estimated_state_queue;
xQueueHandle motor_torque_queue;

SemaphoreHandle_t mqtt_sensor_sem;
SemaphoreHandle_t mqtt_estimated_state_sem;
SemaphoreHandle_t mqtt_motor_torque_sem;

sensorData_t mqtt_sensor;
state_t mqtt_state;
motor_torque_t mqtt_controller;

#endif  // COMMON_VARIABLES_H
