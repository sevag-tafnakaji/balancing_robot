#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <math.h>

#include "common_variables.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// LQR gain TODO: add integral gain
#define K_1_1 -0.1204507
#define K_1_2 0.175460
#define K_1_3 -0.4201054
#define K_1_4 0.170
#define K_2_1 0.1083171
#define K_2_2 0.1811022
#define K_2_3 -1.5147
#define K_2_4 -0.1786299

state_t current_state_estimate;
motor_torque_t torques;

TickType_t xControllerFrequency = pdMS_TO_TICKS(5);
TickType_t xStateQueueRecieveBlockTime = pdMS_TO_TICKS(5);
TickType_t xTorqueQueueSendBlockTime = portMAX_DELAY;

esp_err_t read_from_estimate_queue(state_t*);

// TODO: Add reference tracking

void controller_task(void* arg);

#endif
