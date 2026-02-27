#include "controller.h"

static const char* controller_tag = "Controller";

esp_err_t read_from_estimate_queue(state_t* dest) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response =
      xQueueReceive(estimated_state_queue, dest, xStateQueueRecieveBlockTime);

  if (response == pdTRUE)
    return ESP_OK;
  else
    return ESP_ERR_NO_MEM;
}

esp_err_t write_to_torques_queue(motor_torque_t* data) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response =
      xQueueSendToBack(motor_torque_queue, data, xTorqueQueueSendBlockTime);
  if (response == pdTRUE)
    return ESP_OK;
  else
    return ESP_ERR_NO_MEM;
}

void controller_task(void* arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();

  int counter = 0;

  esp_err_t err;

  while (1) {
    if (!calibration_finished) {
      continue;
    }
    // blocking action:
    if ((err = read_from_estimate_queue(&current_state_estimate) != ESP_OK)) {
      ESP_LOGE(controller_tag,
               "Failed when attempting to read state estimate from queue: %d",
               err);
    }

    float x, v, pitch, omega;
    x = current_state_estimate.x;
    v = current_state_estimate.v;
    pitch = current_state_estimate.pitch;
    omega = current_state_estimate.omega;

    torques.T_left = -K_1_1 * x - K_1_2 * v - K_1_3 * pitch - K_1_4 * omega;
    torques.T_right = -K_1_1 * x - K_1_2 * v - K_1_3 * pitch - K_1_4 * omega;

    if (counter % 5 == 0) {
      ESP_LOGD(
          controller_tag,
          "State Estimates: (%d.%d, %d.%d, %d.%d, %d.%d), counter: %d",
          (int)(current_state_estimate.x),
          (int)(fabs(current_state_estimate.x) * 100) % 100,
          (int)(current_state_estimate.v),
          (int)(fabs(current_state_estimate.v) * 100) % 100,
          (int)(current_state_estimate.pitch * 180 / M_PI),
          (int)(fabs(current_state_estimate.pitch * 180 / M_PI) * 100) % 100,
          (int)(current_state_estimate.omega),
          (int)(fabs(current_state_estimate.omega) * 100) % 100, counter);
      ESP_LOGD(controller_tag,
               "Motor Toruqes - Left: %d.%d, Right: %d.%d. Counter: %d",
               (int)(torques.T_left), (int)(fabs(torques.T_left) * 100) % 100,
               (int)(torques.T_right), (int)(fabs(torques.T_right) * 100) % 100,
               counter);
    }

    counter++;

    // blocking action:
    if (write_to_torques_queue(&torques) != ESP_OK) {
      ESP_LOGE(controller_tag,
               "Failed when attempting to send generated motor tasks to queue");
      continue;
    }

    vTaskDelayUntil(&xLastWakeTime, xControllerFrequency);
  }
}
