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
    return ESP_FAIL;
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
    return ESP_FAIL;
  ;
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

    if (mqtt_initialised && counter % 300 == 0) {
      xSemaphoreTake(mqtt_motor_torque_sem, pdMS_TO_TICKS(5));
      mqtt_controller = torques;
      xSemaphoreGive(mqtt_motor_torque_sem);
    }

    if (counter % 5 == 0) {
      ESP_LOGI(controller_tag, "State Estimates: (%f, %f, %f, %f), counter: %d",
               current_state_estimate.x, current_state_estimate.v,
               current_state_estimate.pitch * 180 / M_PI,
               current_state_estimate.omega, counter);
      ESP_LOGI(controller_tag,
               "Motor Toruqes - Left: %f, Right: %f. Counter: %d",
               torques.T_left, torques.T_right, counter);
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
