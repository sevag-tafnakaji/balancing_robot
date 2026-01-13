
#include "system_semaphores.h"

#include "esp_log.h"

static const char* semaphore_tag = "semaphores";

void semaphores_init() {
  ESP_LOGI(semaphore_tag, "Initialising semaphores");
  motors_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(motors_sem);

  sensors_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sensors_sem);

  estimator_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(estimator_sem);

  states_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(states_sem);
}
