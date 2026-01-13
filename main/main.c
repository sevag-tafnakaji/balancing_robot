#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "estimator.c"
#include "mpu6050.c"
#include "system_semaphores.c"
#include "tb6612fng.c"

static const char* main_tag = "main";

void app_main(void) {
  semaphores_init();

  ESP_LOGI(main_tag, "Beginning tasks");

  // start i2c task
  xTaskCreate(mpu6050_task, "mpu6050 reading task", 2048, NULL, 20, NULL);

  xTaskCreate(estimate_task, "Angle estimator task", 2048, NULL, 10, NULL);

  // xTaskCreate(driver_task, "driver_test", 2048, NULL, 20, NULL);
}
