#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mpu6050.c"
#include "tb6612fng.c"

static const char* main_tag = "main";

void app_main(void) {
  ESP_LOGI(main_tag, "Beginning tasks");
  // start i2c task
  xTaskCreate(mpu6050_task, "mpu6050 reading task", 2048, NULL, 10, NULL);

  xTaskCreate(driver_task, "driver_test", 2048, NULL, 20, NULL);
}
