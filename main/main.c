#include "common_variables.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "estimator.c"
#include "mpu6050.c"
#include "sample_multivariate_gaussian.c"
#include "system_semaphores.c"
#include "tb6612fng.c"

static const char* main_tag = "main";

void app_main(void) {
  semaphores_init();

  ESP_LOGI(main_tag, "Beginning tasks");

  // int dim = 3;
  // float** cov = initialise_matrix(dim);

  // cov[0][0] = 4.0f;
  // cov[0][1] = 12.0f;
  // cov[0][2] = -16.0f;
  // cov[1][0] = 12.0f;
  // cov[1][1] = 37.0f;
  // cov[1][2] = -43.0f;
  // cov[2][0] = -16.0f;
  // cov[2][1] = -43.0f;
  // cov[2][2] = 98.0f;

  // ESP_LOGI(
  //     main_tag,
  //     "[[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d,
  //     %d.%d]]", (int)cov[0][0], (int)(fabs(cov[0][0]) * 100) % 100,
  //     (int)cov[0][1], (int)(fabs(cov[0][1]) * 100) % 100, (int)cov[0][2],
  //     (int)(fabs(cov[0][2]) * 100) % 100, (int)cov[1][0],
  //     (int)(fabs(cov[1][0]) * 100) % 100, (int)cov[1][1],
  //     (int)(fabs(cov[1][1]) * 100) % 100, (int)cov[1][2],
  //     (int)(fabs(cov[1][2]) * 100) % 100, (int)cov[2][0],
  //     (int)(fabs(cov[2][0]) * 100) % 100, (int)cov[2][1],
  //     (int)(fabs(cov[2][1]) * 100) % 100, (int)cov[2][2],
  //     (int)(fabs(cov[2][2]) * 100) % 100);

  // Expected result: [[2, 6, -8], [0, 1, 5], [0, 0, 3]]
  // test_chol(cov, dim);

  raw_sensor_queue = xQueueCreate(150, sizeof(sensorData_t));

  if (raw_sensor_queue == NULL) {
    ESP_LOGE(main_tag,
             "Error when building queue for raw sensor data. Stopping.");
  } else {
    // start i2c task
    xTaskCreate(mpu6050_task, "mpu6050 reading task", 2048, NULL, 20, NULL);

    // vTaskDelay(pdMS_TO_TICKS(5000));

    xTaskCreate(estimate_task, "Angle estimator task", 2048, NULL, 10, NULL);

    // xTaskCreate(driver_task, "driver_test", 2048, NULL, 20, NULL);
  }
}
