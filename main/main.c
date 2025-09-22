#include "mpu6050.c"
#include "tb6612fng.c"

#include "esp_log.h"
#include "driver/gpio.h"

static const char *main_tag = "main";

void app_main(void)
{

    ESP_LOGI(main_tag, "Beginning tasks");
    // start i2c task
    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);

    xTaskCreate(driver_task, "driver_test", 2048, NULL, 20, NULL);
}
