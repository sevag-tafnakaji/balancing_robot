#include "mpu6050.c"

static const char *main_tag = "main";

void app_main(void)
{

    ESP_LOGI(main_tag, "Beginning MPU6050 reading task");
    // start i2c task
    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);
}
