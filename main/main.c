#include "mpu6050.c"
#include "webpage.c"

static const char *main_tag = "main";

void app_main(void)
{
    ESP_LOGI(main_tag, "Beginning MPU6050 reading task");
    // start i2c task
    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    ESP_LOGI(main_tag, "Beginning websocket task");
    xTaskCreate(&openssl_server_task, "openssl_server", 8192, NULL, 15, NULL);
}
