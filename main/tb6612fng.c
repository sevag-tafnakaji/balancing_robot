
#include "tb6612fng.h"

/**
 * TODO:
 * convert mode + PWM into PWM being baked into mode pins (https://forum.pololu.com/t/tb6612fng-dual-motor-driver-and-pwm-signals/5697). Reduces number of pins needed
 * Using RX pin makes motor B run a bit during boot, and messes with logging. Try to find other pins to use.
 * Initial duty setting not done correctly? motors stick with the duties set during PWM initialisation. (used pwm_start after initialisation, seems to fix)
 * After first duty change, one motor doesn't turn sometimes. (seems to be an issue with duty cycle or voltage or PWM freq being too small. Seems to be mainly voltage? Increase duty cycle or freq to accommodate)
 * sometimes doesn't boot into the correct place and need to reset into it
 */

void set_motor_mode_A(int mode)
{
    ESP_ERROR_CHECK(gpio_set_level(A_IN_1, (uint32_t)(mode & 0x1)));
    ESP_ERROR_CHECK(gpio_set_level(A_IN_2, (uint32_t)(mode & 0x10)));
}

void set_motor_mode_B(int mode)
{
    ESP_ERROR_CHECK(gpio_set_level(B_IN_1, (uint32_t)(mode & 0x1)));
    ESP_ERROR_CHECK(gpio_set_level(B_IN_2, (uint32_t)(mode & 0x10)));
}

void set_motor_mode(int mode, char motor)
{
    ESP_LOGD(tb6612fng_tag, "Setting motor %c to mode %d", motor, mode);
    if (motor == 'A')
        set_motor_mode_A(mode);
    else if (motor == 'B')
        set_motor_mode_B(mode);
}

void set_both_motor_modes(int mode)
{
    set_motor_mode(mode, 'A');
    set_motor_mode(mode, 'B');
}

void set_motor_duty(float duty, char motor)
{
    uint32_t ui_duty = pwm_period * duty;
    uint32_t ui_duties[] = {0, 0};
    if (motor == 'A')
    {
        ui_duties[0] = ui_duty;
    }
    else if (motor == 'B')
        ui_duties[1] = ui_duty;

    uint32_t *pDuties = ui_duties;

    ESP_ERROR_CHECK(pwm_set_duties(pDuties));
    ESP_ERROR_CHECK(pwm_start());
}

void set_motor_duties(float duty_A, float duty_B)
{
    uint32_t ui_duties[] = {pwm_period * duty_A, pwm_period * duty_B};
    uint32_t *pDuties = ui_duties;

    ESP_ERROR_CHECK(pwm_set_duties(pDuties));
    ESP_ERROR_CHECK(pwm_start());
}

void initialise_pwms()
{
    uint32_t pins[] = {PWM_A, PWM_B};
    // uint32_t duties[] = {pwm_period, pwm_period};
    uint32_t duties[] = {40, 40};

    ESP_ERROR_CHECK(pwm_init(pwm_period, duties, 2, pins));

    // ESP_ERROR_CHECK(pwm_set_phase(1u, 0.0f));
    // initialise_pwm(pwm_period, 1.0f, 2u, PWM_B);
    // ESP_ERROR_CHECK(pwm_set_phase(2u, 0.0f));

    float phases[] = {0.0f, 0.0f};
    // float phases[] = {0.0f};
    // float *pPhases = phases;

    ESP_ERROR_CHECK(pwm_set_phases(phases));
    ESP_ERROR_CHECK(pwm_start());
}

void initialise_gpio()
{
    gpio_config_t io_conf;
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void initialise_driver()
{
    // initialise GPIO pins
    initialise_gpio();

    // keep STBY on high during init
    gpio_set_level(STBY, 1u);

    // // // initialise PWM pins
    initialise_pwms();

    // // // Set initial motor modes
    set_both_motor_modes(MOTOR_SHORT_BREAK);
    // set_motor_mode_A(MOTOR_SHORT_BREAK);
}

void driver_task(void *args)
{
    initialise_driver();

    pwm_start();

    while (1)
    {

        ESP_LOGI(tb6612fng_tag, "Rotating slowly CW for 2 sec");
        // set_motor_duty(0.2, 'A');
        set_motor_duties(0.2, 0.2);

        // Rotate CW slowly for 2 sec
        set_both_motor_modes(MOTOR_CW);

        vTaskDelay(2000 / portTICK_RATE_MS);

        // Stop for 1 second

        ESP_LOGI(tb6612fng_tag, "Stopping for 1 sec");

        set_both_motor_modes(MOTOR_SHORT_BREAK);

        vTaskDelay(1000 / portTICK_RATE_MS);

        // Rotate CCW faster for 2 seconds

        ESP_LOGI(tb6612fng_tag, "Rotating quickly CCW for 2 sec");

        // set_motor_duty(0.8, 'A');
        set_motor_duties(0.8, 0.8);

        set_both_motor_modes(MOTOR_CCW);

        vTaskDelay(2000 / portTICK_RATE_MS);

        // Stop for 1 second

        ESP_LOGI(tb6612fng_tag, "Stopping for 1 sec");

        set_both_motor_modes(MOTOR_SHORT_BREAK);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
