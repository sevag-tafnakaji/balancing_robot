#ifndef TB6612FNG_H
#define TB6612FNG_H

// GPIO pin definitions
#define PWM_A GPIO_NUM_2   // D4
#define A_IN_2 GPIO_NUM_14 // D5
#define A_IN_1 GPIO_NUM_12 // D6
#define STBY GPIO_NUM_13   // D7
#define B_IN_1 GPIO_NUM_15 // D8
#define B_IN_2 GPIO_NUM_3  // RX
#define PWM_B GPIO_NUM_1   // TX

#define GPIO_OUTPUT_PIN_SEL ((1ULL << PWM_A) | (1ULL << PWM_B) | (1ULL << A_IN_1) | (1ULL << A_IN_2) | (1ULL << B_IN_1) | (1ULL << B_IN_2) | (1ULL << STBY)) // Combine output pins to bitmap
// #define GPIO_OUTPUT_PIN_SEL ((1ULL << PWM_A) | (1ULL << A_IN_1) | (1ULL << A_IN_2) | (1ULL << STBY)) // Combine output pins to bitmap

// Values for motor modes
#define MOTOR_STOP 0x00        // Mode In pins: LL
#define MOTOR_CCW 0x01         // Mode In pins: LH
#define MOTOR_CW 0x10          // Mode In pins: HL
#define MOTOR_SHORT_BREAK 0x11 // Mode In pins: HH

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pwm.h"

static const char *tb6612fng_tag = "TB6612FNG";

// 100 Hz to microseconds
uint32_t pwm_period = ((1.0f / 100.0f) * 1000000); // microseconds

#endif // TB6612FNG_H
