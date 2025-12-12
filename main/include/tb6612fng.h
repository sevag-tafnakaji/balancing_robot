#ifndef TB6612FNG_H
#define TB6612FNG_H

// GPIO pin definitions
#define A_IN_2 GPIO_NUM_0  // D3
#define A_IN_1 GPIO_NUM_2  // D4
#define STBY GPIO_NUM_14   // D5
#define B_IN_1 GPIO_NUM_12 // D6
#define B_IN_2 GPIO_NUM_13 // D7

/* #define GPIO_OUTPUT_PIN_SEL \
   ((1ULL << PWM_A) | (1ULL << PWM_B) | (1ULL << A_IN_1) | (1ULL << A_IN_2) |  \
    (1ULL << B_IN_1) | (1ULL << B_IN_2) | \ (1ULL << STBY)) // Combine output
   pins to bitmap */
/*#define GPIO_OUTPUT_PIN_SEL \
  ((1ULL << PWM_A) | (1ULL << A_IN_1) | (1ULL << A_IN_2) |                     \
   (1ULL << STBY)) // Combine output pins to bitmap */
#define GPIO_OUTPUT_PIN_SEL                                                    \
  ((1ULL << A_IN_1) | (1ULL << A_IN_2) | (1ULL << B_IN_1) | (1ULL << B_IN_2) | \
   (1ULL << STBY)) // Combine output pins to bitmap
// #define GPIO_OUTPUT_PIN_SEL ((1ULL << PWM_A) | (1ULL << A_IN_1) | (1ULL <<
// A_IN_2) | (1ULL << STBY)) // Combine output pins to bitmap

// Values for motor modes
// #define MOTOR_STOP 0x00        // Mode In pins: LL
// #define MOTOR_CCW 0x01         // Mode In pins: LH
// #define MOTOR_CW 0x10          // Mode In pins: HL
// #define MOTOR_SHORT_BREAK 0x11 // Mode In pins: HH

enum motor_dir { MOTOR_STOP, MOTOR_CW, MOTOR_CCW, MOTOR_SHORT_BREAK };

// PWM period 50000us(1Khz)
#define PWM_PERIOD 50000

#include "driver/gpio.h"
#include "driver/pwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *tb6612fng_tag = "TB6612FNG";
int curr_mode_A = MOTOR_STOP;
int curr_mode_B = MOTOR_STOP;

void set_motor_mode_A(int);
void set_motor_mode_B(int);

void set_motor_mode(int, char);

void set_both_motor_modes(int);

void set_motor_duties(float, float);

void initialise_pwms();

void initialise_gpio();

void initialise_driver();

void driver_task(void *);

#endif // TB6612FNG_H
