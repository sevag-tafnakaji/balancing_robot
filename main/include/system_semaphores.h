#ifndef SYS_SEMAPHORE_H
#define SYS_SEMAPHORE_H

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t motors_sem;
static SemaphoreHandle_t sensors_sem;
static SemaphoreHandle_t estimator_sem;
static SemaphoreHandle_t states_sem;

void semaphores_init();

#endif  // SYS_SEMAPHORE_H
