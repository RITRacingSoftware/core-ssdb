/**
  * @file   main.c
  * @brief  Main application code
  */

#include "main.h"

#include <stdbool.h>
#include <stdio.h>

#include "CAN/driver_can.h"
#include "FrontSSDB/driver_front.h"
#include "RearSSDB/driver_rear.h"
#include "ssdb_config.h"
#include "clock.h"
#include "gpio.h"
#include "boot.h"
#include "error_handler.h"
#include "sensor_dbc.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stm32g4xx_hal.h>


void heartbeat_task(void *pvParameters) {
    (void) pvParameters;
    while(true) {
        core_GPIO_toggle_heartbeat();
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

void collect_sensors_task(void *pvParameters) {
    (void) pvParameters;
    TickType_t nextWakeTime = xTaskGetTickCount();

    while (true) {
#ifdef TARGET_REAR
        SSDB_rear_collect_sensors();
        vTaskDelayUntil(&nextWakeTime, SSDB_REAR_LOOP_DELAY * portTICK_PERIOD_MS);
#endif
#ifdef TARGET_FRONT
        SSDB_front_collect_sensors();
        vTaskDelayUntil(&nextWakeTime, SSDB_FRONT_LOOP_DELAY * portTICK_PERIOD_MS);
#endif
    }
}

int main(void) {
    HAL_Init();

    // Drivers
    core_heartbeat_init(GPIOC, GPIO_PIN_8);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) error_handler();
    if (!CAN_init()) error_handler();
    core_boot_init();
#ifdef TARGET_REAR
    if (!SSDB_rear_init()) error_handler();
#endif
#ifdef TARGET_FRONT
    if (!SSDB_front_init()) error_handler();
#endif

    int err = xTaskCreate(heartbeat_task,
        "heartbeat",
        1000,
        NULL,
        4,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }
    err = xTaskCreate(collect_sensors_task,
        "collect_sensors",
        1000,
        NULL,
        4,
        NULL);
    if (err != pdPASS) {
        error_handler();
    }

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // hand control over to FreeRTOS
    vTaskStartScheduler();

    // we should not get here ever
    error_handler();
    return 1;
}

// Called when stack overflows from rtos
// Not needed in header, since included in FreeRTOS-Kernel/include/task.h
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName) {
    (void) xTask;
    (void) pcTaskName;

    error_handler();
}
