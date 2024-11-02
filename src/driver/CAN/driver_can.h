#include <stm32g4xx.h>

#ifndef DRIVER_CAN_H
#define DRIVER_CAN_H

bool CAN_init();
bool CAN_sensor_transmit(uint32_t id, uint8_t dlc, uint64_t data);
bool CAN_sensor_transmit_extended(uint32_t id, uint8_t dlc, uint8_t *data);
bool CAN_sensor_transmit_task();
bool CAN_main_transmit(uint32_t id, uint8_t dlc, uint64_t data);
bool CAN_main_transmit_task();

#endif
