#include "can.h"
#include "driver_can.h"
#include "ssdb_config.h"
#include <stdint.h>
#include <stdbool.h>

bool CAN_init() {
    if (!core_CAN_init(CAN_SENSOR)) return false;
    if (!core_CAN_init(CAN_MAIN)) return false;
    return true;
}

bool CAN_sensor_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    return core_CAN_add_message_to_tx_queue(CAN_SENSOR, id, dlc, data);
}

bool CAN_sensor_transmit_extended(uint32_t id, uint8_t dlc, uint8_t *data) {
    return core_CAN_add_extended_message_to_tx_queue(CAN_SENSOR, id, dlc, data);
}

bool CAN_sensor_transmit_task() {
    return core_CAN_send_from_tx_queue_task(CAN_SENSOR);
}

bool CAN_main_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    return core_CAN_add_message_to_tx_queue(CAN_MAIN, id, dlc, data);
}

bool CAN_main_transmit_task() {
    return core_CAN_send_from_tx_queue_task(CAN_MAIN);
}

