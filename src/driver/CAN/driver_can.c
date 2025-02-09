#include "can.h"
#include "driver_can.h"
#include "ssdb_config.h"
#include <stdint.h>
#include <stdbool.h>

#define WAIT_TX(can) while ((can->PSR & 0x18) == 0x18)

bool CAN_init() {
    if (!core_CAN_init(CAN_SENSOR)) return false;
    if (!core_CAN_init(CAN_MAIN)) return false;
    return true;
}

bool CAN_sensor_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    if (!core_CAN_send_message(CAN_SENSOR, id, dlc, data)) return false;
    WAIT_TX(CAN_SENSOR);
    return true;
}

bool CAN_sensor_transmit_extended(uint32_t id, uint8_t dlc, uint8_t *data) {
    if (!core_CAN_send_fd_message(CAN_SENSOR, id, dlc, data)) return false;
    WAIT_TX(CAN_SENSOR);
    return true;
}

bool CAN_main_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    if (!core_CAN_send_message(CAN_MAIN, id, dlc, data)) return false;
    WAIT_TX(CAN_MAIN);
    return true;
}

