/**
  * @file   driver_can.c
  * @brief  CAN driver functions
  */

#include "can.h"
#include "driver_can.h"
#include "ssdb_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  Wait until an FDCAN module leaves the transmitting state
  */
#define WAIT_TX(can) while ((can->PSR & 0x18) == 0x18)

/**
  * @brief  Initialize the main and sensor CAN busses
  * @retval 1 if all FDCAN modules initialized successfully
  * @retval 0 otherwise
  */
bool CAN_init() {
    if (!core_CAN_init(CAN_SENSOR, 1000000)) return false;
    if (!core_CAN_init(CAN_MAIN, 1000000)) return false;
    return true;
}

/**
  * @brief  Transmit a message on the sensor bus and wait for transmission to finish
  * @param  id ID of the message
  * @param  dlc Length of the message being transmitted
  * @param  data Data, packed least significant byte first as a uint64_t
  * @retval 1 if the message was added to the hardware CAN queue successfully
  * @retval 0 otherwise
  */
bool CAN_sensor_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    if (!core_CAN_send_message(CAN_SENSOR, id, dlc, data)) return false;
    WAIT_TX(CAN_SENSOR);
    return true;
}

/**
  * @brief  Transmit an FDCAN extended message on the sensor bus and wait for 
  *         transmission to finish
  * @param  id ID of the message
  * @param  dlc Length of the message being transmitted. If dlc is not a valid
  *         data length for an extended CAN message, the length is rounded up.
  * @param  data Pointer to a byte array containing the data to be transmitted
  * @retval 1 if the message was added to the hardware CAN queue successfully
  * @retval 0 otherwise
  */
bool CAN_sensor_transmit_extended(uint32_t id, uint8_t dlc, uint8_t *data) {
    if (!core_CAN_send_fd_message(CAN_SENSOR, id, dlc, data)) return false;
    WAIT_TX(CAN_SENSOR);
    return true;
}

/**
  * @brief  Transmit a message on the main bus and wait for transmission to finish
  * @param  id ID of the message
  * @param  dlc Length of the message being transmitted
  * @param  data Data, packed least significant byte first as a uint64_t
  * @retval 1 if the message was added to the hardware CAN queue successfully
  * @retval 0 otherwise
  */
bool CAN_main_transmit(uint32_t id, uint8_t dlc, uint64_t data) {
    if (!core_CAN_send_message(CAN_MAIN, id, dlc, data)) return false;
    WAIT_TX(CAN_MAIN);
    return true;
}

