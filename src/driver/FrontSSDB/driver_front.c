/**
  * @file   driver_front.c
  * @brief  Functions for initializing and collecting data on the front SSDB
  */
#include <stdint.h>
#include <stdbool.h>

#include "driver_front.h"
#include "CAN/driver_can.h"
#include "ssdb_config.h"
#include "adc.h"
#include "rtt.h"

#include "sensor_dbc.h"
#include "main_dbc.h"

static struct sensor_dbc_ssdb_suspension_front_t data_suspension;
static struct main_dbc_ssdb_front_t data_front;
static uint64_t can_data;
static uint8_t dlc;

/**
  * @brief  Initialize the peripherals required for the front SSDB
  * @retval 1 if initialization succeeded
  * @retval 0 otherwise
  */
bool SSDB_front_init() {
    core_ADC_init(ADC2);
    core_ADC_init(ADC5);
    core_ADC_setup_pin(SSDB_FRONT_LEFT_PORT, SSDB_FRONT_LEFT_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_RIGHT_PORT, SSDB_FRONT_RIGHT_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_BRAKE_PORT, SSDB_FRONT_BRAKE_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_STEERING_PORT, SSDB_FRONT_STEERING_PIN, 1);
    return true;
}

/**
  * @brief  Collect sensor data on the front SSDB and transmit over CAN
  */
void SSDB_front_collect_sensors() {
    core_ADC_read_channel(SSDB_FRONT_BRAKE_PORT, SSDB_FRONT_BRAKE_PIN, &(data_front.ssdb_brake_pressure_front_raw));
    core_ADC_read_channel(SSDB_FRONT_STEERING_PORT, SSDB_FRONT_STEERING_PIN, &(data_front.ssdb_steering_angle_raw));
    dlc = main_dbc_ssdb_front_pack((uint8_t*)(&can_data), &data_front, 8);
    CAN_main_transmit(MAIN_DBC_SSDB_FRONT_FRAME_ID, dlc, can_data);

    core_ADC_read_channel(SSDB_FRONT_LEFT_PORT, SSDB_FRONT_LEFT_PIN, &(data_suspension.ssdb_suspension_fl));
    core_ADC_read_channel(SSDB_FRONT_RIGHT_PORT, SSDB_FRONT_RIGHT_PIN, &(data_suspension.ssdb_suspension_fr));
    dlc = sensor_dbc_ssdb_suspension_front_pack((uint8_t*)(&can_data), &data_suspension, 8);
    CAN_sensor_transmit(SENSOR_DBC_SSDB_SUSPENSION_FRONT_FRAME_ID, dlc, can_data);

    rprintf("main %d, %d; sensor %d, %d\n", data_front.ssdb_steering_angle_raw, data_front.ssdb_brake_pressure_front_raw, data_suspension.ssdb_suspension_fr, data_suspension.ssdb_suspension_fl);
}
