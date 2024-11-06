#include <stdint.h>
#include <stdbool.h>

#include "driver_front.h"
#include "CAN/driver_can.h"
#include "ssdb_config.h"
#include "adc.h"

#include "formula_sensor_dbc.h"
#include "formula_main_dbc.h"

bool SSDB_front_init() {
    core_ADC_init(ADC2);
    core_ADC_setup_pin(SSDB_FRONT_LEFT_PORT, SSDB_FRONT_LEFT_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_RIGHT_PORT, SSDB_FRONT_RIGHT_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_BRAKE_PORT, SSDB_FRONT_BRAKE_PIN, 1);
    core_ADC_setup_pin(SSDB_FRONT_STEERING_PORT, SSDB_FRONT_STEERING_PIN, 1);
    return true;
}

void SSDB_front_collect_sensors() {
    uint64_t data = 0x0000;
    uint8_t dlc = 2;

    struct formula_sensor_dbc_ssdb_suspension_fl_m_t data_fl;
    core_ADC_read_channel(SSDB_FRONT_LEFT_PORT, SSDB_FRONT_LEFT_PIN, &(data_fl.ssdb_suspension_fl));
    dlc = formula_sensor_dbc_ssdb_suspension_fl_m_pack((uint8_t*)(&data), &data_fl, 8);
    CAN_sensor_transmit(FORMULA_SENSOR_DBC_SSDB_SUSPENSION_FL_M_FRAME_ID, dlc, data);

    struct formula_sensor_dbc_ssdb_suspension_fr_m_t data_fr;
    core_ADC_read_channel(SSDB_FRONT_RIGHT_PORT, SSDB_FRONT_RIGHT_PIN, &(data_fr.ssdb_suspension_fr));
    dlc = formula_sensor_dbc_ssdb_suspension_fr_m_pack((uint8_t*)(&data), &data_fr, 8);
    CAN_sensor_transmit(FORMULA_SENSOR_DBC_SSDB_SUSPENSION_FR_M_FRAME_ID, dlc, data);
    
    struct formula_main_dbc_ssdb_brake_pressure_m_t data_brake;
    core_ADC_read_channel(SSDB_FRONT_BRAKE_PORT, SSDB_FRONT_BRAKE_PIN, &(data_brake.ssdb_brake_pressure));
    dlc = formula_main_dbc_ssdb_brake_pressure_m_pack((uint8_t*)(&data), &data_brake, 8);
    CAN_main_transmit(FORMULA_MAIN_DBC_SSDB_BRAKE_PRESSURE_M_FRAME_ID, dlc, data);
    
    struct formula_main_dbc_ssdb_steering_angle_m_t data_steer;
    core_ADC_read_channel(SSDB_FRONT_STEERING_PORT, SSDB_FRONT_STEERING_PIN, &(data_steer.ssdb_steering_angle));
    dlc = formula_main_dbc_ssdb_steering_angle_m_pack((uint8_t*)(&data), &data_steer, 8);
    CAN_main_transmit(FORMULA_MAIN_DBC_SSDB_STEERING_ANGLE_M_FRAME_ID, dlc, data);
}
