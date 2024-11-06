#include <stdint.h>
#include <stdbool.h>

#include "driver_rear.h"
#include "CAN/driver_can.h"
#include "IMU/driver_imu.h"
#include "ssdb_config.h"
#include "adc.h"
#include "usart.h"

#include "formula_sensor_dbc.h"

static uint8_t imubuf[512];
static uint8_t can_imubuf[64];
static uint32_t imubuflen = 0;
static imu_result_t parsed_imu_data;

bool SSDB_rear_init() {
    core_ADC_init(ADC1);
    core_ADC_init(ADC2);
    core_ADC_setup_pin(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, 1);
    core_ADC_setup_pin(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, 1);
    core_USART_init(USART3, 500000);
    core_USART_start_rx(USART3, imubuf, &imubuflen);
    return true;
}

void SSDB_rear_collect_sensors() {
    uint64_t data = 0x0000;
    uint8_t dlc = 2;

    struct formula_sensor_dbc_ssdb_suspension_rl_m_t data_rl;
    core_ADC_read_channel(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, &(data_rl.ssdb_suspension_rl));
    dlc = formula_sensor_dbc_ssdb_suspension_rl_m_pack((uint8_t*)(&data), &data_rl, 8);
    CAN_sensor_transmit(FORMULA_SENSOR_DBC_SSDB_SUSPENSION_RL_M_FRAME_ID, dlc, data);

    struct formula_sensor_dbc_ssdb_suspension_rr_m_t data_rr;
    core_ADC_read_channel(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, &(data_rr.ssdb_suspension_rr));
    dlc = formula_sensor_dbc_ssdb_suspension_rr_m_pack((uint8_t*)(&data), &data_rr, 8);
    CAN_sensor_transmit(FORMULA_SENSOR_DBC_SSDB_SUSPENSION_RL_M_FRAME_ID, dlc, data);

    if (imubuflen) {
        struct formula_sensor_dbc_vector_nav_t data_imu;
        core_USART_update_disable(USART3);
        if (imu_parse(imubuf, &parsed_imu_data)) {
            data_imu.vector_nav_accel_x = parsed_imu_data.AccelX;
            data_imu.vector_nav_accel_y = parsed_imu_data.AccelY;
            data_imu.vector_nav_accel_z = parsed_imu_data.AccelZ;
            data_imu.vector_nav_angular_rate_x = parsed_imu_data.AngularRateX;
            data_imu.vector_nav_angular_rate_y = parsed_imu_data.AngularRateY;
            data_imu.vector_nav_angular_rate_z = parsed_imu_data.AngularRateZ;
            data_imu.vector_nav_pos_lla_l = parsed_imu_data.PosLlaL;
            data_imu.vector_nav_pos_lla_o = parsed_imu_data.PosLlaO;
            data_imu.vector_nav_pos_lla_a = parsed_imu_data.PosLlaA;
            data_imu.vector_nav_vel_ned_n = parsed_imu_data.VelNedN;
            data_imu.vector_nav_vel_ned_e = parsed_imu_data.VelNedE;
            data_imu.vector_nav_vel_ned_d = parsed_imu_data.VelNedD;
            dlc = formula_sensor_dbc_vector_nav_pack(can_imubuf, &data_imu, 64);
            CAN_sensor_transmit_extended(FORMULA_SENSOR_DBC_VECTOR_NAV_FRAME_ID, dlc, can_imubuf);
        }
        imubuflen = 0;
        core_USART_update_enable(USART3);
    }
}
