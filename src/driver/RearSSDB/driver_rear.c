/**
  * @file   driver_rear.c
  * @brief  Functions for initializing and collecting data on the rear SSDB
  */
#include <stdint.h>
#include <stdbool.h>

#include "driver_rear.h"
#include "CAN/driver_can.h"
#include "IMU/driver_imu.h"
#include "ssdb_config.h"
#include "adc.h"
#include "usart.h"

#include "sensor_dbc.h"
#include "main_dbc.h"

static uint8_t can_imubuf[64];
static uint32_t imubuflen = 0;
static imu_result_t parsed_imu_data;
static struct sensor_dbc_vector_nav_t data_imu;
static struct main_dbc_ssdb_vector_nav6_t data_imu_velocity;

static struct sensor_dbc_ssdb_suspension_rear_t data_suspension;
static uint64_t can_data;
static uint8_t dlc;

#define WAIT_TX(can) while ((can->PSR & 0x18) == 0x18)

void SSDB_USART_callback(uint8_t *rxbuf, uint32_t rxbuflen) {
    uint8_t dlc = 2;
    if (imu_parse(rxbuf, rxbuflen, &parsed_imu_data)) {
        /*data_imu.vector_nav_accel_x = parsed_imu_data.AccelX;
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
        data_imu.vector_nav_vel_ned_d = parsed_imu_data.VelNedD;*/

        data_imu_velocity.vector_nav_vel_ned_n = parsed_imu_data.VelNedN;
        data_imu_velocity.vector_nav_vel_ned_e = parsed_imu_data.VelNedE;
        dlc = main_dbc_ssdb_vector_nav6_pack(&can_imubuf, &data_imu_velocity, 8);
        WAIT_TX(CAN_MAIN);
        CAN_main_transmit(MAIN_DBC_SSDB_VECTOR_NAV6_FRAME_ID, dlc, &can_imubuf);

        /*dlc = sensor_dbc_vector_nav_pack(&can_imubuf, &data_imu, 64);
        CAN_sensor_transmit_extended(SENSOR_DBC_VECTOR_NAV_FRAME_ID, dlc, &can_imubuf);*/

    }
}

bool SSDB_rear_init() {
    core_ADC_init(ADC1);
    core_ADC_init(ADC2);
    core_ADC_setup_pin(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, 1);
    core_ADC_setup_pin(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, 1);
    core_USART_init(USART3, 921600);
    //core_USART_start_rx(USART3, imubuf, &imubuflen);
    core_USART_register_callback(USART3, &SSDB_USART_callback);
    return true;
}

void SSDB_rear_collect_sensors() {
    uint64_t data = 0x0000;
    uint8_t dlc = 2;

    core_ADC_read_channel(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, &(data_suspension.ssdb_suspension_rl));
    core_ADC_read_channel(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, &(data_suspension.ssdb_suspension_rr));
    dlc = sensor_dbc_ssdb_suspension_rear_pack((uint8_t*)(&can_data), &data_suspension, 8);
    CAN_sensor_transmit(SENSOR_DBC_SSDB_SUSPENSION_REAR_FRAME_ID, dlc, can_data);
}
