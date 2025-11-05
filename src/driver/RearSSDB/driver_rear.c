/**
  * @file   driver_rear.c
  * @brief  Functions for initializing and collecting data on the rear SSDB
  */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "driver_rear.h"
#include "CAN/driver_can.h"
#include "IMU/driver_imu.h"
#include "ssdb_config.h"
#include "adc.h"
#include "usart.h"
#include "rtt.h"

#include "sensor_dbc.h"
#include "main_dbc.h"

/*

BO_ 400 VectorNav0: 8 VectorNav
 SG_ VectorNav_AccelX : 0|32@1+ (1,0) [0|0] "m/s^2" Vector__XXX
 SG_ VectorNav_AccelY : 32|32@1+ (1,0) [0|0] "m/s^2" Vector__XXX

BO_ 401 VectorNav1: 8 VectorNav
 SG_ VectorNav_AccelZ : 0|32@1+ (1,0) [0|0] "m/s^2" Vector__XXX
 SG_ VectorNav_AngularRateX : 32|32@1+ (1,0) [0|0] "rad/s" Vector__XXX

BO_ 402 VectorNav2: 8 VectorNav
 SG_ VectorNav_AngularRateY : 0|32@1+ (1,0) [0|0] "rad/s" Vector__XXX
 SG_ VectorNav_AngularRateZ : 32|32@1+ (1,0) [0|0] "rad/s" Vector__XXX

BO_ 403 VectorNav3: 8 VectorNav
 SG_ VectorNav_PosLlaL : 0|64@1+ (1,0) [0|0] "deg" Vector__XXX

BO_ 404 VectorNav4: 8 VectorNav
 SG_ VectorNav_PosLlaO : 0|64@1+ (1,0) [0|0] "deg" Vector__XXX

BO_ 405 VectorNav5: 8 VectorNav
 SG_ VectorNav_PosLlaA : 0|64@1+ (1,0) [0|0] "m" Vector__XXX

BO_ 406 VectorNav6: 8 VectorNav
 SG_ VectorNav_VelNedN : 0|32@1+ (1,0) [0|0] "m/s" Vector__XXX
 SG_ VectorNav_VelNedE : 32|32@1+ (1,0) [0|0] "m/s" Vector__XXX

BO_ 407 VectorNav7: 4 VectorNav
 SG_ VectorNav_VelNedD : 0|32@1+ (1,0) [0|0] "m/s" Vector__XXX
*/

static uint8_t __attribute__((aligned(8))) can_imubuf[64];
//static uint32_t imubuflen = 0;
static imu_result_t parsed_imu_data;
//static struct sensor_dbc_vector_nav_t data_imu;
static struct main_dbc_vector_nav0_t data_imu_msg0;
static struct main_dbc_vector_nav2_t data_imu_msg2;
static struct main_dbc_vector_nav6_t data_imu_msg6;

// static struct sensor_dbc_vector_nav0_t data_imu_msg0;
static struct sensor_dbc_vector_nav1_t data_imu_msg1;
// static struct sensor_dbc_vector_nav2_t data_imu_msg2;
static struct sensor_dbc_vector_nav3_t data_imu_msg3;
static struct sensor_dbc_vector_nav4_t data_imu_msg4;
static struct sensor_dbc_vector_nav5_t data_imu_msg5;
// static struct sensor_dbc_vector_nav6_t data_imu_msg6;
static struct sensor_dbc_vector_nav7_t data_imu_msg7;
static struct sensor_dbc_vector_nav8_t data_imu_msg8;
static struct sensor_dbc_vector_nav10_t data_imu_msg10;

static struct sensor_dbc_ssdb_suspension_rear_t data_suspension;
static uint64_t can_data;
static uint8_t dlc;

#define WAIT_TX(can) while ((can->PSR & 0x18) == 0x18)

uint8_t msg_counter = 0;
// vn_setup == 0: VectorNAV has not sent anything
// vn_setup == 1: VectorNAV has sent its first message and is awaiting config
// vn_setup == 2: VectorNAV is configured and data is being sent over CAN
uint8_t vn_setup = 0;

/**
  * @brief  VectorNAV UART receive timeout callback
  *
  * This function is called by the core library whenever a block of data is
  * received from the VectorNAV.
  *
  * @param  rxbuf Pointer to the location where the received data is stored
  * @param  rxbuflen Number of bytes received
  */
void SSDB_USART_callback(uint8_t *rxbuf, uint32_t rxbuflen) {

    /*
    for (int i = 0; i < rxbuflen; i++)
    {
        rprintf("%02x ", rxbuf[i]);
    }
    rprintf("\n\n\n");
    */

    uint8_t dlc = 2;
    if (rxbuf[0] == '$') {
        rxbuf[rxbuflen+1] = 0;
        rprintf("Received from VN: %s\n", rxbuf);
    } else if (vn_setup < 2) { 
        vn_setup = 1;
    } else if (imu_parse(rxbuf, rxbuflen, &parsed_imu_data)) {
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
        
        // X and Y accel to main bus
        data_imu_msg0.vector_nav_accel_x = parsed_imu_data.AccelX;
        data_imu_msg0.vector_nav_accel_y = parsed_imu_data.AccelY;
        dlc = main_dbc_vector_nav0_pack(can_imubuf, &data_imu_msg0, 8);
        WAIT_TX(CAN_MAIN);
        CAN_main_transmit(MAIN_DBC_VECTOR_NAV0_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Y and Z rate to main bus
        data_imu_msg2.vector_nav_angular_rate_y = parsed_imu_data.AngularRateY;
        data_imu_msg2.vector_nav_angular_rate_z = parsed_imu_data.AngularRateZ;
        dlc = main_dbc_vector_nav2_pack(can_imubuf, &data_imu_msg2, 8);
        WAIT_TX(CAN_MAIN);
        CAN_main_transmit(MAIN_DBC_VECTOR_NAV2_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Velocity data to main bus
        data_imu_msg6.vector_nav_vel_body_x = parsed_imu_data.VelBodyX;
        data_imu_msg6.vector_nav_vel_body_y = parsed_imu_data.VelBodyY;
        dlc = main_dbc_vector_nav6_pack(can_imubuf, &data_imu_msg6, 8);
        WAIT_TX(CAN_MAIN);
        CAN_main_transmit(MAIN_DBC_VECTOR_NAV6_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);  

        // Z accel and X rate to sensor bus
        data_imu_msg1.vector_nav_accel_z = parsed_imu_data.AccelZ;
        data_imu_msg1.vector_nav_angular_rate_x = parsed_imu_data.AngularRateX;
        dlc = sensor_dbc_vector_nav1_pack(can_imubuf, &data_imu_msg1, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV1_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Latitude to sensor bus
        data_imu_msg3.vector_nav_pos_lla_l = parsed_imu_data.PosLlaL;
        dlc = sensor_dbc_vector_nav3_pack(can_imubuf, &data_imu_msg3, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV3_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Longitude to sensor bus
        data_imu_msg4.vector_nav_pos_lla_o = parsed_imu_data.PosLlaO;
        dlc = sensor_dbc_vector_nav4_pack(can_imubuf, &data_imu_msg4, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV4_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Altitude to sensor bus
        data_imu_msg5.vector_nav_pos_lla_a = parsed_imu_data.PosLlaA;
        dlc = sensor_dbc_vector_nav5_pack(can_imubuf, &data_imu_msg5, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV5_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Z velocity and Y angle to sensor bus
        data_imu_msg7.vector_nav_vel_body_z = parsed_imu_data.VelBodyZ;
        data_imu_msg7.vector_nav_ypr_y = parsed_imu_data.YprY;
        dlc = sensor_dbc_vector_nav7_pack(can_imubuf, &data_imu_msg7, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV7_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // P and R angles to sensor bus
        data_imu_msg8.vector_nav_ypr_p = parsed_imu_data.YprP;
        data_imu_msg8.vector_nav_ypr_r = parsed_imu_data.YprR;
        dlc = sensor_dbc_vector_nav8_pack(can_imubuf, &data_imu_msg8, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV8_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        // Body slip angle to sensor bus
        if (parsed_imu_data.VelBodyX == 0 && parsed_imu_data.VelBodyY == 0) {
            data_imu_msg10.vector_nav_body_slip = 0;
        } else {
            data_imu_msg10.vector_nav_body_slip = atan2f(parsed_imu_data.VelBodyY, parsed_imu_data.VelBodyX);
        }
        data_imu_msg8.vector_nav_ypr_r = parsed_imu_data.YprR;
        dlc = sensor_dbc_vector_nav10_pack(can_imubuf, &data_imu_msg8, 8);
        WAIT_TX(CAN_SENSOR);
        CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV10_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);

        if (msg_counter == 0) {
            // Satellite info is only output every 100ms
            can_imubuf[0] = parsed_imu_data.NumSats1;
            can_imubuf[1] = parsed_imu_data.NumSats2;
            *((uint16_t*)(can_imubuf+2)) = parsed_imu_data.InsStatus;
            WAIT_TX(CAN_SENSOR);
            CAN_sensor_transmit(SENSOR_DBC_VECTOR_NAV9_FRAME_ID, dlc, ((uint64_t*)&can_imubuf)[0]);
        }
        msg_counter++;
        if (msg_counter >= 10) msg_counter = 0;

        /*dlc = sensor_dbc_vector_nav_pack(&can_imubuf, &data_imu, 64);
        CAN_sensor_transmit_extended(SENSOR_DBC_VECTOR_NAV_FRAME_ID, dlc, &can_imubuf);*/

    }
}

/**
  * @brief  Initialize the peripherals required for the rear SSDB
  * @retval 1 if initialization succeeded
  * @retval 0 otherwise
  */
bool SSDB_rear_init() {
    uint32_t i;
    core_ADC_init(ADC1);
    core_ADC_init(ADC2);
    core_ADC_setup_pin(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, 1);
    core_ADC_setup_pin(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, 1);
    core_USART_init(USART3, 921600);
    core_USART_register_callback(USART3, &SSDB_USART_callback);
    //core_USART_start_rx(USART3, imubuf, &imubuflen);
    
    //uprintf(USART3, "$VNWRG,26,-1.0,-0.0,-0.0,-0.0,1.0,-0.0,-0.0,-0.0,-1.0*71\n");
    //for (i=0; i < 100000; i++);
    //uprintf(USART3, "$VNWRG,57,-0.1278636,0,0.49977039999999995*41\n");
    //for (i=0; i < 100000; i++);
    //uprintf(USART3, "$VNWRG,93,1.5960598,00.000,-0.31562039999999997,0.0127,0.0127,0.0127*54\n");
    //for (i=0; i < 100000; i++);
    // [] indicates a group is enabled

    // common, [time], [IMU], [GNSS], [attitude], [INS], [GNSS2]

    //uprintf(USART3, "$VNASY,0*4F\n");
    //for (i=0; i < 100000; i++);
    //uprintf(USART3, "$VNRRG,75*71\n");
    //for (i=0; i < 100000; i++);
    /*uprintf(USART3, "$VNRRG,76*XX\n");
    for (i=0; i < 100000; i++);
    uprintf(USART3, "$VNRRG,77*XX\n");
    for (i=0; i < 100000; i++);*/
    rprintf("Initialized VectorNAV\n");
    //uprintf(USART3, "$VNWNV*57\n");
    //for (i=0; i < 100000; i++);

    return true;
}

/**
  * @brief  Collect sensor data on the rear SSDB and transmit over CAN
  */
void SSDB_rear_collect_sensors() {
    core_ADC_read_channel(SSDB_REAR_LEFT_PORT, SSDB_REAR_LEFT_PIN, &(data_suspension.ssdb_suspension_rl));
    core_ADC_read_channel(SSDB_REAR_RIGHT_PORT, SSDB_REAR_RIGHT_PIN, &(data_suspension.ssdb_suspension_rr));
    dlc = sensor_dbc_ssdb_suspension_rear_pack((uint8_t*)(&can_data), &data_suspension, 8);
    CAN_sensor_transmit(SENSOR_DBC_SSDB_SUSPENSION_REAR_FRAME_ID, dlc, can_data);
    if (vn_setup == 1) {
        rprintf("Sending config\n");
        uprintf(USART3, "$VNWRG,75,2,4,7E,0240,0600,0018,0002,000B,0018*XX\n");
        vn_setup = 2;
    }
}
