/**
  * @file   driver_imu.c
  * @brief  Generic functions for parsing data from a VectorNAV VN300 IMU
  *
  * The VectorNAV can be configured to output data from any combination of
  * seven groups. Each group consists of a number of fields which are 
  * arranged into 'types'. This three-level hierarchy allows the user freedom
  * to select which fields are to be output over the UART.
  *
  * <table>
  * <tr>
  *   <th>Group</th>
  *   <th>Type</th>
  *   <th>Bit</th>
  *   <th>Field</th>
  *   <th>Type</th>
  *   <th>Units</th>
  *   <th>Description</th>
  * </tr>
  * <tr>
  * <td rowspan="16">time</td>
  * <td rowspan="1">TimeStartup</td><td rowspan="1">0</td>
  * <td>TimeStartup</td><td>uint64_t</td><td>ns</td><td>Time since start-up, based on internal TCXO of the microcontroller (20 ppm accuracy).</td></tr><tr>
  * <td rowspan="1">TimeGps</td><td rowspan="1">1</td>
  * <td>TimeGps</td><td>uint64_t</td><td>ns</td><td>Absolute Gps time (since 1980).</td></tr><tr>
  * <td rowspan="1">GpsTow</td><td rowspan="1">2</td>
  * <td>GpsTow</td><td>uint64_t</td><td>ns</td><td>GPS time of week.</td></tr><tr>
  * <td rowspan="1">GpsWeek</td><td rowspan="1">3</td>
  * <td>GpsWeek</td><td>uint16_t</td><td>-</td><td>The current GPS week.</td></tr><tr>
  * <td rowspan="1">TimeSyncIn</td><td rowspan="1">4</td>
  * <td>TimeSyncIn</td><td>uint64_t</td><td>ns</td><td>Time since last SyncIn trigger.</td></tr><tr>
  * <td rowspan="1">TimeGpsPps</td><td rowspan="1">5</td>
  * <td>TimeGpsPps</td><td>uint64_i</td><td>ns</td><td>Time since last Gps PPS trigger.</td></tr><tr>
  * <td rowspan="7">TimeUtc</td><td rowspan="7">6</td>
  * <td>Year</td><td>int8_i</td><td>yr</td><td>Number of years before or after 2000.</td></tr><tr>
  * <td>Month</td><td>uint8_i</td><td>mo</td><td>Month of the year [1 to 12].</td></tr><tr>
  * <td>Day</td><td>uint8_i</td><td>d</td><td>Day of the month [1 to 31].</td></tr><tr>
  * <td>Hour</td><td>uint8_i</td><td>hr</td><td>Hour of the day [0 to 23].</td></tr><tr>
  * <td>Minute</td><td>uint8_i</td><td>min</td><td>Minute of the hour [0 to 59].</td></tr><tr>
  * <td>Second</td><td>uint8_i</td><td>s</td><td>Second of the minute [0 to 59].</td></tr><tr>
  * <td>FracSec</td><td>uint16_i</td><td>ms</td><td>Fraction of a second [0 to 999].</td></tr><tr>
  * <td rowspan="1">SyncInCnt</td><td rowspan="1">7</td>
  * <td>SyncInCnt</td><td>uint32_t</td><td>-</td><td>Number of SyncIn trigger events.</td></tr><tr>
  * <td rowspan="1">SyncOutCnt</td><td rowspan="1">8</td>
  * <td>SyncOutCnt</td><td>uint32_t</td><td>-</td><td>Number of SyncOut trigger events.</td></tr><tr>
  * <td rowspan="1">TimeStatus</td><td rowspan="1">9</td>
  * <td>TimeStatus</td><td>uint8</td><td>BITS</td><td>Time validity status.</td></tr><tr>
  * <td rowspan="28">imu</td>
  * <td rowspan="3">UncompMag</td><td rowspan="3">1</td>
  * <td>UncompMagX</td><td>float</td><td>G</td><td>Magnetometer body-frame x-axis.</td></tr><tr>
  * <td>UncompMagY</td><td>float</td><td>G</td><td>Magnetometer body-frame y-axis.</td></tr><tr>
  * <td>UncompMagZ</td><td>float</td><td>G</td><td>Magnetometer body-frame z-axis.</td></tr><tr>
  * <td rowspan="3">UncompAccel</td><td rowspan="3">2</td>
  * <td>UncompAccX</td><td>float</td><td>m/s2</td><td>Accelerometer body-frame x-axis.</td></tr><tr>
  * <td>UncompAccY</td><td>float</td><td>m/s2</td><td>Accelerometer body-frame y-axis.</td></tr><tr>
  * <td>UncompAccZ</td><td>float</td><td>m/s2</td><td>Accelerometer body-frame z-axis.</td></tr><tr>
  * <td rowspan="3">UncompGyro</td><td rowspan="3">3</td>
  * <td>UncompGyroX</td><td>float</td><td>rad/s</td><td>Angular rate body-frame x-axis.</td></tr><tr>
  * <td>UncompGyroY</td><td>float</td><td>rad/s</td><td>Angular rate body-frame y-axis.</td></tr><tr>
  * <td>UncompGyroZ</td><td>float</td><td>rad/s</td><td>Angular rate body-frame z-axis.</td></tr><tr>
  * <td rowspan="1">Temperature</td><td rowspan="1">4</td>
  * <td>Temperature</td><td>float</td><td>&deg;C</td><td>Sensor temperature.</td></tr><tr>
  * <td rowspan="1">Pressure</td><td rowspan="1">5</td>
  * <td>Pressure</td><td>float</td><td>kPa</td><td>Barometric pressure.</td></tr><tr>
  * <td rowspan="4">DeltaTheta</td><td rowspan="4">6</td>
  * <td>DeltaTime</td><td>float</td><td>s</td><td>Duration of integration interval.</td></tr><tr>
  * <td>DeltaThetaX</td><td>float</td><td>deg</td><td>Integrated rotation vector x-axis.</td></tr><tr>
  * <td>DeltaThetaY</td><td>float</td><td>deg</td><td>Integrated rotation vector y-axis.</td></tr><tr>
  * <td>DeltaThetaZ</td><td>float</td><td>deg</td><td>Integrated rotation vector z-axis.</td></tr><tr>
  * <td rowspan="3">DeltaVel</td><td rowspan="3">7</td>
  * <td>DeltaVelX</td><td>float</td><td>m/s</td><td>Integrated velocity x-axis.</td></tr><tr>
  * <td>DeltaVelY</td><td>float</td><td>m/s</td><td>Integrated velocity y-axis.</td></tr><tr>
  * <td>DeltaVelZ</td><td>float</td><td>m/s</td><td>Integrated velocity z-axis.</td></tr><tr>
  * <td rowspan="3">Mag</td><td rowspan="3">8</td>
  * <td>MagX</td><td>float</td><td>G</td><td>Compensated magnetometer measurement in the body-frame x-axis.</td></tr><tr>
  * <td>MagY</td><td>float</td><td>G</td><td>Compensated magnetometer measurement in the body-frame y-axis.</td></tr><tr>
  * <td>MagZ</td><td>float</td><td>G</td><td>Compensated magnetometer measurement in the body-frame z-axis.</td></tr><tr>
  * <td rowspan="3">Accel</td><td rowspan="3">9</td>
  * <td>AccelX</td><td>float</td><td>m/s2</td><td>Compensated accelerometer measurement in the body-frame x-axis.</td></tr><tr>
  * <td>AccelY</td><td>float</td><td>m/s2</td><td>Compensated accelerometer measurement in the body-frame y-axis.</td></tr><tr>
  * <td>AccelZ</td><td>float</td><td>m/s2</td><td>Compensated accelerometer measurement in the body-frame z-axis.</td></tr><tr>
  * <td rowspan="3">AngularRate</td><td rowspan="3">10</td>
  * <td>GyroX</td><td>float</td><td>rad/s</td><td>Compensated angular rate measurement in the body-frame x-axis.</td></tr><tr>
  * <td>GyroY</td><td>float</td><td>rad/s</td><td>Compensated angular rate measurement in the body-frame y-axis.</td></tr><tr>
  * <td>GyroZ</td><td>float</td><td>rad/s</td><td>Compensated angular rate measurement in the body-frame z-axis.</td></tr><tr>
  * <td rowspan="1">SensSat</td><td rowspan="1">11</td>
  * <td>SensSat</td><td>uint16_t</td><td>BITS</td><td>Sensor saturation flags.</td></tr><tr>
  * <td rowspan="36">gnss</td>
  * <td rowspan="7">TimeUtc</td><td rowspan="7">0</td>
  * <td>Year</td><td>int8_t</td><td>yr</td><td>Number of years before or after 2000.</td></tr><tr>
  * <td>Month</td><td>uint8_t</td><td>mo</td><td>Month of the year [1 to 12].</td></tr><tr>
  * <td>Day</td><td>uint8_t</td><td>d</td><td>Day of the month [1 to 31].</td></tr><tr>
  * <td>Hour</td><td>uint8_t</td><td>hr</td><td>Hour of the day [0 to 23].</td></tr><tr>
  * <td>Minute</td><td>uint8_t</td><td>min</td><td>Minute of the hour [0 to 59].</td></tr><tr>
  * <td>Second</td><td>uint8_t</td><td>s</td><td>Second of the minute [0 to 59].</td></tr><tr>
  * <td>FracSec</td><td>int16_t</td><td>ms</td><td>Fraction of a second. Typically positive, but may be negative.</td></tr><tr>
  * <td rowspan="1">GpsTow</td><td rowspan="1">1</td>
  * <td>GpsTow</td><td>uint64_t</td><td>ns</td><td>GPS time of week.</td></tr><tr>
  * <td rowspan="1">GpsWeek</td><td rowspan="1">2</td>
  * <td>GpsWeek</td><td>uint16_t</td><td>-</td><td>GPS week.</td></tr><tr>
  * <td rowspan="1">NumSats</td><td rowspan="1">3</td>
  * <td>NumSats</td><td>uint8_t</td><td>-</td><td>Number of satellites tracked by GNSS receiver.</td></tr><tr>
  * <td rowspan="1">GnssFix</td><td rowspan="1">4</td>
  * <td>GnssFix</td><td>uint8_t</td><td>ENUM</td><td>Type of GNSS fix.</td></tr><tr>
  * <td rowspan="3">GnssPosLla</td><td rowspan="3">5</td>
  * <td>Lat</td><td>double</td><td>deg</td><td>GNSS geodetic latitude.</td></tr><tr>
  * <td>Lon</td><td>double</td><td>deg</td><td>GNSS longitude.</td></tr><tr>
  * <td>Alt</td><td>double</td><td>m</td><td>GNSS altitude above WGS84 ellipsoid.</td></tr><tr>
  * <td rowspan="2">GnssPosEcef</td><td rowspan="2">6</td>
  * <td>PosX</td><td>double</td><td>m</td><td>GNSS position in ECEF-frame x-axis.</td></tr><tr>
  * <td>PosY</td><td>double</td><td>m</td><td>GNSS position in ECEF-frame y-axis.</td></tr><tr>
  * <td rowspan="3">GnssVelNed</td><td rowspan="3">7</td>
  * <td>VelN</td><td>float</td><td>m/s</td><td>GNSS velocity in North direction.</td></tr><tr>
  * <td>VelE</td><td>float</td><td>m/s</td><td>GNSS velocity in East direction.</td></tr><tr>
  * <td>VelD</td><td>float</td><td>m/s</td><td>GNSS velocity in Down direction.</td></tr><tr>
  * <td rowspan="3">GnssVelEcef</td><td rowspan="3">8</td>
  * <td>VelX</td><td>float</td><td>m/s</td><td>GNSS velocity in ECEF-frame x-axis.</td></tr><tr>
  * <td>VelY</td><td>float</td><td>m/s</td><td>GNSS velocity in ECEF-frame y-axis.</td></tr><tr>
  * <td>VelZ</td><td>float</td><td>m/s</td><td>GNSS velocity in ECEF-frame z-axis.</td></tr><tr>
  * <td rowspan="3">GnssPosUncertainty</td><td rowspan="3">9</td>
  * <td>PosUncertaintyN</td><td>float</td><td>m</td><td>GNSS position uncertainty, North direction.</td></tr><tr>
  * <td>PosUncertaintyE</td><td>float</td><td>m</td><td>GNSS position uncertainty, East direction.</td></tr><tr>
  * <td>PosUncertaintyD</td><td>float</td><td>m</td><td>GNSS position uncertainty, Down direction.</td></tr><tr>
  * <td rowspan="1">GnssVelUncertainty</td><td rowspan="1">10</td>
  * <td>GnssVelUncertainty</td><td>float</td><td>m/s</td><td>GNSS velocity uncertainty (scalar).</td></tr><tr>
  * <td rowspan="1">GnssTimeUncertainty</td><td rowspan="1">11</td>
  * <td>GnssTimeUncertainty</td><td>float</td><td>s</td><td>GNSS time uncertainty.</td></tr><tr>
  * <td rowspan="2">GnssTimeInfo</td><td rowspan="2">12</td>
  * <td>GnssTimeStatus</td><td>uint8_t</td><td>–</td><td>GNSS time validity status. Same as TimeStatus in Time Group</td></tr><tr>
  * <td>LeapSeconds</td><td>int8_t</td><td>s</td><td>Current number of leap seconds.</td></tr><tr>
  * <td rowspan="7">GnssDop</td><td rowspan="7">13</td>
  * <td>Gdop</td><td>float</td><td>-</td><td>Dilution of precision.</td></tr><tr>
  * <td>Pdop</td><td>float</td><td>-</td><td>Position dilution of precision.</td></tr><tr>
  * <td>Tdop</td><td>float</td><td>-</td><td>Time dilution of precision.</td></tr><tr>
  * <td>Vdop</td><td>float</td><td>-</td><td>Vertical dilution of precision.</td></tr><tr>
  * <td>Hdop</td><td>float</td><td>-</td><td>Horizontal dilution of precision.</td></tr><tr>
  * <td>Ndop</td><td>float</td><td>-</td><td>North dilution of precision.</td></tr><tr>
  * <td>Edop</td><td>float</td><td>-</td><td>East dilution of precision.</td></tr><tr>
  * </table>
  *  - `common`
  *  - `time`
  *  - `imu`
  *  - `gnss`
  *  - `attitude`
  *  - `ins`
  *  - `gnss2`
  *
  */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "IMU/driver_imu.h"

static const uint32_t imu_group_sizes[] = {0, 0, 12, 0, 0, 11, 0};

static const uint32_t imu_offsets[] = {
    // common
    // time
    // imu
    0, MEMBER_OFFSET(UncompMagX), MEMBER_OFFSET(UncompAccelX), MEMBER_OFFSET(UncompGyroX), MEMBER_OFFSET(Temperature), MEMBER_OFFSET(Pressure), MEMBER_OFFSET(DeltaThetaT), MEMBER_OFFSET(DeltaVelX), MEMBER_OFFSET(MagX), MEMBER_OFFSET(AccelX), MEMBER_OFFSET(AngularRateX), MEMBER_OFFSET(SensSat),
    // gnss
    // attitude
    // ins
    MEMBER_OFFSET(InsStatus), MEMBER_OFFSET(PosLlaL), MEMBER_OFFSET(PosEcefX), MEMBER_OFFSET(VelBodyX), MEMBER_OFFSET(VelNedN), MEMBER_OFFSET(VelEcefX), MEMBER_OFFSET(MagEcefX), MEMBER_OFFSET(AccelEcefX), MEMBER_OFFSET(LinAccelEcefX), MEMBER_OFFSET(PosU), MEMBER_OFFSET(VelU),
    // gnss2
};

static const uint32_t imu_lengths[] = {
    // common
    // time
    // imu
    0, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2,
    // gnss
    // attitude
    // ins
    2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4,
    // gnss2
};

/**
  * @brief  Parse IMU data from a byte array
  * @param  buf Pointer to the buffer where received IMU data is stored
  * @param  buflen Length of the byte array in buf
  * @param  data Pointer to an imu_result_t struct where the parsed data is stored
  * @retval 0 if there was an error while parsing
  * @retval 1 otherwise
  */
bool imu_parse(uint8_t *buf, uint16_t buflen, imu_result_t *data) {
    if (buf[0] != 0xFA) return false;
    // mask_ptr points to the mask word associated with the group currently
    // being processed. 
    uint16_t mask_ptr = 2;
    uint16_t ptr = 2;
    // Scroll ptr ahead to the first data byte
    for (uint8_t i=0; i < 7; i++) {
        if (buf[1] & (1<<i)) ptr += 2;
    }
    uint16_t lut_idx = 0;
    uint16_t mask;
    // Iterate over all of the groups
    for (uint8_t i=0; i < 7; i++) {
        // Check if the group is present in the data by checking the
        // corresponding bit in the group mask byte (buf[1])
        if (buf[1] & (1<<i)) {
            // Extract the element mask word
            mask = *((uint16_t*)(buf + mask_ptr));
            // Iterate over all of the elements in the group and process them
            // if the corresponding bit in the group's element mask word is set
            for (uint8_t j=0; j < imu_group_sizes[i]; j++) {
                if (mask & (1<<j)) {
                    memcpy(((uint8_t*)data) + imu_offsets[lut_idx + j], buf+ptr, imu_lengths[lut_idx + j]);
                    ptr += imu_lengths[lut_idx + j];
                }
            }
            mask_ptr += 2;
        }
        lut_idx += imu_group_sizes[i];
    }
    return true;
}

