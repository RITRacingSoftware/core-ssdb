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

bool imu_parse(uint8_t *buf, imu_result_t *data) {
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

