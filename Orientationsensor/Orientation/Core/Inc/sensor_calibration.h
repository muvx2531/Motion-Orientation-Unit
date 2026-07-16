#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cal_storage.h"
#include "dps310.h"
#include "lis3mdl.h"
#include "mpu6500.h"
#include <stdint.h>

typedef enum
{
  SENSOR_CAL_OK = 0,
  SENSOR_CAL_ERROR = 1,
  SENSOR_CAL_ERROR_INVALID_ARG = 2,
  SENSOR_CAL_ERROR_SENSOR = 3,
  SENSOR_CAL_ERROR_NOT_ENOUGH_SAMPLES = 4,
  SENSOR_CAL_ERROR_BAD_DATA = 5
} sensor_cal_status_t;

typedef enum
{
  SENSOR_CAL_ACCEL_POS_X = 0,
  SENSOR_CAL_ACCEL_NEG_X,
  SENSOR_CAL_ACCEL_POS_Y,
  SENSOR_CAL_ACCEL_NEG_Y,
  SENSOR_CAL_ACCEL_POS_Z,
  SENSOR_CAL_ACCEL_NEG_Z,
  SENSOR_CAL_ACCEL_POSITION_COUNT
} sensor_cal_accel_position_t;

typedef struct
{
  float x;
  float y;
  float z;
} sensor_cal_vector_t;

typedef struct
{
  sensor_cal_vector_t mean[SENSOR_CAL_ACCEL_POSITION_COUNT];
  uint8_t valid[SENSOR_CAL_ACCEL_POSITION_COUNT];
} sensor_cal_accel_six_position_t;

typedef struct
{
  sensor_cal_vector_t min_gauss;
  sensor_cal_vector_t max_gauss;
  uint32_t sample_count;
} sensor_cal_mag_minmax_t;

sensor_cal_status_t sensor_cal_mpu6500_gyro_zero(mpu6500_t *imu,
                                                 uint32_t sample_count,
                                                 uint32_t sample_delay_ms,
                                                 cal_mpu6500_t *calibration);
sensor_cal_status_t sensor_cal_mpu6500_accel_collect_position(mpu6500_t *imu,
                                                              sensor_cal_accel_position_t position,
                                                              uint32_t sample_count,
                                                              uint32_t sample_delay_ms,
                                                              sensor_cal_accel_six_position_t *accel_data);
sensor_cal_status_t sensor_cal_mpu6500_accel_compute_six_position(const sensor_cal_accel_six_position_t *accel_data,
                                                                  float accel_lsb_per_g,
                                                                  cal_mpu6500_t *calibration);
sensor_cal_status_t sensor_cal_mpu6500_accel_single_position(mpu6500_t *imu,
                                                             const sensor_cal_vector_t *expected_g,
                                                             uint32_t sample_count,
                                                             uint32_t sample_delay_ms,
                                                             cal_mpu6500_t *calibration);
void sensor_cal_accel_six_position_clear(sensor_cal_accel_six_position_t *accel_data);

void sensor_cal_lis3mdl_mag_minmax_start(sensor_cal_mag_minmax_t *state);
sensor_cal_status_t sensor_cal_lis3mdl_mag_minmax_update(lis3mdl_t *mag,
                                                         sensor_cal_mag_minmax_t *state);
sensor_cal_status_t sensor_cal_lis3mdl_mag_minmax_finish(const sensor_cal_mag_minmax_t *state,
                                                         cal_lis3mdl_t *calibration);

sensor_cal_status_t sensor_cal_dps310_reference_current(dps310_t *prs,
                                                        const dps310_calibration_t *dps_calibration,
                                                        dps310_oversampling_t pressure_oversampling,
                                                        dps310_oversampling_t temperature_oversampling,
                                                        float reference_altitude_m,
                                                        cal_dps310_t *calibration);

void sensor_cal_apply_mpu6500_accel_raw(const cal_mpu6500_t *calibration,
                                        const mpu6500_vector_raw_t *raw,
                                        sensor_cal_vector_t *corrected_raw);
void sensor_cal_apply_mpu6500_gyro_raw(const cal_mpu6500_t *calibration,
                                       const mpu6500_vector_raw_t *raw,
                                       sensor_cal_vector_t *corrected_raw);
void sensor_cal_apply_lis3mdl_gauss(const cal_lis3mdl_t *calibration,
                                    const lis3mdl_vector_t *raw_gauss,
                                    lis3mdl_vector_t *corrected_gauss);

#ifdef __cplusplus
}
#endif

#endif
