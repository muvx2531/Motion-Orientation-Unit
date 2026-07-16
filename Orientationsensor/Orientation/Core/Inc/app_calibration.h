#ifndef APP_CALIBRATION_H
#define APP_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cal_storage.h"
#include "dps310.h"
#include "lis3mdl.h"
#include "mpu6500.h"
#include "sensor_calibration.h"
#include <stdint.h>

#define APP_CAL_SAMPLE_COUNT             400U
#define APP_CAL_SAMPLE_DELAY_MS          2U
#define APP_MAG_CAL_UPDATE_PERIOD_MS     20U
#define APP_DPS310_REFERENCE_ALTITUDE_M  0.0f

typedef enum
{
  APP_CAL_CMD_NONE = 0,
  APP_CAL_CMD_GYRO_ZERO = 1,
  APP_CAL_CMD_ACCEL_POS_X = 2,
  APP_CAL_CMD_ACCEL_NEG_X = 3,
  APP_CAL_CMD_ACCEL_POS_Y = 4,
  APP_CAL_CMD_ACCEL_NEG_Y = 5,
  APP_CAL_CMD_ACCEL_POS_Z = 6,
  APP_CAL_CMD_ACCEL_NEG_Z = 7,
  APP_CAL_CMD_ACCEL_COMPUTE_SAVE = 8,
  APP_CAL_CMD_MAG_START = 9,
  APP_CAL_CMD_MAG_FINISH_SAVE = 10,
  APP_CAL_CMD_DPS310_REF_SAVE = 11,
  APP_CAL_CMD_SAVE = 12,
  APP_CAL_CMD_LOAD_DEFAULTS = 13
} app_cal_command_t;

typedef struct
{
  mpu6500_t *imu;
  lis3mdl_t *mag;
  dps310_t *prs;
  dps310_calibration_t *prs_sensor_calibration;
  lis3mdl_calibration_t *mag_runtime_calibration;
  cal_storage_block_t block;
  sensor_cal_accel_six_position_t accel_data;
  sensor_cal_mag_minmax_t mag_data;
  cal_storage_status_t flash_status;
  sensor_cal_status_t status;
  app_cal_command_t command;
  uint8_t busy;
  uint8_t loaded_from_flash;
  uint8_t using_defaults;
  uint8_t mag_active;
  uint32_t mag_sample_count;
  uint32_t save_count;
} app_calibration_t;

void app_calibration_init(app_calibration_t *app,
                          mpu6500_t *imu,
                          lis3mdl_t *mag,
                          dps310_t *prs,
                          dps310_calibration_t *prs_sensor_calibration,
                          lis3mdl_calibration_t *mag_runtime_calibration);
void app_calibration_process(app_calibration_t *app);
void app_calibration_update_mag(app_calibration_t *app);
void app_calibration_apply_stored(app_calibration_t *app);
void app_calibration_request(app_calibration_t *app, app_cal_command_t command);

#ifdef __cplusplus
}
#endif

#endif
