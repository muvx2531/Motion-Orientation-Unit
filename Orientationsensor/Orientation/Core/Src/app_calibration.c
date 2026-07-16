#include "app_calibration.h"
#include <string.h>

static void app_calibration_save_ok(app_calibration_t *app);

void app_calibration_init(app_calibration_t *app,
                          mpu6500_t *imu,
                          lis3mdl_t *mag,
                          dps310_t *prs,
                          dps310_calibration_t *prs_sensor_calibration,
                          lis3mdl_calibration_t *mag_runtime_calibration)
{
  if (app == NULL)
  {
    return;
  }

  memset(app, 0, sizeof(*app));
  app->imu = imu;
  app->mag = mag;
  app->prs = prs;
  app->prs_sensor_calibration = prs_sensor_calibration;
  app->mag_runtime_calibration = mag_runtime_calibration;
  app->flash_status = cal_storage_load(&app->block);
  if (app->flash_status != CAL_STORAGE_OK)
  {
    cal_storage_set_defaults(&app->block);
    app->flash_status = cal_storage_save(&app->block);
    app->loaded_from_flash = 0U;
    app->using_defaults = (app->flash_status == CAL_STORAGE_OK) ? 0U : 1U;
  }
  else
  {
    app->loaded_from_flash = 1U;
    app->using_defaults = 0U;
  }

  app_calibration_apply_stored(app);
  sensor_cal_accel_six_position_clear(&app->accel_data);
  app->command = APP_CAL_CMD_NONE;
}

void app_calibration_process(app_calibration_t *app)
{
  app_cal_command_t command;

  if ((app == NULL) || (app->busy != 0U) || (app->command == APP_CAL_CMD_NONE))
  {
    return;
  }

  app->busy = 1U;
  command = app->command;
  app->command = APP_CAL_CMD_NONE;

  switch (command)
  {
    case APP_CAL_CMD_GYRO_ZERO:
      app->status = sensor_cal_mpu6500_gyro_zero(app->imu,
                                                 APP_CAL_SAMPLE_COUNT,
                                                 APP_CAL_SAMPLE_DELAY_MS,
                                                 &app->block.mpu6500);
      break;

    case APP_CAL_CMD_ACCEL_POS_X:
    case APP_CAL_CMD_ACCEL_NEG_X:
    case APP_CAL_CMD_ACCEL_POS_Y:
    case APP_CAL_CMD_ACCEL_NEG_Y:
    case APP_CAL_CMD_ACCEL_POS_Z:
    case APP_CAL_CMD_ACCEL_NEG_Z:
      app->status = sensor_cal_mpu6500_accel_collect_position(
          app->imu,
          (sensor_cal_accel_position_t)(command - APP_CAL_CMD_ACCEL_POS_X),
          APP_CAL_SAMPLE_COUNT,
          APP_CAL_SAMPLE_DELAY_MS,
          &app->accel_data);
      break;

    case APP_CAL_CMD_ACCEL_COMPUTE_SAVE:
      app->status = sensor_cal_mpu6500_accel_compute_six_position(&app->accel_data,
                                                                  app->imu->accel_lsb_per_g,
                                                                  &app->block.mpu6500);
      if (app->status == SENSOR_CAL_OK)
      {
        app->flash_status = cal_storage_save(&app->block);
        app_calibration_save_ok(app);
      }
      break;

    case APP_CAL_CMD_MAG_START:
      sensor_cal_lis3mdl_mag_minmax_start(&app->mag_data);
      app->mag_sample_count = 0U;
      app->mag_active = 1U;
      app->status = SENSOR_CAL_OK;
      break;

    case APP_CAL_CMD_MAG_FINISH_SAVE:
      app->mag_active = 0U;
      app->status = sensor_cal_lis3mdl_mag_minmax_finish(&app->mag_data, &app->block.lis3mdl);
      if (app->status == SENSOR_CAL_OK)
      {
        app_calibration_apply_stored(app);
        app->flash_status = cal_storage_save(&app->block);
        app_calibration_save_ok(app);
      }
      break;

    case APP_CAL_CMD_DPS310_REF_SAVE:
      app->status = sensor_cal_dps310_reference_current(app->prs,
                                                        app->prs_sensor_calibration,
                                                        DPS310_OVERSAMPLING_1,
                                                        DPS310_OVERSAMPLING_1,
                                                        APP_DPS310_REFERENCE_ALTITUDE_M,
                                                        &app->block.dps310);
      if (app->status == SENSOR_CAL_OK)
      {
        app->flash_status = cal_storage_save(&app->block);
        app_calibration_save_ok(app);
      }
      break;

    case APP_CAL_CMD_SAVE:
      app->flash_status = cal_storage_save(&app->block);
      if (app->flash_status == CAL_STORAGE_OK)
      {
        app_calibration_save_ok(app);
        app->status = SENSOR_CAL_OK;
      }
      else
      {
        app->status = SENSOR_CAL_ERROR;
      }
      break;

    case APP_CAL_CMD_LOAD_DEFAULTS:
      cal_storage_set_defaults(&app->block);
      app_calibration_apply_stored(app);
      sensor_cal_accel_six_position_clear(&app->accel_data);
      app->loaded_from_flash = 0U;
      app->using_defaults = 1U;
      app->status = SENSOR_CAL_OK;
      break;

    default:
      app->status = SENSOR_CAL_ERROR_INVALID_ARG;
      break;
  }

  app->busy = 0U;
}

void app_calibration_update_mag(app_calibration_t *app)
{
  static uint32_t last_update_ms = 0U;
  uint32_t now_ms;

  if ((app == NULL) || (app->mag_active == 0U) || (app->busy != 0U))
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - last_update_ms) < APP_MAG_CAL_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_update_ms = now_ms;

  app->status = sensor_cal_lis3mdl_mag_minmax_update(app->mag, &app->mag_data);
  if (app->status == SENSOR_CAL_OK)
  {
    app->mag_sample_count = app->mag_data.sample_count;
  }
}

void app_calibration_apply_stored(app_calibration_t *app)
{
  lis3mdl_calibration_t *mag_cal;

  if ((app == NULL) || (app->mag_runtime_calibration == NULL))
  {
    return;
  }

  mag_cal = app->mag_runtime_calibration;
  mag_cal->hard_iron_gauss.x = app->block.lis3mdl.hard_iron_gauss[0];
  mag_cal->hard_iron_gauss.y = app->block.lis3mdl.hard_iron_gauss[1];
  mag_cal->hard_iron_gauss.z = app->block.lis3mdl.hard_iron_gauss[2];

  mag_cal->soft_iron_matrix[0][0] = app->block.lis3mdl.soft_iron_matrix[0][0];
  mag_cal->soft_iron_matrix[0][1] = app->block.lis3mdl.soft_iron_matrix[0][1];
  mag_cal->soft_iron_matrix[0][2] = app->block.lis3mdl.soft_iron_matrix[0][2];
  mag_cal->soft_iron_matrix[1][0] = app->block.lis3mdl.soft_iron_matrix[1][0];
  mag_cal->soft_iron_matrix[1][1] = app->block.lis3mdl.soft_iron_matrix[1][1];
  mag_cal->soft_iron_matrix[1][2] = app->block.lis3mdl.soft_iron_matrix[1][2];
  mag_cal->soft_iron_matrix[2][0] = app->block.lis3mdl.soft_iron_matrix[2][0];
  mag_cal->soft_iron_matrix[2][1] = app->block.lis3mdl.soft_iron_matrix[2][1];
  mag_cal->soft_iron_matrix[2][2] = app->block.lis3mdl.soft_iron_matrix[2][2];
}

void app_calibration_request(app_calibration_t *app, app_cal_command_t command)
{
  if (app == NULL)
  {
    return;
  }

  app->command = command;
}

static void app_calibration_save_ok(app_calibration_t *app)
{
  if ((app != NULL) && (app->flash_status == CAL_STORAGE_OK))
  {
    app->loaded_from_flash = 1U;
    app->using_defaults = 0U;
    app->save_count++;
  }
}
