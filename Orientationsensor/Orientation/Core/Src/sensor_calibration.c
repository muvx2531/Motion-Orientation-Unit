#include "sensor_calibration.h"
#include <float.h>
#include <string.h>

static sensor_cal_status_t sensor_cal_mpu6500_collect_mean(mpu6500_t *imu,
                                                           uint32_t sample_count,
                                                           uint32_t sample_delay_ms,
                                                           sensor_cal_vector_t *accel_mean,
                                                           sensor_cal_vector_t *gyro_mean);
static float sensor_cal_absf(float value);

sensor_cal_status_t sensor_cal_mpu6500_gyro_zero(mpu6500_t *imu,
                                                 uint32_t sample_count,
                                                 uint32_t sample_delay_ms,
                                                 cal_mpu6500_t *calibration)
{
  sensor_cal_vector_t gyro_mean;
  sensor_cal_status_t status;

  if ((imu == NULL) || (calibration == NULL))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  status = sensor_cal_mpu6500_collect_mean(imu, sample_count, sample_delay_ms, NULL, &gyro_mean);
  if (status != SENSOR_CAL_OK)
  {
    return status;
  }

  calibration->gyro_offset_raw[0] = gyro_mean.x;
  calibration->gyro_offset_raw[1] = gyro_mean.y;
  calibration->gyro_offset_raw[2] = gyro_mean.z;

  return SENSOR_CAL_OK;
}

sensor_cal_status_t sensor_cal_mpu6500_accel_collect_position(mpu6500_t *imu,
                                                              sensor_cal_accel_position_t position,
                                                              uint32_t sample_count,
                                                              uint32_t sample_delay_ms,
                                                              sensor_cal_accel_six_position_t *accel_data)
{
  sensor_cal_vector_t accel_mean;
  sensor_cal_status_t status;

  if ((imu == NULL) || (accel_data == NULL) || (position >= SENSOR_CAL_ACCEL_POSITION_COUNT))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  status = sensor_cal_mpu6500_collect_mean(imu, sample_count, sample_delay_ms, &accel_mean, NULL);
  if (status != SENSOR_CAL_OK)
  {
    return status;
  }

  accel_data->mean[position] = accel_mean;
  accel_data->valid[position] = 1U;

  return SENSOR_CAL_OK;
}

sensor_cal_status_t sensor_cal_mpu6500_accel_compute_six_position(const sensor_cal_accel_six_position_t *accel_data,
                                                                  float accel_lsb_per_g,
                                                                  cal_mpu6500_t *calibration)
{
  float dx;
  float dy;
  float dz;
  uint32_t i;

  if ((accel_data == NULL) || (calibration == NULL) || (accel_lsb_per_g <= 0.0f))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  for (i = 0U; i < SENSOR_CAL_ACCEL_POSITION_COUNT; i++)
  {
    if (accel_data->valid[i] == 0U)
    {
      return SENSOR_CAL_ERROR_NOT_ENOUGH_SAMPLES;
    }
  }

  dx = accel_data->mean[SENSOR_CAL_ACCEL_POS_X].x - accel_data->mean[SENSOR_CAL_ACCEL_NEG_X].x;
  dy = accel_data->mean[SENSOR_CAL_ACCEL_POS_Y].y - accel_data->mean[SENSOR_CAL_ACCEL_NEG_Y].y;
  dz = accel_data->mean[SENSOR_CAL_ACCEL_POS_Z].z - accel_data->mean[SENSOR_CAL_ACCEL_NEG_Z].z;

  if ((sensor_cal_absf(dx) < accel_lsb_per_g) ||
      (sensor_cal_absf(dy) < accel_lsb_per_g) ||
      (sensor_cal_absf(dz) < accel_lsb_per_g))
  {
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  calibration->accel_offset_raw[0] = (accel_data->mean[SENSOR_CAL_ACCEL_POS_X].x +
                                      accel_data->mean[SENSOR_CAL_ACCEL_NEG_X].x) * 0.5f;
  calibration->accel_offset_raw[1] = (accel_data->mean[SENSOR_CAL_ACCEL_POS_Y].y +
                                      accel_data->mean[SENSOR_CAL_ACCEL_NEG_Y].y) * 0.5f;
  calibration->accel_offset_raw[2] = (accel_data->mean[SENSOR_CAL_ACCEL_POS_Z].z +
                                      accel_data->mean[SENSOR_CAL_ACCEL_NEG_Z].z) * 0.5f;

  calibration->accel_scale[0] = (2.0f * accel_lsb_per_g) / dx;
  calibration->accel_scale[1] = (2.0f * accel_lsb_per_g) / dy;
  calibration->accel_scale[2] = (2.0f * accel_lsb_per_g) / dz;

  return SENSOR_CAL_OK;
}

sensor_cal_status_t sensor_cal_mpu6500_accel_single_position(mpu6500_t *imu,
                                                             const sensor_cal_vector_t *expected_g,
                                                             uint32_t sample_count,
                                                             uint32_t sample_delay_ms,
                                                             cal_mpu6500_t *calibration)
{
  sensor_cal_vector_t accel_mean;
  sensor_cal_status_t status;

  if ((imu == NULL) || (expected_g == NULL) || (calibration == NULL))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  status = sensor_cal_mpu6500_collect_mean(imu, sample_count, sample_delay_ms, &accel_mean, NULL);
  if (status != SENSOR_CAL_OK)
  {
    return status;
  }

  calibration->accel_offset_raw[0] = accel_mean.x - (expected_g->x * imu->accel_lsb_per_g);
  calibration->accel_offset_raw[1] = accel_mean.y - (expected_g->y * imu->accel_lsb_per_g);
  calibration->accel_offset_raw[2] = accel_mean.z - (expected_g->z * imu->accel_lsb_per_g);

  if ((calibration->accel_scale[0] == 0.0f) ||
      (calibration->accel_scale[1] == 0.0f) ||
      (calibration->accel_scale[2] == 0.0f))
  {
    calibration->accel_scale[0] = 1.0f;
    calibration->accel_scale[1] = 1.0f;
    calibration->accel_scale[2] = 1.0f;
  }

  return SENSOR_CAL_OK;
}

void sensor_cal_accel_six_position_clear(sensor_cal_accel_six_position_t *accel_data)
{
  if (accel_data == NULL)
  {
    return;
  }

  memset(accel_data, 0, sizeof(*accel_data));
}

void sensor_cal_lis3mdl_mag_minmax_start(sensor_cal_mag_minmax_t *state)
{
  if (state == NULL)
  {
    return;
  }

  state->min_gauss.x = FLT_MAX;
  state->min_gauss.y = FLT_MAX;
  state->min_gauss.z = FLT_MAX;
  state->max_gauss.x = -FLT_MAX;
  state->max_gauss.y = -FLT_MAX;
  state->max_gauss.z = -FLT_MAX;
  state->sample_count = 0U;
}

sensor_cal_status_t sensor_cal_lis3mdl_mag_minmax_update(lis3mdl_t *mag,
                                                         sensor_cal_mag_minmax_t *state)
{
  lis3mdl_vector_t sample;
  lis3mdl_status_t status;

  if ((mag == NULL) || (state == NULL))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  status = lis3mdl_read_gauss(mag, &sample);
  if (status != LIS3MDL_OK)
  {
    return SENSOR_CAL_ERROR_SENSOR;
  }

  if (sample.x < state->min_gauss.x) { state->min_gauss.x = sample.x; }
  if (sample.y < state->min_gauss.y) { state->min_gauss.y = sample.y; }
  if (sample.z < state->min_gauss.z) { state->min_gauss.z = sample.z; }
  if (sample.x > state->max_gauss.x) { state->max_gauss.x = sample.x; }
  if (sample.y > state->max_gauss.y) { state->max_gauss.y = sample.y; }
  if (sample.z > state->max_gauss.z) { state->max_gauss.z = sample.z; }
  state->sample_count++;

  return SENSOR_CAL_OK;
}

sensor_cal_status_t sensor_cal_lis3mdl_mag_minmax_finish(const sensor_cal_mag_minmax_t *state,
                                                         cal_lis3mdl_t *calibration)
{
  float radius_x;
  float radius_y;
  float radius_z;
  float average_radius;

  if ((state == NULL) || (calibration == NULL))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  if (state->sample_count < 50U)
  {
    return SENSOR_CAL_ERROR_NOT_ENOUGH_SAMPLES;
  }

  radius_x = (state->max_gauss.x - state->min_gauss.x) * 0.5f;
  radius_y = (state->max_gauss.y - state->min_gauss.y) * 0.5f;
  radius_z = (state->max_gauss.z - state->min_gauss.z) * 0.5f;

  if ((radius_x <= 0.0f) || (radius_y <= 0.0f) || (radius_z <= 0.0f))
  {
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  average_radius = (radius_x + radius_y + radius_z) / 3.0f;

  calibration->hard_iron_gauss[0] = (state->max_gauss.x + state->min_gauss.x) * 0.5f;
  calibration->hard_iron_gauss[1] = (state->max_gauss.y + state->min_gauss.y) * 0.5f;
  calibration->hard_iron_gauss[2] = (state->max_gauss.z + state->min_gauss.z) * 0.5f;

  memset(calibration->soft_iron_matrix, 0, sizeof(calibration->soft_iron_matrix));
  calibration->soft_iron_matrix[0][0] = average_radius / radius_x;
  calibration->soft_iron_matrix[1][1] = average_radius / radius_y;
  calibration->soft_iron_matrix[2][2] = average_radius / radius_z;

  return SENSOR_CAL_OK;
}

sensor_cal_status_t sensor_cal_dps310_reference_current(dps310_t *prs,
                                                        const dps310_calibration_t *dps_calibration,
                                                        dps310_oversampling_t pressure_oversampling,
                                                        dps310_oversampling_t temperature_oversampling,
                                                        float reference_altitude_m,
                                                        cal_dps310_t *calibration)
{
  dps310_sample_t sample;
  dps310_status_t status;

  if ((prs == NULL) || (dps_calibration == NULL) || (calibration == NULL))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  status = dps310_read_compensated(prs,
                                   dps_calibration,
                                   pressure_oversampling,
                                   temperature_oversampling,
                                   &sample);
  if (status != DPS310_OK)
  {
    return SENSOR_CAL_ERROR_SENSOR;
  }

  calibration->reference_pressure_pa = sample.pressure_pa;
  calibration->reference_altitude_m = reference_altitude_m;

  return SENSOR_CAL_OK;
}

void sensor_cal_apply_mpu6500_accel_raw(const cal_mpu6500_t *calibration,
                                        const mpu6500_vector_raw_t *raw,
                                        sensor_cal_vector_t *corrected_raw)
{
  if ((calibration == NULL) || (raw == NULL) || (corrected_raw == NULL))
  {
    return;
  }

  corrected_raw->x = ((float)raw->x - calibration->accel_offset_raw[0]) * calibration->accel_scale[0];
  corrected_raw->y = ((float)raw->y - calibration->accel_offset_raw[1]) * calibration->accel_scale[1];
  corrected_raw->z = ((float)raw->z - calibration->accel_offset_raw[2]) * calibration->accel_scale[2];
}

void sensor_cal_apply_mpu6500_gyro_raw(const cal_mpu6500_t *calibration,
                                       const mpu6500_vector_raw_t *raw,
                                       sensor_cal_vector_t *corrected_raw)
{
  if ((calibration == NULL) || (raw == NULL) || (corrected_raw == NULL))
  {
    return;
  }

  corrected_raw->x = (float)raw->x - calibration->gyro_offset_raw[0];
  corrected_raw->y = (float)raw->y - calibration->gyro_offset_raw[1];
  corrected_raw->z = (float)raw->z - calibration->gyro_offset_raw[2];
}

void sensor_cal_apply_lis3mdl_gauss(const cal_lis3mdl_t *calibration,
                                    const lis3mdl_vector_t *raw_gauss,
                                    lis3mdl_vector_t *corrected_gauss)
{
  lis3mdl_vector_t centered;

  if ((calibration == NULL) || (raw_gauss == NULL) || (corrected_gauss == NULL))
  {
    return;
  }

  centered.x = raw_gauss->x - calibration->hard_iron_gauss[0];
  centered.y = raw_gauss->y - calibration->hard_iron_gauss[1];
  centered.z = raw_gauss->z - calibration->hard_iron_gauss[2];

  corrected_gauss->x = (calibration->soft_iron_matrix[0][0] * centered.x) +
                       (calibration->soft_iron_matrix[0][1] * centered.y) +
                       (calibration->soft_iron_matrix[0][2] * centered.z);
  corrected_gauss->y = (calibration->soft_iron_matrix[1][0] * centered.x) +
                       (calibration->soft_iron_matrix[1][1] * centered.y) +
                       (calibration->soft_iron_matrix[1][2] * centered.z);
  corrected_gauss->z = (calibration->soft_iron_matrix[2][0] * centered.x) +
                       (calibration->soft_iron_matrix[2][1] * centered.y) +
                       (calibration->soft_iron_matrix[2][2] * centered.z);
}

static sensor_cal_status_t sensor_cal_mpu6500_collect_mean(mpu6500_t *imu,
                                                           uint32_t sample_count,
                                                           uint32_t sample_delay_ms,
                                                           sensor_cal_vector_t *accel_mean,
                                                           sensor_cal_vector_t *gyro_mean)
{
  uint32_t i;
  float accel_sum_x = 0.0f;
  float accel_sum_y = 0.0f;
  float accel_sum_z = 0.0f;
  float gyro_sum_x = 0.0f;
  float gyro_sum_y = 0.0f;
  float gyro_sum_z = 0.0f;

  if ((imu == NULL) || (sample_count == 0U))
  {
    return SENSOR_CAL_ERROR_INVALID_ARG;
  }

  for (i = 0U; i < sample_count; i++)
  {
    mpu6500_raw_sample_t sample;
    if (mpu6500_read_raw(imu, &sample) != MPU6500_OK)
    {
      return SENSOR_CAL_ERROR_SENSOR;
    }

    accel_sum_x += (float)sample.accel.x;
    accel_sum_y += (float)sample.accel.y;
    accel_sum_z += (float)sample.accel.z;
    gyro_sum_x += (float)sample.gyro.x;
    gyro_sum_y += (float)sample.gyro.y;
    gyro_sum_z += (float)sample.gyro.z;

    if (sample_delay_ms > 0U)
    {
      HAL_Delay(sample_delay_ms);
    }
  }

  if (accel_mean != NULL)
  {
    accel_mean->x = accel_sum_x / (float)sample_count;
    accel_mean->y = accel_sum_y / (float)sample_count;
    accel_mean->z = accel_sum_z / (float)sample_count;
  }

  if (gyro_mean != NULL)
  {
    gyro_mean->x = gyro_sum_x / (float)sample_count;
    gyro_mean->y = gyro_sum_y / (float)sample_count;
    gyro_mean->z = gyro_sum_z / (float)sample_count;
  }

  return SENSOR_CAL_OK;
}

static float sensor_cal_absf(float value)
{
  return (value < 0.0f) ? -value : value;
}
