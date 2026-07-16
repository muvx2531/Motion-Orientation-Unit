#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define MPU6500_WHO_AM_I_EXPECTED        0x70U

typedef enum
{
  MPU6500_OK = 0,
  MPU6500_ERROR = 1,
  MPU6500_ERROR_INVALID_ARG = 2,
  MPU6500_ERROR_HAL = 3,
  MPU6500_ERROR_WHO_AM_I = 4,
  MPU6500_ERROR_TIMEOUT = 5
} mpu6500_status_t;

typedef enum
{
  MPU6500_GYRO_FS_250DPS = 0,
  MPU6500_GYRO_FS_500DPS,
  MPU6500_GYRO_FS_1000DPS,
  MPU6500_GYRO_FS_2000DPS
} mpu6500_gyro_fs_t;

typedef enum
{
  MPU6500_ACCEL_FS_2G = 0,
  MPU6500_ACCEL_FS_4G,
  MPU6500_ACCEL_FS_8G,
  MPU6500_ACCEL_FS_16G
} mpu6500_accel_fs_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} mpu6500_vector_raw_t;

typedef struct
{
  float x;
  float y;
  float z;
} mpu6500_vector_t;

typedef struct
{
  mpu6500_vector_raw_t accel;
  mpu6500_vector_raw_t gyro;
  int16_t temperature;
} mpu6500_raw_sample_t;

typedef struct
{
  mpu6500_vector_t accel_g;
  mpu6500_vector_t gyro_dps;
  float temperature_c;
} mpu6500_scaled_sample_t;

typedef struct
{
  mpu6500_gyro_fs_t gyro_fs;
  mpu6500_accel_fs_t accel_fs;
  uint8_t dlpf_cfg;
  uint8_t sample_rate_divider;
} mpu6500_config_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint32_t timeout_ms;
  mpu6500_config_t config;
  float accel_lsb_per_g;
  float gyro_lsb_per_dps;
} mpu6500_t;

mpu6500_status_t mpu6500_init(mpu6500_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin,
                              const mpu6500_config_t *config);
mpu6500_status_t mpu6500_read_who_am_i(mpu6500_t *dev, uint8_t *who_am_i);
mpu6500_status_t mpu6500_read_raw(mpu6500_t *dev, mpu6500_raw_sample_t *sample);
mpu6500_status_t mpu6500_read_scaled(mpu6500_t *dev, mpu6500_scaled_sample_t *sample);
mpu6500_status_t mpu6500_read_accel_raw(mpu6500_t *dev, mpu6500_vector_raw_t *accel);
mpu6500_status_t mpu6500_read_gyro_raw(mpu6500_t *dev, mpu6500_vector_raw_t *gyro);
mpu6500_status_t mpu6500_read_temperature_raw(mpu6500_t *dev, int16_t *temperature);
mpu6500_status_t mpu6500_write_register(mpu6500_t *dev, uint8_t reg, uint8_t value);
mpu6500_status_t mpu6500_read_register(mpu6500_t *dev, uint8_t reg, uint8_t *value);
mpu6500_status_t mpu6500_read_registers(mpu6500_t *dev, uint8_t reg, uint8_t *data, uint16_t length);
float mpu6500_accel_lsb_per_g(mpu6500_accel_fs_t accel_fs);
float mpu6500_gyro_lsb_per_dps(mpu6500_gyro_fs_t gyro_fs);

#ifdef __cplusplus
}
#endif

#endif
