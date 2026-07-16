#ifndef CAL_STORAGE_H
#define CAL_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define CAL_STORAGE_FLASH_ADDR           0x0801F800UL
#define CAL_STORAGE_FLASH_PAGE           63U
#define CAL_STORAGE_FLASH_SIZE           0x800U
#define CAL_STORAGE_MAGIC                0x4D4F5543UL
#define CAL_STORAGE_VERSION              1U
#define CAL_STORAGE_VALID_FLAG           0x00000001UL

typedef enum
{
  CAL_STORAGE_OK = 0,
  CAL_STORAGE_ERROR = 1,
  CAL_STORAGE_ERROR_INVALID_ARG = 2,
  CAL_STORAGE_ERROR_INVALID_DATA = 3,
  CAL_STORAGE_ERROR_FLASH = 4,
  CAL_STORAGE_ERROR_VERIFY = 5
} cal_storage_status_t;

typedef struct
{
  float accel_offset_raw[3];
  float accel_scale[3];
  float gyro_offset_raw[3];
} cal_mpu6500_t;

typedef struct
{
  float hard_iron_gauss[3];
  float soft_iron_matrix[3][3];
} cal_lis3mdl_t;

typedef struct
{
  float reference_pressure_pa;
  float reference_altitude_m;
} cal_dps310_t;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t sequence;
  uint32_t flags;
  cal_mpu6500_t mpu6500;
  cal_lis3mdl_t lis3mdl;
  cal_dps310_t dps310;
  uint32_t reserved[16];
  uint32_t crc32;
} cal_storage_block_t;

cal_storage_status_t cal_storage_load(cal_storage_block_t *block);
cal_storage_status_t cal_storage_save(const cal_storage_block_t *block);
void cal_storage_set_defaults(cal_storage_block_t *block);
uint8_t cal_storage_is_valid(const cal_storage_block_t *block);
uint32_t cal_storage_crc32(const void *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
