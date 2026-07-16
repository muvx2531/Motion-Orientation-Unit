#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define LIS3MDL_WHO_AM_I_EXPECTED        0x3DU

typedef enum
{
  LIS3MDL_OK = 0,
  LIS3MDL_ERROR = 1,
  LIS3MDL_ERROR_INVALID_ARG = 2,
  LIS3MDL_ERROR_HAL = 3,
  LIS3MDL_ERROR_WHO_AM_I = 4,
  LIS3MDL_ERROR_TIMEOUT = 5
} lis3mdl_status_t;

typedef enum
{
  LIS3MDL_FS_4_GAUSS = 0,
  LIS3MDL_FS_8_GAUSS,
  LIS3MDL_FS_12_GAUSS,
  LIS3MDL_FS_16_GAUSS
} lis3mdl_full_scale_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} lis3mdl_vector_raw_t;

typedef struct
{
  float x;
  float y;
  float z;
} lis3mdl_vector_t;

typedef struct
{
  lis3mdl_vector_t hard_iron_gauss;
  float soft_iron_matrix[3][3];
} lis3mdl_calibration_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint32_t timeout_ms;
  lis3mdl_full_scale_t full_scale;
  float lsb_per_gauss;
} lis3mdl_t;

lis3mdl_status_t lis3mdl_init(lis3mdl_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin);
lis3mdl_status_t lis3mdl_read_who_am_i(lis3mdl_t *dev, uint8_t *who_am_i);
lis3mdl_status_t lis3mdl_check_who_am_i(lis3mdl_t *dev);
lis3mdl_status_t lis3mdl_configure_continuous(lis3mdl_t *dev, lis3mdl_full_scale_t full_scale);
lis3mdl_status_t lis3mdl_read_raw(lis3mdl_t *dev, lis3mdl_vector_raw_t *raw);
lis3mdl_status_t lis3mdl_read_gauss(lis3mdl_t *dev, lis3mdl_vector_t *mag_gauss);
lis3mdl_status_t lis3mdl_read_calibrated(lis3mdl_t *dev,
                                         const lis3mdl_calibration_t *calibration,
                                         lis3mdl_vector_t *mag_gauss);
void lis3mdl_calibration_identity(lis3mdl_calibration_t *calibration);
float lis3mdl_lsb_per_gauss(lis3mdl_full_scale_t full_scale);
lis3mdl_status_t lis3mdl_write_register(lis3mdl_t *dev, uint8_t reg, uint8_t value);
lis3mdl_status_t lis3mdl_read_register(lis3mdl_t *dev, uint8_t reg, uint8_t *value);
lis3mdl_status_t lis3mdl_read_registers(lis3mdl_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
