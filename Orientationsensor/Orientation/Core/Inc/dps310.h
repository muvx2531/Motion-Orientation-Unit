#ifndef DPS310_H
#define DPS310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define DPS310_PRODUCT_ID_REG_EXPECTED   0x10U
#define DPS310_PRODUCT_ID_MASK           0xF0U
#define DPS310_COEFFICIENT_LENGTH        18U

typedef enum
{
  DPS310_OK = 0,
  DPS310_ERROR = 1,
  DPS310_ERROR_INVALID_ARG = 2,
  DPS310_ERROR_HAL = 3,
  DPS310_ERROR_PRODUCT_ID = 4,
  DPS310_ERROR_TIMEOUT = 5
} dps310_status_t;

typedef enum
{
  DPS310_OVERSAMPLING_1 = 0,
  DPS310_OVERSAMPLING_2,
  DPS310_OVERSAMPLING_4,
  DPS310_OVERSAMPLING_8,
  DPS310_OVERSAMPLING_16,
  DPS310_OVERSAMPLING_32,
  DPS310_OVERSAMPLING_64,
  DPS310_OVERSAMPLING_128
} dps310_oversampling_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint32_t timeout_ms;
} dps310_t;

typedef struct
{
  uint8_t data[DPS310_COEFFICIENT_LENGTH];
} dps310_coefficients_raw_t;

typedef struct
{
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
} dps310_calibration_t;

typedef struct
{
  int32_t pressure_raw;
  int32_t temperature_raw;
  float pressure_pa;
  float temperature_c;
} dps310_sample_t;

dps310_status_t dps310_init(dps310_t *dev,
                            SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port,
                            uint16_t cs_pin);
dps310_status_t dps310_read_product_id(dps310_t *dev, uint8_t *product_id);
dps310_status_t dps310_check_product_id(dps310_t *dev);
dps310_status_t dps310_read_coefficients_raw(dps310_t *dev, dps310_coefficients_raw_t *coefficients);
dps310_status_t dps310_read_calibration(dps310_t *dev, dps310_calibration_t *calibration);
dps310_status_t dps310_configure_continuous(dps310_t *dev,
                                            dps310_oversampling_t pressure_oversampling,
                                            dps310_oversampling_t temperature_oversampling);
dps310_status_t dps310_read_pressure_raw(dps310_t *dev, int32_t *pressure_raw);
dps310_status_t dps310_read_temperature_raw(dps310_t *dev, int32_t *temperature_raw);
dps310_status_t dps310_read_compensated(dps310_t *dev,
                                        const dps310_calibration_t *calibration,
                                        dps310_oversampling_t pressure_oversampling,
                                        dps310_oversampling_t temperature_oversampling,
                                        dps310_sample_t *sample);
dps310_status_t dps310_write_register(dps310_t *dev, uint8_t reg, uint8_t value);
dps310_status_t dps310_read_register(dps310_t *dev, uint8_t reg, uint8_t *value);
dps310_status_t dps310_read_registers(dps310_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
