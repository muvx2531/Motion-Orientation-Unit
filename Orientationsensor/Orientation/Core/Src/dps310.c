#include "dps310.h"

#define DPS310_REG_PRS_B2                0x00U
#define DPS310_REG_TMP_B2                0x03U
#define DPS310_REG_PRS_CFG               0x06U
#define DPS310_REG_TMP_CFG               0x07U
#define DPS310_REG_MEAS_CFG              0x08U
#define DPS310_REG_COEF_BASE             0x10U
#define DPS310_REG_PRODUCT_ID            0x0DU
#define DPS310_REG_COEF_SRCE             0x28U

#define DPS310_SPI_READ_BIT              0x80U
#define DPS310_SPI_WRITE_MASK            0x7FU
#define DPS310_DEFAULT_TIMEOUT_MS        100U
#define DPS310_TMP_COEF_SRCE_MASK        0x80U
#define DPS310_MEAS_CFG_CONT_PRS_TMP     0x07U

static dps310_status_t dps310_spi_write(dps310_t *dev, const uint8_t *data, uint16_t length);
static int16_t dps310_sign_extend_12(uint16_t value);
static int32_t dps310_sign_extend_20(uint32_t value);
static int32_t dps310_sign_extend_24(uint32_t value);
static float dps310_scaling_factor(dps310_oversampling_t oversampling);

dps310_status_t dps310_init(dps310_t *dev,
                            SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port,
                            uint16_t cs_pin)
{
  if ((dev == NULL) || (hspi == NULL) || (cs_port == NULL))
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->timeout_ms = DPS310_DEFAULT_TIMEOUT_MS;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  return dps310_check_product_id(dev);
}

dps310_status_t dps310_read_product_id(dps310_t *dev, uint8_t *product_id)
{
  return dps310_read_register(dev, DPS310_REG_PRODUCT_ID, product_id);
}

dps310_status_t dps310_check_product_id(dps310_t *dev)
{
  uint8_t product_id;
  dps310_status_t status;

  status = dps310_read_product_id(dev, &product_id);
  if (status != DPS310_OK)
  {
    return status;
  }

  if ((product_id & DPS310_PRODUCT_ID_MASK) != DPS310_PRODUCT_ID_REG_EXPECTED)
  {
    return DPS310_ERROR_PRODUCT_ID;
  }

  return DPS310_OK;
}

dps310_status_t dps310_read_coefficients_raw(dps310_t *dev, dps310_coefficients_raw_t *coefficients)
{
  if (coefficients == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  return dps310_read_registers(dev, DPS310_REG_COEF_BASE, coefficients->data, DPS310_COEFFICIENT_LENGTH);
}

dps310_status_t dps310_read_calibration(dps310_t *dev, dps310_calibration_t *calibration)
{
  dps310_coefficients_raw_t raw;
  dps310_status_t status;

  if (calibration == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  status = dps310_read_coefficients_raw(dev, &raw);
  if (status != DPS310_OK)
  {
    return status;
  }

  calibration->c0 = dps310_sign_extend_12(((uint16_t)raw.data[0] << 4) |
                                          ((uint16_t)raw.data[1] >> 4));
  calibration->c1 = dps310_sign_extend_12((((uint16_t)raw.data[1] & 0x0FU) << 8) |
                                          raw.data[2]);
  calibration->c00 = dps310_sign_extend_20(((uint32_t)raw.data[3] << 12) |
                                           ((uint32_t)raw.data[4] << 4) |
                                           ((uint32_t)raw.data[5] >> 4));
  calibration->c10 = dps310_sign_extend_20((((uint32_t)raw.data[5] & 0x0FU) << 16) |
                                           ((uint32_t)raw.data[6] << 8) |
                                           raw.data[7]);
  calibration->c01 = (int16_t)(((uint16_t)raw.data[8] << 8) | raw.data[9]);
  calibration->c11 = (int16_t)(((uint16_t)raw.data[10] << 8) | raw.data[11]);
  calibration->c20 = (int16_t)(((uint16_t)raw.data[12] << 8) | raw.data[13]);
  calibration->c21 = (int16_t)(((uint16_t)raw.data[14] << 8) | raw.data[15]);
  calibration->c30 = (int16_t)(((uint16_t)raw.data[16] << 8) | raw.data[17]);

  return DPS310_OK;
}

dps310_status_t dps310_configure_continuous(dps310_t *dev,
                                            dps310_oversampling_t pressure_oversampling,
                                            dps310_oversampling_t temperature_oversampling)
{
  dps310_status_t status;
  uint8_t coefficient_source;
  uint8_t pressure_cfg;
  uint8_t temperature_cfg;

  if ((pressure_oversampling > DPS310_OVERSAMPLING_128) ||
      (temperature_oversampling > DPS310_OVERSAMPLING_128))
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  pressure_cfg = (uint8_t)pressure_oversampling;

  status = dps310_read_register(dev, DPS310_REG_COEF_SRCE, &coefficient_source);
  if (status != DPS310_OK)
  {
    return status;
  }

  temperature_cfg = (uint8_t)((coefficient_source & DPS310_TMP_COEF_SRCE_MASK) |
                              (uint8_t)temperature_oversampling);

  status = dps310_write_register(dev, DPS310_REG_PRS_CFG, pressure_cfg);
  if (status != DPS310_OK)
  {
    return status;
  }

  status = dps310_write_register(dev, DPS310_REG_TMP_CFG, temperature_cfg);
  if (status != DPS310_OK)
  {
    return status;
  }

  return dps310_write_register(dev, DPS310_REG_MEAS_CFG, DPS310_MEAS_CFG_CONT_PRS_TMP);
}

dps310_status_t dps310_read_pressure_raw(dps310_t *dev, int32_t *pressure_raw)
{
  uint8_t data[3];
  dps310_status_t status;

  if (pressure_raw == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  status = dps310_read_registers(dev, DPS310_REG_PRS_B2, data, sizeof(data));
  if (status != DPS310_OK)
  {
    return status;
  }

  *pressure_raw = dps310_sign_extend_24(((uint32_t)data[0] << 16) |
                                        ((uint32_t)data[1] << 8) |
                                        data[2]);

  return DPS310_OK;
}

dps310_status_t dps310_read_temperature_raw(dps310_t *dev, int32_t *temperature_raw)
{
  uint8_t data[3];
  dps310_status_t status;

  if (temperature_raw == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  status = dps310_read_registers(dev, DPS310_REG_TMP_B2, data, sizeof(data));
  if (status != DPS310_OK)
  {
    return status;
  }

  *temperature_raw = dps310_sign_extend_24(((uint32_t)data[0] << 16) |
                                           ((uint32_t)data[1] << 8) |
                                           data[2]);

  return DPS310_OK;
}

dps310_status_t dps310_read_compensated(dps310_t *dev,
                                        const dps310_calibration_t *calibration,
                                        dps310_oversampling_t pressure_oversampling,
                                        dps310_oversampling_t temperature_oversampling,
                                        dps310_sample_t *sample)
{
  dps310_status_t status;
  float pressure_scaled;
  float temperature_scaled;

  if ((calibration == NULL) || (sample == NULL) ||
      (pressure_oversampling > DPS310_OVERSAMPLING_128) ||
      (temperature_oversampling > DPS310_OVERSAMPLING_128))
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  status = dps310_read_pressure_raw(dev, &sample->pressure_raw);
  if (status != DPS310_OK)
  {
    return status;
  }

  status = dps310_read_temperature_raw(dev, &sample->temperature_raw);
  if (status != DPS310_OK)
  {
    return status;
  }

  pressure_scaled = (float)sample->pressure_raw / dps310_scaling_factor(pressure_oversampling);
  temperature_scaled = (float)sample->temperature_raw / dps310_scaling_factor(temperature_oversampling);

  sample->temperature_c = ((float)calibration->c0 * 0.5f) +
                          ((float)calibration->c1 * temperature_scaled);
  sample->pressure_pa = (float)calibration->c00 +
                        (pressure_scaled * ((float)calibration->c10 +
                        (pressure_scaled * ((float)calibration->c20 +
                        (pressure_scaled * (float)calibration->c30))))) +
                        (temperature_scaled * (float)calibration->c01) +
                        (temperature_scaled * pressure_scaled *
                        ((float)calibration->c11 + (pressure_scaled * (float)calibration->c21)));

  return DPS310_OK;
}

dps310_status_t dps310_write_register(dps310_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  if (dev == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  tx[0] = reg & DPS310_SPI_WRITE_MASK;
  tx[1] = value;

  return dps310_spi_write(dev, tx, sizeof(tx));
}

dps310_status_t dps310_read_register(dps310_t *dev, uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  return dps310_read_registers(dev, reg, value, 1U);
}

dps310_status_t dps310_read_registers(dps310_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx_header;
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (length == 0U))
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  tx_header = reg | DPS310_SPI_READ_BIT;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(dev->hspi, &tx_header, 1U, dev->timeout_ms);
  if (hal_status == HAL_OK)
  {
    hal_status = HAL_SPI_Receive(dev->hspi, data, length, dev->timeout_ms);
  }
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return DPS310_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return DPS310_ERROR_HAL;
  }

  return DPS310_OK;
}

static int16_t dps310_sign_extend_12(uint16_t value)
{
  if ((value & 0x0800U) != 0U)
  {
    value |= 0xF000U;
  }

  return (int16_t)value;
}

static int32_t dps310_sign_extend_20(uint32_t value)
{
  if ((value & 0x00080000UL) != 0UL)
  {
    value |= 0xFFF00000UL;
  }

  return (int32_t)value;
}

static int32_t dps310_sign_extend_24(uint32_t value)
{
  if ((value & 0x00800000UL) != 0UL)
  {
    value |= 0xFF000000UL;
  }

  return (int32_t)value;
}

static float dps310_scaling_factor(dps310_oversampling_t oversampling)
{
  switch (oversampling)
  {
    case DPS310_OVERSAMPLING_1:
      return 524288.0f;
    case DPS310_OVERSAMPLING_2:
      return 1572864.0f;
    case DPS310_OVERSAMPLING_4:
      return 3670016.0f;
    case DPS310_OVERSAMPLING_8:
      return 7864320.0f;
    case DPS310_OVERSAMPLING_16:
      return 253952.0f;
    case DPS310_OVERSAMPLING_32:
      return 516096.0f;
    case DPS310_OVERSAMPLING_64:
      return 1040384.0f;
    case DPS310_OVERSAMPLING_128:
      return 2088960.0f;
    default:
      return 524288.0f;
  }
}

static dps310_status_t dps310_spi_write(dps310_t *dev, const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (length == 0U))
  {
    return DPS310_ERROR_INVALID_ARG;
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, length, dev->timeout_ms);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return DPS310_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return DPS310_ERROR_HAL;
  }

  return DPS310_OK;
}
