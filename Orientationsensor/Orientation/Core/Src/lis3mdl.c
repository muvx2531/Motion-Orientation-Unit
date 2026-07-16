#include "lis3mdl.h"

#define LIS3MDL_REG_WHO_AM_I             0x0FU
#define LIS3MDL_REG_CTRL_REG1            0x20U
#define LIS3MDL_REG_CTRL_REG2            0x21U
#define LIS3MDL_REG_CTRL_REG3            0x22U
#define LIS3MDL_REG_CTRL_REG4            0x23U
#define LIS3MDL_REG_CTRL_REG5            0x24U
#define LIS3MDL_REG_OUT_X_L              0x28U

#define LIS3MDL_SPI_READ_BIT             0x80U
#define LIS3MDL_SPI_AUTO_INCREMENT_BIT   0x40U
#define LIS3MDL_SPI_WRITE_MASK           0x3FU
#define LIS3MDL_DEFAULT_TIMEOUT_MS       100U
#define LIS3MDL_CTRL_REG1_10HZ_UHP       0x70U
#define LIS3MDL_CTRL_REG3_CONTINUOUS     0x00U
#define LIS3MDL_CTRL_REG4_Z_UHP          0x0CU
#define LIS3MDL_CTRL_REG5_BDU            0x40U

static lis3mdl_status_t lis3mdl_spi_write(lis3mdl_t *dev, const uint8_t *data, uint16_t length);
static int16_t lis3mdl_le16_to_i16(const uint8_t *data);
static uint8_t lis3mdl_full_scale_bits(lis3mdl_full_scale_t full_scale);

lis3mdl_status_t lis3mdl_init(lis3mdl_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin)
{
  if ((dev == NULL) || (hspi == NULL) || (cs_port == NULL))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->timeout_ms = LIS3MDL_DEFAULT_TIMEOUT_MS;
  dev->full_scale = LIS3MDL_FS_4_GAUSS;
  dev->lsb_per_gauss = lis3mdl_lsb_per_gauss(dev->full_scale);

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  return lis3mdl_check_who_am_i(dev);
}

lis3mdl_status_t lis3mdl_read_who_am_i(lis3mdl_t *dev, uint8_t *who_am_i)
{
  return lis3mdl_read_register(dev, LIS3MDL_REG_WHO_AM_I, who_am_i);
}

lis3mdl_status_t lis3mdl_check_who_am_i(lis3mdl_t *dev)
{
  uint8_t who_am_i;
  lis3mdl_status_t status;

  status = lis3mdl_read_who_am_i(dev, &who_am_i);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  if (who_am_i != LIS3MDL_WHO_AM_I_EXPECTED)
  {
    return LIS3MDL_ERROR_WHO_AM_I;
  }

  return LIS3MDL_OK;
}

lis3mdl_status_t lis3mdl_configure_continuous(lis3mdl_t *dev, lis3mdl_full_scale_t full_scale)
{
  lis3mdl_status_t status;

  if ((dev == NULL) || (full_scale > LIS3MDL_FS_16_GAUSS))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  status = lis3mdl_write_register(dev, LIS3MDL_REG_CTRL_REG1, LIS3MDL_CTRL_REG1_10HZ_UHP);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  status = lis3mdl_write_register(dev, LIS3MDL_REG_CTRL_REG2, lis3mdl_full_scale_bits(full_scale));
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  status = lis3mdl_write_register(dev, LIS3MDL_REG_CTRL_REG3, LIS3MDL_CTRL_REG3_CONTINUOUS);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  status = lis3mdl_write_register(dev, LIS3MDL_REG_CTRL_REG4, LIS3MDL_CTRL_REG4_Z_UHP);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  status = lis3mdl_write_register(dev, LIS3MDL_REG_CTRL_REG5, LIS3MDL_CTRL_REG5_BDU);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  dev->full_scale = full_scale;
  dev->lsb_per_gauss = lis3mdl_lsb_per_gauss(full_scale);

  return LIS3MDL_OK;
}

lis3mdl_status_t lis3mdl_read_raw(lis3mdl_t *dev, lis3mdl_vector_raw_t *raw)
{
  uint8_t data[6];
  lis3mdl_status_t status;

  if (raw == NULL)
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  status = lis3mdl_read_registers(dev, LIS3MDL_REG_OUT_X_L, data, sizeof(data));
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  raw->x = lis3mdl_le16_to_i16(&data[0]);
  raw->y = lis3mdl_le16_to_i16(&data[2]);
  raw->z = lis3mdl_le16_to_i16(&data[4]);

  return LIS3MDL_OK;
}

lis3mdl_status_t lis3mdl_read_gauss(lis3mdl_t *dev, lis3mdl_vector_t *mag_gauss)
{
  lis3mdl_vector_raw_t raw;
  lis3mdl_status_t status;

  if ((dev == NULL) || (mag_gauss == NULL))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  status = lis3mdl_read_raw(dev, &raw);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  mag_gauss->x = (float)raw.x / dev->lsb_per_gauss;
  mag_gauss->y = (float)raw.y / dev->lsb_per_gauss;
  mag_gauss->z = (float)raw.z / dev->lsb_per_gauss;

  return LIS3MDL_OK;
}

lis3mdl_status_t lis3mdl_read_calibrated(lis3mdl_t *dev,
                                         const lis3mdl_calibration_t *calibration,
                                         lis3mdl_vector_t *mag_gauss)
{
  lis3mdl_vector_t uncalibrated;
  lis3mdl_vector_t centered;
  lis3mdl_status_t status;

  if ((calibration == NULL) || (mag_gauss == NULL))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  status = lis3mdl_read_gauss(dev, &uncalibrated);
  if (status != LIS3MDL_OK)
  {
    return status;
  }

  centered.x = uncalibrated.x - calibration->hard_iron_gauss.x;
  centered.y = uncalibrated.y - calibration->hard_iron_gauss.y;
  centered.z = uncalibrated.z - calibration->hard_iron_gauss.z;

  mag_gauss->x = (calibration->soft_iron_matrix[0][0] * centered.x) +
                 (calibration->soft_iron_matrix[0][1] * centered.y) +
                 (calibration->soft_iron_matrix[0][2] * centered.z);
  mag_gauss->y = (calibration->soft_iron_matrix[1][0] * centered.x) +
                 (calibration->soft_iron_matrix[1][1] * centered.y) +
                 (calibration->soft_iron_matrix[1][2] * centered.z);
  mag_gauss->z = (calibration->soft_iron_matrix[2][0] * centered.x) +
                 (calibration->soft_iron_matrix[2][1] * centered.y) +
                 (calibration->soft_iron_matrix[2][2] * centered.z);

  return LIS3MDL_OK;
}

void lis3mdl_calibration_identity(lis3mdl_calibration_t *calibration)
{
  if (calibration == NULL)
  {
    return;
  }

  calibration->hard_iron_gauss.x = 0.0f;
  calibration->hard_iron_gauss.y = 0.0f;
  calibration->hard_iron_gauss.z = 0.0f;
  calibration->soft_iron_matrix[0][0] = 1.0f;
  calibration->soft_iron_matrix[0][1] = 0.0f;
  calibration->soft_iron_matrix[0][2] = 0.0f;
  calibration->soft_iron_matrix[1][0] = 0.0f;
  calibration->soft_iron_matrix[1][1] = 1.0f;
  calibration->soft_iron_matrix[1][2] = 0.0f;
  calibration->soft_iron_matrix[2][0] = 0.0f;
  calibration->soft_iron_matrix[2][1] = 0.0f;
  calibration->soft_iron_matrix[2][2] = 1.0f;
}

float lis3mdl_lsb_per_gauss(lis3mdl_full_scale_t full_scale)
{
  switch (full_scale)
  {
    case LIS3MDL_FS_4_GAUSS:
      return 6842.0f;
    case LIS3MDL_FS_8_GAUSS:
      return 3421.0f;
    case LIS3MDL_FS_12_GAUSS:
      return 2281.0f;
    case LIS3MDL_FS_16_GAUSS:
      return 1711.0f;
    default:
      return 6842.0f;
  }
}

lis3mdl_status_t lis3mdl_write_register(lis3mdl_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  if (dev == NULL)
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  tx[0] = reg & LIS3MDL_SPI_WRITE_MASK;
  tx[1] = value;

  return lis3mdl_spi_write(dev, tx, sizeof(tx));
}

lis3mdl_status_t lis3mdl_read_register(lis3mdl_t *dev, uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  return lis3mdl_read_registers(dev, reg, value, 1U);
}

lis3mdl_status_t lis3mdl_read_registers(lis3mdl_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx_header;
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (length == 0U))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  tx_header = reg | LIS3MDL_SPI_READ_BIT;
  if (length > 1U)
  {
    tx_header |= LIS3MDL_SPI_AUTO_INCREMENT_BIT;
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(dev->hspi, &tx_header, 1U, dev->timeout_ms);
  if (hal_status == HAL_OK)
  {
    hal_status = HAL_SPI_Receive(dev->hspi, data, length, dev->timeout_ms);
  }
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return LIS3MDL_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return LIS3MDL_ERROR_HAL;
  }

  return LIS3MDL_OK;
}

static int16_t lis3mdl_le16_to_i16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static uint8_t lis3mdl_full_scale_bits(lis3mdl_full_scale_t full_scale)
{
  return (uint8_t)(((uint8_t)full_scale & 0x03U) << 5);
}

static lis3mdl_status_t lis3mdl_spi_write(lis3mdl_t *dev, const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (length == 0U))
  {
    return LIS3MDL_ERROR_INVALID_ARG;
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, length, dev->timeout_ms);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return LIS3MDL_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return LIS3MDL_ERROR_HAL;
  }

  return LIS3MDL_OK;
}
