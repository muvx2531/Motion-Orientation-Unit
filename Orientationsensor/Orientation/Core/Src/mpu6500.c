#include "mpu6500.h"

#define MPU6500_REG_SMPLRT_DIV           0x19U
#define MPU6500_REG_CONFIG               0x1AU
#define MPU6500_REG_GYRO_CONFIG          0x1BU
#define MPU6500_REG_ACCEL_CONFIG         0x1CU
#define MPU6500_REG_ACCEL_CONFIG2        0x1DU
#define MPU6500_REG_ACCEL_XOUT_H         0x3BU
#define MPU6500_REG_TEMP_OUT_H           0x41U
#define MPU6500_REG_GYRO_XOUT_H          0x43U
#define MPU6500_REG_USER_CTRL            0x6AU
#define MPU6500_REG_PWR_MGMT_1           0x6BU
#define MPU6500_REG_PWR_MGMT_2           0x6CU
#define MPU6500_REG_WHO_AM_I             0x75U

#define MPU6500_SPI_READ_BIT             0x80U
#define MPU6500_SPI_WRITE_MASK           0x7FU
#define MPU6500_DEVICE_RESET_BIT         0x80U
#define MPU6500_PWR_MGMT_1_CLK_PLL_XGYRO 0x01U
#define MPU6500_USER_CTRL_I2C_IF_DIS     0x10U
#define MPU6500_DEFAULT_TIMEOUT_MS       100U

static mpu6500_status_t mpu6500_spi_transfer(mpu6500_t *dev,
                                             uint8_t *tx,
                                             uint8_t *rx,
                                             uint16_t length);
static mpu6500_status_t mpu6500_apply_config(mpu6500_t *dev);
static int16_t mpu6500_be16_to_i16(const uint8_t *data);
static uint8_t mpu6500_gyro_fs_bits(mpu6500_gyro_fs_t gyro_fs);
static uint8_t mpu6500_accel_fs_bits(mpu6500_accel_fs_t accel_fs);

mpu6500_status_t mpu6500_init(mpu6500_t *dev,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port,
                              uint16_t cs_pin,
                              const mpu6500_config_t *config)
{
  uint8_t who_am_i;

  if ((dev == NULL) || (hspi == NULL) || (cs_port == NULL))
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->timeout_ms = MPU6500_DEFAULT_TIMEOUT_MS;

  if (config != NULL)
  {
    dev->config = *config;
  }
  else
  {
    dev->config.gyro_fs = MPU6500_GYRO_FS_500DPS;
    dev->config.accel_fs = MPU6500_ACCEL_FS_4G;
    dev->config.dlpf_cfg = 3U;
    dev->config.sample_rate_divider = 9U;
  }

  dev->accel_lsb_per_g = mpu6500_accel_lsb_per_g(dev->config.accel_fs);
  dev->gyro_lsb_per_dps = mpu6500_gyro_lsb_per_dps(dev->config.gyro_fs);

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  if (mpu6500_write_register(dev, MPU6500_REG_PWR_MGMT_1, MPU6500_DEVICE_RESET_BIT) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  HAL_Delay(100U);

  if (mpu6500_read_who_am_i(dev, &who_am_i) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (who_am_i != MPU6500_WHO_AM_I_EXPECTED)
  {
    return MPU6500_ERROR_WHO_AM_I;
  }

  if (mpu6500_apply_config(dev) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  return MPU6500_OK;
}

mpu6500_status_t mpu6500_read_who_am_i(mpu6500_t *dev, uint8_t *who_am_i)
{
  return mpu6500_read_register(dev, MPU6500_REG_WHO_AM_I, who_am_i);
}

mpu6500_status_t mpu6500_read_raw(mpu6500_t *dev, mpu6500_raw_sample_t *sample)
{
  uint8_t data[14];
  mpu6500_status_t status;

  if (sample == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_registers(dev, MPU6500_REG_ACCEL_XOUT_H, data, sizeof(data));
  if (status != MPU6500_OK)
  {
    return status;
  }

  sample->accel.x = mpu6500_be16_to_i16(&data[0]);
  sample->accel.y = mpu6500_be16_to_i16(&data[2]);
  sample->accel.z = mpu6500_be16_to_i16(&data[4]);
  sample->temperature = mpu6500_be16_to_i16(&data[6]);
  sample->gyro.x = mpu6500_be16_to_i16(&data[8]);
  sample->gyro.y = mpu6500_be16_to_i16(&data[10]);
  sample->gyro.z = mpu6500_be16_to_i16(&data[12]);

  return MPU6500_OK;
}

mpu6500_status_t mpu6500_read_scaled(mpu6500_t *dev, mpu6500_scaled_sample_t *sample)
{
  mpu6500_raw_sample_t raw;
  mpu6500_status_t status;

  if ((dev == NULL) || (sample == NULL))
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_raw(dev, &raw);
  if (status != MPU6500_OK)
  {
    return status;
  }

  sample->accel_g.x = (float)raw.accel.x / dev->accel_lsb_per_g;
  sample->accel_g.y = (float)raw.accel.y / dev->accel_lsb_per_g;
  sample->accel_g.z = (float)raw.accel.z / dev->accel_lsb_per_g;
  sample->gyro_dps.x = (float)raw.gyro.x / dev->gyro_lsb_per_dps;
  sample->gyro_dps.y = (float)raw.gyro.y / dev->gyro_lsb_per_dps;
  sample->gyro_dps.z = (float)raw.gyro.z / dev->gyro_lsb_per_dps;
  sample->temperature_c = ((float)raw.temperature / 333.87f) + 21.0f;

  return MPU6500_OK;
}

mpu6500_status_t mpu6500_read_accel_raw(mpu6500_t *dev, mpu6500_vector_raw_t *accel)
{
  uint8_t data[6];
  mpu6500_status_t status;

  if (accel == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_registers(dev, MPU6500_REG_ACCEL_XOUT_H, data, sizeof(data));
  if (status != MPU6500_OK)
  {
    return status;
  }

  accel->x = mpu6500_be16_to_i16(&data[0]);
  accel->y = mpu6500_be16_to_i16(&data[2]);
  accel->z = mpu6500_be16_to_i16(&data[4]);

  return MPU6500_OK;
}

mpu6500_status_t mpu6500_read_gyro_raw(mpu6500_t *dev, mpu6500_vector_raw_t *gyro)
{
  uint8_t data[6];
  mpu6500_status_t status;

  if (gyro == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_registers(dev, MPU6500_REG_GYRO_XOUT_H, data, sizeof(data));
  if (status != MPU6500_OK)
  {
    return status;
  }

  gyro->x = mpu6500_be16_to_i16(&data[0]);
  gyro->y = mpu6500_be16_to_i16(&data[2]);
  gyro->z = mpu6500_be16_to_i16(&data[4]);

  return MPU6500_OK;
}

mpu6500_status_t mpu6500_read_temperature_raw(mpu6500_t *dev, int16_t *temperature)
{
  uint8_t data[2];
  mpu6500_status_t status;

  if (temperature == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_registers(dev, MPU6500_REG_TEMP_OUT_H, data, sizeof(data));
  if (status != MPU6500_OK)
  {
    return status;
  }

  *temperature = mpu6500_be16_to_i16(data);
  return MPU6500_OK;
}

mpu6500_status_t mpu6500_write_register(mpu6500_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  if (dev == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  tx[0] = reg & MPU6500_SPI_WRITE_MASK;
  tx[1] = value;

  return mpu6500_spi_transfer(dev, tx, NULL, sizeof(tx));
}

mpu6500_status_t mpu6500_read_register(mpu6500_t *dev, uint8_t reg, uint8_t *value)
{
  mpu6500_status_t status;

  if (value == NULL)
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  status = mpu6500_read_registers(dev, reg, value, 1U);
  return status;
}

mpu6500_status_t mpu6500_read_registers(mpu6500_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx_header;
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (length == 0U))
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  tx_header = reg | MPU6500_SPI_READ_BIT;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(dev->hspi, &tx_header, 1U, dev->timeout_ms);
  if (hal_status == HAL_OK)
  {
    hal_status = HAL_SPI_Receive(dev->hspi, data, length, dev->timeout_ms);
  }
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return MPU6500_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  return MPU6500_OK;
}

float mpu6500_accel_lsb_per_g(mpu6500_accel_fs_t accel_fs)
{
  switch (accel_fs)
  {
    case MPU6500_ACCEL_FS_2G:
      return 16384.0f;
    case MPU6500_ACCEL_FS_4G:
      return 8192.0f;
    case MPU6500_ACCEL_FS_8G:
      return 4096.0f;
    case MPU6500_ACCEL_FS_16G:
      return 2048.0f;
    default:
      return 8192.0f;
  }
}

float mpu6500_gyro_lsb_per_dps(mpu6500_gyro_fs_t gyro_fs)
{
  switch (gyro_fs)
  {
    case MPU6500_GYRO_FS_250DPS:
      return 131.0f;
    case MPU6500_GYRO_FS_500DPS:
      return 65.5f;
    case MPU6500_GYRO_FS_1000DPS:
      return 32.8f;
    case MPU6500_GYRO_FS_2000DPS:
      return 16.4f;
    default:
      return 65.5f;
  }
}

static mpu6500_status_t mpu6500_spi_transfer(mpu6500_t *dev,
                                             uint8_t *tx,
                                             uint8_t *rx,
                                             uint16_t length)
{
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (tx == NULL) || (length == 0U))
  {
    return MPU6500_ERROR_INVALID_ARG;
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  if (rx != NULL)
  {
    hal_status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, length, dev->timeout_ms);
  }
  else
  {
    hal_status = HAL_SPI_Transmit(dev->hspi, tx, length, dev->timeout_ms);
  }
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (hal_status == HAL_TIMEOUT)
  {
    return MPU6500_ERROR_TIMEOUT;
  }

  if (hal_status != HAL_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  return MPU6500_OK;
}

static mpu6500_status_t mpu6500_apply_config(mpu6500_t *dev)
{
  if (mpu6500_write_register(dev, MPU6500_REG_PWR_MGMT_1, MPU6500_PWR_MGMT_1_CLK_PLL_XGYRO) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_PWR_MGMT_2, 0x00U) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_USER_CTRL, MPU6500_USER_CTRL_I2C_IF_DIS) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_SMPLRT_DIV, dev->config.sample_rate_divider) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_CONFIG, dev->config.dlpf_cfg & 0x07U) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_GYRO_CONFIG, mpu6500_gyro_fs_bits(dev->config.gyro_fs)) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_ACCEL_CONFIG, mpu6500_accel_fs_bits(dev->config.accel_fs)) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  if (mpu6500_write_register(dev, MPU6500_REG_ACCEL_CONFIG2, dev->config.dlpf_cfg & 0x07U) != MPU6500_OK)
  {
    return MPU6500_ERROR_HAL;
  }

  return MPU6500_OK;
}

static int16_t mpu6500_be16_to_i16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint8_t mpu6500_gyro_fs_bits(mpu6500_gyro_fs_t gyro_fs)
{
  return (uint8_t)(((uint8_t)gyro_fs & 0x03U) << 3);
}

static uint8_t mpu6500_accel_fs_bits(mpu6500_accel_fs_t accel_fs)
{
  return (uint8_t)(((uint8_t)accel_fs & 0x03U) << 3);
}
