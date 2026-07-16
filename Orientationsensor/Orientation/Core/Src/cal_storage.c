#include "cal_storage.h"
#include <stddef.h>
#include <string.h>

#define CAL_STORAGE_CRC_INIT             0xFFFFFFFFUL
#define CAL_STORAGE_CRC_POLY             0xEDB88320UL

static const cal_storage_block_t *cal_storage_flash_block(void);
static uint32_t cal_storage_crc_length(void);
static cal_storage_status_t cal_storage_program_block(const cal_storage_block_t *block);

cal_storage_status_t cal_storage_load(cal_storage_block_t *block)
{
  const cal_storage_block_t *stored;

  if (block == NULL)
  {
    return CAL_STORAGE_ERROR_INVALID_ARG;
  }

  stored = cal_storage_flash_block();
  memcpy(block, stored, sizeof(*block));

  if (cal_storage_is_valid(block) == 0U)
  {
    cal_storage_set_defaults(block);
    return CAL_STORAGE_ERROR_INVALID_DATA;
  }

  return CAL_STORAGE_OK;
}

cal_storage_status_t cal_storage_save(const cal_storage_block_t *block)
{
  cal_storage_block_t prepared;
  cal_storage_status_t status;
  cal_storage_block_t verify;

  if (block == NULL)
  {
    return CAL_STORAGE_ERROR_INVALID_ARG;
  }

  prepared = *block;
  prepared.magic = CAL_STORAGE_MAGIC;
  prepared.version = CAL_STORAGE_VERSION;
  prepared.length = (uint16_t)sizeof(prepared);
  prepared.flags |= CAL_STORAGE_VALID_FLAG;
  prepared.crc32 = cal_storage_crc32(&prepared, cal_storage_crc_length());

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = CAL_STORAGE_FLASH_PAGE;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return CAL_STORAGE_ERROR_FLASH;
  }

  status = cal_storage_program_block(&prepared);
  HAL_FLASH_Lock();

  if (status != CAL_STORAGE_OK)
  {
    return status;
  }

  memcpy(&verify, cal_storage_flash_block(), sizeof(verify));
  if (cal_storage_is_valid(&verify) == 0U)
  {
    return CAL_STORAGE_ERROR_VERIFY;
  }

  return CAL_STORAGE_OK;
}

void cal_storage_set_defaults(cal_storage_block_t *block)
{
  uint32_t i;

  if (block == NULL)
  {
    return;
  }

  memset(block, 0, sizeof(*block));

  block->magic = CAL_STORAGE_MAGIC;
  block->version = CAL_STORAGE_VERSION;
  block->length = (uint16_t)sizeof(*block);
  block->flags = 0U;

  block->mpu6500.accel_scale[0] = 1.0f;
  block->mpu6500.accel_scale[1] = 1.0f;
  block->mpu6500.accel_scale[2] = 1.0f;

  for (i = 0U; i < 3U; i++)
  {
    block->lis3mdl.soft_iron_matrix[i][i] = 1.0f;
  }

  block->dps310.reference_pressure_pa = 101325.0f;
  block->dps310.reference_altitude_m = 0.0f;
  block->crc32 = cal_storage_crc32(block, cal_storage_crc_length());
}

uint8_t cal_storage_is_valid(const cal_storage_block_t *block)
{
  uint32_t expected_crc;

  if (block == NULL)
  {
    return 0U;
  }

  if ((block->magic != CAL_STORAGE_MAGIC) ||
      (block->version != CAL_STORAGE_VERSION) ||
      (block->length != sizeof(*block)) ||
      ((block->flags & CAL_STORAGE_VALID_FLAG) == 0U))
  {
    return 0U;
  }

  expected_crc = cal_storage_crc32(block, cal_storage_crc_length());
  return (expected_crc == block->crc32) ? 1U : 0U;
}

uint32_t cal_storage_crc32(const void *data, uint32_t length)
{
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = CAL_STORAGE_CRC_INIT;
  uint32_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < length; i++)
  {
    crc ^= bytes[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (crc >> 1) ^ CAL_STORAGE_CRC_POLY;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

static const cal_storage_block_t *cal_storage_flash_block(void)
{
  return (const cal_storage_block_t *)CAL_STORAGE_FLASH_ADDR;
}

static uint32_t cal_storage_crc_length(void)
{
  return (uint32_t)offsetof(cal_storage_block_t, crc32);
}

static cal_storage_status_t cal_storage_program_block(const cal_storage_block_t *block)
{
  const uint8_t *src = (const uint8_t *)block;
  uint32_t address = CAL_STORAGE_FLASH_ADDR;
  uint32_t words = (uint32_t)((sizeof(*block) + sizeof(uint64_t) - 1U) / sizeof(uint64_t));
  uint32_t i;

  for (i = 0U; i < words; i++)
  {
    uint64_t data = 0xFFFFFFFFFFFFFFFFULL;
    uint32_t remaining = (uint32_t)sizeof(*block) - (i * (uint32_t)sizeof(uint64_t));
    uint32_t chunk = (remaining >= sizeof(uint64_t)) ? (uint32_t)sizeof(uint64_t) : remaining;

    memcpy(&data, &src[i * sizeof(uint64_t)], chunk);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) != HAL_OK)
    {
      return CAL_STORAGE_ERROR_FLASH;
    }
    address += sizeof(uint64_t);
  }

  return CAL_STORAGE_OK;
}
