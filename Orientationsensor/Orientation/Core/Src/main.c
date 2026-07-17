/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_calibration.h"
#include "dps310.h"
#include "lis3mdl.h"
#include "madgwickFilter.h"
#include "mpu6500.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float x;
  float y;
  float z;
} vec3f_t;

typedef struct
{
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
} euler_angles_t;

typedef struct
{
  uint64_t timestamp_us;
  vec3f_t accel_mps2;
  vec3f_t gyro_dps;
  vec3f_t mag_uT;
  float pressure_pa;
  float temperature_c;
  euler_angles_t euler;
} telemetry_sample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Uncomment DO_CALIBRATE_SENSORS only when you want to run one-time calibration.
 * After calibration is saved, comment it again and flash the board again.
 */
//#define DO_CALIBRATE_SENSORS

#ifdef DO_CALIBRATE_SENSORS
#define APP_CALIBRATE_MAG_MINMAX_DURATION_MS  30000U
#define APP_CALIBRATE_LOAD_DEFAULTS_FIRST     0U
#define APP_CALIBRATE_ACCEL_FROM_USER_VALUES  1U

#define USER_CAL_ACCEL_POS_X_X_RAW            0.0f
#define USER_CAL_ACCEL_POS_X_Y_RAW            0.0f
#define USER_CAL_ACCEL_POS_X_Z_RAW            0.0f

#define USER_CAL_ACCEL_NEG_X_X_RAW            0.0f
#define USER_CAL_ACCEL_NEG_X_Y_RAW            0.0f
#define USER_CAL_ACCEL_NEG_X_Z_RAW            0.0f

#define USER_CAL_ACCEL_POS_Y_X_RAW            0.0f
#define USER_CAL_ACCEL_POS_Y_Y_RAW            0.0f
#define USER_CAL_ACCEL_POS_Y_Z_RAW            0.0f

#define USER_CAL_ACCEL_NEG_Y_X_RAW            0.0f
#define USER_CAL_ACCEL_NEG_Y_Y_RAW            0.0f
#define USER_CAL_ACCEL_NEG_Y_Z_RAW            0.0f

#define USER_CAL_ACCEL_POS_Z_X_RAW            0.0f
#define USER_CAL_ACCEL_POS_Z_Y_RAW            0.0f
#define USER_CAL_ACCEL_POS_Z_Z_RAW            0.0f

#define USER_CAL_ACCEL_NEG_Z_X_RAW            0.0f
#define USER_CAL_ACCEL_NEG_Z_Y_RAW            0.0f
#define USER_CAL_ACCEL_NEG_Z_Z_RAW            0.0f
#endif

#define APP_SENSOR_READ_PERIOD_MS             10U
#define APP_TELEMETRY_PERIOD_MS               100U
#define APP_STANDARD_GRAVITY_MPS2             9.80665f
#define APP_GAUSS_TO_MICROTESLA               100.0f
#define APP_DEG_TO_RAD                        0.017453292519943295f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static mpu6500_t imu;
static lis3mdl_t mag;
static dps310_t prs;

static mpu6500_config_t imu_config = {
  .gyro_fs = MPU6500_GYRO_FS_500DPS,
  .accel_fs = MPU6500_ACCEL_FS_4G,
  .dlpf_cfg = 3U,
  .sample_rate_divider = 9U
};

static lis3mdl_calibration_t mag_calibration;
static dps310_calibration_t prs_calibration;
static app_calibration_t calibration_app;

static mpu6500_raw_sample_t imu_raw_sample;
static sensor_cal_vector_t accel_compensated_raw;
static sensor_cal_vector_t gyro_compensated_raw;
static lis3mdl_vector_t mag_raw_gauss;
static lis3mdl_vector_t mag_compensated_gauss;
static dps310_sample_t prs_compensated_sample;
static telemetry_sample_t telemetry_sample;
static uint32_t telemetry_last_send_ms = 0U;

volatile mpu6500_status_t imu_status = MPU6500_ERROR;
volatile lis3mdl_status_t mag_status = LIS3MDL_ERROR;
volatile dps310_status_t prs_status = DPS310_ERROR;
volatile mpu6500_status_t imu_read_status = MPU6500_ERROR;
volatile lis3mdl_status_t mag_read_status = LIS3MDL_ERROR;
volatile dps310_status_t prs_read_status = DPS310_ERROR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void App_SensorsInit(void);
static void App_SensorsCalibrationSection(void);
static void App_SensorsReadAndCompensate(void);
static void App_OrientationUpdateMadgwick(void);
static void App_TelemetryUpdateSample(uint64_t timestamp_us);
static void App_TelemetrySendCdc(void);
static int32_t App_FloatToInt32Scaled(float value, float scale);
static uint64_t App_TimestampUs(void);
#ifdef DO_CALIBRATE_SENSORS
static void App_LoadUserAccelCalibrationSamples(void);
static void App_CalibrateMagnetometerBlocking(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  App_SensorsInit();
  App_SensorsCalibrationSection();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint64_t timestamp_us = App_TimestampUs();

    App_SensorsReadAndCompensate();
    App_OrientationUpdateMadgwick();
    App_TelemetryUpdateSample(timestamp_us);
    App_TelemetrySendCdc();
    HAL_Delay(APP_SENSOR_READ_PERIOD_MS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFUL;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_MAG_CS_Pin|SPI_PRS_CS_Pin|SPI_IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_FSync_Pin|LED_Blink_Pin|GPS_PowerON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MAG_INT_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAG_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_MAG_CS_Pin SPI_PRS_CS_Pin SPI_IMU_CS_Pin */
  GPIO_InitStruct.Pin = SPI_MAG_CS_Pin|SPI_PRS_CS_Pin|SPI_IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_FSync_Pin LED_Blink_Pin GPS_PowerON_Pin */
  GPIO_InitStruct.Pin = IMU_FSync_Pin|LED_Blink_Pin|GPS_PowerON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void App_SensorsInit(void)
{
  app_calibration_init(&calibration_app, &imu, &mag, &prs, &prs_calibration, &mag_calibration);

  HAL_GPIO_WritePin(SPI_MAG_CS_GPIO_Port, SPI_MAG_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_PRS_CS_GPIO_Port, SPI_PRS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_IMU_CS_GPIO_Port, SPI_IMU_CS_Pin, GPIO_PIN_SET);

  imu_status = mpu6500_init(&imu, &hspi1, SPI_IMU_CS_GPIO_Port, SPI_IMU_CS_Pin, &imu_config);

  mag_status = lis3mdl_init(&mag, &hspi1, SPI_MAG_CS_GPIO_Port, SPI_MAG_CS_Pin);
  if (mag_status == LIS3MDL_OK)
  {
    mag_status = lis3mdl_configure_continuous(&mag, LIS3MDL_FS_4_GAUSS);
  }

  prs_status = dps310_init(&prs, &hspi1, SPI_PRS_CS_GPIO_Port, SPI_PRS_CS_Pin);
  if (prs_status == DPS310_OK)
  {
    prs_status = dps310_read_calibration(&prs, &prs_calibration);
  }
  if (prs_status == DPS310_OK)
  {
    prs_status = dps310_configure_continuous(&prs, DPS310_OVERSAMPLING_1, DPS310_OVERSAMPLING_1);
  }
}

static void App_SensorsCalibrationSection(void)
{
#ifdef DO_CALIBRATE_SENSORS
  #if (APP_CALIBRATE_LOAD_DEFAULTS_FIRST != 0U)
    app_calibration_request(&calibration_app, APP_CAL_CMD_LOAD_DEFAULTS);
    app_calibration_process(&calibration_app);
  #endif

  if (imu_status == MPU6500_OK)
  {
    app_calibration_request(&calibration_app, APP_CAL_CMD_GYRO_ZERO);
    app_calibration_process(&calibration_app);

  #if (APP_CALIBRATE_ACCEL_FROM_USER_VALUES != 0U)
    App_LoadUserAccelCalibrationSamples();
    app_calibration_request(&calibration_app, APP_CAL_CMD_ACCEL_COMPUTE_SAVE);
    app_calibration_process(&calibration_app);
  #endif
  }

  if (prs_status == DPS310_OK)
  {
    app_calibration_request(&calibration_app, APP_CAL_CMD_DPS310_REF_SAVE);
    app_calibration_process(&calibration_app);
  }

  if (mag_status == LIS3MDL_OK)
  {
    App_CalibrateMagnetometerBlocking();
  }
#endif

  app_calibration_apply_stored(&calibration_app);
}

static void App_SensorsReadAndCompensate(void)
{
  if (imu_status == MPU6500_OK)
  {
    imu_read_status = mpu6500_read_raw(&imu, &imu_raw_sample);
    if (imu_read_status == MPU6500_OK)
    {
      sensor_cal_apply_mpu6500_accel_raw(&calibration_app.block.mpu6500,
                                         &imu_raw_sample.accel,
                                         &accel_compensated_raw);
      sensor_cal_apply_mpu6500_gyro_raw(&calibration_app.block.mpu6500,
                                        &imu_raw_sample.gyro,
                                        &gyro_compensated_raw);
    }
  }

  if (mag_status == LIS3MDL_OK)
  {
    mag_read_status = lis3mdl_read_gauss(&mag, &mag_raw_gauss);
    if (mag_read_status == LIS3MDL_OK)
    {
      sensor_cal_apply_lis3mdl_gauss(&calibration_app.block.lis3mdl,
                                     &mag_raw_gauss,
                                     &mag_compensated_gauss);
    }
  }

  if (prs_status == DPS310_OK)
  {
    prs_read_status = dps310_read_compensated(&prs,
                                             &prs_calibration,
                                             DPS310_OVERSAMPLING_1,
                                             DPS310_OVERSAMPLING_1,
                                             &prs_compensated_sample);
  }
}

static void App_OrientationUpdateMadgwick(void)
{
  float accel_g_x;
  float accel_g_y;
  float accel_g_z;
  float gyro_rad_s_x;
  float gyro_rad_s_y;
  float gyro_rad_s_z;

  if ((imu_status != MPU6500_OK) || (imu_read_status != MPU6500_OK))
  {
    return;
  }

  accel_g_x = accel_compensated_raw.x / imu.accel_lsb_per_g;
  accel_g_y = accel_compensated_raw.y / imu.accel_lsb_per_g;
  accel_g_z = accel_compensated_raw.z / imu.accel_lsb_per_g;

  gyro_rad_s_x = (gyro_compensated_raw.x / imu.gyro_lsb_per_dps) * APP_DEG_TO_RAD;
  gyro_rad_s_y = (gyro_compensated_raw.y / imu.gyro_lsb_per_dps) * APP_DEG_TO_RAD;
  gyro_rad_s_z = (gyro_compensated_raw.z / imu.gyro_lsb_per_dps) * APP_DEG_TO_RAD;

  if ((mag_status == LIS3MDL_OK) && (mag_read_status == LIS3MDL_OK))
  {
    marg_filter(accel_g_x,
                accel_g_y,
                accel_g_z,
                gyro_rad_s_x,
                gyro_rad_s_y,
                gyro_rad_s_z,
                mag_compensated_gauss.x,
                mag_compensated_gauss.y,
                mag_compensated_gauss.z);
  }
  else
  {
    imu_filter(accel_g_x,
               accel_g_y,
               accel_g_z,
               gyro_rad_s_x,
               gyro_rad_s_y,
               gyro_rad_s_z);
  }

  eulerAngles(q_est,
              &telemetry_sample.euler.roll_deg,
              &telemetry_sample.euler.pitch_deg,
              &telemetry_sample.euler.yaw_deg);
}

static void App_TelemetryUpdateSample(uint64_t timestamp_us)
{
  telemetry_sample.timestamp_us = timestamp_us;

  if ((imu_status == MPU6500_OK) && (imu_read_status == MPU6500_OK))
  {
    telemetry_sample.accel_mps2.x = (accel_compensated_raw.x / imu.accel_lsb_per_g) * APP_STANDARD_GRAVITY_MPS2;
    telemetry_sample.accel_mps2.y = (accel_compensated_raw.y / imu.accel_lsb_per_g) * APP_STANDARD_GRAVITY_MPS2;
    telemetry_sample.accel_mps2.z = (accel_compensated_raw.z / imu.accel_lsb_per_g) * APP_STANDARD_GRAVITY_MPS2;

    telemetry_sample.gyro_dps.x = gyro_compensated_raw.x / imu.gyro_lsb_per_dps;
    telemetry_sample.gyro_dps.y = gyro_compensated_raw.y / imu.gyro_lsb_per_dps;
    telemetry_sample.gyro_dps.z = gyro_compensated_raw.z / imu.gyro_lsb_per_dps;
  }

  if ((mag_status == LIS3MDL_OK) && (mag_read_status == LIS3MDL_OK))
  {
    telemetry_sample.mag_uT.x = mag_compensated_gauss.x * APP_GAUSS_TO_MICROTESLA;
    telemetry_sample.mag_uT.y = mag_compensated_gauss.y * APP_GAUSS_TO_MICROTESLA;
    telemetry_sample.mag_uT.z = mag_compensated_gauss.z * APP_GAUSS_TO_MICROTESLA;
  }

  if ((prs_status == DPS310_OK) && (prs_read_status == DPS310_OK))
  {
    telemetry_sample.pressure_pa = prs_compensated_sample.pressure_pa;
    telemetry_sample.temperature_c = prs_compensated_sample.temperature_c;
  }
}

static void App_TelemetrySendCdc(void)
{
  static char frame[384];
  int length;
  uint32_t now_ms = HAL_GetTick();
  uint32_t timestamp_us_32 = (uint32_t)telemetry_sample.timestamp_us;

  if ((now_ms - telemetry_last_send_ms) < APP_TELEMETRY_PERIOD_MS)
  {
    return;
  }
  telemetry_last_send_ms = now_ms;

  length = snprintf(frame,
                    sizeof(frame),
                    "TUS=%lu,ACC_MPS2_1000=%ld,%ld,%ld,GYRO_MDPS=%ld,%ld,%ld,"
                    "MAG_UT100=%ld,%ld,%ld,P_PA100=%ld,T_C100=%ld,"
                    "ROLL_MDEG=%ld,PITCH_MDEG=%ld,YAW_MDEG=%ld\r\n",
                    (unsigned long)timestamp_us_32,
                    (long)App_FloatToInt32Scaled(telemetry_sample.accel_mps2.x, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.accel_mps2.y, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.accel_mps2.z, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.gyro_dps.x, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.gyro_dps.y, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.gyro_dps.z, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.mag_uT.x, 100.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.mag_uT.y, 100.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.mag_uT.z, 100.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.pressure_pa, 100.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.temperature_c, 100.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.euler.roll_deg, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.euler.pitch_deg, 1000.0f),
                    (long)App_FloatToInt32Scaled(telemetry_sample.euler.yaw_deg, 1000.0f));

  if ((length > 0) && (length < (int)sizeof(frame)))
  {
    (void)CDC_Transmit_FS((uint8_t *)frame, (uint16_t)length);
  }
}

static int32_t App_FloatToInt32Scaled(float value, float scale)
{
  float scaled = value * scale;

  if (scaled >= 0.0f)
  {
    return (int32_t)(scaled + 0.5f);
  }

  return (int32_t)(scaled - 0.5f);
}

static uint64_t App_TimestampUs(void)
{
  return (uint64_t)__HAL_TIM_GET_COUNTER(&htim2);
}

#ifdef DO_CALIBRATE_SENSORS
static void App_LoadUserAccelCalibrationSamples(void)
{
  sensor_cal_accel_six_position_t *accel_data = &calibration_app.accel_data;

  sensor_cal_accel_six_position_clear(accel_data);

  accel_data->mean[SENSOR_CAL_ACCEL_POS_X].x = USER_CAL_ACCEL_POS_X_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_X].y = USER_CAL_ACCEL_POS_X_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_X].z = USER_CAL_ACCEL_POS_X_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_POS_X] = 1U;

  accel_data->mean[SENSOR_CAL_ACCEL_NEG_X].x = USER_CAL_ACCEL_NEG_X_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_X].y = USER_CAL_ACCEL_NEG_X_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_X].z = USER_CAL_ACCEL_NEG_X_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_NEG_X] = 1U;

  accel_data->mean[SENSOR_CAL_ACCEL_POS_Y].x = USER_CAL_ACCEL_POS_Y_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_Y].y = USER_CAL_ACCEL_POS_Y_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_Y].z = USER_CAL_ACCEL_POS_Y_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_POS_Y] = 1U;

  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Y].x = USER_CAL_ACCEL_NEG_Y_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Y].y = USER_CAL_ACCEL_NEG_Y_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Y].z = USER_CAL_ACCEL_NEG_Y_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_NEG_Y] = 1U;

  accel_data->mean[SENSOR_CAL_ACCEL_POS_Z].x = USER_CAL_ACCEL_POS_Z_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_Z].y = USER_CAL_ACCEL_POS_Z_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_POS_Z].z = USER_CAL_ACCEL_POS_Z_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_POS_Z] = 1U;

  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Z].x = USER_CAL_ACCEL_NEG_Z_X_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Z].y = USER_CAL_ACCEL_NEG_Z_Y_RAW;
  accel_data->mean[SENSOR_CAL_ACCEL_NEG_Z].z = USER_CAL_ACCEL_NEG_Z_Z_RAW;
  accel_data->valid[SENSOR_CAL_ACCEL_NEG_Z] = 1U;
}

static void App_CalibrateMagnetometerBlocking(void)
{
  uint32_t sample_index;
  uint32_t sample_count;

  app_calibration_request(&calibration_app, APP_CAL_CMD_MAG_START);
  app_calibration_process(&calibration_app);

  sample_count = APP_CALIBRATE_MAG_MINMAX_DURATION_MS / APP_MAG_CAL_UPDATE_PERIOD_MS;
  for (sample_index = 0U; sample_index < sample_count; sample_index++)
  {
    app_calibration_update_mag(&calibration_app);
    HAL_Delay(APP_MAG_CAL_UPDATE_PERIOD_MS);
  }

  app_calibration_request(&calibration_app, APP_CAL_CMD_MAG_FINISH_SAVE);
  app_calibration_process(&calibration_app);
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
