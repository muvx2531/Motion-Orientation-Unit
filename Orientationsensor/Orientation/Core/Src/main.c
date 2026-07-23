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
#include <math.h>
#include <stdarg.h>
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

typedef enum
{
  APP_EULER_MODE_ZYX = 0U,
  APP_EULER_MODE_ZXY = 1U
} app_euler_mode_t;

typedef struct
{
  uint64_t timestamp_us;
  vec3f_t accel_mps2;
  vec3f_t gyro_dps;
  vec3f_t mag_uT;
  float pressure_pa;
  float temperature_c;
  euler_angles_t euler;
  app_euler_mode_t euler_mode;
} telemetry_sample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Uncomment DO_CALIBRATE_SENSORS only when you want to run one-time calibration.
 * After calibration is saved, comment it again and flash the board again.
 */
//#define DO_CALIBRATE_SENSORS

/*
 * Manual calibration modes. Enable only one sensor at a time.
 * Calibration mode prints to USB CDC and stops normal telemetry.
 */
//#define CAL_GYRO_ENABLE
//#define CAL_ACCEL_ENABLE
//#define CAL_MAG_ENABLE

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
#define APP_IMU_PERIOD_US                     10000U
#define APP_MAG_PERIOD_US                     50000U
#define APP_PRS_PERIOD_US                     100000U
#define APP_TELEMETRY_PERIOD_US               100000U
#define APP_ORIENTATION_DT_DEFAULT_S          0.01f
#define APP_ORIENTATION_DT_MIN_S              0.001f
#define APP_ORIENTATION_DT_MAX_S              0.05f
#define APP_CDC_PRINT_RETRY_COUNT             20U
#define APP_CDC_PRINT_RETRY_DELAY_MS          2U
#define APP_STANDARD_GRAVITY_MPS2             9.80665f
#define APP_GAUSS_TO_MICROTESLA               100.0f
#define APP_DEG_TO_RAD                        0.017453292519943295f
#define APP_RAD_TO_DEG                        57.29577951308232f
#define APP_MADGWICK_IMU_ONLY_TEST            0U
#define APP_EULER_SWITCH_TO_ZXY_DEG           80.0f
#define APP_EULER_SWITCH_TO_ZYX_DEG           75.0f
#define APP_EULER_ZXY_SINGULAR_MARGIN_DEG     80.0f

#if defined(CAL_GYRO_ENABLE) || defined(CAL_ACCEL_ENABLE) || defined(CAL_MAG_ENABLE)
#define APP_CAL_MODE_ACTIVE                   1U
#else
#define APP_CAL_MODE_ACTIVE                   0U
#endif

#if (defined(CAL_GYRO_ENABLE) && defined(CAL_ACCEL_ENABLE)) || \
    (defined(CAL_GYRO_ENABLE) && defined(CAL_MAG_ENABLE)) || \
    (defined(CAL_ACCEL_ENABLE) && defined(CAL_MAG_ENABLE))
#error "Enable only one manual calibration mode at a time."
#endif

#ifdef CAL_GYRO_ENABLE
#define APP_GYRO_CAL_WARMUP_MS                10000U
#define APP_GYRO_CAL_SAMPLE_COUNT             2000U
#define APP_GYRO_CAL_SAMPLE_DELAY_MS          10U
#define APP_GYRO_CAL_PRINT_PERIOD             100U
#define APP_GYRO_CAL_VERIFY_SAMPLE_COUNT      500U
#define APP_GYRO_CAL_VERIFY_DELAY_MS          10U
#define APP_GYRO_CAL_MAX_STDDEV_MDPS          80.0f
#define APP_GYRO_CAL_MAX_RANGE_MDPS           500.0f
#define APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS     50.0f
#define APP_GYRO_CAL_MAX_ACCEL_RANGE_RAW      400.0f
#endif

#ifdef CAL_ACCEL_ENABLE
#define APP_ACCEL_CAL_MOVE_TIME_MS            10000U
#define APP_ACCEL_CAL_SETTLE_TIME_MS          3000U
#define APP_ACCEL_CAL_SAMPLE_COUNT            500U
#define APP_ACCEL_CAL_SAMPLE_DELAY_MS         10U
#define APP_ACCEL_CAL_PRINT_PERIOD            100U
#define APP_ACCEL_CAL_MAX_RANGE_RAW           300.0f
#define APP_ACCEL_CAL_MAX_GRAVITY_ERROR_RAW   900.0f
#define APP_ACCEL_CAL_MIN_DOMINANT_RAW_SCALE  0.70f
#define APP_ACCEL_CAL_MAX_CROSS_RAW_SCALE     0.45f
#define APP_ACCEL_CAL_MAX_GYRO_MEAN_MDPS      300.0f
#endif

#ifdef CAL_MAG_ENABLE
#define APP_MAG_CAL_DURATION_MS               60000U
#define APP_MAG_CAL_SAMPLE_DELAY_MS           20U
#define APP_MAG_CAL_PRINT_PERIOD              50U
#define APP_MAG_CAL_ACTION_PERIOD_MS          10000U
#define APP_MAG_CAL_MIN_SAMPLE_COUNT          2000U
#define APP_MAG_CAL_MIN_RANGE_GAUSS           0.25f
#define APP_MAG_CAL_MAX_RANGE_RATIO           2.50f
#define APP_MAG_CAL_MIN_FIELD_UT              25.0f
#define APP_MAG_CAL_MAX_FIELD_UT              75.0f
#endif

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

#if (APP_CAL_MODE_ACTIVE == 0U)
static mpu6500_raw_sample_t imu_raw_sample;
static sensor_cal_vector_t accel_compensated_raw;
static sensor_cal_vector_t gyro_compensated_raw;
static lis3mdl_vector_t mag_raw_gauss;
static lis3mdl_vector_t mag_compensated_gauss;
static dps310_sample_t prs_compensated_sample;
static telemetry_sample_t telemetry_sample;
static uint32_t app_last_imu_us = 0U;
static uint32_t app_last_mag_us = 0U;
static uint32_t app_last_prs_us = 0U;
static uint32_t app_last_telemetry_us = 0U;
static uint8_t app_scheduler_started = 0U;
static app_euler_mode_t app_euler_mode = APP_EULER_MODE_ZYX;
#endif

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
#if (APP_CAL_MODE_ACTIVE == 0U)
static void App_NormalModeProcess(uint32_t now_us);
static void App_ReadImuAndCompensate(void);
static void App_ReadMagAndCompensate(void);
static void App_ReadPressureCompensated(void);
static void App_OrientationUpdateMadgwick(float delta_t_s);
static void App_TelemetryUpdateSample(uint64_t timestamp_us);
static void App_TelemetrySendCdc(void);
static void App_UpdateDisplayEuler(const struct quaternion *q);
static void App_QuaternionToEulerZyx(const struct quaternion *q, euler_angles_t *euler);
static void App_QuaternionToEulerZxy(const struct quaternion *q, euler_angles_t *euler);
static float App_ClampFloat(float value, float min_value, float max_value);
static float App_ClampOrientationDeltaTime(uint32_t delta_us);
static uint64_t App_TimestampUs(void);
#endif
static int32_t App_FloatToInt32Scaled(float value, float scale);
#if (APP_CAL_MODE_ACTIVE != 0U)
static void App_CdcPrint(const char *format, ...);
#endif
#ifdef DO_CALIBRATE_SENSORS
static void App_LoadUserAccelCalibrationSamples(void);
static void App_CalibrateMagnetometerBlocking(void);
#endif
#ifdef CAL_GYRO_ENABLE
static void App_CalibrateGyroManualBlocking(void);
#endif
#ifdef CAL_ACCEL_ENABLE
static void App_CalibrateAccelManualBlocking(void);
static sensor_cal_status_t App_AccelCalCollectFace(sensor_cal_accel_position_t position,
                                                   const char *label,
                                                   uint8_t dominant_axis,
                                                   float expected_sign);
#endif
#ifdef CAL_MAG_ENABLE
static void App_CalibrateMagManualBlocking(void);
static const char *App_MagCalActionText(uint32_t elapsed_ms);
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
#ifdef CAL_GYRO_ENABLE
  App_CalibrateGyroManualBlocking();
#endif
#ifdef CAL_ACCEL_ENABLE
  App_CalibrateAccelManualBlocking();
#endif
#ifdef CAL_MAG_ENABLE
  App_CalibrateMagManualBlocking();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (APP_CAL_MODE_ACTIVE == 0U)
    App_NormalModeProcess((uint32_t)App_TimestampUs());
#else
    HAL_Delay(APP_SENSOR_READ_PERIOD_MS);
#endif
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim2.Init.Period = 4.294967295E9;
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

#if (APP_CAL_MODE_ACTIVE == 0U)
static void App_NormalModeProcess(uint32_t now_us)
{
  if (app_scheduler_started == 0U)
  {
    app_last_imu_us = now_us;
    app_last_mag_us = now_us;
    app_last_prs_us = now_us;
    app_last_telemetry_us = now_us;
    app_scheduler_started = 1U;
    return;
  }

  if ((uint32_t)(now_us - app_last_mag_us) >= APP_MAG_PERIOD_US)
  {
    app_last_mag_us += APP_MAG_PERIOD_US;
    App_ReadMagAndCompensate();
  }

  if ((uint32_t)(now_us - app_last_prs_us) >= APP_PRS_PERIOD_US)
  {
    app_last_prs_us += APP_PRS_PERIOD_US;
    App_ReadPressureCompensated();
  }

  if ((uint32_t)(now_us - app_last_imu_us) >= APP_IMU_PERIOD_US)
  {
    uint32_t delta_us = (uint32_t)(now_us - app_last_imu_us);
    float delta_t_s = App_ClampOrientationDeltaTime(delta_us);

    app_last_imu_us = now_us;
    App_ReadImuAndCompensate();
    App_OrientationUpdateMadgwick(delta_t_s);
    App_TelemetryUpdateSample(now_us);
  }

  if ((uint32_t)(now_us - app_last_telemetry_us) >= APP_TELEMETRY_PERIOD_US)
  {
    app_last_telemetry_us += APP_TELEMETRY_PERIOD_US;
    App_TelemetrySendCdc();
  }
}

static void App_ReadImuAndCompensate(void)
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
}

static void App_ReadMagAndCompensate(void)
{
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
}

static void App_ReadPressureCompensated(void)
{
  if (prs_status == DPS310_OK)
  {
    prs_read_status = dps310_read_compensated(&prs,
                                             &prs_calibration,
                                             DPS310_OVERSAMPLING_1,
                                             DPS310_OVERSAMPLING_1,
                                             &prs_compensated_sample);
  }
}

static void App_OrientationUpdateMadgwick(float delta_t_s)
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

  madgwick_set_delta_t(delta_t_s);

#if (APP_MADGWICK_IMU_ONLY_TEST == 0U)
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
#endif
  {
    imu_filter(accel_g_x,
               accel_g_y,
               accel_g_z,
               gyro_rad_s_x,
               gyro_rad_s_y,
               gyro_rad_s_z);
  }

  App_UpdateDisplayEuler(&q_est);
}

static float App_ClampOrientationDeltaTime(uint32_t delta_us)
{
  float delta_t_s = (float)delta_us * 0.000001f;

  if (delta_t_s < APP_ORIENTATION_DT_MIN_S)
  {
    return APP_ORIENTATION_DT_DEFAULT_S;
  }

  if (delta_t_s > APP_ORIENTATION_DT_MAX_S)
  {
    return APP_ORIENTATION_DT_MAX_S;
  }

  return delta_t_s;
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
  uint32_t timestamp_us_32 = (uint32_t)telemetry_sample.timestamp_us;
  uint32_t timestamp_s = timestamp_us_32 / 1000000U;
  uint32_t timestamp_ms = (timestamp_us_32 % 1000000U) / 1000U;
  uint32_t timestamp_subms_us = timestamp_us_32 % 1000U;

  length = snprintf(frame,
                    sizeof(frame),
                    "TS=%lu.%03lu.%03lu,ACC_MPS2_1000=%ld,%ld,%ld,GYRO_MDPS=%ld,%ld,%ld,"
                    "MAG_UT100=%ld,%ld,%ld,P_PA100=%ld,T_C100=%ld,"
                    "ROLL_MDEG=%ld,PITCH_MDEG=%ld,YAW_MDEG=%ld,EULER_MODE=%lu,"
                    "Q_X1000000=%ld,%ld,%ld,%ld\r\n",
                    (unsigned long)timestamp_s,
                    (unsigned long)timestamp_ms,
                    (unsigned long)timestamp_subms_us,
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
                    (long)App_FloatToInt32Scaled(telemetry_sample.euler.yaw_deg, 1000.0f),
                    (unsigned long)telemetry_sample.euler_mode,
                    (long)App_FloatToInt32Scaled(q_est.q2, 1000000.0f),
                    (long)App_FloatToInt32Scaled(q_est.q3, 1000000.0f),
                    (long)App_FloatToInt32Scaled(q_est.q4, 1000000.0f),
                    (long)App_FloatToInt32Scaled(q_est.q1, 1000000.0f));

  if ((length > 0) && (length < (int)sizeof(frame)))
  {
    (void)CDC_Transmit_FS((uint8_t *)frame, (uint16_t)length);
  }
}
#endif

#if (APP_CAL_MODE_ACTIVE == 0U)
static void App_UpdateDisplayEuler(const struct quaternion *q)
{
  euler_angles_t zyx;
  euler_angles_t zxy;

  App_QuaternionToEulerZyx(q, &zyx);
  App_QuaternionToEulerZxy(q, &zxy);

  if (app_euler_mode == APP_EULER_MODE_ZYX)
  {
    if ((fabsf(zyx.pitch_deg) > APP_EULER_SWITCH_TO_ZXY_DEG) &&
        (fabsf(zxy.roll_deg) < APP_EULER_ZXY_SINGULAR_MARGIN_DEG))
    {
      app_euler_mode = APP_EULER_MODE_ZXY;
    }
  }
  else
  {
    if (fabsf(zyx.pitch_deg) < APP_EULER_SWITCH_TO_ZYX_DEG)
    {
      app_euler_mode = APP_EULER_MODE_ZYX;
    }
  }

  if (app_euler_mode == APP_EULER_MODE_ZXY)
  {
    telemetry_sample.euler = zxy;
  }
  else
  {
    telemetry_sample.euler = zyx;
  }

  telemetry_sample.euler_mode = app_euler_mode;
  HAL_GPIO_WritePin(LED_Blink_GPIO_Port,
                    LED_Blink_Pin,
                    (app_euler_mode == APP_EULER_MODE_ZXY) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_QuaternionToEulerZyx(const struct quaternion *q, euler_angles_t *euler)
{
  float sin_pitch;

  if ((q == NULL) || (euler == NULL))
  {
    return;
  }

  euler->roll_deg = atan2f((2.0f * ((q->q1 * q->q2) + (q->q3 * q->q4))),
                           (1.0f - (2.0f * ((q->q2 * q->q2) + (q->q3 * q->q3))))) * APP_RAD_TO_DEG;

  sin_pitch = 2.0f * ((q->q1 * q->q3) - (q->q4 * q->q2));
  sin_pitch = App_ClampFloat(sin_pitch, -1.0f, 1.0f);
  euler->pitch_deg = asinf(sin_pitch) * APP_RAD_TO_DEG;

  euler->yaw_deg = atan2f((2.0f * ((q->q1 * q->q4) + (q->q2 * q->q3))),
                          (1.0f - (2.0f * ((q->q3 * q->q3) + (q->q4 * q->q4))))) * APP_RAD_TO_DEG;
}

static void App_QuaternionToEulerZxy(const struct quaternion *q, euler_angles_t *euler)
{
  float r01;
  float r11;
  float r20;
  float r21;
  float r22;
  float sin_roll;

  if ((q == NULL) || (euler == NULL))
  {
    return;
  }

  r01 = 2.0f * ((q->q2 * q->q3) - (q->q1 * q->q4));
  r11 = 1.0f - (2.0f * ((q->q2 * q->q2) + (q->q4 * q->q4)));
  r20 = 2.0f * ((q->q2 * q->q4) - (q->q1 * q->q3));
  r21 = 2.0f * ((q->q3 * q->q4) + (q->q1 * q->q2));
  r22 = 1.0f - (2.0f * ((q->q2 * q->q2) + (q->q3 * q->q3)));

  sin_roll = App_ClampFloat(r21, -1.0f, 1.0f);
  euler->roll_deg = asinf(sin_roll) * APP_RAD_TO_DEG;
  euler->pitch_deg = atan2f(-r20, r22) * APP_RAD_TO_DEG;
  euler->yaw_deg = atan2f(-r01, r11) * APP_RAD_TO_DEG;
}
#endif

static float App_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
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

#if (APP_CAL_MODE_ACTIVE == 0U)
static uint64_t App_TimestampUs(void)
{
  return (uint64_t)__HAL_TIM_GET_COUNTER(&htim2);
}
#endif

#if (APP_CAL_MODE_ACTIVE != 0U)
static void App_CdcPrint(const char *format, ...)
{
  static char text[192];
  va_list args;
  int length;
  uint32_t retry;

  va_start(args, format);
  length = vsnprintf(text, sizeof(text), format, args);
  va_end(args);

  if ((length <= 0) || (length >= (int)sizeof(text)))
  {
    return;
  }

  for (retry = 0U; retry < APP_CDC_PRINT_RETRY_COUNT; retry++)
  {
    if (CDC_Transmit_FS((uint8_t *)text, (uint16_t)length) == USBD_OK)
    {
      return;
    }
    HAL_Delay(APP_CDC_PRINT_RETRY_DELAY_MS);
  }
}
#endif

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

#ifdef CAL_GYRO_ENABLE
static void App_CalibrateGyroManualBlocking(void)
{
  uint32_t i;
  uint32_t warmup_steps;
  mpu6500_raw_sample_t sample;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  float sum_sq_x = 0.0f;
  float sum_sq_y = 0.0f;
  float sum_sq_z = 0.0f;
  float mean_x;
  float mean_y;
  float mean_z;
  float std_x;
  float std_y;
  float std_z;
  float min_x = 0.0f;
  float min_y = 0.0f;
  float min_z = 0.0f;
  float max_x = 0.0f;
  float max_y = 0.0f;
  float max_z = 0.0f;
  float accel_min_x = 0.0f;
  float accel_min_y = 0.0f;
  float accel_min_z = 0.0f;
  float accel_max_x = 0.0f;
  float accel_max_y = 0.0f;
  float accel_max_z = 0.0f;
  float verify_sum_x = 0.0f;
  float verify_sum_y = 0.0f;
  float verify_sum_z = 0.0f;
  float verify_mean_mdps_x;
  float verify_mean_mdps_y;
  float verify_mean_mdps_z;
  uint8_t pass = 1U;

  App_CdcPrint("\r\nGYRO_CAL: START\r\n");
  App_CdcPrint("GYRO_CAL: PLACE_BOARD_STILL\r\n");

  if (imu_status != MPU6500_OK)
  {
    App_CdcPrint("GYRO_CAL: FAIL IMU_NOT_READY status=%ld\r\n", (long)imu_status);
    return;
  }

  warmup_steps = APP_GYRO_CAL_WARMUP_MS / 100U;
  App_CdcPrint("GYRO_CAL: WARMUP_MS=%lu\r\n", (unsigned long)APP_GYRO_CAL_WARMUP_MS);
  for (i = 0U; i < warmup_steps; i++)
  {
    if (mpu6500_read_raw(&imu, &sample) != MPU6500_OK)
    {
      App_CdcPrint("GYRO_CAL: FAIL WARMUP_READ_ERROR\r\n");
      return;
    }

    App_CdcPrint("GYRO_RAW: X=%ld,Y=%ld,Z=%ld,ACC=%ld,%ld,%ld\r\n",
                 (long)sample.gyro.x,
                 (long)sample.gyro.y,
                 (long)sample.gyro.z,
                 (long)sample.accel.x,
                 (long)sample.accel.y,
                 (long)sample.accel.z);
    HAL_Delay(100U);
  }

  App_CdcPrint("GYRO_CAL: COLLECT samples=%lu delay_ms=%lu\r\n",
               (unsigned long)APP_GYRO_CAL_SAMPLE_COUNT,
               (unsigned long)APP_GYRO_CAL_SAMPLE_DELAY_MS);

  for (i = 0U; i < APP_GYRO_CAL_SAMPLE_COUNT; i++)
  {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;

    if (mpu6500_read_raw(&imu, &sample) != MPU6500_OK)
    {
      App_CdcPrint("GYRO_CAL: FAIL COLLECT_READ_ERROR sample=%lu\r\n", (unsigned long)i);
      return;
    }

    gx = (float)sample.gyro.x;
    gy = (float)sample.gyro.y;
    gz = (float)sample.gyro.z;
    ax = (float)sample.accel.x;
    ay = (float)sample.accel.y;
    az = (float)sample.accel.z;

    if (i == 0U)
    {
      min_x = max_x = gx;
      min_y = max_y = gy;
      min_z = max_z = gz;
      accel_min_x = accel_max_x = ax;
      accel_min_y = accel_max_y = ay;
      accel_min_z = accel_max_z = az;
    }

    if (gx < min_x) { min_x = gx; }
    if (gy < min_y) { min_y = gy; }
    if (gz < min_z) { min_z = gz; }
    if (gx > max_x) { max_x = gx; }
    if (gy > max_y) { max_y = gy; }
    if (gz > max_z) { max_z = gz; }

    if (ax < accel_min_x) { accel_min_x = ax; }
    if (ay < accel_min_y) { accel_min_y = ay; }
    if (az < accel_min_z) { accel_min_z = az; }
    if (ax > accel_max_x) { accel_max_x = ax; }
    if (ay > accel_max_y) { accel_max_y = ay; }
    if (az > accel_max_z) { accel_max_z = az; }

    sum_x += gx;
    sum_y += gy;
    sum_z += gz;
    sum_sq_x += gx * gx;
    sum_sq_y += gy * gy;
    sum_sq_z += gz * gz;

    if ((i % APP_GYRO_CAL_PRINT_PERIOD) == 0U)
    {
      App_CdcPrint("GYRO_RAW: sample=%lu,X=%ld,Y=%ld,Z=%ld,ACC=%ld,%ld,%ld\r\n",
                   (unsigned long)i,
                   (long)sample.gyro.x,
                   (long)sample.gyro.y,
                   (long)sample.gyro.z,
                   (long)sample.accel.x,
                   (long)sample.accel.y,
                   (long)sample.accel.z);
    }

    HAL_Delay(APP_GYRO_CAL_SAMPLE_DELAY_MS);
  }

  mean_x = sum_x / (float)APP_GYRO_CAL_SAMPLE_COUNT;
  mean_y = sum_y / (float)APP_GYRO_CAL_SAMPLE_COUNT;
  mean_z = sum_z / (float)APP_GYRO_CAL_SAMPLE_COUNT;
  std_x = sqrtf((sum_sq_x / (float)APP_GYRO_CAL_SAMPLE_COUNT) - (mean_x * mean_x));
  std_y = sqrtf((sum_sq_y / (float)APP_GYRO_CAL_SAMPLE_COUNT) - (mean_y * mean_y));
  std_z = sqrtf((sum_sq_z / (float)APP_GYRO_CAL_SAMPLE_COUNT) - (mean_z * mean_z));

  App_CdcPrint("GYRO_CAL: OFFSET_RAW_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(mean_x, 1000.0f),
               (long)App_FloatToInt32Scaled(mean_y, 1000.0f),
               (long)App_FloatToInt32Scaled(mean_z, 1000.0f));
  App_CdcPrint("GYRO_CAL: STD_MDPS=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled((std_x / imu.gyro_lsb_per_dps), 1000.0f),
               (long)App_FloatToInt32Scaled((std_y / imu.gyro_lsb_per_dps), 1000.0f),
               (long)App_FloatToInt32Scaled((std_z / imu.gyro_lsb_per_dps), 1000.0f));
  App_CdcPrint("GYRO_CAL: RANGE_MDPS=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(((max_x - min_x) / imu.gyro_lsb_per_dps), 1000.0f),
               (long)App_FloatToInt32Scaled(((max_y - min_y) / imu.gyro_lsb_per_dps), 1000.0f),
               (long)App_FloatToInt32Scaled(((max_z - min_z) / imu.gyro_lsb_per_dps), 1000.0f));
  App_CdcPrint("GYRO_CAL: ACC_RANGE_RAW=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(accel_max_x - accel_min_x, 1.0f),
               (long)App_FloatToInt32Scaled(accel_max_y - accel_min_y, 1.0f),
               (long)App_FloatToInt32Scaled(accel_max_z - accel_min_z, 1.0f));

  if (((std_x / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_STDDEV_MDPS) { pass = 0U; }
  if (((std_y / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_STDDEV_MDPS) { pass = 0U; }
  if (((std_z / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_STDDEV_MDPS) { pass = 0U; }
  if ((((max_x - min_x) / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_RANGE_MDPS) { pass = 0U; }
  if ((((max_y - min_y) / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_RANGE_MDPS) { pass = 0U; }
  if ((((max_z - min_z) / imu.gyro_lsb_per_dps) * 1000.0f) > APP_GYRO_CAL_MAX_RANGE_MDPS) { pass = 0U; }
  if ((accel_max_x - accel_min_x) > APP_GYRO_CAL_MAX_ACCEL_RANGE_RAW) { pass = 0U; }
  if ((accel_max_y - accel_min_y) > APP_GYRO_CAL_MAX_ACCEL_RANGE_RAW) { pass = 0U; }
  if ((accel_max_z - accel_min_z) > APP_GYRO_CAL_MAX_ACCEL_RANGE_RAW) { pass = 0U; }

  if (pass == 0U)
  {
    App_CdcPrint("GYRO_CAL: FAIL MOVEMENT_OR_NOISE_TOO_HIGH\r\n");
    return;
  }

  calibration_app.block.mpu6500.gyro_offset_raw[0] = mean_x;
  calibration_app.block.mpu6500.gyro_offset_raw[1] = mean_y;
  calibration_app.block.mpu6500.gyro_offset_raw[2] = mean_z;
  app_calibration_apply_stored(&calibration_app);

  App_CdcPrint("GYRO_CAL: VERIFY samples=%lu\r\n",
               (unsigned long)APP_GYRO_CAL_VERIFY_SAMPLE_COUNT);
  for (i = 0U; i < APP_GYRO_CAL_VERIFY_SAMPLE_COUNT; i++)
  {
    sensor_cal_vector_t corrected;

    if (mpu6500_read_raw(&imu, &sample) != MPU6500_OK)
    {
      App_CdcPrint("GYRO_CAL: FAIL VERIFY_READ_ERROR sample=%lu\r\n", (unsigned long)i);
      return;
    }

    sensor_cal_apply_mpu6500_gyro_raw(&calibration_app.block.mpu6500, &sample.gyro, &corrected);
    verify_sum_x += corrected.x / imu.gyro_lsb_per_dps;
    verify_sum_y += corrected.y / imu.gyro_lsb_per_dps;
    verify_sum_z += corrected.z / imu.gyro_lsb_per_dps;

    if ((i % APP_GYRO_CAL_PRINT_PERIOD) == 0U)
    {
      App_CdcPrint("GYRO_VERIFY_MDPS: sample=%lu,X=%ld,Y=%ld,Z=%ld\r\n",
                   (unsigned long)i,
                   (long)App_FloatToInt32Scaled(corrected.x / imu.gyro_lsb_per_dps, 1000.0f),
                   (long)App_FloatToInt32Scaled(corrected.y / imu.gyro_lsb_per_dps, 1000.0f),
                   (long)App_FloatToInt32Scaled(corrected.z / imu.gyro_lsb_per_dps, 1000.0f));
    }

    HAL_Delay(APP_GYRO_CAL_VERIFY_DELAY_MS);
  }

  verify_mean_mdps_x = (verify_sum_x / (float)APP_GYRO_CAL_VERIFY_SAMPLE_COUNT) * 1000.0f;
  verify_mean_mdps_y = (verify_sum_y / (float)APP_GYRO_CAL_VERIFY_SAMPLE_COUNT) * 1000.0f;
  verify_mean_mdps_z = (verify_sum_z / (float)APP_GYRO_CAL_VERIFY_SAMPLE_COUNT) * 1000.0f;

  App_CdcPrint("GYRO_CAL: VERIFY_MEAN_MDPS=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(verify_mean_mdps_x, 1.0f),
               (long)App_FloatToInt32Scaled(verify_mean_mdps_y, 1.0f),
               (long)App_FloatToInt32Scaled(verify_mean_mdps_z, 1.0f));

  if ((verify_mean_mdps_x > APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS) ||
      (verify_mean_mdps_x < -APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS) ||
      (verify_mean_mdps_y > APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS) ||
      (verify_mean_mdps_y < -APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS) ||
      (verify_mean_mdps_z > APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS) ||
      (verify_mean_mdps_z < -APP_GYRO_CAL_MAX_VERIFY_MEAN_MDPS))
  {
    App_CdcPrint("GYRO_CAL: FAIL VERIFY_MEAN_TOO_HIGH\r\n");
    return;
  }

  app_calibration_request(&calibration_app, APP_CAL_CMD_SAVE);
  app_calibration_process(&calibration_app);
  if ((calibration_app.status == SENSOR_CAL_OK) &&
      (calibration_app.flash_status == CAL_STORAGE_OK))
  {
    App_CdcPrint("GYRO_CAL: PASS SAVED_TO_FLASH\r\n");
  }
  else
  {
    App_CdcPrint("GYRO_CAL: FAIL FLASH_SAVE status=%ld flash=%ld\r\n",
                 (long)calibration_app.status,
                 (long)calibration_app.flash_status);
  }
}
#endif

#ifdef CAL_ACCEL_ENABLE
static void App_CalibrateAccelManualBlocking(void)
{
  sensor_cal_status_t status;

  App_CdcPrint("\r\nACC_CAL: START\r\n");
  App_CdcPrint("ACC_CAL: Keep cable loose. Follow each face command from terminal.\r\n");

  if (imu_status != MPU6500_OK)
  {
    App_CdcPrint("ACC_CAL: FAIL IMU_NOT_READY status=%ld\r\n", (long)imu_status);
    return;
  }

  sensor_cal_accel_six_position_clear(&calibration_app.accel_data);

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_POS_X, "POS_X_UP", 0U, 1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_NEG_X, "NEG_X_UP", 0U, -1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_POS_Y, "POS_Y_UP", 1U, 1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_NEG_Y, "NEG_Y_UP", 1U, -1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_POS_Z, "POS_Z_UP", 2U, 1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  status = App_AccelCalCollectFace(SENSOR_CAL_ACCEL_NEG_Z, "NEG_Z_UP", 2U, -1.0f);
  if (status != SENSOR_CAL_OK) { return; }

  App_CdcPrint("ACC_CAL: COMPUTE\r\n");
  calibration_app.status = sensor_cal_mpu6500_accel_compute_six_position(&calibration_app.accel_data,
                                                                         imu.accel_lsb_per_g,
                                                                         &calibration_app.block.mpu6500);
  if (calibration_app.status != SENSOR_CAL_OK)
  {
    App_CdcPrint("ACC_CAL: FAIL COMPUTE status=%ld\r\n", (long)calibration_app.status);
    return;
  }

  app_calibration_apply_stored(&calibration_app);

  App_CdcPrint("ACC_CAL: OFFSET_RAW_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_offset_raw[0], 1000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_offset_raw[1], 1000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_offset_raw[2], 1000.0f));
  App_CdcPrint("ACC_CAL: SCALE_X1000000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_scale[0], 1000000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_scale[1], 1000000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.mpu6500.accel_scale[2], 1000000.0f));

  app_calibration_request(&calibration_app, APP_CAL_CMD_SAVE);
  app_calibration_process(&calibration_app);
  if ((calibration_app.status == SENSOR_CAL_OK) &&
      (calibration_app.flash_status == CAL_STORAGE_OK))
  {
    App_CdcPrint("ACC_CAL: PASS SAVED_TO_FLASH\r\n");
  }
  else
  {
    App_CdcPrint("ACC_CAL: FAIL FLASH_SAVE status=%ld flash=%ld\r\n",
                 (long)calibration_app.status,
                 (long)calibration_app.flash_status);
  }
}

static sensor_cal_status_t App_AccelCalCollectFace(sensor_cal_accel_position_t position,
                                                   const char *label,
                                                   uint8_t dominant_axis,
                                                   float expected_sign)
{
  uint32_t i;
  uint32_t countdown;
  mpu6500_raw_sample_t sample;
  sensor_cal_vector_t gyro_corrected;
  sensor_cal_vector_t mean;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  float min_x = 0.0f;
  float min_y = 0.0f;
  float min_z = 0.0f;
  float max_x = 0.0f;
  float max_y = 0.0f;
  float max_z = 0.0f;
  float gyro_sum_x = 0.0f;
  float gyro_sum_y = 0.0f;
  float gyro_sum_z = 0.0f;
  float gravity_mag;
  float dominant_value;
  float cross_a;
  float cross_b;
  float dominant_min_raw = imu.accel_lsb_per_g * APP_ACCEL_CAL_MIN_DOMINANT_RAW_SCALE;
  float cross_max_raw = imu.accel_lsb_per_g * APP_ACCEL_CAL_MAX_CROSS_RAW_SCALE;
  float gyro_mean_mdps_x;
  float gyro_mean_mdps_y;
  float gyro_mean_mdps_z;

  App_CdcPrint("\r\nACC_CAL: STEP PLACE_%s\r\n", label);
  for (countdown = APP_ACCEL_CAL_MOVE_TIME_MS / 1000U; countdown > 0U; countdown--)
  {
    App_CdcPrint("ACC_CAL: MOVE_NOW %lu\r\n", (unsigned long)countdown);
    HAL_Delay(1000U);
  }

  App_CdcPrint("ACC_CAL: DO_NOT_TOUCH %s\r\n", label);
  for (countdown = APP_ACCEL_CAL_SETTLE_TIME_MS / 1000U; countdown > 0U; countdown--)
  {
    App_CdcPrint("ACC_CAL: SETTLING %lu\r\n", (unsigned long)countdown);
    HAL_Delay(1000U);
  }

  App_CdcPrint("ACC_CAL: COLLECT %s samples=%lu delay_ms=%lu\r\n",
               label,
               (unsigned long)APP_ACCEL_CAL_SAMPLE_COUNT,
               (unsigned long)APP_ACCEL_CAL_SAMPLE_DELAY_MS);

  for (i = 0U; i < APP_ACCEL_CAL_SAMPLE_COUNT; i++)
  {
    float ax;
    float ay;
    float az;

    if (mpu6500_read_raw(&imu, &sample) != MPU6500_OK)
    {
      App_CdcPrint("ACC_CAL: FAIL READ_ERROR %s sample=%lu\r\n",
                   label,
                   (unsigned long)i);
      return SENSOR_CAL_ERROR_SENSOR;
    }

    ax = (float)sample.accel.x;
    ay = (float)sample.accel.y;
    az = (float)sample.accel.z;

    if (i == 0U)
    {
      min_x = max_x = ax;
      min_y = max_y = ay;
      min_z = max_z = az;
    }

    if (ax < min_x) { min_x = ax; }
    if (ay < min_y) { min_y = ay; }
    if (az < min_z) { min_z = az; }
    if (ax > max_x) { max_x = ax; }
    if (ay > max_y) { max_y = ay; }
    if (az > max_z) { max_z = az; }

    sum_x += ax;
    sum_y += ay;
    sum_z += az;

    sensor_cal_apply_mpu6500_gyro_raw(&calibration_app.block.mpu6500, &sample.gyro, &gyro_corrected);
    gyro_sum_x += gyro_corrected.x / imu.gyro_lsb_per_dps;
    gyro_sum_y += gyro_corrected.y / imu.gyro_lsb_per_dps;
    gyro_sum_z += gyro_corrected.z / imu.gyro_lsb_per_dps;

    if ((i % APP_ACCEL_CAL_PRINT_PERIOD) == 0U)
    {
      App_CdcPrint("ACC_RAW: %s sample=%lu,X=%ld,Y=%ld,Z=%ld,GYRO_MDPS=%ld,%ld,%ld\r\n",
                   label,
                   (unsigned long)i,
                   (long)sample.accel.x,
                   (long)sample.accel.y,
                   (long)sample.accel.z,
                   (long)App_FloatToInt32Scaled(gyro_corrected.x / imu.gyro_lsb_per_dps, 1000.0f),
                   (long)App_FloatToInt32Scaled(gyro_corrected.y / imu.gyro_lsb_per_dps, 1000.0f),
                   (long)App_FloatToInt32Scaled(gyro_corrected.z / imu.gyro_lsb_per_dps, 1000.0f));
    }

    HAL_Delay(APP_ACCEL_CAL_SAMPLE_DELAY_MS);
  }

  mean.x = sum_x / (float)APP_ACCEL_CAL_SAMPLE_COUNT;
  mean.y = sum_y / (float)APP_ACCEL_CAL_SAMPLE_COUNT;
  mean.z = sum_z / (float)APP_ACCEL_CAL_SAMPLE_COUNT;
  gravity_mag = sqrtf((mean.x * mean.x) + (mean.y * mean.y) + (mean.z * mean.z));
  gyro_mean_mdps_x = (gyro_sum_x / (float)APP_ACCEL_CAL_SAMPLE_COUNT) * 1000.0f;
  gyro_mean_mdps_y = (gyro_sum_y / (float)APP_ACCEL_CAL_SAMPLE_COUNT) * 1000.0f;
  gyro_mean_mdps_z = (gyro_sum_z / (float)APP_ACCEL_CAL_SAMPLE_COUNT) * 1000.0f;

  if (dominant_axis == 0U)
  {
    dominant_value = mean.x * expected_sign;
    cross_a = fabsf(mean.y);
    cross_b = fabsf(mean.z);
  }
  else if (dominant_axis == 1U)
  {
    dominant_value = mean.y * expected_sign;
    cross_a = fabsf(mean.x);
    cross_b = fabsf(mean.z);
  }
  else
  {
    dominant_value = mean.z * expected_sign;
    cross_a = fabsf(mean.x);
    cross_b = fabsf(mean.y);
  }

  App_CdcPrint("ACC_CAL: MEAN_%s=%ld,%ld,%ld\r\n",
               label,
               (long)App_FloatToInt32Scaled(mean.x, 1.0f),
               (long)App_FloatToInt32Scaled(mean.y, 1.0f),
               (long)App_FloatToInt32Scaled(mean.z, 1.0f));
  App_CdcPrint("ACC_CAL: RANGE_RAW_%s=%ld,%ld,%ld\r\n",
               label,
               (long)App_FloatToInt32Scaled(max_x - min_x, 1.0f),
               (long)App_FloatToInt32Scaled(max_y - min_y, 1.0f),
               (long)App_FloatToInt32Scaled(max_z - min_z, 1.0f));
  App_CdcPrint("ACC_CAL: GRAVITY_RAW_%s=%ld\r\n",
               label,
               (long)App_FloatToInt32Scaled(gravity_mag, 1.0f));
  App_CdcPrint("ACC_CAL: GYRO_MEAN_MDPS_%s=%ld,%ld,%ld\r\n",
               label,
               (long)App_FloatToInt32Scaled(gyro_mean_mdps_x, 1.0f),
               (long)App_FloatToInt32Scaled(gyro_mean_mdps_y, 1.0f),
               (long)App_FloatToInt32Scaled(gyro_mean_mdps_z, 1.0f));

  if (((max_x - min_x) > APP_ACCEL_CAL_MAX_RANGE_RAW) ||
      ((max_y - min_y) > APP_ACCEL_CAL_MAX_RANGE_RAW) ||
      ((max_z - min_z) > APP_ACCEL_CAL_MAX_RANGE_RAW))
  {
    App_CdcPrint("ACC_CAL: FAIL %s ACC_RANGE_TOO_HIGH\r\n", label);
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  if (fabsf(gravity_mag - imu.accel_lsb_per_g) > APP_ACCEL_CAL_MAX_GRAVITY_ERROR_RAW)
  {
    App_CdcPrint("ACC_CAL: FAIL %s GRAVITY_MAG_BAD\r\n", label);
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  if ((dominant_value < dominant_min_raw) ||
      (cross_a > cross_max_raw) ||
      (cross_b > cross_max_raw))
  {
    App_CdcPrint("ACC_CAL: FAIL %s WRONG_FACE_OR_ALIGNMENT\r\n", label);
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  if ((fabsf(gyro_mean_mdps_x) > APP_ACCEL_CAL_MAX_GYRO_MEAN_MDPS) ||
      (fabsf(gyro_mean_mdps_y) > APP_ACCEL_CAL_MAX_GYRO_MEAN_MDPS) ||
      (fabsf(gyro_mean_mdps_z) > APP_ACCEL_CAL_MAX_GYRO_MEAN_MDPS))
  {
    App_CdcPrint("ACC_CAL: FAIL %s GYRO_MEAN_TOO_HIGH\r\n", label);
    return SENSOR_CAL_ERROR_BAD_DATA;
  }

  calibration_app.accel_data.mean[position] = mean;
  calibration_app.accel_data.valid[position] = 1U;
  App_CdcPrint("ACC_CAL: PASS_%s\r\n", label);

  return SENSOR_CAL_OK;
}
#endif

#ifdef CAL_MAG_ENABLE
static void App_CalibrateMagManualBlocking(void)
{
  sensor_cal_mag_minmax_t mag_data;
  sensor_cal_status_t cal_status;
  lis3mdl_vector_t sample;
  uint32_t sample_index;
  uint32_t sample_count;
  uint32_t elapsed_ms;
  float range_x;
  float range_y;
  float range_z;
  float min_range;
  float max_range;
  float radius_x;
  float radius_y;
  float radius_z;
  float average_radius;
  float estimated_field_uT;

  App_CdcPrint("\r\nMAG_CAL: START\r\n");
  App_CdcPrint("MAG_CAL: Move away from metal, magnets, speakers, laptop edges.\r\n");
  App_CdcPrint("MAG_CAL: Keep USB cable loose and away from sensor.\r\n");
  App_CdcPrint("MAG_CAL: North direction is NOT required.\r\n");
  App_CdcPrint("MAG_CAL: Rotate slowly. Do not shake.\r\n");
  App_CdcPrint("MAG_CAL: Keep board in same area. Do not walk around.\r\n");

  if (mag_status != LIS3MDL_OK)
  {
    App_CdcPrint("MAG_CAL: FAIL MAG_NOT_READY status=%ld\r\n", (long)mag_status);
    return;
  }

  sensor_cal_lis3mdl_mag_minmax_start(&mag_data);
  sample_count = APP_MAG_CAL_DURATION_MS / APP_MAG_CAL_SAMPLE_DELAY_MS;

  App_CdcPrint("MAG_CAL: COLLECT duration_ms=%lu samples=%lu delay_ms=%lu\r\n",
               (unsigned long)APP_MAG_CAL_DURATION_MS,
               (unsigned long)sample_count,
               (unsigned long)APP_MAG_CAL_SAMPLE_DELAY_MS);

  for (sample_index = 0U; sample_index < sample_count; sample_index++)
  {
    elapsed_ms = sample_index * APP_MAG_CAL_SAMPLE_DELAY_MS;

    if ((elapsed_ms % APP_MAG_CAL_ACTION_PERIOD_MS) == 0U)
    {
      uint32_t time_left = (APP_MAG_CAL_DURATION_MS - elapsed_ms) / 1000U;
      App_CdcPrint("MAG_CAL: TIME_LEFT %lu\r\n", (unsigned long)time_left);
      App_CdcPrint("MAG_CAL: ACTION %s\r\n", App_MagCalActionText(elapsed_ms));
    }

    if (lis3mdl_read_gauss(&mag, &sample) != LIS3MDL_OK)
    {
      App_CdcPrint("MAG_CAL: FAIL READ_ERROR sample=%lu\r\n", (unsigned long)sample_index);
      return;
    }

    if (sample.x < mag_data.min_gauss.x) { mag_data.min_gauss.x = sample.x; }
    if (sample.y < mag_data.min_gauss.y) { mag_data.min_gauss.y = sample.y; }
    if (sample.z < mag_data.min_gauss.z) { mag_data.min_gauss.z = sample.z; }
    if (sample.x > mag_data.max_gauss.x) { mag_data.max_gauss.x = sample.x; }
    if (sample.y > mag_data.max_gauss.y) { mag_data.max_gauss.y = sample.y; }
    if (sample.z > mag_data.max_gauss.z) { mag_data.max_gauss.z = sample.z; }
    mag_data.sample_count++;

    if ((sample_index % APP_MAG_CAL_PRINT_PERIOD) == 0U)
    {
      range_x = mag_data.max_gauss.x - mag_data.min_gauss.x;
      range_y = mag_data.max_gauss.y - mag_data.min_gauss.y;
      range_z = mag_data.max_gauss.z - mag_data.min_gauss.z;
      App_CdcPrint("MAG_RAW_GAUSS_X1000: sample=%lu,X=%ld,Y=%ld,Z=%ld\r\n",
                   (unsigned long)sample_index,
                   (long)App_FloatToInt32Scaled(sample.x, 1000.0f),
                   (long)App_FloatToInt32Scaled(sample.y, 1000.0f),
                   (long)App_FloatToInt32Scaled(sample.z, 1000.0f));
      App_CdcPrint("MAG_RANGE_GAUSS_X1000: X=%ld,Y=%ld,Z=%ld\r\n",
                   (long)App_FloatToInt32Scaled(range_x, 1000.0f),
                   (long)App_FloatToInt32Scaled(range_y, 1000.0f),
                   (long)App_FloatToInt32Scaled(range_z, 1000.0f));
    }

    HAL_Delay(APP_MAG_CAL_SAMPLE_DELAY_MS);
  }

  range_x = mag_data.max_gauss.x - mag_data.min_gauss.x;
  range_y = mag_data.max_gauss.y - mag_data.min_gauss.y;
  range_z = mag_data.max_gauss.z - mag_data.min_gauss.z;
  min_range = range_x;
  max_range = range_x;
  if (range_y < min_range) { min_range = range_y; }
  if (range_z < min_range) { min_range = range_z; }
  if (range_y > max_range) { max_range = range_y; }
  if (range_z > max_range) { max_range = range_z; }

  radius_x = range_x * 0.5f;
  radius_y = range_y * 0.5f;
  radius_z = range_z * 0.5f;
  average_radius = (radius_x + radius_y + radius_z) / 3.0f;
  estimated_field_uT = average_radius * APP_GAUSS_TO_MICROTESLA;

  App_CdcPrint("MAG_CAL: SAMPLE_COUNT=%lu\r\n", (unsigned long)mag_data.sample_count);
  App_CdcPrint("MAG_CAL: MIN_GAUSS_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(mag_data.min_gauss.x, 1000.0f),
               (long)App_FloatToInt32Scaled(mag_data.min_gauss.y, 1000.0f),
               (long)App_FloatToInt32Scaled(mag_data.min_gauss.z, 1000.0f));
  App_CdcPrint("MAG_CAL: MAX_GAUSS_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(mag_data.max_gauss.x, 1000.0f),
               (long)App_FloatToInt32Scaled(mag_data.max_gauss.y, 1000.0f),
               (long)App_FloatToInt32Scaled(mag_data.max_gauss.z, 1000.0f));
  App_CdcPrint("MAG_CAL: RANGE_GAUSS_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(range_x, 1000.0f),
               (long)App_FloatToInt32Scaled(range_y, 1000.0f),
               (long)App_FloatToInt32Scaled(range_z, 1000.0f));
  App_CdcPrint("MAG_CAL: EST_FIELD_UT=%ld\r\n",
               (long)App_FloatToInt32Scaled(estimated_field_uT, 1.0f));

  if (mag_data.sample_count < APP_MAG_CAL_MIN_SAMPLE_COUNT)
  {
    App_CdcPrint("MAG_CAL: FAIL NOT_ENOUGH_SAMPLES\r\n");
    return;
  }

  if ((range_x < APP_MAG_CAL_MIN_RANGE_GAUSS) ||
      (range_y < APP_MAG_CAL_MIN_RANGE_GAUSS) ||
      (range_z < APP_MAG_CAL_MIN_RANGE_GAUSS))
  {
    App_CdcPrint("MAG_CAL: FAIL RANGE_TOO_SMALL\r\n");
    App_CdcPrint("MAG_CAL: Retry and rotate more in all 3D directions, especially the small-range axis.\r\n");
    return;
  }

  if ((min_range <= 0.0f) || ((max_range / min_range) > APP_MAG_CAL_MAX_RANGE_RATIO))
  {
    App_CdcPrint("MAG_CAL: FAIL COVERAGE_UNBALANCED\r\n");
    App_CdcPrint("MAG_CAL: Retry with more upside-down and diagonal rotations.\r\n");
    return;
  }

  if ((estimated_field_uT < APP_MAG_CAL_MIN_FIELD_UT) ||
      (estimated_field_uT > APP_MAG_CAL_MAX_FIELD_UT))
  {
    App_CdcPrint("MAG_CAL: FAIL FIELD_OUT_OF_RANGE\r\n");
    App_CdcPrint("MAG_CAL: Move farther from metal/magnets and retry.\r\n");
    return;
  }

  App_CdcPrint("MAG_CAL: COMPUTE\r\n");
  cal_status = sensor_cal_lis3mdl_mag_minmax_finish(&mag_data, &calibration_app.block.lis3mdl);
  calibration_app.status = cal_status;
  if (cal_status != SENSOR_CAL_OK)
  {
    App_CdcPrint("MAG_CAL: FAIL COMPUTE status=%ld\r\n", (long)cal_status);
    return;
  }

  app_calibration_apply_stored(&calibration_app);

  App_CdcPrint("MAG_CAL: HARD_IRON_GAUSS_X1000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.hard_iron_gauss[0], 1000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.hard_iron_gauss[1], 1000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.hard_iron_gauss[2], 1000.0f));
  App_CdcPrint("MAG_CAL: SOFT_SCALE_X1000000=%ld,%ld,%ld\r\n",
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.soft_iron_matrix[0][0], 1000000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.soft_iron_matrix[1][1], 1000000.0f),
               (long)App_FloatToInt32Scaled(calibration_app.block.lis3mdl.soft_iron_matrix[2][2], 1000000.0f));

  app_calibration_request(&calibration_app, APP_CAL_CMD_SAVE);
  app_calibration_process(&calibration_app);
  if ((calibration_app.status == SENSOR_CAL_OK) &&
      (calibration_app.flash_status == CAL_STORAGE_OK))
  {
    App_CdcPrint("MAG_CAL: PASS SAVED_TO_FLASH\r\n");
  }
  else
  {
    App_CdcPrint("MAG_CAL: FAIL FLASH_SAVE status=%ld flash=%ld\r\n",
                 (long)calibration_app.status,
                 (long)calibration_app.flash_status);
  }
}

static const char *App_MagCalActionText(uint32_t elapsed_ms)
{
  if (elapsed_ms < 10000U)
  {
    return "STEP 1/6 FLAT_YAW: keep board mostly flat, slowly rotate like compass heading.";
  }
  if (elapsed_ms < 20000U)
  {
    return "STEP 2/6 ROLL: slowly roll left edge up, then right edge up.";
  }
  if (elapsed_ms < 30000U)
  {
    return "STEP 3/6 PITCH: slowly pitch front edge up, then back edge up.";
  }
  if (elapsed_ms < 40000U)
  {
    return "STEP 4/6 UPSIDE_DOWN: turn board upside down and rotate slowly.";
  }
  if (elapsed_ms < 50000U)
  {
    return "STEP 5/6 DIAGONAL: hold one corner up, rotate, then another corner up.";
  }
  return "STEP 6/6 FULL_3D: slow random 3D rotation, cover any missing directions.";
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
