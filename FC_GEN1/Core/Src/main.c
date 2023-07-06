/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <pthread.h>
#include "string.h"
#include "globals.h"
#include "uart.h"
#include "printf.h"
#include "canal.h"
#include "adc.h"
#include "rtscheduler.h"
#include "canal_fc_messages.h"
#include "controller_autogen.h"
#include "rtwtypes.h"
#include "timers_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t driverSpeakerEn; 
uint8_t brakeLightEn;
uint8_t statusLightDutyCycle; 
uint16_t statusLightFreq;
uint16_t adc_values[NUM_ADC_CHANNELS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void readFromAdc(void);
void setControlSystemInputs(void);
void getControlSystemOutputs(void);
void transmitToAMKMotors(void);
void transmitToBMS(void);
void setDigitalOutputs(void);
void setPWMOutputs(void);
void getRollingAvg(uint16_t* data_points, uint16_t* output_averages);

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

UART_st debug_uart = {
	.uart_num = 3,
	.huart = &huart3,
	.bit_position = LSB_First,
	.baudrate = UART_115200,
	.mode = UART_TX_RX,
	.datasize = UART_Datasize_8,
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RtScheduler_tasks tasks = {
	[eTASK1_5MS] = {
		readFromAdc,
		setControlSystemInputs,
		controller_autogen_step,
		getControlSystemOutputs,
		transmitToAMKMotors,
		transmitToBMS,
		setDigitalOutputs,
	},
	[eTASK2_500MS] = {
			setPWMOutputs,
	},
};
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Printf_Init(&debug_uart);
  CanAL_Init(&pt1_can);
  CanAL_Init(&veh_can);
  ADC_Init(&adc1);
  Timer_Init(&tim2);
  PWM_Init(&status_led_pwm);
  controller_autogen_initialize();

  RtScheduler_startRunning(tasks);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 1;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 1;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = ENABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(START_BUTTON_LED_EN_GPIO_Port, START_BUTTON_LED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_SPI_CS_Pin */
  GPIO_InitStruct.Pin = IMU_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_LED_EN_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_LED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(START_BUTTON_LED_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRAKE_LIGHT_EN_Pin RTDS_EN_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_N_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_BUTTON_N_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Step Functions

void readFromAdc() {
  ADC_Scan(&adc1);
  uint16_t raw_adc_values[NUM_ADC_CHANNELS] = {
		  [APPS1_IDX] = apps1_buffer[0],
		  [APPS2_IDX] = apps2_buffer[0],
		  [STEERING_ANGLE_SENSOR_IDX] = steering_angle_sensor_buffer[0],
		  [BPPS_BUFFERED_IDX] = bpps_buffered_buffer[0],
  };

  getRollingAvg(raw_adc_values, adc_values);
}

void setControlSystemInputs() {
  controller_autogen_U.AMK_ActualVelocity_R = AMK1_ActualValues1.AMK_ActualVelocity;
  controller_autogen_U.AMK_MagnetizingCurrent_R = AMK1_ActualValues1.AMK_MagnetizingCurrent;
  controller_autogen_U.AMK_TorqueCurrent_R = AMK1_ActualValues1.AMK_TorqueCurrent;
  controller_autogen_U.AMK_bDcOn_R = AMK1_ActualValues1.AMK_bDcOn;
  controller_autogen_U.AMK_bDerating_R = AMK1_ActualValues1.AMK_bDerating;
  controller_autogen_U.AMK_bError_R = AMK1_ActualValues1.AMK_bError;
  controller_autogen_U.AMK_bInverterOn_R = AMK1_ActualValues1.AMK_bInverterOn;
  controller_autogen_U.AMK_bQuitDcOn_R = AMK1_ActualValues1.AMK_bQuitDcOn;
  controller_autogen_U.AMK_bQuitInverterOn_R = AMK1_ActualValues1.AMK_bQuitInverterOn;
  controller_autogen_U.AMK_bSystemReady_R = AMK1_ActualValues1.AMK_bSystemReady;
  controller_autogen_U.AMK_bWarn_R = AMK1_ActualValues1.AMK_bWarn;
  controller_autogen_U.AMK_ErrorInfo_R = AMK1_ActualValues2.AMK_ErrorInfo;
  controller_autogen_U.AMK_TempIGBT_R = AMK1_ActualValues2.AMK_TempIGBT;
  controller_autogen_U.AMK_TempInverter_R = AMK1_ActualValues2.AMK_TempInverter;
  controller_autogen_U.AMK_TempMotor_R = AMK1_ActualValues2.AMK_TempMotor;
  controller_autogen_U.AMK_ActualVelocity_L = AMK0_ActualValues1.AMK_ActualVelocity;
  controller_autogen_U.AMK_MagnetizingCurrent_L = AMK0_ActualValues1.AMK_MagnetizingCurrent;
  controller_autogen_U.AMK_TorqueCurrent_L = AMK0_ActualValues1.AMK_TorqueCurrent;
  controller_autogen_U.AMK_bDcOn_L = AMK0_ActualValues1.AMK_bDcOn;
  controller_autogen_U.AMK_bDerating_L = AMK0_ActualValues1.AMK_bDerating;
  controller_autogen_U.AMK_bError_L = AMK0_ActualValues1.AMK_bError;
  controller_autogen_U.AMK_bInverterOn_L = AMK0_ActualValues1.AMK_bInverterOn;
  controller_autogen_U.AMK_bQuitDcOn_L = AMK0_ActualValues1.AMK_bQuitDcOn;
  controller_autogen_U.AMK_bQuitInverterOn_L = AMK0_ActualValues1.AMK_bQuitInverterOn;
  controller_autogen_U.AMK_bSystemReady_L = AMK0_ActualValues1.AMK_bSystemReady;
  controller_autogen_U.AMK_bWarn_L = AMK0_ActualValues1.AMK_bWarn;
  controller_autogen_U.AMK_ErrorInfo_L = AMK0_ActualValues2.AMK_ErrorInfo;
  controller_autogen_U.AMK_TempIGBT_L = AMK0_ActualValues2.AMK_TempIGBT;
  controller_autogen_U.AMK_TempInverter_L = AMK0_ActualValues2.AMK_TempInverter;
  controller_autogen_U.AMK_TempMotor_L = AMK0_ActualValues2.AMK_TempMotor;
  controller_autogen_U.DI_V_SteeringAngle = adc_values[STEERING_ANGLE_SENSOR_IDX];
  controller_autogen_U.DI_V_BrakePedalPos = adc_values[BPPS_BUFFERED_IDX];
  controller_autogen_U.DI_b_DriverButton = HAL_GPIO_ReadPin(START_BUTTON_N_GPIO_Port, START_BUTTON_N_Pin);
  controller_autogen_U.DI_V_AccelPedalPos1 = adc_values[APPS1_IDX];
  controller_autogen_U.DI_V_AccelPedalPos2 = adc_values[APPS2_IDX];
  controller_autogen_U.BM_b_prechrgContactorSts = Contactor_Feedback.Pack_Precharge_Feedback;
  controller_autogen_U.BM_b_HVposContactorSts = Contactor_Feedback.Pack_Positive_Feedback;
  controller_autogen_U.BM_b_HVnegContactorSts = Contactor_Feedback.Pack_Negative_Feedback;
}

void getControlSystemOutputs() {
  AMK1_SetPoints1.AMK_bInverterOn = controller_autogen_Y.AMK_bInverterOn_tx_R;
  AMK1_SetPoints1.AMK_bDcOn = controller_autogen_Y.AMK_bDcOn_tx_R;
  AMK1_SetPoints1.AMK_bEnable = controller_autogen_Y.AMK_bEnable_R;
  AMK1_SetPoints1.AMK_bErrorReset = controller_autogen_Y.AMK_bErrorReset_R;
  AMK1_SetPoints1.AMK_TargetVelocity = controller_autogen_Y.AMK_TargetVelocity_R;
  AMK1_SetPoints1.AMK_TorqueLimitPositiv = controller_autogen_Y.AMK_TorqueLimitPositiv_R;
  AMK1_SetPoints1.AMK_TorqueLimitNegativ = controller_autogen_Y.AMK_TorqueLimitNegativ_R;

  AMK0_SetPoints1.AMK_bInverterOn = controller_autogen_Y.AMK_bInverterOn_tx_L;
  AMK0_SetPoints1.AMK_bDcOn = controller_autogen_Y.AMK_bDcOn_tx_L;
  AMK0_SetPoints1.AMK_bEnable = controller_autogen_Y.AMK_bEnable_L;
  AMK0_SetPoints1.AMK_bErrorReset = controller_autogen_Y.AMK_bErrorReset_L;
  AMK0_SetPoints1.AMK_TargetVelocity = controller_autogen_Y.AMK_TargetVelocity_L;
  AMK0_SetPoints1.AMK_TorqueLimitPositiv = controller_autogen_Y.AMK_TorqueLimitPositiv_L;
  AMK0_SetPoints1.AMK_TorqueLimitNegativ = controller_autogen_Y.AMK_TorqueLimitNegativ_L;

  Contactor_States.Pack_Precharge = controller_autogen_Y.BM_b_prechargeContactorCMD;
  Contactor_States.Pack_Positive = controller_autogen_Y.BM_b_HVposContactorCMD;
  Contactor_States.Pack_Negative = controller_autogen_Y.BM_b_HVnegContactorCMD;

  driverSpeakerEn = controller_autogen_Y.DI_b_driverSpeaker;
  brakeLightEn = controller_autogen_Y.DI_b_brakeLightEn;
  status_led_pwm.duty = controller_autogen_Y.DI_p_PWMstatusLightCycle;
  tim2.freq_hz = controller_autogen_Y.DI_p_PWMstatusLightFreq;
}

void setDigitalOutputs() {
  HAL_GPIO_WritePin(RTDS_EN_GPIO_Port, RTDS_EN_Pin, driverSpeakerEn);
  HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, brakeLightEn);
}

void setPWMOutputs(){
  static uint8_t prevDutyCycle = 0;
  static uint32_t prevFreq = 0;

  if (tim2.freq_hz != prevFreq || status_led_pwm.duty != prevDutyCycle) {
	Timer_Init(&tim2);
	PWM_Init(&status_led_pwm);

	prevDutyCycle = status_led_pwm.duty;
	prevFreq = tim2.freq_hz;
  }
}

void transmitToAMKMotors() {
	CanAL_Transmit(&pt1_can, AMK0_SETPOINTS1_CANAL_ID);
	CanAL_Transmit(&pt1_can, AMK1_SETPOINTS1_CANAL_ID);
}

void transmitToBMS() {
	CanAL_Transmit(&veh_can, CONTACTOR_STATES_CANAL_ID);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	TeCanALRet ret = CanAL_Receive(&pt1_can);
	if (ret != CANAL_OK) {
		CANAL_PRINT("Could not recognize message\n\r");
	}
}

void getRollingAvg(uint16_t* data_points, uint16_t* output_averages)
{
    static uint32_t sums [NUM_ADC_CHANNELS] = {};
    static uint16_t num_vals = 0;
    static uint16_t value_windows[NUM_ADC_CHANNELS][ADC_ROLLING_AVG_WIN_SIZE] = {};

    if (num_vals == ADC_ROLLING_AVG_WIN_SIZE)
    {
        for (int i = 0; i<NUM_ADC_CHANNELS; i++)
        {
            sums[i] = sums[i] - value_windows[i][0] + data_points[i];
            output_averages[i] = sums[i]/num_vals;

            //Shift the values
            for (int j = 0; j < ADC_ROLLING_AVG_WIN_SIZE - 1; j++)
            {
                value_windows[i][j] = value_windows[i][j+1];
            }
            value_windows[i][ADC_ROLLING_AVG_WIN_SIZE - 1] = data_points[i];
        }
    }
    else
    {
        num_vals++;
        for (int i = 0; i < NUM_ADC_CHANNELS; i++)
        {
            sums[i] += data_points[i];
            value_windows[i][num_vals-1] = data_points[i];
            output_averages[i] = sums[i]/(num_vals);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
