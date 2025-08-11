/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "lsm6dsl_reg.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t rx_buffer_size = 64;
uint8_t rx_buffer[64];
volatile uint8_t rx_index = 0;
uint8_t message_received = 0;

// ============= Variable du mode Manuel =============
volatile uint32_t IC1_Value1 = 0;
volatile uint32_t IC1_Value2 = 0;
volatile uint8_t IC1_Captured = 0;
volatile float lin = 0.0f;

volatile uint32_t IC2_Value1 = 0;
volatile uint32_t IC2_Value2 = 0;
volatile uint8_t IC2_Captured = 0;
volatile float ang = 0.0f;

volatile uint32_t IC3_Value1 = 0;
volatile uint32_t IC3_Value2 = 0;
volatile uint8_t IC3_Captured = 0;
volatile uint32_t pulse_width_SWITCH;
volatile uint8_t switch_state = 0; // 0 = RC / 1 = Autonomous // 0 Par defaut !!

// ============= Variable des Capteurs =============

extern I2C_HandleTypeDef hi2c2;

static stmdev_ctx_t imu_ctx;
float accel[3], gyro[3];
char sensors_buffer[100];
uint32_t last_send = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Update_Motors(int16_t motorA, int16_t motorB);
void processCompleteMessage(char *msg);
bool CheckSwitchMode(float pulse_width_SWITCH);
void IMU_Init(void);
bool IMU_Read(float *accel_data, float *gyro_data);

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	// ============= Capteurs =============

	IMU_Init();

	// ============= Ecoute du mode Manuel =============
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // LIN
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // ANG
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); // Switch (TIM1_CH1)

	// ============= Envoie des PWM =============
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Moteur Droit (TIM3_CH2)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Moteur Gauche (TIM3_CH3)

	// Initialisation PWM neutre
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); // 1.5ms
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500); // 1.5ms

	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	printf("Démarrage de l'application...\n\r");
	printf("Control Mode :: Manual\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	while (true) {
		/* USER CODE BEGIN WHILE */

		uint32_t current_time = HAL_GetTick();

		if (current_time - last_send > 100) {
			if (IMU_Read(accel, gyro)) {
				last_send = current_time;

				int len = snprintf(sensors_buffer, sizeof(sensors_buffer),
						"IMU,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
						current_time, accel[0], accel[1], accel[2], gyro[0],
						gyro[1], gyro[2]);

				HAL_UART_Transmit(&huart3, (uint8_t*) sensors_buffer, len,
						HAL_MAX_DELAY);
			}
		}

		if (CheckSwitchMode(pulse_width_SWITCH)) {
			if (switch_state == 0) {
				printf("Control Mode :: Manual\n\r");
			} else {
				printf("Control Mode :: Automatic\n\r");
			}
		}
		if (switch_state == 0) { // Mode Télécommande
			int16_t pwm_left = 1500 + (int) ((lin - ang) * 500);
			int16_t pwm_right = 1500 + (int) ((lin + ang) * 500);
			Update_Motors(pwm_left, pwm_right);
		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F12981;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ARD_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============= Callback d'interruption PWM =============
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint32_t diff;

	// TIM2 - Commandes RC (Linéaire et Angulaire)
	if (htim->Instance == TIM3) {
		switch (htim->Channel) {
		case HAL_TIM_ACTIVE_CHANNEL_1: // LIN
			if (IC1_Captured == 0) {
				IC1_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				IC1_Captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (IC1_Captured == 1) {
				IC1_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				if (IC1_Value2 >= IC1_Value1)
					diff = IC1_Value2 - IC1_Value1;
				else
					diff = (htim->Instance->ARR - IC1_Value1) + IC1_Value2 + 1;

				float pulse_width_us = (float) diff;
				if (pulse_width_us < 1000)
					pulse_width_us = 1000;
				if (pulse_width_us > 2000)
					pulse_width_us = 2000;
				lin = (pulse_width_us - 1500.0f) / 500.0f;

				IC1_Captured = 0;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_RISING);
			}
			break;

		case HAL_TIM_ACTIVE_CHANNEL_2: // ANG
			if (IC2_Captured == 0) {
				IC2_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				IC2_Captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
						TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (IC2_Captured == 1) {
				IC2_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				if (IC2_Value2 >= IC2_Value1)
					diff = IC2_Value2 - IC2_Value1;
				else
					diff = (htim->Instance->ARR - IC2_Value1) + IC2_Value2 + 1;

				float pulse_width_us = (float) diff;
				if (pulse_width_us < 1000)
					pulse_width_us = 1000;
				if (pulse_width_us > 2000)
					pulse_width_us = 2000;
				ang = (pulse_width_us - 1500.0f) / 500.0f;

				IC2_Captured = 0;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
						TIM_INPUTCHANNELPOLARITY_RISING);
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4: // ANG
			if (IC3_Captured == 0) {
				IC3_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				IC3_Captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
						TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (IC3_Captured == 1) {
				IC3_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				if (IC3_Value2 >= IC3_Value1)
					pulse_width_SWITCH = IC3_Value2 - IC3_Value1;
				else
					pulse_width_SWITCH = (htim->Instance->ARR - IC3_Value1)
							+ IC3_Value2 + 1;

				// printf("[SWITCH] Pulse width: %lu us\n\r", pulse_width_SWITCH);
				IC3_Captured = 0;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
						TIM_INPUTCHANNELPOLARITY_RISING);
			}
			break;
		default:
			break;
		}
	}
}

// Fonction de contrôle moteur
void Update_Motors(int16_t pwm_left, int16_t pwm_right) {
	// Clamping des valeurs
	pwm_left = (pwm_left < 1000) ? 1000 : (pwm_left > 2000) ? 2000 : pwm_left;
	pwm_right = (pwm_right < 1000) ? 1000 :
				(pwm_right > 2000) ? 2000 : pwm_right;

	// Commande Moteurs
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_left); // Moteur Gauche (TIM3_CH3)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_right); // Moteur Droit (TIM3_CH2)
}

// ============= Gestion UART =============
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static char buffer[128];
	static uint8_t idx = 0;

	if (rx_buffer[0] == '\n' || idx >= sizeof(buffer) - 1) {
		buffer[idx] = '\0';
		processCompleteMessage(buffer);
		idx = 0;
	} else if (rx_buffer[0] >= 32 && rx_buffer[0] <= 126) {
		buffer[idx++] = rx_buffer[0];
	}

	// Relance la réception sur le premier octet
	HAL_UART_Receive_IT(huart, &rx_buffer[0], 1);
}

void processCompleteMessage(char *msg) {
	float lin_val, ang_val;
	if (sscanf(msg, "L:%f A:%f", &lin_val, &ang_val) == 2) {
		printf("Message valide: %s\n\r", msg);

		// Calcul des PWM
		int16_t pwm_left = 1500 + (int) ((lin_val - ang_val) * 500);
		int16_t pwm_right = 1500 + (int) ((lin_val + ang_val) * 500);

		Update_Motors(pwm_left, pwm_right);
	}
}

bool CheckSwitchMode(float pulse_width_SWITCH) {
	static uint32_t last_switch_time = 0;
	if (HAL_GetTick() - last_switch_time < 200)
		return false; // Anti-rebond 200ms

	if ((switch_state == 1) && (pulse_width_SWITCH < 1800)) {
		// Retour en mode Manuel
		HAL_UART_AbortReceive(&huart3);
		HAL_NVIC_DisableIRQ(USART3_IRQn);

		// Réactivation des captures RC
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

		switch_state = 0;
		last_switch_time = HAL_GetTick();
		return true;
	} else if ((switch_state == 0) && (pulse_width_SWITCH > 1800)) {
		// Passage en mode Automatique
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);

		HAL_NVIC_EnableIRQ(USART3_IRQn);
		HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);

		switch_state = 1;
		last_switch_time = HAL_GetTick();
		return true;
	}
	return false;
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	printf("Erreur UART! Réinitialisation...\n");
	HAL_UART_AbortReceive(huart);
	HAL_UART_Receive_IT(huart, &rx_buffer[0], 1);
}

// ============= Gestion Capteurs =============

#define LSM6DSL_I2C_ADDRESS (0x6A << 1) // 0xD4 en 8 bits adresse pour HAL!


static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
		uint16_t Len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write((I2C_HandleTypeDef*) handle,
			(LSM6DSL_I2C_ADDRESS), Reg, I2C_MEMADD_SIZE_8BIT, Bufp, Len, 1000);
	return (status == HAL_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
		uint16_t Len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read((I2C_HandleTypeDef*) handle,
			(LSM6DSL_I2C_ADDRESS), Reg, I2C_MEMADD_SIZE_8BIT, Bufp, Len, 1000);
	return (status == HAL_OK) ? 0 : -1;
}

void IMU_Init(void) {

	imu_ctx.handle = (void*) &hi2c2;
	imu_ctx.read_reg = platform_read;
	imu_ctx.write_reg = platform_write;

	uint8_t whoamI;
	lsm6dsl_device_id_get(&imu_ctx, &whoamI);
	if (whoamI != LSM6DSL_ID) {
		printf("Erreur IMU non détectée\n");
	}

	lsm6dsl_reset_set(&imu_ctx, PROPERTY_ENABLE);
	HAL_Delay(100);
	uint8_t rst;
	do {
		lsm6dsl_reset_get(&imu_ctx, &rst);
	} while (rst);

	lsm6dsl_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);
	lsm6dsl_xl_full_scale_set(&imu_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&imu_ctx, LSM6DSL_2000dps);
	lsm6dsl_xl_data_rate_set(&imu_ctx, LSM6DSL_XL_ODR_52Hz);
	lsm6dsl_gy_data_rate_set(&imu_ctx, LSM6DSL_GY_ODR_52Hz);
}


bool IMU_Read(float *accel_data, float *gyro_data) {

    int16_t raw_accel[3], raw_gyro[3];

    // Lecture accéléromètre
    int32_t ret_accel = lsm6dsl_acceleration_raw_get(&imu_ctx, raw_accel);
    if (ret_accel != 0) {
        printf("Erreur lecture accel: %ld\n\r", ret_accel);
        return false;
    }

    // Lecture gyroscope
    int32_t ret_gyro = lsm6dsl_angular_rate_raw_get(&imu_ctx, raw_gyro);
    if (ret_gyro != 0) {
        printf("Erreur lecture gyro: %ld\n\r", ret_gyro);
        return false;
    }

    // Conversion en g (acc) et dps (gyro)
    accel_data[0] = raw_accel[0] * 0.061f / 1000.0f;
    accel_data[1] = raw_accel[1] * 0.061f / 1000.0f;
    accel_data[2] = raw_accel[2] * 0.061f / 1000.0f;

    gyro_data[0] = raw_gyro[0] * 70.0f / 1000.0f;
    gyro_data[1] = raw_gyro[1] * 70.0f / 1000.0f;
    gyro_data[2] = raw_gyro[2] * 70.0f / 1000.0f;

    return true;
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
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
