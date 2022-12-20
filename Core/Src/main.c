/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {

	GPIO_TypeDef *LS_Pin;

	uint16_t LSP_Number;

	TIM_HandleTypeDef HS_Tim;

	uint16_t HST_Channel;

} Mosfet_Driver_Typedef;

typedef struct {

	GPIO_TypeDef *H_Pin;

	uint16_t HP_Number;

} Hall_Sensor_Typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void Motor_Driver(Mosfet_Driver_Typedef MD_one, Mosfet_Driver_Typedef MD_two,
		Mosfet_Driver_Typedef MD_three, uint8_t Hall_T);
void Mosfet_Control(Mosfet_Driver_Typedef MD_one, Mosfet_Driver_Typedef MD_two,
					Mosfet_Driver_Typedef MD_three, uint8_t case_var, uint8_t button);

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
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Init(&htim3);

	Mosfet_Driver_Typedef MD_One = { 0 };
	MD_One.LS_Pin = GPIOB;
	MD_One.LSP_Number = GPIO_PIN_10;
	MD_One.HS_Tim = htim2;
	MD_One.HST_Channel = TIM_CHANNEL_4;

	Mosfet_Driver_Typedef MD_Two;
	MD_Two.LS_Pin = GPIOB;
	MD_Two.LSP_Number = GPIO_PIN_0;
	MD_Two.HS_Tim = htim3;
	MD_Two.HST_Channel = TIM_CHANNEL_4;

	Mosfet_Driver_Typedef MD_Three;
	MD_Three.LS_Pin = GPIOA;
	MD_Three.LSP_Number = GPIO_PIN_6;
	MD_Three.HS_Tim = htim3;
	MD_Three.HST_Channel = TIM_CHANNEL_2;
	// --

	// Hall Sensor Typedef
	Hall_Sensor_Typedef Hall_A;
	Hall_A.H_Pin = GPIOB;
	Hall_A.HP_Number = GPIO_PIN_7;

	Hall_Sensor_Typedef Hall_B;
	Hall_B.H_Pin = GPIOB;
	Hall_B.HP_Number = GPIO_PIN_8;

	Hall_Sensor_Typedef Hall_C;
	Hall_C.H_Pin = GPIOB;
	Hall_C.HP_Number = GPIO_PIN_9;

	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t HA_Res = 0;
	uint8_t HB_Res = 0;
	uint8_t HC_Res = 0;
	uint8_t HT_Res = 0;
	uint8_t Button = 0;

	uint8_t Prvs_button = 3;
	uint8_t Case = 0 ;

	uint8_t Is_break = 0;

	uint8_t Prvs_hall = 0;
	uint8_t Index = 0;
	uint8_t Is_limit = 1;

	char List_hall[50] = {0};

	/*
	 Hall List One => 1, 2, 3, 4, 5, 6
	 Hall List Two => 2, 4, 6, 1, 3, 5
	 Hall List Three => 3, 5, 4, 1, 2, 6
	 */

	while (1){

		Button = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		Is_break = Button;

		/* HAL SENSÖR OKUMASI*/
		HA_Res = HAL_GPIO_ReadPin(Hall_A.H_Pin, Hall_A.HP_Number);
		HB_Res = HAL_GPIO_ReadPin(Hall_B.H_Pin, Hall_B.HP_Number);
		HC_Res = HAL_GPIO_ReadPin(Hall_C.H_Pin, Hall_C.HP_Number);

		HB_Res = HB_Res << 1;
		HC_Res = HC_Res << 2;
		HT_Res = HC_Res | HB_Res | HA_Res;


		/* SÜRERSE GENEL SÜRÜCÜ*/
		if (Is_break) {
			HAL_TIM_PWM_Stop(&MD_One.HS_Tim, MD_One.HST_Channel);
			HAL_TIM_PWM_Stop(&MD_Two.HS_Tim, MD_Two.HST_Channel);
			HAL_TIM_PWM_Stop(&MD_Three.HS_Tim, MD_Three.HST_Channel);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MD_One.LS_Pin, MD_One.LSP_Number, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MD_Two.LS_Pin, MD_Two.LSP_Number, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MD_Three.LS_Pin, MD_Three.LSP_Number, GPIO_PIN_RESET);

			}
		else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			Motor_Driver(MD_One, MD_Two, MD_Three, HT_Res);
		}


		/* MOSFET KONTROL ABİMİZ

		if (Prvs_button != Button) {
			Mosfet_Control(MD_One, MD_Two, MD_Three, Case, Button);
			Prvs_button = Button ;
			Case++ ;
			if (Case == 7) {
				Case = 0;
			}
		}*/

		/* HAL DEPOLAYICI
		 if ((Prvs_hall != HT_Res)& (Is_limit)) {
		 List_hall[Index] = HT_Res;
		 Prvs_hall = HT_Res;
		 Index ++ ;
		 if (Index == 50) {
		 Is_limit = 0;
		 }}
		 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Motor_Driver(Mosfet_Driver_Typedef MD_one, Mosfet_Driver_Typedef MD_two,
		Mosfet_Driver_Typedef MD_three, uint8_t Hall_T) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	switch (Hall_T) {

	case 1:
		// 2 > 1
		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_three.HS_Tim, MD_three.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_one.HS_Tim, MD_one.HST_Channel);

		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_two.HS_Tim, MD_two.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_two.HS_Tim, MD_two.HST_Channel, 10);

		break;
	case 2:
		// 3 > 2
		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_one.HS_Tim, MD_one.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_two.HS_Tim, MD_two.HST_Channel);

		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_three.HS_Tim, MD_three.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_three.HS_Tim, MD_three.HST_Channel, 10);

		break;
	case 3:
		// 3 > 1
		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_two.HS_Tim, MD_two.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_one.HS_Tim, MD_one.HST_Channel);

		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_three.HS_Tim, MD_three.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_three.HS_Tim, MD_three.HST_Channel, 10);

		break;
	case 4:
		// 1 > 3
		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_two.HS_Tim, MD_two.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_three.HS_Tim, MD_three.HST_Channel);

		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_one.HS_Tim, MD_one.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_one.HS_Tim, MD_one.HST_Channel, 10);

		break;
	case 5:
		// 2 > 3
		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_one.HS_Tim, MD_one.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_three.HS_Tim, MD_three.HST_Channel);

		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_two.HS_Tim, MD_two.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_two.HS_Tim, MD_two.HST_Channel, 10);

		break;
	case 6:
		// 1 > 2
		HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&MD_three.HS_Tim, MD_three.HST_Channel);
		HAL_TIM_PWM_Stop(&MD_two.HS_Tim, MD_two.HST_Channel);

		HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&MD_one.HS_Tim, MD_one.HST_Channel);
		__HAL_TIM_SET_COMPARE(&MD_one.HS_Tim, MD_one.HST_Channel, 10);

		break;
	default:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		break;
	}

}

void Mosfet_Control(Mosfet_Driver_Typedef MD_one, Mosfet_Driver_Typedef MD_two,
					Mosfet_Driver_Typedef MD_three,uint8_t case_var, uint8_t button){

	HAL_TIM_PWM_Stop(&MD_one.HS_Tim, MD_one.HST_Channel);
	HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(&MD_two.HS_Tim, MD_two.HST_Channel);
	HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(&MD_three.HS_Tim, MD_three.HST_Channel);
	HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_SET);

	switch (case_var) {
		case 1:
			HAL_TIM_PWM_Start(&MD_one.HS_Tim, MD_one.HST_Channel);
			__HAL_TIM_SET_COMPARE(&MD_one.HS_Tim, MD_one.HST_Channel,50);
			break;
		case 2:
			HAL_GPIO_WritePin(MD_one.LS_Pin, MD_one.LSP_Number, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_TIM_PWM_Start(&MD_two.HS_Tim, MD_two.HST_Channel);
			__HAL_TIM_SET_COMPARE(&MD_two.HS_Tim, MD_two.HST_Channel,50);
			break;
		case 4:
			HAL_GPIO_WritePin(MD_two.LS_Pin, MD_two.LSP_Number, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_TIM_PWM_Start(&MD_three.HS_Tim, MD_three.HST_Channel);
			__HAL_TIM_SET_COMPARE(&MD_three.HS_Tim, MD_three.HST_Channel,50);
			break;
		case 6:
			HAL_GPIO_WritePin(MD_three.LS_Pin, MD_three.LSP_Number, GPIO_PIN_RESET);
			break;
		default:
			break;
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
	__disable_irq();
	while (1) {
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
