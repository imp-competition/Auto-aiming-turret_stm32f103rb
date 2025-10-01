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
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Buffer_Size 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t hori_Txraw[Buffer_Size];
uint8_t hori_Rxraw[Buffer_Size];
uint8_t ver_Txraw[Buffer_Size];
uint8_t ver_Rxraw[Buffer_Size];
uint8_t degree[Buffer_Size];

uint8_t hori_temp[1];
uint8_t ver_temp[1];
size_t hori_rx_index = 0;
size_t ver_rx_index = 0;
const char *init_msg = "UART Connect! \r\n";
const char *btn_msg = "Button \r\n";
volatile int hori_rec_counter = 0;
volatile int ver_rec_counter = 0;
volatile int btn_counter = 0;
typedef enum state{idle, servo0, servo90, servo180}servo;
volatile int hori_g_target_deg = 0;
volatile int ver_g_target_deg = 0;
int ver_val;
int hori_val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint16_t deg_to_us(int deg)
{
    // 안전 범위(0~180)로 클램프. 필요시 0~120 등으로 변경 가능
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;

    // 0°=1000us, 180°=2000us 선형 매핑
    // us = 1000 + (deg/180)*1000
    return (uint16_t)(1000 + ((float)deg * 1000.0f / 180.0f));
}
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart3, (uint8_t*) init_msg, strlen(init_msg),
	HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart3, hori_temp, 1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	HAL_UART_Transmit(&huart1, (uint8_t*) init_msg, strlen(init_msg),
		HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart1, ver_temp, 1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	uint16_t init_deg = deg_to_us(90);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, init_deg);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, init_deg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_StatusTypeDef RX_Status = HAL_UART_Receive(&huart2, temp, 1,
//		HAL_MAX_DELAY);
//		if (RX_Status == HAL_OK) {
//			Rxraw[rx_index++] = temp[0];
//
//			if ((rx_index >= 2) && (Rxraw[rx_index - 2] == '\r')
//					&& (Rxraw[rx_index - 1] == '\n')) {
//				for (int i = 0; i < rx_index; i++) {
//					Txraw[i] = Rxraw[i];
//				}
//				HAL_UART_Transmit(&huart2, Txraw, rx_index, HAL_MAX_DELAY);
//				memset(Rxraw, 0, Buffer_Size);
//				rx_index = 0;
//			}
//
//			if (rx_index >= Buffer_Size) {
//				memset(Rxraw, 0, Buffer_Size);
//				rx_index = 0;
//
//			}
//		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//btn_counter++;
	//HAL_UART_Transmit(&huart3, (uint8_t*) btn_msg, strlen(btn_msg),
	//HAL_MAX_DELAY);
	 //uint16_t us = deg_to_us(g_target_deg);

	    // 권장: HAL 매크로로 비교값 설정 (가독성/이식성 좋음)
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, deg_to_us(90));
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, deg_to_us(90));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	    {
			hori_rec_counter++;
			hori_Rxraw[hori_rx_index++] = hori_temp[0];

	        // 줄바꿈(명령 종료) 체크
	        if((hori_temp[0] == '\r') || (hori_temp[0] == '\n'))
	        {
	            // 문자열 종료 처리
	            if (hori_rx_index > 0) {
	                // 끝문자를 0으로 만들어 C문자열 완성
	                //if (rx_index >= Buffer_Size) rx_index = Buffer_Size - 1;
	                //Rxraw[rx_index] = '\0';

	                // 공백 제거(선택) — 단순하게 atoi만 사용해도 됨
	                // atoi는 "30", "  60", "90\r" 같은 것도 잘 파싱

	            	hori_val = atoi((char*)hori_Rxraw);

	                // 여기서 마지막 목표 각도 갱신 (예: 30, 60, 90)
	                hori_g_target_deg = hori_val;   // EXTI에서 이 값을 사용
	                uint16_t hori_us = deg_to_us(hori_g_target_deg);
	                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, hori_us);
	                memset(hori_Rxraw, 0, sizeof(hori_Rxraw));
	                // (선택) OK 응답
	                // const char *ok = "OK\r\n";
	                // HAL_UART_Transmit(&huart3, (uint8_t*)ok, strlen(ok), HAL_MAX_DELAY);
	            }

	            // 다음 명령을 위한 버퍼 리셋
	            hori_rx_index = 0;
	        }
	        else
	        {
	            // 버퍼 오버런 방지
	            if (hori_rx_index >= Buffer_Size) {
	            	hori_rx_index = 0;
	            }
	        }

	        // 다음 1바이트 재수신
	        HAL_UART_Receive_IT(&huart3, hori_temp, 1);
	    }

	if(huart->Instance == USART1)
		    {
				ver_rec_counter++;
				ver_Rxraw[ver_rx_index++] = ver_temp[0];

		        // 줄바꿈(명령 종료) 체크
		        if((ver_temp[0] == '\r') || (ver_temp[0] == '\n'))
		        {
		            // 문자열 종료 처리
		            if (ver_rx_index > 0) {
		                // 끝문자를 0으로 만들어 C문자열 완성
		                //if (rx_index >= Buffer_Size) rx_index = Buffer_Size - 1;
		                //Rxraw[rx_index] = '\0';

		                // 공백 제거(선택) — 단순하게 atoi만 사용해도 됨
		                // atoi는 "30", "  60", "90\r" 같은 것도 잘 파싱

		            	ver_val = atoi((char*)ver_Rxraw);

		                // 여기서 마지막 목표 각도 갱신 (예: 30, 60, 90)
		            	ver_g_target_deg = ver_val;   // EXTI에서 이 값을 사용
		                uint16_t ver_us = deg_to_us(ver_g_target_deg);
		                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ver_us);
		                memset(ver_Rxraw, 0, sizeof(ver_Rxraw));
		                // (선택) OK 응답
		                // const char *ok = "OK\r\n";
		                // HAL_UART_Transmit(&huart3, (uint8_t*)ok, strlen(ok), HAL_MAX_DELAY);
		            }

		            // 다음 명령을 위한 버퍼 리셋
		            ver_rx_index = 0;
		        }
		        else
		        {
		            // 버퍼 오버런 방지
		            if (ver_rx_index >= Buffer_Size) {
		            	ver_rx_index = 0;
		            }
		        }

		        // 다음 1바이트 재수신
		        HAL_UART_Receive_IT(&huart1, ver_temp, 1);
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
