/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define APPLICATION_ADDRESS  0x08005000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char txbuf[8]={'\0'};
int CanData;
int WhlData=0,head_c=0;
int steer=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t  TxData[8];
uint32_t id;
uint8_t  RxData[8];
uint32_t TxMailbox;
char defal[32]={'\0'};
void can_transmitSingleByte(uint8_t Txid,uint8_t Txdata);
int Constrain(int au32_IN, int au32_MIN, int au32_MAX);
void CanPid(int str, double kp, double ki, double kd);
int MAP(int au32_IN, int au32_INmin, int au32_INmax, int au32_OUTmin,int au32_OUTmax);
void Init_PWM(void);
void SerialWrite(char * uart2Data);
void CAN_SetFilter(uint32_t id,uint32_t mask,uint8_t isExtended,uint8_t filter);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__enable_irq();
	//SCB->VTOR = APPLICATION_ADDRESS;
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SerialWrite("PID Application is running \n");
  HAL_Delay(1000);

  HAL_CAN_Start(&hcan);
  CAN_SetFilter(0x12,0x7FF,0,0);
  CAN_SetFilter(0x104,0x7FF,0,1);
  CAN_SetFilter(0x691,0x7FF,0,2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  Init_PWM();
 // Init_PWM();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
//		HAL_Delay(200);

		CanPid(51,410.0,0.0001,0.0);

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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void can_transmitSingleByte(uint8_t Txid,uint8_t Txdata) {
	/***
	 * Single byte data transmit function
	 * DLC Value define 2 - 16 bit
	 * Single ID transmit CAN ID
	 * ID size is restrict to 8 bit size i.e max value = 256
	 */
    TxHeader.IDE = CAN_ID_STD;  /*--- Global variable TxHeader ---*/
    TxHeader.StdId = Txid;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;           /*--- 2 Bytes = 2x8 = 16 bits ----*/

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }
    else{
    	TxData[0] = Txdata;
    }

}
void Init_PWM(void){
	int Rcv;
	while(Rcv < 4096)
	               {
	               TIM2->CCR1 = Rcv;
	               Rcv += 10;
	               HAL_Delay(10);
	               }

	               while(Rcv > 0)
	               {
	               TIM2->CCR1 = Rcv;
	               Rcv -= 10;
	               HAL_Delay(10);
	               }
}

void CanPid(int str, double kp, double ki, double kd){
	uint32_t start_t, final_t = 0;
	      double elapsedTime = 0.0, error, prev_error = 0.0;
	      double integral=0.0, derivative=0.0;
	      int PIDCAN_value =0,RcvData=0;
	      while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
	     	             if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData)
	     	                     != HAL_OK) {
	     	                 Error_Handler();
	     	             }
	     	             start_t = HAL_GetTick();
	     	             if (RxHeader.StdId == 0x12) {
	     	             			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	     	             			RcvData = RxData[0];


	     	             elapsedTime = (double) (start_t - final_t);
	     	             error = (double) (str - RcvData);
	     	             integral += (double) (error * elapsedTime);
	     	             derivative = (double) (error - prev_error) / elapsedTime;
	     	             PIDCAN_value = (int) (kp * error + ki * integral + kd * derivative);
	     	             int dutyCANCycle = Constrain(PIDCAN_value, -4096, 4096);
	     	             prev_error = error;
	     	             final_t = start_t;
	     	             int values = MAP(dutyCANCycle, -4096, 4096, 0, 4096);
	     	             TIM2->CCR1 = values;

	     	             }

	     	  }
}
int MAP(int au32_IN, int au32_INmin, int au32_INmax, int au32_OUTmin,int au32_OUTmax) {
	return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
			/ (au32_INmax - au32_INmin)) + au32_OUTmin);
}

int Constrain(int au32_IN, int au32_MIN, int au32_MAX) {
	if (au32_IN < au32_MIN) {
		return au32_MIN;
	} else if (au32_IN > au32_MAX) {
		return au32_MAX;
	} else {
		return au32_IN;
	}
}


void SerialWrite(char * uart2Data)
{
	//while (huart2.gState == HAL_UART_STATE_READY ){
	HAL_StatusTypeDef uart_status;
	uart_status = HAL_UART_Transmit(&huart2, (uint8_t *)uart2Data,strlen(uart2Data), 1000);
	if(uart_status != HAL_OK){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		HAL_Delay(1000);
	  }
	//}


}


void CAN_SetFilter(uint32_t id,uint32_t mask,uint8_t isExtended,uint8_t filter){

	CAN_FilterTypeDef sfilterConfig;
	sfilterConfig.FilterBank = filter;
	sfilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sfilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	if(isExtended){
		sfilterConfig.FilterIdHigh = (id>>13)& 0xffff;
		sfilterConfig.FilterIdLow = (id<<3)&0xffff;
		sfilterConfig.FilterMaskIdHigh = (mask>>13)& 0xffff;
		sfilterConfig.FilterMaskIdLow = (mask<<3)&0xffff;
	}
	else{
		sfilterConfig.FilterIdHigh = (id<<5)& 0xffff;
		sfilterConfig.FilterIdLow = 0x0000;
		sfilterConfig.FilterMaskIdHigh = (mask<<5)& 0xffff;
		sfilterConfig.FilterMaskIdLow = 0x0000;
	}
	sfilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sfilterConfig.FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan, &sfilterConfig) != HAL_OK) {
	  		Error_Handler();
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
