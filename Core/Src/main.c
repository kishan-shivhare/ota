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
#include"string.h"
#include<stdio.h>
#include<stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define raw_rx_buf_size    300
#define file_chunk_size    256
#define AT_BUF_SIZE        45
#define FLASH_APP_ADDR     0x8005000
#define APP_SIZE_BYTES     (44 * 1024)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
 //
 //
uint32_t current_offset = 0;
uint8_t at_rx_buf[AT_BUF_SIZE]; //
uint8_t ota_rx_buf[raw_rx_buf_size];


int i, j = 0;

uint8_t status = 9;
uint8_t attempt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SerialWrite(char *data);
void ec200u_reset();
HAL_StatusTypeDef Send_and_read_AT_Command_(const char *cmd,const char *expected_response, uint8_t max_retries, uint32_t timeout_ms);
HAL_StatusTypeDef Send_AT_command(const char *cmd, uint8_t *response,uint16_t response_size, uint32_t timeout_ms);
bool is_app_flash_erased(void);
void Erase_application_area();
void goto_application(void);
void perform_OTA();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET);

    SerialWrite("Bootloader Started\n");
    ec200u_reset();

	HAL_Delay(2000);
	perform_OTA();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void SerialWrite(char *data) {
	HAL_StatusTypeDef result;
	result = HAL_UART_Transmit(&huart2, (uint8_t*) data, strlen(data),
	HAL_MAX_DELAY);
	if (result != HAL_OK) {
		Error_Handler();
	}
}
void ec200u_reset(){
	SerialWrite("Reseting the Ec200u module\n");
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,RESET);
	HAL_Delay(5000);

}
HAL_StatusTypeDef Send_and_read_AT_Command_(const char *cmd,const char *expected_response, uint8_t max_retries, uint32_t timeout_ms) {
	i++;
	HAL_StatusTypeDef status;
	char at_cmd[80];
	uint16_t RxLen;
	for (attempt = 1; attempt <= max_retries; attempt++) {
		memset(ota_rx_buf, 0, raw_rx_buf_size);
		sprintf(at_cmd, "%s\r\n", cmd);
		status = HAL_UART_Transmit(&huart1, (uint8_t*) at_cmd, strlen(at_cmd),
				timeout_ms);
		if (status != HAL_OK)
			return status;
		// Wait for Trasmit to complete
		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
			;
		__HAL_UART_FLUSH_DRREGISTER(&huart1);
		RxLen = 0;
		status = HAL_UARTEx_ReceiveToIdle(&huart1, ota_rx_buf, raw_rx_buf_size - 1,
				&RxLen, timeout_ms);
		if(strstr(cmd,"AT+QFREAD")!= NULL){
			return status;
		}

		if (status == HAL_OK) {
			if (strstr((char*) ota_rx_buf, expected_response) != NULL) {
//i=10;
				HAL_UART_Transmit(&huart2, (uint8_t*) at_cmd, strlen(at_cmd),timeout_ms);
				HAL_UART_Transmit(&huart2,  ota_rx_buf, strlen(ota_rx_buf),timeout_ms);
				return HAL_OK;
			}

		}

		HAL_Delay(100);  // Optional delay before retry
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) at_cmd, strlen(at_cmd),timeout_ms);
	HAL_UART_Transmit(&huart2,  ota_rx_buf, strlen(ota_rx_buf),timeout_ms);
	// All retries failed
	return HAL_ERROR;
}

//HAL_StatusTypeDef Send_AT_command(const char *cmd, uint8_t *response,uint16_t response_size, uint32_t timeout_ms) {
//
//	HAL_StatusTypeDef status;
//	uint16_t RxLen;
//	memset(response, 0, response_size);
//	//__HAL_UART_FLUSH_DRREGISTER(&huart2);
//	char at_cmd[20];
//	sprintf(at_cmd, "%s\r\n", cmd);
//	status = HAL_UART_Transmit(&huart1, (uint8_t*) at_cmd, strlen(at_cmd),
//			timeout_ms);
//	if (status != HAL_OK)
//		return status;
//
//	__HAL_UART_FLUSH_DRREGISTER(&huart1);
//	RxLen = 0;
//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
//		;
//	HAL_UARTEx_ReceiveToIdle(&huart1, response, response_size - 1, &RxLen,
//			timeout_ms);
//	return status;
//
//}
void check_status(){
		if(status == HAL_ERROR){
				SerialWrite("AT response is wrong or something went wrong. Reset again\n");
				SerialWrite("Jumping to the existing application\n");

				goto_application();
			}
	}
bool is_app_flash_erased(void) {
    for (uint32_t addr = FLASH_APP_ADDR; addr < FLASH_APP_ADDR + APP_SIZE_BYTES;addr += 4)
    {
        if (*(uint32_t*)addr != 0xFFFFFFFF) {
            return false; // Not erased
        }
    }
    return true; // Fully erased
}
void Erase_application_area() {
	//SerialWrite("Erasing existing application\n");
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	uint32_t PageError = 0;

	// 1. Calculate number of pages to erase (round up)
	uint32_t numPages = (APP_SIZE_BYTES + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

	// 2. Erase application region
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_APP_ADDR;
	erase.NbPages = numPages;
	HAL_FLASHEx_Erase(&erase, &PageError);
	HAL_FLASH_Lock();


	if(is_app_flash_erased){
		SerialWrite("Application erased successfully\n");
	}
	else{
		SerialWrite("Error in Application erasing\n");
	}
}
void goto_application(void) {
    SerialWrite("Goto application started\n");

    uint32_t appStackPointer = *(volatile uint32_t*) FLASH_APP_ADDR;
    uint32_t appResetHandler = *(volatile uint32_t*) (FLASH_APP_ADDR + 4);

    // Validate stack pointer (to avoid jumping into empty flash)
    if ((appStackPointer & 0x2FFE0000) != 0x20000000) {
        SerialWrite("No valid application found. OTA required.\n");
        return;
    }

    HAL_UART_DeInit(&huart1);
    HAL_UART_DeInit(&huart2);
    HAL_RCC_DeInit();
    HAL_DeInit();
   __disable_irq();
//
    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Optional: Reset external module
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, RESET);

    // Set vector table base
    SCB->VTOR = FLASH_APP_ADDR;

    // Set stack pointer and jump
    __set_MSP(appStackPointer);
    void (*resetHandler)(void) = (void (*)(void)) appResetHandler;
    resetHandler();
}


void perform_OTA() {

	uint8_t file_data[file_chunk_size];
	uint32_t bytes_written = 0;
	uint16_t actual_data_len;
	int filehandle, filesize;
	char at[30];
	//HAL_Delay(5000);
	SerialWrite("OTA starting\n");

	status = Send_and_read_AT_Command_("AT", "\r\nOK\r\n", 2, 1000);
	check_status();
	HAL_Delay(1000);


	status = Send_and_read_AT_Command_("AT+CPIN?","\r\n+CPIN: READY\r\n\r\nOK\r\n", 5, 1000);
	check_status();
	HAL_Delay(1000);
	//SerialWrite("Deleting the file \n");
	status = Send_and_read_AT_Command_("AT+QFDEL=\"*\"", "\r\nOK\r\n", 5, 1000);
	check_status();
	HAL_Delay(1000);

	status = Send_and_read_AT_Command_("AT+QICSGP=1,1,\"internet\",\"\",\"\",0","\r\nOK\r\n", 5, 1000);
	HAL_Delay(2000);

	status = Send_and_read_AT_Command_("AT+QIACT?", "\r\nOK\r\n", 5, 1000);
	check_status();
	HAL_Delay(1000);

	if (strstr((char*)ota_rx_buf, "+QIACT: 1,1,1,\"") == NULL) {
		status = Send_and_read_AT_Command_("AT+QIACT=1", "\r\nOK\r\n", 5, 3000);
		HAL_Delay(1000);
		status = Send_and_read_AT_Command_("AT+QIACT?", "+QIACT: 1,1,1,\"", 5, 1000);
		HAL_Delay(1000);// Valid response

	}

	status = Send_and_read_AT_Command_("AT+QHTTPCFG=\"contextid\",1","\r\nOK\r\n", 5, 1000);
	check_status();
	HAL_Delay(1000);


	status = Send_and_read_AT_Command_("AT+QHTTPURL=79,200", "\r\nCONNECT\r\n",5, 2000);
	check_status();
	HAL_Delay(1000);


	status =Send_and_read_AT_Command_("https://raw.githubusercontent.com/kishan-shivhare/ota/main/f103_application.bin","\r\nOK\r\n", 5, 2000);
	check_status();
	HAL_Delay(1000);


	status = Send_and_read_AT_Command_("AT+QHTTPGET=400", "\r\nOK\r\n", 5,1000);
	check_status();
	HAL_Delay(3000);


	status = Send_and_read_AT_Command_("AT+QHTTPREADFILE=\"UFS:application.bin\",400", "\r\nOK\r\n", 5,2000);
	check_status();
	HAL_Delay(1000);

	SerialWrite("Check if any file not closed previously, closing the opened file \n");
	status = Send_and_read_AT_Command_("AT+QFCLOSE=1027", "\r\nOK\r\n", 1,2000);
	HAL_Delay(1000);

	status = Send_and_read_AT_Command_("AT+QFLST=\"UFS:application.bin\"","\r\n+QFLST: \"UFS:application.bin\",", 1, 1000);
	check_status();
	HAL_Delay(2000);

	sscanf((char*) ota_rx_buf, "\r\n+QFLST: \"UFS:application.bin\",%d",&filesize);

	status = Send_and_read_AT_Command_("AT+QFOPEN=\"UFS:application.bin\",0","\r\n+QFOPEN: ", 1, 2000);
	check_status();
	HAL_Delay(1000);


	sscanf((char*) ota_rx_buf, "\r\n+QFOPEN: %d", &filehandle);
	if(filesize && filehandle){
		char cmd[64];
		sprintf(cmd, "Downloaded filesize:%d,%d\n",filesize, filehandle);
		SerialWrite(cmd);
		Erase_application_area();
	}
	else{
		SerialWrite("file not found for OTA, closing the opened file and jumping to the existing application\n");
		sprintf(at, "AT+QFCLOSE=%d", filehandle);
		status = Send_and_read_AT_Command_(at, "\r\nOK\r\n", 1, 2000);
		goto_application();
		return;
	}
	//SerialWrite("Writing downloaded file to the application area\n");
	HAL_Delay(1000);
	HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < filesize; i += file_chunk_size) {
		j++;
		char cmd[64];
		sprintf(cmd, "AT+QFREAD=%d,%d", filehandle, file_chunk_size);

		//Send_AT_command(cmd, ota_rx_buf, raw_rx_buf_size, 2000);
		status = Send_and_read_AT_Command_(cmd,"\r\nCONNECT ", 1, 1000);
		check_status();
		// Search for "CONNECT"
		char *connect_ptr = strstr((char*) ota_rx_buf, "CONNECT");
		if (!connect_ptr)
			return HAL_ERROR;

		// Find actual data start (after \r\n following CONNECT)
		char *data_start = strstr(connect_ptr, "\r\n");
		if (!data_start)
			return HAL_ERROR;
		data_start += 2; // Skip \r\n

		sscanf((char*) ota_rx_buf, "\r\nCONNECT %d\r\n", &actual_data_len);

		memcpy(file_data, data_start, actual_data_len);

		for (uint32_t k = 0; k < actual_data_len; k += 4) {
			uint32_t word = *(uint32_t*) (file_data + k);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_APP_ADDR + i + k,word);
		}
		bytes_written += actual_data_len;

	}
	//SerialWrite("file write successfully \n");
	//SerialWrite("Closing the file \n");
	sprintf(at, "AT+QFCLOSE=%d", filehandle);
	status = Send_and_read_AT_Command_(at, "\r\nOK\r\n", 1, 2000);
	check_status();
	HAL_Delay(1000);

	//SerialWrite("Checking whole file is written successfully \n");
	if (bytes_written == filesize) {
		SerialWrite("OTA done\n");
		goto_application();
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
