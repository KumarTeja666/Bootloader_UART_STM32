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
#include"string.h"
#include"math.h"
#include"firmware.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TARGET_FLASH_ADDR 0x08000000
#define BIN_SIZE          8084     // Update with your actual .bin size

#define ACK  0x79
#define NACK 0x1F

#define STM32_FLASH_START_ADDR 0x08000000
#define MAX_CHUNK_SIZE 256
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCB_AIRCR (*(volatile uint32_t*)0xE000ED0C)  // Define AIRCR register address
#define VECTKEY   (0x05FA << 16)                     // VECTKEY value to unlock AIRCR
#define SYSRESETREQ (1 << 2)                         // SYSRESETREQ bit position
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t status=0;
uint8_t commands[256] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Utility: Send command and get ACK
#include "stm32f3xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1; // Your UART instance

#define ACK   0x79
#define NACK  0x1F

HAL_StatusTypeDef send_command(uint8_t cmd) {
	uint8_t packet[2] = {cmd, (uint8_t)(cmd ^ 0xFF)};
	HAL_UART_Transmit(&huart1, packet, 2, HAL_MAX_DELAY);
	uint8_t ack;
	HAL_UART_Receive(&huart1, &ack, 1, HAL_MAX_DELAY);
	return (ack == ACK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef send_get_command(void) {
	uint8_t cmd[2] = {0x00, 0xFF}; // Get command and its XOR
	HAL_StatusTypeDef status;

	// 1. Send the Get command
	status = HAL_UART_Transmit(&huart1, cmd, 2, HAL_MAX_DELAY);
	//if (status != HAL_OK) return status;

	// 2. Receive the first ACK
	uint8_t ack;
	status = HAL_UART_Receive(&huart1, &ack, 1, HAL_MAX_DELAY);
	//    if (status != HAL_OK || ack != ACK) {
	//        return HAL_ERROR;
	//    }

	// 3. Receive N (number of command bytes - 1)
	uint8_t n;
	status = HAL_UART_Receive(&huart1, &n, 1, HAL_MAX_DELAY);
	//if (status != HAL_OK) return status;

	// 4. Receive bootloader version
	uint8_t version;
	status = HAL_UART_Receive(&huart1, &version, 1, HAL_MAX_DELAY);
	//if (status != HAL_OK) return status;

	// 5. Receive N+1 command codes
	memset(commands,0,256);
	uint8_t num_cmds = n ;
	//uint8_t commands[256] = {0}; // just in case, enough size
	status = HAL_UART_Receive(&huart1, commands, num_cmds, HAL_MAX_DELAY);
	//if (status != HAL_OK) return status;

	// 6. Final ACK
	uint8_t final_ack;
	status = HAL_UART_Receive(&huart1, &final_ack, 1, HAL_MAX_DELAY);
	if (status != HAL_OK || final_ack != ACK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}


// Utility: Send 32-bit address + checksum
HAL_StatusTypeDef send_address(uint32_t address) {
	uint8_t addr_buf[5];
	addr_buf[0] = (address >> 24) & 0xFF;
	addr_buf[1] = (address >> 16) & 0xFF;
	addr_buf[2] = (address >> 8) & 0xFF;
	addr_buf[3] = address & 0xFF;
	addr_buf[4] = addr_buf[0] ^ addr_buf[1] ^ addr_buf[2] ^ addr_buf[3];

	HAL_UART_Transmit(&huart1, addr_buf, 5, HAL_MAX_DELAY);

	uint8_t ack;
	if (HAL_UART_Receive(&huart1, &ack, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	return (ack == ACK) ? HAL_OK : HAL_ERROR;
}

// Utility: Write data chunk (max 256 bytes)
HAL_StatusTypeDef send_data_chunk(const uint8_t *data, uint32_t length) {
	if (length == 0 || length > 256) return HAL_ERROR;

	uint8_t buffer[1 + 256 + 1];  // len + data + checksum
	buffer[0] = length - 1;

	uint8_t checksum = buffer[0];
	for (uint32_t i = 0; i < length; i++) {
		buffer[1 + i] = data[i];
		checksum ^= data[i];
	}
	buffer[1 + length] = checksum;

	HAL_UART_Transmit(&huart1, buffer, length + 2, HAL_MAX_DELAY);

	uint8_t ack;
	if (HAL_UART_Receive(&huart1, &ack, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	return (ack == ACK) ? HAL_OK : HAL_ERROR;
}

void send_firmware_to_target(void) {
	uint32_t offset = 0;
	uint32_t current_address = STM32_FLASH_START_ADDR;

	while (offset < firmware_size) {
		uint32_t chunk_size = ((firmware_size - offset) > MAX_CHUNK_SIZE)
                            		  ? MAX_CHUNK_SIZE
                            				  : (firmware_size - offset);

		if (send_command(0x31) != HAL_OK) {
			while(1);
		}

		if (send_address(current_address) != HAL_OK) {
			while(1);
		}

		if (send_data_chunk(&firmware_bin[offset], chunk_size) != HAL_OK) {
			while(1);
		}

		current_address += chunk_size;
		offset += chunk_size;

		HAL_Delay(10);  // Optional: small delay to ensure target is ready
	}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(250);



	status=send_command(0x7f);
	if(status==1)
	{
		while(1);
	}
	else
	{
		status=send_get_command();
	}

	HAL_Delay(1);
	status=send_command(0x44);
	if(status==1)
	{
		while(1);
	}
	else
	{
		uint8_t ack;
		HAL_Delay(1);
		uint8_t global_erase[3] = {0xFF, 0xFF, 0x00};
		HAL_UART_Transmit(&huart1, global_erase, 3, HAL_MAX_DELAY);
		HAL_UART_Receive(&huart1, &ack, 1, HAL_MAX_DELAY);
		if (ack != 0x79)
		{
			status=1;
		}
	}

	if(status==1)
	{
		while(1);
	}
	else
	{
       send_firmware_to_target();
	}

	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 0);

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(G_LED_GPIO_Port, G_LED_Pin);
		HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : G_LED_Pin */
  GPIO_InitStruct.Pin = G_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(G_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
