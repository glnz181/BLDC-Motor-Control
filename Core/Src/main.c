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
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "queue.h"

extern QueueHandle_t motorTxQueue;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t data[10];
	uint8_t length;

} MotorCommand;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data[12];
volatile int32_t last_speed_raw = 0;
volatile uint8_t speed_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart1, rx_data, 12);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void send_enable(void) {

	MotorCommand cmd;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x00; // Enable service
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	cmd.data[0] = (ExtId >> 24) & 0xFF;
	cmd.data[1] = (ExtId >> 16) & 0xFF;
	cmd.data[2] = (ExtId >> 8) & 0xFF;
	cmd.data[3] = (ExtId >> 0) & 0xFF;
	cmd.data[4] = 1;        // DLC
	cmd.data[5] = 0x01;     // Data (Enable command)

	cmd.length = 6;

	xQueueSend(motorTxQueue, &cmd, 0);
}

void send_speed(int32_t current_speed) {

	MotorCommand cmd;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x41; // Set speed service
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	cmd.data[0] = (ExtId >> 24) & 0xFF;
	cmd.data[1] = (ExtId >> 16) & 0xFF;
	cmd.data[2] = (ExtId >> 8) & 0xFF;
	cmd.data[3] = (ExtId >> 0) & 0xFF;
	cmd.data[4] = 0x05;        // DLC
	cmd.data[5] = 0x20;
	cmd.data[6] = (current_speed >> 0) & 0xFF;  // Byte0 (LSB)
	cmd.data[7] = (current_speed >> 8) & 0xFF;  // Byte1
	cmd.data[8] = (current_speed >> 16) & 0xFF; // Byte2
	cmd.data[9] = (current_speed >> 24) & 0xFF; // Byte3 (MSB)

	cmd.length = 10;

	// portMAX_DELAY yerine 100 ms verdik. Eğer kuyruk doluysa sonsuza kadar kilitlenmeyecek!
	if (xQueueSend(motorTxQueue, &cmd, 100) != pdPASS) {
		// Eğer 100ms içinde kuyruğa yazamazsa hata bas
		char msg[] = "HATA: Kuyruk Dolu! send_speed kilitlendi.\r\n";
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
	} else {
		// Başarıyla kuyruğa yazdıysa bilgi bas
		char msg[] = "BASARILI: Komut kuyruga eklendi.\r\n";
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
	}
}
void send_reset(void) {

	MotorCommand cmd;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0xFF; // Set speed service
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	cmd.data[0] = (ExtId >> 24) & 0xFF;
	cmd.data[1] = (ExtId >> 16) & 0xFF;
	cmd.data[2] = (ExtId >> 8) & 0xFF;
	cmd.data[3] = (ExtId >> 0) & 0xFF;
	cmd.data[4] = 0x00;        // DLC

	cmd.length = 5;

	xQueueSend(motorTxQueue, &cmd, portMAX_DELAY);
}

void send_get(uint16_t address) {
	MotorCommand cmd;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30; // Get speed service
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	cmd.data[0] = (ExtId >> 24) & 0xFF;
	cmd.data[1] = (ExtId >> 16) & 0xFF;
	cmd.data[2] = (ExtId >> 8) & 0xFF;
	cmd.data[3] = (ExtId >> 0) & 0xFF;

	cmd.data[4] = 0x03;        // DLC

	cmd.data[5] = 0x04; //length
	cmd.data[6] = address & 0xFF;
	cmd.data[7] = (address >> 8) & 0xFF;

	cmd.length = 8;

	xQueueSend(motorTxQueue, &cmd, portMAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		last_speed_raw = (int32_t) ((rx_data[10] << 24) | (rx_data[9] << 16)
				| (rx_data[8] << 8) | (rx_data[7]));

		speed_flag = 1;

		HAL_UART_Receive_IT(&huart1, rx_data, 12);
	}
}

void send_stop(void) {
	MotorCommand cmd;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x41; // MOVE Service ID
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	cmd.data[0] = (ExtId >> 24) & 0xFF;
	cmd.data[1] = (ExtId >> 16) & 0xFF;
	cmd.data[2] = (ExtId >> 8) & 0xFF;
	cmd.data[3] = (ExtId >> 0) & 0xFF;

	cmd.data[4] = 0x05;        // DLC
	cmd.data[5] = 0x81;  // CMD: 0x81 -> Abort Decel. to Stop (Yavaşlayarak Dur)

	cmd.data[6] = 0x00;
	cmd.data[7] = 0x00;
	cmd.data[8] = 0x00;
	cmd.data[9] = 0x00;

	cmd.length = 10;
	xQueueSend(motorTxQueue, &cmd, 100);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
