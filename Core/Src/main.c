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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

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

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_reset(uint32_t axis_id);
void send_enable(uint32_t axis_id);
void send_get_digital_inputs(uint32_t axis_id);
void send_get_digitaoutputs(uint32_t axis_id);
void send_set_acceleration(uint32_t axis_id, int32_t accel);
void send_speed_move(uint32_t axis_id, uint32_t speed);
void send_get_param(uint32_t axis_id, uint8_t addr0, uint8_t addr1);
void send_get_encoder_counter(uint32_t axis_id, uint8_t addr0, uint8_t addr1);
void send_set_can_options(uint32_t axis_id, uint32_t options_value);

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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	send_set_acceleration(0, 100000);

	HAL_Delay(100);

	send_enable(0);              // group 0 all-call
	HAL_Delay(500);

	while (1) {

		send_speed_move(0, 100);   // group 0 all-call
		HAL_Delay(5000);
		HAL_UART_Transmit(&huart3, (uint8_t*) "motor1: ", 10, HAL_MAX_DELAY);

		send_get_encoder_counter(1, 0x00, 0x20);
		HAL_UART_Transmit(&huart3, (uint8_t*) "motor2: ", 10, HAL_MAX_DELAY);

		send_get_encoder_counter(2, 0x00, 0x20);
		HAL_UART_Transmit(&huart3, (uint8_t*) "motor3: ", 10, HAL_MAX_DELAY);

		send_get_encoder_counter(3, 0x00, 0x20);

		HAL_Delay(1000);
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
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void send_reset(uint32_t axis_id) {
	uint8_t uartFrame[6];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0xFF;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 1;
	uartFrame[5] = 0x01;

	HAL_UART_Transmit(&huart1, uartFrame, 6, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart3, uartFrame, 6, HAL_MAX_DELAY);
}

void send_enable(uint32_t axis_id) {
	uint8_t uartFrame[6];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x00;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 1;
	uartFrame[5] = 0x01;

	HAL_UART_Transmit(&huart1, uartFrame, 6, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart3, uartFrame, 6, HAL_MAX_DELAY);
}
void send_get_digital_inputs(uint32_t axis_id) {
	uint8_t uartFrame[7];
	uint8_t rxByte[7];
	char rxBuffer[50];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 4;
	uartFrame[5] = 0x11;
	uartFrame[6] = 0x20; //00 11 20 01 00 00 00

	HAL_UART_Transmit(&huart1, uartFrame, 7, HAL_MAX_DELAY);
	memset(rxByte, 0, sizeof(rxByte));
	HAL_UART_Receive(&huart1, rxByte, 7, 100);

	sprintf(rxBuffer, "RX: %02X %02X %02X %02X %02X %02X %02X\r\n", rxByte[0],
			rxByte[1], rxByte[2], rxByte[3], rxByte[4], rxByte[5], rxByte[6]);

	HAL_UART_Transmit(&huart3, (uint8_t*) rxBuffer, strlen(rxBuffer),
	HAL_MAX_DELAY);
}

void send_get_digitaoutputs(uint32_t axis_id) {

	uint8_t uartFrame[7];
//	uint8_t rxByte[7];
//	char rxBuffer[50];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 4;
	uartFrame[5] = 0x12;
	uartFrame[6] = 0x21; //00 11 20 01 00 00 00

	HAL_UART_Transmit(&huart1, uartFrame, 7, HAL_MAX_DELAY);
//	memset(rxByte, 0, sizeof(rxByte));
//	HAL_UART_Receive(&huart1, rxByte, 7, 100);
//
//	sprintf(rxBuffer, "can out RX: %02X %02X %02X %02X %02X %02X %02X\r\n",
//			rxByte[0], rxByte[1], rxByte[2], rxByte[3], rxByte[4], rxByte[5],
//			rxByte[6]);

//	HAL_UART_Transmit(&huart3, (uint8_t*) rxBuffer, strlen(rxBuffer),
//	HAL_MAX_DELAY);

}

void send_get_encoder_counter(uint32_t axis_id, uint8_t addr0, uint8_t addr1) {
	uint8_t uartFrame[7];
	uint8_t rxByte[11];
	char rxBuffer[100];
	int32_t encoder_counter = 0;

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 4;       // 32-bit read
	uartFrame[5] = addr0;
	uartFrame[6] = addr1;

	HAL_UART_Transmit(&huart1, uartFrame, 7, HAL_MAX_DELAY);

	memset(rxByte, 0, sizeof(rxByte));
	HAL_UART_Receive(&huart1, rxByte, 11, 200);

	encoder_counter = ((rxByte[7] << 24) | (rxByte[8] << 16) | (rxByte[9] << 8)
			| rxByte[10]);

	sprintf(rxBuffer,
			"Axis %lu RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | Counter: %ld\r\n",
			(unsigned long) axis_id, rxByte[0], rxByte[1], rxByte[2], rxByte[3],
			rxByte[4], rxByte[5], rxByte[6], rxByte[7], rxByte[8], rxByte[9],
			rxByte[10], (long) encoder_counter);

	HAL_UART_Transmit(&huart3, (uint8_t*) rxBuffer, strlen(rxBuffer),
	HAL_MAX_DELAY);

}
void send_get_param(uint32_t axis_id, uint8_t addr0, uint8_t addr1) {
	uint8_t uartFrame[7];
	uint8_t rxByte[7];
	char rxBuffer[60];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 4;       // 32-bit read
	uartFrame[5] = addr0;
	uartFrame[6] = addr1;

	HAL_UART_Transmit(&huart1, uartFrame, 7, HAL_MAX_DELAY);

	memset(rxByte, 0, sizeof(rxByte));
	HAL_UART_Receive(&huart1, rxByte, 7, 200);

	sprintf(rxBuffer, "GET RX: %02X %02X %02X %02X %02X %02X %02X\r\n",
			rxByte[0], rxByte[1], rxByte[2], rxByte[3], rxByte[4], rxByte[5],
			rxByte[6]);

	HAL_UART_Transmit(&huart3, (uint8_t*) rxBuffer, strlen(rxBuffer),
	HAL_MAX_DELAY);
}

void send_set_acceleration(uint32_t axis_id, int32_t accel) {
	uint8_t uartFrame[11];
//	uint8_t rx[7];
	//char buffer[60];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x20;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 4;

	uartFrame[5] = 0xA1;
	uartFrame[6] = 0x04;

	uartFrame[7] = (accel >> 24) & 0xFF;
	uartFrame[8] = (accel >> 16) & 0xFF;
	uartFrame[9] = (accel >> 8) & 0xFF;
	uartFrame[10] = (accel >> 0) & 0xFF;

	HAL_UART_Transmit(&huart1, uartFrame, 11, HAL_MAX_DELAY);

//	memset(rx, 0, sizeof(rx));
//	HAL_UART_Receive(&huart1, rx, 7, 200);
//
//	sprintf(buffer, "ACC RX: %02X %02X %02X %02X %02X %02X %02X\r\n", rx[0],
//			rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);
//	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer),
//	HAL_MAX_DELAY);

}

void send_set_can_options(uint32_t axis_id, uint32_t options_value) {
	uint8_t uartFrame[11];
	uint8_t rx[7];
	char buffer[80];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x20;      // SET
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 4;       // uint32 length
	uartFrame[5] = 0xAA;    // Addr0
	uartFrame[6] = 0x04;    // Addr1

	uartFrame[7] = (options_value >> 24) & 0xFF;
	uartFrame[8] = (options_value >> 16) & 0xFF;
	uartFrame[9] = (options_value >> 8) & 0xFF;
	uartFrame[10] = (options_value >> 0) & 0xFF;

	HAL_UART_Transmit(&huart1, uartFrame, 11, HAL_MAX_DELAY);

	memset(rx, 0, sizeof(rx));
	HAL_UART_Receive(&huart1, rx, 7, 200);

	sprintf(buffer, "CAN OPT RX: %02X %02X %02X %02X %02X %02X %02X\r\n", rx[0],
			rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);

	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer),
	HAL_MAX_DELAY);
}

void send_speed_move(uint32_t axis_id, uint32_t speed) {
	uint8_t uartFrame[9];
//	uint8_t rx[7];
//	char buffer[60];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x41;
	uint32_t axis_id_group = (axis_id == 0) ? 0 : 1;
	uint32_t dest_id = (axis_id == 0) ? 0 : axis_id;
	uint32_t source_id = 10;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id_group << 15)
			| (dest_id << 8) | (source_id << 0);

	int32_t speed_iu = speed * 2048;

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 0x20;

	uartFrame[5] = (speed_iu >> 24) & 0xFF;
	uartFrame[6] = (speed_iu >> 16) & 0xFF;
	uartFrame[7] = (speed_iu >> 8) & 0xFF;
	uartFrame[8] = (speed_iu >> 0) & 0xFF;

	HAL_UART_Transmit(&huart1, uartFrame, 9, HAL_MAX_DELAY);

//	memset(rx, 0, sizeof(rx));
//	HAL_UART_Receive(&huart1, rx, 7, 200);
//
//	sprintf(buffer, "MOVE RX: %02X %02X %02X %02X %02X %02X %02X\r\n", rx[0],
//			rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);

//	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer),
//	HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
