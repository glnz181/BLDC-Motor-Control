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
uint8_t rx_data[12]; // Sürücüden gelen cevabı tutacak

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
	HAL_UART_Receive_IT(&huart1, rx_data, 11);

	HAL_UART_Transmit(&huart3, (uint8_t*) "SISTEM TEST\r\n", 13, 100);
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

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
void send_enable(void) {
	uint8_t uartFrame[8];

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

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 1;        // DLC
	uartFrame[5] = 0x01;     // Data (Enable command)
	/* Frame Format:
	 [ID byte3]
	 [ID byte2]
	 [ID byte1]
	 [ID byte0]
	 [DLC]
	 [DATA]
	 */
	HAL_UART_Transmit(&huart1, uartFrame, 8, 10);

}

//void send_brake(void) {
//	uint8_t uartFrame[6];
//
//	uint32_t priority = 3;
//	uint32_t service_bit = 1;
//	uint32_t request_bit = 1;
//	uint32_t service_id = 0x01; // brake service
//	uint32_t axis_id = 1;
//	uint32_t dest_id = 1;
//	uint32_t source_id = 2;
//
//	uint32_t ExtId = (priority << 26) | (service_bit << 25)
//			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
//			| (dest_id << 8) | (source_id << 0);
//
//	uartFrame[0] = (ExtId >> 24) & 0xFF;
//	uartFrame[1] = (ExtId >> 16) & 0xFF;
//	uartFrame[2] = (ExtId >> 8) & 0xFF;
//	uartFrame[3] = (ExtId >> 0) & 0xFF;
//	uartFrame[4] = 1;        // DLC
//	uartFrame[5] = 0x01;     // Data (Enable command)
//	/* Frame Format:
//	 [ID byte3]
//	 [ID byte2]
//	 [ID byte1]
//	 [ID byte0]
//	 [DLC]
//	 [DATA]
//	 */
//	HAL_UART_Transmit(&huart1, uartFrame, 6, HAL_MAX_DELAY);
//
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == GPIO_PIN_0) { // Assuming the button is connected to GPIO_PIN_0
//		send_brake();
//		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13); // Toggle the LED on pin PA5
//	}
//
//}

void send_speed(int32_t current_speed) {
	uint8_t uartFrame[10];

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

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;
	uartFrame[4] = 0x05;        // DLC
	uartFrame[5] = 0x20;
	uartFrame[6] = (current_speed >> 24) & 0xFF; // MSB
	uartFrame[7] = (current_speed >> 16) & 0xFF;
	uartFrame[8] = (current_speed >> 8) & 0xFF;
	uartFrame[9] = (current_speed >> 0) & 0xFF;   // LSB

	HAL_UART_Transmit(&huart1, uartFrame, 10, 10);
}

void send_get(uint16_t address) {

	uint8_t uartFrame[8];

	uint32_t priority = 3;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x30; // Enable service
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	uint32_t ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id << 0);

	uartFrame[0] = (ExtId >> 24) & 0xFF;
	uartFrame[1] = (ExtId >> 16) & 0xFF;
	uartFrame[2] = (ExtId >> 8) & 0xFF;
	uartFrame[3] = (ExtId >> 0) & 0xFF;

	uartFrame[4] = 0x03;               // DLC (3 byte veri gidiyor)

	uartFrame[5] = 0x04;            // Data1: Length=4 (32-bit veri okuyacağız)
	uartFrame[6] = address & 0xFF;    // Data2: Addr0 (Adresin düşük byte'ı)
	uartFrame[7] = (address >> 8) & 0xFF; // Data3: Addr1 (Adresin yüksek byte'ı)

	HAL_UART_Transmit(&huart1, uartFrame, 8, 10);

	// RealTerm'de kendi sorunu da gör:
	HAL_UART_Transmit(&huart3, uartFrame, 8, 10);
	uint8_t nl[] = { 0x0D, 0x0A };
	HAL_UART_Transmit(&huart3, nl, 2, 10);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		// Ham veriyi sayıya dönüştür (Dökümana göre 7. byte'dan başlar)
		int32_t speed_raw = (int32_t) (rx_data[10] << 24) | (rx_data[9] << 16)
				| (rx_data[8] << 8) | (rx_data[7]);

		// RPM cinsinden hesapla
		float rpm = (float) speed_raw / 2048.0f;

		// PUUTY/RealTerm'e OKUNAKLI metin gönder
		printf("Motor Hizi: %.2f RPM\r\n", rpm);

		// Dinlemeyi tekrar aç
		HAL_UART_Receive_IT(&huart1, rx_data, 11);
	}
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
