/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
#define TARGET_AXIS_ID     0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 216 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = { .name = "myTask02", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of myTask02 */
	myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* USER CODE BEGIN 5 */
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	/* Infinite loop */
	for (;;) {
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)
				== HAL_OK) {
			// Gelen mesajın Status bilgisini kontrol et
			if (RxData[0] == 0x00) {
				// Komut başarıyla alındı
			} else if (RxData[0] == 0x01) {
				// Bir hata var!
			}
		}
		osDelay(1);
	}

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;
	char debug_msg[150]; // Mesajları formatlamak için değişken

	// 1. CAN Parametrelerini Belirle (Zaten sende var olan kısımlar)
	uint32_t priority = 7;
	uint32_t service_bit = 1;
	uint32_t request_bit = 1;
	uint32_t service_id = 0x00;
	uint32_t axis_id = 1;
	uint32_t dest_id = 1;
	uint32_t source_id = 2;

	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8; // Gönderilen veri boyutu (1 byte)

	// ExtId Hesapla
	TxHeader.ExtId = (priority << 26) | (service_bit << 25)
			| (request_bit << 24) | (service_id << 16) | (axis_id << 15)
			| (dest_id << 8) | (source_id);

	// Veriyi Hazırla (Enable komutu: 0x01)
	TxData[0] = 0x01;

	/* Infinite loop */
	for (;;) {

		snprintf(debug_msg, sizeof(debug_msg),
				"\r\n--- Yeni Mesaj Hazir ---\r\n"
						"CAN ID (Ext): 0x%08lX\r\n"
						"Data[0]: 0x%02X (Enable Komutu)\r\n"
						"DLC: %lu\r\n", TxHeader.ExtId, TxData[0],
				TxHeader.DLC);

		HAL_UART_Transmit(&huart1, (uint8_t*) debug_msg, strlen(debug_msg),
				100);

		// 2. Mesajı CAN Hattına Gönder
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
				== HAL_OK) {
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "DURUM: CAN Hattina Basildi (Mailbox OK)\r\n",
					41, 100);
		} else {
			uint32_t error = HAL_CAN_GetError(&hcan1);
			snprintf(debug_msg, sizeof(debug_msg),
					"HATA: CAN Basılamadı! Hata Kodu: 0x%08lX\r\n", error);
			HAL_UART_Transmit(&huart1, (uint8_t*) debug_msg, strlen(debug_msg),
					100);
		}

		osDelay(2000); // 2 saniyede bir gönder (Okuması kolay olsun)
	}
	/* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

