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
#include "queue.h"

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
/* USER CODE BEGIN Variables */

QueueHandle_t motorTxQueue;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern void send_enable(void);
extern void send_speed(int32_t current_speed);
extern void send_reset(void);
extern void send_get(uint16_t address);
extern void send_stop(void);

extern volatile int32_t last_speed_raw;
extern volatile uint8_t speed_flag;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 216 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = { .name = "myTask02", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for motorTxQueue */
osMessageQueueId_t motorTxQueueHandle;
const osMessageQueueAttr_t motorTxQueue_attributes = { .name = "motorTxQueue" };

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

	motorTxQueue = xQueueCreate(1, sizeof(MotorCommand));

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

	/* Create the queue(s) */
	/* creation of motorTxQueue */
	motorTxQueueHandle = osMessageQueueNew(16, sizeof(uint16_t),
			&motorTxQueue_attributes);

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

	osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
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
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* USER CODE BEGIN 5 */

	MotorCommand cmd;

	/* Infinite loop */
	for (;;) {
		if (xQueueReceive(motorTxQueue, &cmd, portMAX_DELAY) == pdPASS) {
			// Kuyruktan komutu al ve işleme koy
			// Örneğin, cmd.data ve cmd.lenght kullanarak UART ile gönder
			HAL_UART_Transmit(&huart1, cmd.data, cmd.length, HAL_MAX_DELAY);
			osDelay(10); // Gönderim sonrası kısa bir gecikme ekleyebilirsiniz
		}

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
	//char buf[60];
//	send_reset();
//	osDelay(1000);

	send_enable();
	osDelay(500);

	for (;;) {
		// 1. İLERİ (Artı yön)
		send_speed(40960);
		osDelay(6000); // 3 saniye dön

		send_stop();
		osDelay(2000); // 2 saniye bekle

		// 3. GERİ (Eksi yön)
		send_speed(-40960);
		osDelay(6000); // 3 saniye dön

		send_stop();
		osDelay(2000); // 2 saniye bekle

	}
	/* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

