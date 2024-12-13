/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "openamp.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  RPMSG_SERVICE_NAME	"openamp_test"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile int message_received;
volatile unsigned int received_data;
char *received_data_str;
static struct rpmsg_endpoint rp_endpoint;

/* USER CODE END Variables */
/* Definitions for RX_Task */
osThreadId_t RX_TaskHandle;
const osThreadAttr_t RX_Task_attributes = {
  .name = "RX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TX_Task */
osThreadId_t TX_TaskHandle;
const osThreadAttr_t TX_Task_attributes = {
  .name = "TX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for txSem */
osSemaphoreId_t txSemHandle;
const osSemaphoreAttr_t txSem_attributes = {
  .name = "txSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static int rpmsg_recv_callback (struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
unsigned int receive_message (void);

/* USER CODE END FunctionPrototypes */

void StartRX_Task(void *argument);
void Start_TX_Task(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of txSem */
  txSemHandle = osSemaphoreNew(1, 1, &txSem_attributes);

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
  /* creation of RX_Task */
  RX_TaskHandle = osThreadNew(StartRX_Task, NULL, &RX_Task_attributes);

  /* creation of TX_Task */
  TX_TaskHandle = osThreadNew(Start_TX_Task, NULL, &TX_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRX_Task */
/**
  * @brief  Function implementing the RX_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRX_Task */
void StartRX_Task(void *argument)
{
  /* USER CODE BEGIN StartRX_Task */

	  int32_t status = 0;

	  MAILBOX_Init();

	  if (MX_OPENAMP_Init(RPMSG_REMOTE, NULL) != HAL_OK) {
		  Error_Handler();
	  }

	  status = OPENAMP_create_endpoint (&rp_endpoint, RPMSG_SERVICE_NAME, RPMSG_ADDR_ANY, rpmsg_recv_callback, NULL);
	  if (status < 0) {
		  Error_Handler();
	  }

	  if (osSemaphoreAcquire(txSemHandle, osWaitForever) != osOK) {
		  Error_Handler();
	  }

  /* Infinite loop */
  for(;;)
  {
	  OPENAMP_check_for_message();

	  if (received_data == 10) {
		  OPENAMP_DeInit();

		  osThreadTerminate(TX_TaskHandle);

		  for (uint8_t i = 0; i < 10; i++) {
			  HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
			  osDelay(100);
		  }

		  osThreadTerminate(RX_TaskHandle);
	  }

	  osDelay(10);
  }
  /* USER CODE END StartRX_Task */
}

/* USER CODE BEGIN Header_Start_TX_Task */
/**
* @brief Function implementing the TX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_TX_Task */
void Start_TX_Task(void *argument)
{
  /* USER CODE BEGIN Start_TX_Task */
  /* Infinite loop */
  for(;;)
  {

		if (osSemaphoreAcquire(txSemHandle, osWaitForever) != osOK) {
		  Error_Handler();
		}

		char *data =  pvPortMalloc(100);
//		sprintf (data, "%u\n\r", received_data);
		sprintf (data, "%s\n\r", received_data_str);
		HAL_UART_Transmit(&huart3, (uint8_t *) data, strlen(data), 100);
		vPortFree(data);

		osDelay(1);
  }
  /* USER CODE END Start_TX_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static int rpmsg_recv_callback (struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv) {
//	received_data = *((unsigned int *) data);
	received_data_str = (char *) data;

	if (osSemaphoreRelease(txSemHandle) != osOK) {
	  Error_Handler();
	}

	return 0;
}

/* USER CODE END Application */

