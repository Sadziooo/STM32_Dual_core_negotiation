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
/* USER CODE BEGIN Variables */

#define  RPMSG_CHAN_NAME	"openamp_test"

//uint32_t message = 0;
char msg[] = "Hello baby from CM7";

static volatile int message_received;
static volatile int service_created;
volatile unsigned int received_data;
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static int rpmsg_recv_callback (struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
unsigned int receive_message (void);
void service_destroy_cb (struct rpmsg_endpoint *ept);
void new_service_cb (struct rpmsg_device *rdev, const char *name, uint32_t dest);

/* USER CODE END FunctionPrototypes */

void Start_RX_Task(void *argument);
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
  RX_TaskHandle = osThreadNew(Start_RX_Task, NULL, &RX_Task_attributes);

  /* creation of TX_Task */
  TX_TaskHandle = osThreadNew(Start_TX_Task, NULL, &TX_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_RX_Task */
/**
  * @brief  Function implementing the RX_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_RX_Task */
void Start_RX_Task(void *argument)
{
  /* USER CODE BEGIN Start_RX_Task */

	  MAILBOX_Init();

	  rpmsg_init_ept(&rp_endpoint, RPMSG_CHAN_NAME, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);

	  if (MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb) != HAL_OK) {
		  Error_Handler();
	  }

	  OPENAMP_Wait_EndPointready(&rp_endpoint);

  /* Infinite loop */
  for(;;)
  {
	  if (service_created) {
		  OPENAMP_check_for_message();
	  } else {
		  OPENAMP_DeInit();


		  osThreadTerminate(TX_TaskHandle);

		  for (uint8_t i = 0; i < 10; i++) {
			  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			  osDelay(100);
		  }

		  osThreadTerminate(RX_TaskHandle);

	  }

	  osDelay(1);
  }
  /* USER CODE END Start_RX_Task */
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

		if(OPENAMP_send (&rp_endpoint, msg, strlen(msg) + 1) < 0) {
			Error_Handler();
		}

//		if(OPENAMP_send (&rp_endpoint, &message, sizeof(message)) < 0) {
//			Error_Handler();
//		}
//
//		message++;

		osDelay(500);
  }
  /* USER CODE END Start_TX_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static int rpmsg_recv_callback (struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv) {
	received_data = *((unsigned int *) data);
	message_received = 1;

	return 0;
}

void service_destroy_cb (struct rpmsg_endpoint *ept) {
	service_created = 0;
}

void new_service_cb (struct rpmsg_device *rdev, const char *name, uint32_t dest) {
	OPENAMP_create_endpoint(&rp_endpoint, name, dest, rpmsg_recv_callback, service_destroy_cb);

	service_created = 1;
}

/* USER CODE END Application */

