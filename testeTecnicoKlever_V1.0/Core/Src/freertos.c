/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "comm.h"
#include "adc.h"
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
QueueHandle_t xUartTaskQueue;
QueueHandle_t xLedTaskQueue;
QueueHandle_t xLedTaskQueue;
SemaphoreHandle_t xUartSemaphore;
/* USER CODE END Variables */
osThreadId ADC_TaskHandle;
osThreadId SERIAL_TaskHandle;
osThreadId LED_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ADCTask(void const * argument);
void SerialTask(void const * argument);
void LedTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  xUartSemaphore = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xUartTaskQueue = xQueueCreate(5, sizeof(uint8_t));
  xLedTaskQueue = xQueueCreate(5, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ADC_Task */
  osThreadDef(ADC_Task, ADCTask, osPriorityNormal, 0, 128);
  ADC_TaskHandle = osThreadCreate(osThread(ADC_Task), NULL);

  /* definition and creation of SERIAL_Task */
  osThreadDef(SERIAL_Task, SerialTask, osPriorityBelowNormal, 0, 128);
  SERIAL_TaskHandle = osThreadCreate(osThread(SERIAL_Task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, LedTask, osPriorityLow, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ADCTask */
/**
  * @brief  Function implementing the ADC_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ADCTask */
void ADCTask(void const * argument)
{
  /* USER CODE BEGIN ADCTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t valorAD = HAL_ADC_GetValue(&hadc1);

	uint32_t DummyNotification = 0;
	BaseType_t TaskNotify = xTaskNotifyWait(pdFALSE, pdTRUE, &DummyNotification, 0);
	if(TaskNotify == pdTRUE){
		if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
			SendLeituraADC(valorAD);
			xSemaphoreGive(xUartSemaphore);
		}
	}
    osDelay(100);
  }
  /* USER CODE END ADCTask */
}

/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the SERIAL_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
void SerialTask(void const * argument)
{
  /* USER CODE BEGIN SerialTask */
  char CMDUartRcv = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(xUartTaskQueue, &CMDUartRcv, portMAX_DELAY)){
		  switch(CMDUartRcv){
		  case LIGAR_LED:
				if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
					SendACK();
					xSemaphoreGive(xUartSemaphore);
				}
				xQueueSend(xLedTaskQueue, &CMDUartRcv, portMAX_DELAY);
			  break;
		  case DESLIGAR_LED:
				if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
					SendACK();
					xSemaphoreGive(xUartSemaphore);
				}
				xQueueSend(xLedTaskQueue, &CMDUartRcv, portMAX_DELAY);
			  break;

		  case TOOGLE_LED:
				if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
					SendACK();
					xSemaphoreGive(xUartSemaphore);
				}
				xQueueSend(xLedTaskQueue, &CMDUartRcv, portMAX_DELAY);
			  break;
		  case LOOP_BACK:
				if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
					SendLoopBack();
					xSemaphoreGive(xUartSemaphore);
				}
			  break;

		  case LER_AD:

			  break;

		  case PACOTE_INCORRETO:
				if(xSemaphoreTake(xUartSemaphore, portMAX_DELAY)){
					SendPacoteIncorreto();
					xSemaphoreGive(xUartSemaphore);
				}
			  break;
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END SerialTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void const * argument)
{
  /* USER CODE BEGIN LedTask */
	char CMDLedRcv = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(xLedTaskQueue,&CMDLedRcv, portMAX_DELAY)){
		  switch(CMDLedRcv){
			  case LIGAR_LED:
				  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				  break;
			  case DESLIGAR_LED:
				  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				  break;
			  case TOOGLE_LED:
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  break;
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END LedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
