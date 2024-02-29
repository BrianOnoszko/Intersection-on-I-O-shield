/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file	          : freertos.c
  * @author			  : Brian Onoszko
  * @version		  : 1.0
  * @date			  : 18-12-2023
  * @brief			  : Structure for the RTOS implementation, main part of the code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "timers.h"
#include "project_functions.h"
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

/**
 * Definitions of timers
 */
TimerHandle_t xTimerHorizontal, xTimerVertical, xTimerPedNorth, xTimerPedWest, xTimerGreenDelay;

/**
 *  Definition of states for state machine and initialization state
 */
typedef enum{
	vertical,
	horizontal,
	redMax,
	pedestrianNorthMax,
	pedestrianWestMax
}states;
static states State = vertical;
static states NextState = vertical;

/* USER CODE END Variables */

/**
 *  Definitions for defaultTask
 */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/*
 * Definitions for inputLogicTask
 */
osThreadId_t inputLogicTaskHandle;
const osThreadAttr_t inputLogicTask_attributes = {
  .name = "inputLogicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/*
 * Definitions for plNorthBlink
 */
osThreadId_t plNorthBlinkHandle;
const osThreadAttr_t plNorthBlink_attributes = {
  .name = "plNorthBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/*
 *  Definitions for plWestBlink
 */
osThreadId_t plWestBlinkHandle;
const osThreadAttr_t plWestBlink_attributes = {
  .name = "plWestBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/*
 *  Definitions for outputLogicTask
 */
osThreadId_t outputLogicTaskHandle;
const osThreadAttr_t outputLogicTask_attributes = {
  .name = "outputLogicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/*
 * Definitions for timerCar1
 */
osTimerId_t timerCar1Handle;
const osTimerAttr_t timerCar1_attributes = {
  .name = "timerCar1"
};

/*
 * Definitions for mutex1
 */
osMutexId_t mutex1Handle;
const osMutexAttr_t mutex1_attributes = {
  .name = "mutex1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
 * Function declarations associated with tasks
 */
void StartDefaultTask(void *argument);
void input_logic(void *argument);
void blink_pl_north(void *argument);
void blink_pl_west(void *argument);
void output_logic(void *argument);

/**
 *  Function declaration for timer callback functions
 */
void timerCarCallback (TimerHandle_t Timer_t);
void timerPedCallback (TimerHandle_t Timer_t);
void timerGreenCallback (TimerHandle_t Timer_t);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutex1 */
  mutex1Handle = osMutexNew(&mutex1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timerCar1 */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /**
   * Timer initialization
   */
  xTimerHorizontal = xTimerCreate("timerHorizontal",
		  pdMS_TO_TICKS(100),
		  pdTRUE,
		  0,
		  timerCarCallback);
  xTimerVertical = xTimerCreate("timerVertical",
  		  pdMS_TO_TICKS(100),
  		  pdTRUE,
  		  0,
  		  timerCarCallback);
  xTimerPedNorth = xTimerCreate("timerPedN",
		  pdMS_TO_TICKS(100),
		  pdTRUE,
		  0,
		  timerPedCallback);
  xTimerPedWest = xTimerCreate("timerPedW",
  		  pdMS_TO_TICKS(100),
  		  pdTRUE,
  		  0,
  		  timerPedCallback);
  xTimerGreenDelay = xTimerCreate("timerGreenDelay",
		  pdMS_TO_TICKS(100),
		  pdTRUE,
		  0,
		  timerGreenCallback);


  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of inputLogicTask */
  inputLogicTaskHandle = osThreadNew(input_logic, NULL, &inputLogicTask_attributes);

  /* creation of plNorthBlink */
  plNorthBlinkHandle = osThreadNew(blink_pl_north, NULL, &plNorthBlink_attributes);

  /* creation of plWestBlink */
  plWestBlinkHandle = osThreadNew(blink_pl_west, NULL, &plWestBlink_attributes);

  /* creation of outputLogicTask */
  outputLogicTaskHandle = osThreadNew(output_logic, NULL, &outputLogicTask_attributes);

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
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_input_logic */
/**
* @brief Input logic, task for reading and interpreting input
* @brief Uses HAL library read function for switches, pedestrian buttons use interrupt so task simply checks state of pedestrians
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_input_logic */
void input_logic(void *argument)
{
  /* USER CODE BEGIN input_logic */
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);  //ms to ticks
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  if((HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin) == GPIO_PIN_RESET) || (HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin) == GPIO_PIN_RESET)){
		  waiting_tl_horizontal = 1;
		  if((xTimerIsTimerActive(xTimerHorizontal) == pdFALSE) && (status_tl_horizontal == 0)){
			  xTimerStart(xTimerHorizontal, 0);
		  }
		  if(status_tl_horizontal == 1){
			  xTimerStop(xTimerHorizontal, 0);
			  vTimerSetTimerID(xTimerHorizontal, (void *) 0);
		  }
	  } else if((HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin) == GPIO_PIN_SET) &&(HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin) == GPIO_PIN_SET)){
		  waiting_tl_horizontal = 0;
		  xTimerStop(xTimerHorizontal, 0);
		  vTimerSetTimerID(xTimerHorizontal, (void *) 0);
	  }
	  if((HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin) == GPIO_PIN_RESET) || (HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin) == GPIO_PIN_RESET)){
		  waiting_tl_vertical = 1;
		  if(xTimerIsTimerActive(xTimerVertical) == pdFALSE && status_tl_vertical == 0){
			  xTimerStart(xTimerVertical, 0);
		  }
		  if(status_tl_vertical == 1){
			  xTimerStop(xTimerVertical, 0);
			  vTimerSetTimerID(xTimerVertical, (void *) 0);
		  }
	  } else if((HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin) == GPIO_PIN_SET)){
		  waiting_tl_vertical = 0;
		  xTimerStop(xTimerVertical, 0);
		  vTimerSetTimerID(xTimerVertical, (void *) 0);
	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END input_logic */
}

/* USER CODE BEGIN Header_blink_pl_north */
/**
* @brief North pedestrian crossing logic, Task for north crossing wait light.
* @brief Uses waiting_pl_north state variable triggered by interrupt flag, toggles with toggle_pl_north() function, enables max delay timer
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink_pl_north */
void blink_pl_north(void *argument)
{
  /* USER CODE BEGIN blink_pl_north */
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(toggleFreq);  //ms to ticks
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    if(interrupt_pl_north == 1 && waiting_pl_north == 0){
    	waiting_pl_north = 1;
    	interrupt_pl_north = 0;
    	xTimerStart(xTimerPedNorth, 0);
    }
    if(waiting_pl_north == 1){
    	toggle_pl_north();
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END blink_pl_north */
}

/* USER CODE BEGIN Header_blink_pl_west */
/**
* @brief West pedestrian crossing logic, task for west crossing pedestrian wait light
* @brief Uses waiting_pl_west state variable triggered by interrupt flag, toggles with toggle_pl_west() function, enables max delay timer
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink_pl_west */
void blink_pl_west(void *argument)
{
  /* USER CODE BEGIN blink_pl_west */
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(toggleFreq);  //ms to ticks
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  if(interrupt_pl_west == 1 && waiting_pl_west == 0){
		  waiting_pl_west = 1;
		  interrupt_pl_west = 0;
		  xTimerStart(xTimerPedWest, 0);

	  }
	  if(waiting_pl_west){
		toggle_pl_west();
	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END blink_pl_west */
}

/* USER CODE BEGIN Header_output_logic */
/**
* @brief Output logic, task for reading states and giving the correct output
* @brief Uses state variables to choose the right state that the crossing should be in and displays it by communicating with traffic lights, uses functions from "project_functions.c" file
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_output_logic */
void output_logic(void *argument)
{
  /* USER CODE BEGIN output_logic */
  /* Infinite loop */
  for(;;)
  {
	  State = NextState;
	  switch(State){
	  case vertical:
		  if(waiting_tl_vertical){
			  if(waiting_pl_west){
				  waiting_pl_west = 0;
				  change_pl_west();
				  status_pl_west = 1;
				  osDelay(walkingDelay);
				  change_pl_west();
				  status_pl_west = 0;
			  }
			  NextState = vertical;
		  } else if(waiting_tl_horizontal){
			  change_tl_vertical();
			  change_tl_horizontal();
			  NextState = horizontal;
		  } else if(greenMax){
			  greenMax = 0;
			  change_tl_vertical();
			  change_tl_horizontal();
			  NextState = horizontal;
		  } else {
			  xTimerStart(xTimerGreenDelay,0);
			  if(waiting_pl_west){
				  waiting_pl_west = 0;
				  change_pl_west();
				  status_pl_west = 1;
				  osDelay(walkingDelay);
				  change_pl_west();
				  status_pl_west = 0;
			  }
		  }
		  break;
	  case horizontal:
		  if(waiting_tl_horizontal){
			  if(waiting_pl_north){
				  waiting_pl_north = 0;
				  change_pl_north();
				  status_pl_north = 1;
				  osDelay(walkingDelay);
				  change_pl_north();
				  status_pl_north = 0;
			  }
			  NextState = horizontal;
		  } else if(waiting_tl_vertical){
			  change_tl_horizontal();
			  change_tl_vertical();
			  NextState = vertical;
		  } else if(greenMax){
			  greenMax = 0;
			  change_tl_horizontal();
			  change_tl_vertical();
		  } else {
			  xTimerStart(xTimerGreenDelay, 0);
			  if(waiting_pl_north){
				  waiting_pl_north = 0;
				  change_pl_north();
				  status_pl_north = 1;
				  osDelay(walkingDelay);
				  change_pl_north();
				  status_pl_north = 0;
			  }
		  }
		  break;
	  case redMax:
		  if((status_tl_vertical == 1) && (waiting_tl_horizontal)){
			  change_tl_vertical();
			  change_tl_horizontal();
			  NextState = horizontal;
		  } else if((status_tl_horizontal == 1) && (waiting_tl_vertical)){
			  change_tl_horizontal();
			  change_tl_vertical();
			  NextState = vertical;
		  } else if(status_tl_vertical == 1){
			  NextState = vertical;
		  } else {
			  NextState = horizontal;
		  }
		  break;
	  case pedestrianNorthMax:
		  if(status_tl_horizontal == 0){
			  change_tl_vertical();
			  change_tl_horizontal();
		  }
		  waiting_pl_north = 0;
		  change_pl_north();
		  status_pl_north = 1;
		  osDelay(walkingDelay);
		  change_pl_north();
		  status_pl_north = 0;
		  change_tl_horizontal();
		  change_tl_vertical();
		  NextState = vertical;
		  break;
	  case pedestrianWestMax:
		  if(status_tl_vertical == 0){
			  change_tl_horizontal();
			  change_tl_vertical();
		  }
		  waiting_pl_west = 0;
		  change_pl_west();
		  status_pl_west = 1;
		  osDelay(walkingDelay);
		  change_pl_west();
		  status_pl_west = 0;
		  change_tl_vertical();
		  change_tl_horizontal();
		  NextState = horizontal;
		  break;
	  }
	  vTaskDelay(1000);
  }
  /* USER CODE END output_logic */
}


/**
 * @brief Timer Car callback function, handles car related timer timeouts
 *
 * @param TimerHandle_t xTimer, timer ID is matched to handle the correct timer
 *
 * @retval void
 */
void timerCarCallback (TimerHandle_t xTimer){
	uint32_t expiryCount1;
	uint32_t expiryCount2;
	const uint32_t maxExpiryCount = (redDelayMax/100);
	if(xTimer == xTimerHorizontal){
		expiryCount1 = (uint32_t) pvTimerGetTimerID(xTimer);
		expiryCount1++;
		if(status_tl_horizontal == 1){
			xTimerStop(xTimerHorizontal, 0);
			vTimerSetTimerID(xTimerHorizontal, (void *) 0);
		}
		if(expiryCount1 >= maxExpiryCount){
			xTimerStop(xTimer,0);
			NextState = redMax;
			vTimerSetTimerID(xTimer, (void *) 0);
		} else {
			vTimerSetTimerID(xTimer, (void *)expiryCount1);
		}
	} else if(xTimer == xTimerVertical){
		expiryCount2 = (uint32_t) pvTimerGetTimerID(xTimer);
		expiryCount2++;
		if(status_tl_vertical == 1){
			xTimerStop(xTimerVertical, 0);
			vTimerSetTimerID(xTimerVertical, (void *) 0);
		}
		if(expiryCount2 >= maxExpiryCount){
			xTimerStop(xTimer,0);
			NextState = redMax;
			vTimerSetTimerID(xTimer, (void *) 0);
		} else {
			vTimerSetTimerID(xTimer, (void *)expiryCount2);
		}
	}
}

/**
 * @brief Timer pedestrian callback function, handles pedestrian related timer timeouts
 *
 * @param TimerHandle_t xTimer, uses timer ID to handle the correct timer
 *
 * @retval void
 */
void timerPedCallback (TimerHandle_t xTimer){
	uint32_t expiryCountN;
	uint32_t expiryCountW;
	const uint32_t maxExpiryCount = (pedestrianDelay/100);
	if(xTimer == xTimerPedNorth){
		expiryCountN = (uint32_t) pvTimerGetTimerID(xTimer);
		expiryCountN++;

		if(expiryCountN >= maxExpiryCount && waiting_pl_north){
			vTimerSetTimerID(xTimer, (void *)0);
			xTimerStop(xTimer,0);
			if(State == vertical){
				NextState = pedestrianNorthMax;
			}
		} else {
			vTimerSetTimerID(xTimer, (void *)expiryCountN);
		}
	} else if(xTimer == xTimerPedWest){
		expiryCountW = (uint32_t) pvTimerGetTimerID(xTimer);
		expiryCountW++;

		if(expiryCountW >= maxExpiryCount && waiting_pl_west){
			vTimerSetTimerID(xTimer, (void *)0);
			xTimerStop(xTimer,0);
			if(State == horizontal){
				NextState = pedestrianWestMax;
			}
		} else {
			vTimerSetTimerID(xTimer, (void *)expiryCountW);
		}
	}
}

/**
 * @brief Timer greenDelay callback function, handles timeouts for greenDelay timer
 *
 * @param TimerHandle_t xTimer, increments expiry counter or stops timer
 *
 * @retval void
 */
void timerGreenCallback (TimerHandle_t xTimer){
	uint32_t expiryCountG;
	const uint32_t maxExpiryCount = (greenDelay / 100);
	expiryCountG = (uint32_t) pvTimerGetTimerID(xTimer);
	expiryCountG++;
	if(expiryCountG >= maxExpiryCount){
		vTimerSetTimerID(xTimer, (void *)0);
		xTimerStop(xTimer,0);
		if((waiting_tl_vertical == 0) && (waiting_tl_horizontal == 0)){
			greenMax = 1;
		}
	} else {
		vTimerSetTimerID(xTimer, (void *) expiryCountG);
	}
}
