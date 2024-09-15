/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <semphr.h>
#include <queue.h>
#include <stdbool.h>
#include <led.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for ledblink */
osThreadId_t ledblinkHandle;
const osThreadAttr_t ledblink_attributes = {
  .name = "ledblink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcd_gloq */
osThreadId_t lcd_gloqHandle;
const osThreadAttr_t lcd_gloq_attributes = {
  .name = "lcd_gloq",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for butttonstatus */
osTimerId_t butttonstatusHandle;
const osTimerAttr_t butttonstatus_attributes = {
  .name = "butttonstatus"
};
/* USER CODE BEGIN PV */

typedef struct {


	volatile bool  Sw1_State;
	volatile bool  Sw2_State;

}Sw_state;


typedef struct {


	volatile bool  Led1_State;
	volatile bool  Led2_State;

}Led_state;





SemaphoreHandle_t Mutex; //  avoids race conditions
QueueHandle_t xButtonQueue; // to send data from one task to other task properly
QueueHandle_t xLedState;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void BlinkLed(void *argument);
void lcd_send(void *argument);
void checkbutton(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_clear();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  Mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of butttonstatus */
  butttonstatusHandle = osTimerNew(checkbutton, osTimerPeriodic, NULL, &butttonstatus_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xButtonQueue = xQueueCreate(5, sizeof(Sw_state));
  xLedState    = xQueueCreate(1, sizeof(Led_state));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ledblink */
  ledblinkHandle = osThreadNew(BlinkLed, NULL, &ledblink_attributes);

  /* creation of lcd_gloq */
  lcd_gloqHandle = osThreadNew(lcd_send, NULL, &lcd_gloq_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_BlinkLed */
/**
  * @brief  Function implementing the ledblink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BlinkLed */
void BlinkLed(void *argument)
{
  /* USER CODE BEGIN 5 */
	osTimerStart(butttonstatusHandle, 100);
	Sw_state switchstate;
	Led_state ledstate;

  /* Infinite loop */
  for(;;)
  {
      if (xQueueReceive(xButtonQueue, &switchstate, portMAX_DELAY) == pdPASS)
      {
          xSemaphoreTake(Mutex, portMAX_DELAY);

          	  if(switchstate.Sw1_State == 1 && switchstate.Sw2_State==0 ){
          		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
          		  ledstate.Led1_State = 1;
          		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
          		ledstate.Led2_State = 0;


          	  }
          	  if(switchstate.Sw2_State ==1  && switchstate.Sw1_State==0){
          		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
          		ledstate.Led1_State = 0;
          		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
          		ledstate.Led2_State = 1;

          	  }
          	  if(switchstate.Sw2_State && switchstate.Sw1_State)
          	  {
          		ledstate.Led1_State = ledstate.Led1_State;
          		ledstate.Led2_State = ledstate.Led2_State;
          	  }
          	  if(switchstate.Sw2_State==0  && switchstate.Sw1_State==0){

          		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
          		  ledstate.Led1_State = !ledstate.Led1_State;
          		  osDelay(30);
          		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
          		 ledstate.Led2_State = !ledstate.Led2_State;
          		  osDelay(30);

          	  }




          xSemaphoreGive(Mutex);
      }

    	  xQueueSend(xLedState, &ledstate, 1);

  }




  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_lcd_send */
/**
* @brief Function implementing the lcd_gloq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lcd_send */
void lcd_send(void *argument)
{
  /* USER CODE BEGIN lcd_send */
	Sw_state switchstate;
	Led_state ledstate;
  /* Infinite loop */
  for(;;)

  {

	if (xQueueReceive(xButtonQueue, &switchstate, portMAX_DELAY) == pdPASS){

		//transmit data to lcd
		xSemaphoreTake(Mutex, portMAX_DELAY);
		// transmit
	    lcd_put_cur(0, 10);
	    if (switchstate.Sw1_State) {
	        lcd_send_string("SW1: ON");
	    } else {
	        lcd_send_string("SW1: OFF");
	    }

	    // Display button switch2 state
	    lcd_put_cur(1, 10); // Set cursor to second row, column 10
	    if (switchstate.Sw2_State) {
	        lcd_send_string("SW2: ON");
	    } else {
	        lcd_send_string("SW2: OFF");
	    }


        xSemaphoreGive(Mutex);

	}

	if (xQueueReceive(xLedState, &ledstate, portMAX_DELAY) == pdPASS){

		//transmit data to lcd
		xSemaphoreTake(Mutex, portMAX_DELAY);
		// transmit
	    lcd_put_cur(0, 0);
	    if (ledstate.Led1_State) {
	        lcd_send_string("LED1: ON");
	    } else {
	        lcd_send_string("LED1: OFF");
	    }
	    lcd_put_cur(1, 0); // Set cursor to the second row, first column
	    if (ledstate.Led2_State) {
	        lcd_send_string("LED2: ON");
	    } else {
	        lcd_send_string("LED2: OFF");
	    }

        xSemaphoreGive(Mutex);

	}
  }
  /* USER CODE END lcd_send */
}

/* checkbutton function */
void checkbutton(void *argument)
{
  /* USER CODE BEGIN checkbutton */
	Sw_state switchstate;



	xSemaphoreTake(Mutex,portMAX_DELAY);

		switchstate.Sw1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		switchstate.Sw2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

		xQueueSend(xButtonQueue, &switchstate, portMAX_DELAY);

	xSemaphoreGive(Mutex);




  /* USER CODE END checkbutton */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
