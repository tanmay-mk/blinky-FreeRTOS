/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
//#include "cmsis_os.h"			//not using CMSIS wrapped API's

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//Using FreeRTOS API's directly, copied from cmsis_os.h
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

//application specific library files
#include <stdio.h>			//for printf
#include <stdbool.h>		//for bool keyword
#include <string.h>			//for string operations
#include <ctype.h>			//for isalpha()
#include <stdlib.h>			//for atoi()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//parameters required to manipulate PWM generation
typedef enum parameters {
	none 		= 0,
	frequency 	= 1,
	duty_cycle 	= 2
}parameter_t;

//states that state machine will traverse through
typedef enum states{
	STATE0_START,
	STATE1_RX_CHOICE,
	STATE2_RX_DATA,
	STATE3_CHANGE_PWM
}state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//events that the event group will set/reset/wait for
#define EVT_PRINT_USR_MSG 		(1 << 0)	//just to kick off first execution
#define EVT_VALID_CHOICE		(1 << 1)	//will be set if user inputs a valid choice (frequency or parameter)
#define EVT_VALID_DATA			(1 << 2)	//will be set if user inputs a valid data (within range based on parameter)
#define EVT_INVALID_DATA		(1 << 3)	//will be set if user inputs either invalid data or invalid choice

#define EVT_ALL					(							\
									EVT_PRINT_USR_MSG 	|	\
									EVT_VALID_CHOICE	|	\
									EVT_VALID_DATA		|	\
									EVT_INVALID_DATA		\
								)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
volatile
uint32_t	time_period	=	1000000,		//controls frequency of PWM
			pulse_width	=	500000,			//controls duty cycle of PWM
			dcycle		=	50;				//to convert duty cycle accordingly if frequency is changed

bool		rx_flag		=	false,			//gets set when a data packet is received from user
			choice_done	=	false,			//gets set when valid choice is received
			value_done	=	false;			//gets set when valid data is received

uint8_t		byte,							//used to store user input - only one byte
			buffer[20];						//used to store user input - byte by byte until
											//carriage return is encountered

parameter_t param 		= 	none;			//global variable to store user's choice for operation
uint32_t 	value 		= 	0;				//global variable to store user's desired value of frequency or duty cycle


TaskHandle_t 		UART_Rx_Complete_Handle = NULL;	//task handle for UART_Rx_Complete
TaskHandle_t 		Parse_Data_Handle 		= NULL;	//task handle for Parse_Data
TaskHandle_t 		Control_States_Handle 	= NULL;	//task handle for Control_States

QueueHandle_t		xQueueHandle1 	= NULL;	//queue handle for a message queue
SemaphoreHandle_t 	xMutex 			= NULL;	//variable to handle mutex locks and unlocks
EventGroupHandle_t 	xEventsGroup 	= NULL;	//variable to handle the event group

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/*
 * @brief		:	Middle Priority Task, waits for UART Callback function
 * 					to set the flag. Once the flag is set, it sends the data
 * 					to lower priority task over a message queue.
 *
 * @parameters	:	void *args	:	arguments required to process, if any
 *
 * @returns		:	ideally should never return, in case of an error, calls
 * 					vTaskDelete() to exit gracefully
 */
void UART_Rx_Complete (void *args);

/*
 * @brief		:	Lowest priority task, waits for UART_Rx_Complete to enqueue
 * 					data on message queue. Once data is available on the queue,
 * 					it receives the data, checks its validity and sets appropriate
 * 					events which drive the state machine.
 *
 * @parameters	:	void *args	:	arguments required to process, if any
 *
 * @returns		:	ideally should never return, in case of an error, calls
 * 					vTaskDelete() to exit gracefully
 *
 */
void Parse_Data (void *args);

/*
 *	@brief		:	Highest priority task, checks whether an event is set or not,
 *					and drives the state machine accordingly.
 *
 * @parameters	:	void *args	:	arguments required to process, if any
 *
 * @returns		:	ideally should never return, in case of an error, calls
 * 					vTaskDelete() to exit gracefully
 */
void Control_States (void *args);

/*
 * @brief		:	checks whether the data input by user as a desired value is
 * 					valid or not. Checks for alphabets present and whether the
 * 					value is in the range for a specific parameter selected
 *
 * @parameters	:	uint8_t *data	:	pointer to a string in which the data to
 * 										be validated is stored
 *
 * @returns		:	bool			:	true 	- if data is valid
 * 										false	- if data is invalid
 */
bool is_valid_data(uint8_t *data);

/*
 * @brief		:	checks whether the choice input by user as a desired value is
 * 					valid or not. Checks for alphabets present and whether the
 * 					choice is within the specifications provided
 *
 * @parameters	:	uint8_t *data	:	pointer to a string in which the choice to
 * 										be validated is stored
 *
 * @returns		:	bool			:	true 	- if choice is valid
 * 										false	- if choice is invalid
 */
bool is_valid_choice(uint8_t *data);

/*
 * @brief		:	state machine that drives the entire operations. changes states
 * 					based on events
 *
 * @parameters	:	EventBits_t event	:	event set by the event group
 *
 * @returns 	:	none
 */
void state_machine(EventBits_t event);

/*
 * @brief		:	adjusts the PWM generation based on parameter selected and value
 *					provided
 *
 * @parameters	:	parameter_t parameter	:	parameter that needs to be changed
 * 												either frequency or duty cycle
 *
 * 					uint32_t value			:	desired value to be set for the selected
 * 												parameter
 * 												for frequency, 0 < value < 1M
 * 												for duty cycle, 0 <= value <= 100
 *
 * @returns		:	none
 */
void change_pwm_parameters(parameter_t parameter, uint32_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Retargets the C library printf function to the USART.
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

//this gets called after data reception is complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t i = 0;

	if (byte != '\b')				//if character is not backspace
	{
		buffer[i++] = byte;			//store the received byte in a buffer
		printf("%c", byte);			//print the data
		fflush(stdout);				//force flush to stdout
	}
	else							//if user enters backspace
	{
		if (i == 0)					//this will avoid segmentation fault
		{							//you don't want to access buffer[-1]
			//do nothing if i = 0
		}

		else
		{
			i--;					//decrement index counter so that we can overwrite in buffer
			printf("\b \b");		//erase previously entered character
			fflush(stdout);			//force flush to stdout
		}
	}

	if (byte == '\r')			//user hit enter, data reception complete
	{
		printf("\n");
		rx_flag = true;
		buffer[i++] = '\0';		//put a null character at the end of buffer
		i=0;					//reset index counter to zero for next iteration
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&byte, 1);	//re-arm the interrupt
		return;
	}

	if (i >= 20)		//to avoid seg fault
	{					//should never come here, ideally
		i = 0;
	}

	HAL_UART_Receive_IT(&huart2, (uint8_t *)&byte, 1);
}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&byte, 1); //enable UART Receive Interrupt

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//start generating PWM

  //create events group
  xEventsGroup = xEventGroupCreate();
  if (xEventsGroup == NULL)
  {
	  printf("Failed to create Message Queue... Entering infinite loop\n\r");
	  while(1);
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* Create a mutex type semaphore. */
  xMutex = xSemaphoreCreateMutex();
  if( xMutex == NULL )
  {
	  printf("Failed to create Semaphore Mutex... Entering infinite loop\n\r");
	  while(1);
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  //create message queue

  xQueueHandle1 = xQueueCreate(25, sizeof(uint8_t));
  if (xQueueHandle1 == NULL)
  {
	  printf("Failed to create Message Queue... Entering infinite loop\n\r");
	  while(1);
  }

  /* USER CODE END RTOS_QUEUES */

//  /* Create the thread(s) */
//  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //create tasks
   BaseType_t xStatus = 0;		//to check return status of xTaskCreate()

  //task with highest priority
  	xStatus = xTaskCreate(
		  	  Control_States,
		  	  "Control_States",
			  512,
			  NULL,
			  tskIDLE_PRIORITY+3,
			  &Control_States_Handle);

  	if (xStatus != pdPASS)
  	{
  		printf("Failed to create task Control_States... Entering infinite loop\n\r");
  		while(1);
  	}

  	xStatus = 0;
  	//task with middle priority
    xStatus = xTaskCreate(
  		  	  UART_Rx_Complete,
  		  	  "UART_Rx_Complete",
  			  512,
  			  NULL,
  			  tskIDLE_PRIORITY+2,
			  &UART_Rx_Complete_Handle);

    if (xStatus != pdPASS)
    {
  	  printf("Failed to create task UART_Rx_Complete... Entering infinite loop\n\r");
  	  while(1);
    }

    xStatus = 0;
    //task with lowest priority
    xStatus = xTaskCreate(
  		  	  Parse_Data,
  		  	  "Parse_Data",
			  512,
  			  NULL,
  			  tskIDLE_PRIORITY+1,
			  &Parse_Data_Handle);

    if (xStatus != pdPASS)
    {
  	  printf("Failed to create task Parse_Data... Entering infinite loop\n\r");
  	  while(1);
    }

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //putting it here so that re-generating .ioc file will not erase it
  vTaskStartScheduler();

  //should never come here
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = time_period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse_width;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-------------------------------------------------------------------------
void UART_Rx_Complete (void *args)
{
	uint8_t byte_rcvd;
	bool rx_flag_local;
	static uint32_t i = 0;
	for(;;)
	{
		while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
		rx_flag_local = rx_flag;												//reading global memory
		while(xSemaphoreGive(xMutex) != pdTRUE);

		if(rx_flag_local == true)
		{
			while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
			byte_rcvd = buffer[i++];
			while(xSemaphoreGive(xMutex) != pdTRUE);

			xQueueSend(xQueueHandle1, &byte_rcvd, 100);
			if (byte_rcvd == '\r')
			{
				i = 0;

				while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);		//wait here till we obtain the mutex lock
				rx_flag = false;												//update global variables
				memset (buffer, 0, sizeof(buffer));
				while(xSemaphoreGive(xMutex) != pdTRUE);						//wait here till we release the mutex lock

			}
			vTaskDelay(100);
		}
	}

	//it should never come here
	//in case of accidental exit, gracefully exit
	vTaskDelete(UART_Rx_Complete_Handle);
}

//-------------------------------------------------------------------------
void Parse_Data (void *args)
{
	uint8_t byte_in, data[20];
	static uint32_t i = 0;
	bool choice_done_local = false, value_done_local = false;

	for(;;)
	{
		if (xQueueReceive(xQueueHandle1, &byte_in, 100) == pdTRUE)
		{
			data[i++] = byte_in;
			if (byte_in == '\r' || i == 20)
			{
				while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
				choice_done_local = choice_done;
				value_done_local = value_done;
				while(xSemaphoreGive(xMutex) != pdTRUE);

				if (!choice_done_local)
				{
					if (is_valid_choice((uint8_t *)&data) == true)
					{
						//critical section for global variable
						while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);	//wait here till we obtain the mutex lock
						choice_done = true;											//update global variables
						while(xSemaphoreGive(xMutex) != pdTRUE);					//wait here till we release the mutex lock

						i = 0;
						byte_in = 0;
						xEventGroupSetBits(xEventsGroup,EVT_VALID_CHOICE);
						continue;
					}
					else
					{
						printf("Invalid Input, start over ... \n\r");

						//critical section for global variable
						while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
						choice_done = false;
						while(xSemaphoreGive(xMutex) != pdTRUE);

						xEventGroupSetBits(xEventsGroup,EVT_INVALID_DATA);
					}
				}

				if (choice_done_local && !value_done_local)
				{
					if (is_valid_data((uint8_t *)&data) == true)
					{
						//critical section for global variable
						while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
						value_done = true;
						while(xSemaphoreGive(xMutex) != pdTRUE);

						xEventGroupSetBits(xEventsGroup,EVT_VALID_DATA);
					}
					else
					{
						printf("Invalid Input, start over ... \n\r");

						//critical section for global variable
						while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
						choice_done = false;
						value_done = false;
						while(xSemaphoreGive(xMutex) != pdTRUE);

						xEventGroupSetBits(xEventsGroup,EVT_INVALID_DATA);
					}
				}

				i = 0;
				memset (data, 0, sizeof(data));
			}
			vTaskDelay(100);
		}
	}

	//it should never come here
	//in case of accidental exit, gracefully exit
	vTaskDelete(Parse_Data_Handle);
}


//-------------------------------------------------------------------------
void Control_States (void *args)
{
	EventBits_t uxBits = 0;

	//wait for a short period of time for an event to occur
	//if it occurs, clear it and then enter the super loop
	//in ideal case, this should never happen
	uxBits = xEventGroupWaitBits(
            xEventsGroup  ,
			1,
            pdTRUE,
            pdFALSE,
            1);
	xEventGroupClearBits(xEventsGroup, EVT_ALL);

	for (;;)
	{
		uxBits = 0;
		uxBits = xEventGroupWaitBits(
	            xEventsGroup  ,			/* The event group being tested. */
				EVT_ALL, 				/* The bits within the event group to wait for. */
	            pdTRUE,        			/* set bit should be cleared before returning. */
	            pdFALSE,       			/* Don't wait for all bits, any bit will do. */
	            1000);					/* Wait a maximum of 1000ms for either bit to be set. */
		state_machine(uxBits);
	}

	//it should never come here
	//in case of accidental exit, gracefully exit
	vTaskDelete(Control_States_Handle);
}

//-------------------------------------------------------------------------
void state_machine(EventBits_t event)
{
	state_t current_state;
	static state_t next_state = STATE0_START;

	current_state = next_state;

	switch(current_state)
	{
		case STATE0_START:

			//critical section for global memory access
			while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
			choice_done = false;
			value_done = false;
			while(xSemaphoreGive(xMutex) != pdTRUE);

			printf("Enter 1 for frequency & 2 for Duty Cycle: ");
			fflush(stdout);
			next_state = STATE1_RX_CHOICE;
			break;

		case STATE1_RX_CHOICE:
			next_state = STATE1_RX_CHOICE;
			if (event & EVT_VALID_CHOICE)
			{
				switch(param)
				{
					case frequency:
						printf("Enter frequency in Hz: ");
						fflush(stdout);
						break;

					case duty_cycle:
						printf("Enter duty cycle in %%: ");
						fflush(stdout);
						break;

					case none:
					default:
						break;
				}
				next_state = STATE2_RX_DATA;
			}

			if (event & EVT_INVALID_DATA)
			{
				next_state = STATE0_START;
			}

			break;

		case STATE2_RX_DATA:
			next_state = STATE2_RX_DATA;
			if (event & EVT_VALID_DATA)
			{
				next_state = STATE3_CHANGE_PWM;
			}

			if (event & EVT_INVALID_DATA)
			{
				next_state = STATE0_START;
			}
			break;

		case STATE3_CHANGE_PWM:
			change_pwm_parameters(param, value);
			next_state = STATE0_START;
			break;
	}
}

//-------------------------------------------------------------------------
bool is_valid_choice(uint8_t *data)
{
	uint32_t i;

	uint8_t selec;

	for (i = 0; i < strlen((const char *)data); i++)
	{
		if (data[i] != '\r' && isdigit(data[i]) == 0)
		{
			return false;
		}
	}

	selec = atoi((const char *) data);

	switch (selec)
	{
	case frequency:
		param = frequency;
		break;

	case duty_cycle:
		param = duty_cycle;
		break;

	default:
		return false;
	}

	return true;
}


//-------------------------------------------------------------------------
bool is_valid_data(uint8_t *data)
{
	uint32_t i;

	for (i = 0; i < strlen((const char *)data); i++)
	{
		if (data[i] != '\r' && isdigit(data[i]) == 0)
		{
			return false;
		}
	}

	value = atoi((const char*)data);

	if ((param == frequency  && (value <= 0 || value > 1000000)) ||
		(param == duty_cycle && (value < 0  || value > 100)))
	{
		return false;
	}

	return true;
}

//-------------------------------------------------------------------------
void change_pwm_parameters(parameter_t parameter, uint32_t value)
{
	//de-initialize timer, prevent risk of interruption
	if (HAL_TIM_PWM_DeInit(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	if ((parameter == frequency  && (value <= 0 || value > 1000000)) ||
		(parameter == duty_cycle && (value < 0  || value > 100)))
	{
		parameter = 0;
	}

	switch(parameter)
	{
		case frequency:

			//critical section for global memory access
			while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
			time_period = 1000000/value;				//adjust frequency accordingly
			pulse_width = (time_period/100)*dcycle;		//pulse width should remain constant in %
			while(xSemaphoreGive(xMutex) != pdTRUE);

			break;

		case duty_cycle:

			//critical section for global memory access
			while(xSemaphoreTake(xMutex, (TickType_t) 10) != pdTRUE);
			pulse_width = (time_period/100)*value;
			dcycle = value;
			while(xSemaphoreGive(xMutex) != pdTRUE);

			break;

		default:
			printf("Wrong parameter!!--\n\r");
			break;
	}

	//re-initialize the timer with updated values
	MX_TIM2_Init();

	//start generating PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
//-------------------------------------------------------------------------

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{
//  /* USER CODE BEGIN 5 */
////  /* Infinite loop */
////  for(;;)
////  {
////    osDelay(1);
////  }
//  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
