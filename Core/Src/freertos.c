/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "adc.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct {
	
	float							controlTemperature;							//0-1024 'C

	uint8_t						endCondition;
	
	uint16_t					endTimeSecond;
	
	uint16_t					endOvenTemperature;
	
	uint16_t					endCentralTemperature;
	
} OVEN_TempHandleTypeDef;


typedef struct {
	
	uint8_t						controlMode;										//steam, spary, steam+spary
	
	uint8_t						controlHumidity;								//0-10, 0->0%, 1->10%, ..., 10->100%
	
	uint8_t						endCondition;										//end by time
	
	uint16_t					endTimeSecond;
	
} OVEN_HumiHandleTypeDef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//define endCondition
#define END_BY_TIME															00U
#define END_BY_OVEN_TEMPERATURE									01U
#define END_BY_CENTRAL_TEMPERATURE							02U


//define controlMode
#define STEAM_MODE															00U
#define SPRAY_MODE															01U
#define STEAM_SPRAY_MODE												02U

//输出口配置宏定义
#define OUTPUT_OVEN_HEATER											5			//烤箱加热丝输出口
#define OUTPUT_WATER_HEATER											7			//水箱加热丝输出口
#define OUTPUT_OVEN_SPRAY_VALVE									14		//烤箱箱体喷水阀门
#define OUTPUT_STEAM_TANK_VALVE									15		//蒸汽水箱进水阀门
#define OUTPUT_BREATHER_VALVE										16		//烤箱箱体通气阀门

//热电偶配置宏定义
#define TEMP_OVEN_MAIN													0			//烤箱温度计
#define TEMP_STEAM_TANK													1			//水箱温度计

#define WATER_TEMP															95.0	//水箱加热温度

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t input_tx_flag;
float spi_temperature[8] = {0};

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ledTaskHandle;
osThreadId usartTaskHandle;
osThreadId spiTaskHandle;
osThreadId inputTaskHandle;
osThreadId outputTaskHandle;
osThreadId adcTaskHandle;
osThreadId tempTaskHandle;
osThreadId humiTaskHandle;
osMessageQId tempQueueHandle;
osMessageQId humiQueueHandle;
osMessageQId outputQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LedTask(void const * argument);
void UsartTask(void const * argument);
void SpiTask(void const * argument);
void InputTask(void const * argument);
void OutputTask(void const * argument);
void AdcTask(void const * argument);
void TempTask(void const * argument);
void HumiTask(void const * argument);

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of tempQueue */
  osMessageQDef(tempQueue, 8, OVEN_TempHandleTypeDef);
  tempQueueHandle = osMessageCreate(osMessageQ(tempQueue), NULL);

  /* definition and creation of humiQueue */
  osMessageQDef(humiQueue, 8, OVEN_HumiHandleTypeDef);
  humiQueueHandle = osMessageCreate(osMessageQ(humiQueue), NULL);

  /* definition and creation of outputQueue */
  osMessageQDef(outputQueue, 16, uint16_t);
  outputQueueHandle = osMessageCreate(osMessageQ(outputQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of usartTask */
  osThreadDef(usartTask, UsartTask, osPriorityIdle, 0, 128);
  usartTaskHandle = osThreadCreate(osThread(usartTask), NULL);

  /* definition and creation of spiTask */
  osThreadDef(spiTask, SpiTask, osPriorityIdle, 0, 128);
  spiTaskHandle = osThreadCreate(osThread(spiTask), NULL);

  /* definition and creation of inputTask */
  osThreadDef(inputTask, InputTask, osPriorityIdle, 0, 128);
  inputTaskHandle = osThreadCreate(osThread(inputTask), NULL);

  /* definition and creation of outputTask */
  osThreadDef(outputTask, OutputTask, osPriorityIdle, 0, 128);
  outputTaskHandle = osThreadCreate(osThread(outputTask), NULL);

  /* definition and creation of adcTask */
  osThreadDef(adcTask, AdcTask, osPriorityIdle, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of tempTask */
  osThreadDef(tempTask, TempTask, osPriorityIdle, 0, 128);
  tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

  /* definition and creation of humiTask */
  osThreadDef(humiTask, HumiTask, osPriorityIdle, 0, 128);
  humiTaskHandle = osThreadCreate(osThread(humiTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void const * argument)
{
  /* USER CODE BEGIN LedTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
    osDelay(1000);
  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_UsartTask */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsartTask */
void UsartTask(void const * argument)
{
  /* USER CODE BEGIN UsartTask */
	
	OVEN_TempHandleTypeDef usart_ovenTemperatureHandle;
	OVEN_HumiHandleTypeDef usart_ovenHumidityHandle;
	
	uint8_t data_head1;
	uint8_t data_head2;
	uint8_t data_address;
	uint8_t data_length;
	uint8_t data_event;
	uint8_t data_checksum;
	uint8_t data_tail1;
	uint8_t data_tail2;
	uint8_t data_rx_err;
	
	UART_BSP_INIT();
	
  /* Infinite loop */
  for(;;)
	{
		
		//1.receive data, check data format
		
		data_checksum = 0;
		data_rx_err = 0;
		
		data_head1 = UART2_GET_DATA();
		data_head2 = UART2_GET_DATA();
		if(data_head1 != RX_DATA_HEAD1 || data_head2 != RX_DATA_HEAD2)		//head err
		{
			data_rx_err = 1;
		}
		else		//head ok
		{
			data_address = UART2_GET_DATA();
			if(data_address != RX_DATA_ADDRESS)		//address err
			{
				data_rx_err = 1;
			}
			else		//address ok
			{
				//get data length , event and detail data
				data_length = UART2_GET_DATA();
				data_event = UART2_GET_DATA();
				data_checksum += data_length;
				data_checksum += data_event;
				for(uint8_t i = 0; i < data_length; i++)
				{
					G_RX_BUFF_HANDLE[i] = UART2_GET_DATA();
					data_checksum += G_RX_BUFF_HANDLE[i];
				}
				if(data_checksum != UART2_GET_DATA())		//checksum err
				{
					data_rx_err = 1;
				}
				else		//checksum ok
				{
					data_tail1 = UART2_GET_DATA();
					data_tail2 = UART2_GET_DATA();
					if(data_tail1 != RX_DATA_TAIL1 || data_tail2 != RX_DATA_TAIL2)		//tail err
					{
						data_rx_err = 1;
					}
					else		//tail ok
					{
						HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);		//Communication complete, Toggle LED
						
						G_UART2_TXBUFFER[0] = data_event;
						for(uint8_t i = 0; i < data_length; i++)
						{
							G_UART2_TXBUFFER[0] = G_RX_BUFF_HANDLE[i];
						}
					}
				}
			}
		}
		
		//2.if data_rx_err == 0(no err), then check data_event(things to do) and get G_RX_BUFF_HANDLE(data)
		
		if(data_rx_err == 0)
		{
			switch (data_event)
			{
				
				//test gpio
				case 0x70:
				{
					HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);
				}break;
				
				//test motor
				case 0x71: 
				{
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0700);
					osDelay(5000);
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0600);
					osDelay(5000);
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0500);
					osDelay(5000);
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0400);
					osDelay(5000);
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0300);
					osDelay(5000);
					MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
					osDelay(5000);
					//speed 100-300 is same?
				}break;
				
				//start transmit input status
				case 0x72:
				{
					input_tx_flag = 1;
				}break;
				
				//stop transmit input status
				case 0x73:
				{
					input_tx_flag = 0;
				}break;
				
				//control output
				case 0x74:
				{
					uint8_t output_num = G_RX_BUFF_HANDLE[0];
					uint8_t output_status = G_RX_BUFF_HANDLE[1];
					if(output_status == 1)
					{
						GPIO_OUTPUT_OPEN(output_num);
					}
					else if(output_status == 0)
					{
						GPIO_OUTPUT_CLOSE(output_num);
					}
				}break;
				
				//tempeature control
				case 0x75:
				{
					
					uint16_t tempeature = G_RX_BUFF_HANDLE[0] << 8 | G_RX_BUFF_HANDLE[1];
					usart_ovenTemperatureHandle.controlTemperature = ( float ) tempeature;
					usart_ovenTemperatureHandle.endCondition = G_RX_BUFF_HANDLE[2];
					switch (usart_ovenTemperatureHandle.endCondition)
					{
						case END_BY_TIME:
						{
							usart_ovenTemperatureHandle.endTimeSecond = G_RX_BUFF_HANDLE[3] << 8 | G_RX_BUFF_HANDLE[4];
						}break;
						case END_BY_OVEN_TEMPERATURE:
						{
							usart_ovenTemperatureHandle.endOvenTemperature = G_RX_BUFF_HANDLE[3] << 8 | G_RX_BUFF_HANDLE[4];
						}break;
						case END_BY_CENTRAL_TEMPERATURE:
						{
							usart_ovenTemperatureHandle.endCentralTemperature = G_RX_BUFF_HANDLE[3] << 8 | G_RX_BUFF_HANDLE[4];
						}break;
					}
					
					xQueueSend(tempQueueHandle, ( void * ) &usart_ovenTemperatureHandle, ( TickType_t ) 0);
					printf("tempeature queue send finish.\n");
					
				}break;
				
				//humidity control
				case 0x76:
				{
					
					usart_ovenHumidityHandle.controlMode = G_RX_BUFF_HANDLE[0];
					usart_ovenHumidityHandle.controlHumidity = G_RX_BUFF_HANDLE[1];
					usart_ovenHumidityHandle.endCondition = G_RX_BUFF_HANDLE[2];
					switch (usart_ovenHumidityHandle.endCondition)
					{
						case END_BY_TIME:
						{
							usart_ovenHumidityHandle.endTimeSecond = G_RX_BUFF_HANDLE[3] << 8 | G_RX_BUFF_HANDLE[4];
						}break;
					}
					
					xQueueSend(humiQueueHandle, ( void * ) &usart_ovenHumidityHandle, ( TickType_t ) 0);
					printf("humidity queue send finish.\n");
					
				}break;
				
				//end control
				case 0x77:
				{
					osDelay(200);
					printf("end, open motor and air valve.\n");
					GPIO_OUTPUT_OPEN(OUTPUT_BREATHER_VALVE);
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0600);
					osDelay(30000);
					GPIO_OUTPUT_CLOSE(OUTPUT_BREATHER_VALVE);
					MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
					printf("end.\n");
				}break;
				
				//suspend all
				case 0x78:
				{
					printf("now suspend output task.\n");
					GPIO_OUTPUT_CLOSEALL();
					MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
					vTaskSuspend( tempTaskHandle );
					vTaskSuspend( humiTaskHandle );
				}break;
				
				//resume all
				case 0x79:
				{
					printf("now resume output task.\n");
					vTaskResume( tempTaskHandle );
					vTaskResume( humiTaskHandle );
				}break;
				
				//open check
				case 0x80:
				{
					printf("now check open.\n");
					if(spi_temperature[0] == (float) 4096.0)
					{
						printf("cannot find T1, please check\n");
					}
					if(spi_temperature[1] == (float) 4096.0)
					{
						printf("cannot find T2, please check\n");
					}
					//打开进水阀
					GPIO_OUTPUT_OPEN(OUTPUT_STEAM_TANK_VALVE);
					while(1)
					{
						if(!GPIO_INPUT(1))
						{
							osDelay(10);
							if(!GPIO_INPUT(1))
							{
								GPIO_OUTPUT_CLOSE(OUTPUT_STEAM_TANK_VALVE);
								break;
							}
						}
						osDelay(50);
					}
				}break;
				
			}
		}
		
		osDelay(10);
  }
  /* USER CODE END UsartTask */
}

/* USER CODE BEGIN Header_SpiTask */
/**
* @brief Function implementing the spiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SpiTask */
void SpiTask(void const * argument)
{
  /* USER CODE BEGIN SpiTask */
	
	uint16_t max6675_data[8] = {0};
	
  /* Infinite loop */
  for(;;)
  {
		for(uint8_t i = 0; i < 8; i++)
		{
			max6675_data[i] = SPI_MAX6675(i+1);
			if(max6675_data[i] & 0x04)	//检测不到热电偶
			{
				spi_temperature[i] = (float) 4096.0;
			}
			else
			{
				spi_temperature[i] = (float) (((max6675_data[i] >> 3) & ~(0xF << 12)) * 0.25f);
			}
			osDelay(50);
		}
		if(input_tx_flag == 1)
		{
			printf("spi_temperature:%f,  %f,  %f\n", spi_temperature[0], spi_temperature[1], spi_temperature[2]);
		}
    osDelay(100);
  }
  /* USER CODE END SpiTask */
}

/* USER CODE BEGIN Header_InputTask */
/**
* @brief Function implementing the inputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputTask */
void InputTask(void const * argument)
{
  /* USER CODE BEGIN InputTask */
	
	uint8_t input_data[20] = {0};
	
  /* Infinite loop */
  for(;;)
  {
		for(uint8_t i = 0; i < 20; i++)
		{
			input_data[i] = GPIO_INPUT(i+1);
		}
		// if(input_tx_flag == 1)
		// {
		// 	printf("gpio: %u%u%u%u%u %u%u%u%u%u %u%u%u%u%u %u%u%u%u%u\n", input_data[0], input_data[1], input_data[2], input_data[3], input_data[4],
		// 																																input_data[5], input_data[6], input_data[7], input_data[8], input_data[9],
		// 																																input_data[10],input_data[11],input_data[12],input_data[13],input_data[14],
		// 																																input_data[15],input_data[16],input_data[17],input_data[18],input_data[19]);
		// }
    osDelay(1000);
  }
  /* USER CODE END InputTask */
}

/* USER CODE BEGIN Header_OutputTask */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputTask */
void OutputTask(void const * argument)
{
  /* USER CODE BEGIN OutputTask */
	uint8_t status = 0;
  /* Infinite loop */
  for(;;)
  {
		if(!GPIO_INPUT(2) && status == 0)
		{
			printf("now suspend output task.\n");
			GPIO_OUTPUT_CLOSEALL();
			MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
			vTaskSuspend( tempTaskHandle );
			vTaskSuspend( humiTaskHandle );
			status = 1;
		}
		else if(GPIO_INPUT(2) && status == 1)
		{
			printf("now resume output task.\n");
			vTaskResume( tempTaskHandle );
			vTaskResume( humiTaskHandle );
			status = 0;
		}
		
		osDelay(100);
  }
  /* USER CODE END OutputTask */
}

/* USER CODE BEGIN Header_AdcTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AdcTask */
void AdcTask(void const * argument)
{
  /* USER CODE BEGIN AdcTask */
	
	uint32_t adc_data[3];
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 6) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AdcTask */
}

/* USER CODE BEGIN Header_TempTask */
/**
* @brief Function implementing the tempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TempTask */
void TempTask(void const * argument)
{
  /* USER CODE BEGIN TempTask */
	
	uint8_t temp_control_queue_flag = 0;
	uint8_t temp_end_flag = 0;
	uint32_t temp_time_counts;
	uint8_t temp_motor_counts;
	uint16_t temp_motor_speed;
	uint8_t temp_motor_status;
	uint8_t moshi;
	
	OVEN_TempHandleTypeDef temp_ovenTemperatureHandle;
	
  /* Infinite loop */
  for(;;)
  {
		//接收队列消息
		if(tempQueueHandle != NULL)
		{
      if( xQueueReceive( tempQueueHandle,
                         &( temp_ovenTemperatureHandle ),
                         ( TickType_t ) 10 ) == pdPASS )
      {
				temp_control_queue_flag = 1;
      }
		}
		
		//若接收到了队列消息，则开始控制
		if(temp_control_queue_flag == 1)
		{
			temp_time_counts = 0;
			temp_motor_status = 0;
			temp_motor_speed = 0x0700;
			temp_motor_counts = 0;
			switch (temp_ovenTemperatureHandle.endCondition)
			{
				case END_BY_TIME:
				{
					osDelay(200);
					printf("Control temperatrue is: %f\n", temp_ovenTemperatureHandle.controlTemperature);
					osDelay(200);
					printf("End by time,start controling...\n");
					osDelay(200);
					for(temp_time_counts = 0; temp_time_counts < temp_ovenTemperatureHandle.endTimeSecond; temp_time_counts++)
					{
						//temperature control
						if(spi_temperature[TEMP_OVEN_MAIN] < temp_ovenTemperatureHandle.controlTemperature)
						{
							GPIO_OUTPUT_OPEN(OUTPUT_OVEN_HEATER);
						}
						else
						{
							GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_HEATER);
						}
						printf("Current temperature is: %.2f, %.2f, %.2f.\n", spi_temperature[TEMP_OVEN_MAIN], spi_temperature[TEMP_STEAM_TANK], spi_temperature[2]);
						//motor control
						
						//1.motor speed
						//风机控制模式，1代表蒸的时候风机减速的模式
						//2代表风机正常烤的时候不减速的模式
						moshi = 1;
						temp_motor_speed = 0x0600;
						
						if(moshi == 1)
						{
							if((temp_time_counts % 615) < 255)
							{
								temp_motor_speed = 0x600;
							}
							if(255 <= (temp_time_counts % 615) && (temp_time_counts % 615) < 315)
							{
								temp_motor_speed = 0x600;
							}
							else if(315 <= (temp_time_counts % 615) && (temp_time_counts % 615) < 375)
							{
								temp_motor_speed = 0x500;
							}
							else if(375 <= (temp_time_counts % 615) && (temp_time_counts % 615) < 435)
							{
								temp_motor_speed = 0x400;
							}
							else if(435 <= (temp_time_counts % 615) && (temp_time_counts % 615) < 495)
							{
								temp_motor_speed = 0x300;
							}
							else if(495 <= (temp_time_counts % 615) && (temp_time_counts % 615) < 615)
							{
								temp_motor_speed = 0x200;
							}
						}
						else if(moshi == 2)
						{
								temp_motor_speed = 0x600;
						}
						
						//2.motor direction
						if(temp_motor_status == 0)
						{
							if(temp_motor_counts < 112)
							{
								MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, temp_motor_speed);
								temp_motor_counts++;
							}
							else
							{
								MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, temp_motor_speed);
								temp_motor_status = 1;
								temp_motor_counts = 0;
							}
						}
						else if(temp_motor_status == 1)
						{
							if(temp_motor_counts < 6)
							{
								MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CCW, 0x0100);
								temp_motor_counts++;
							}
							else
							{
								MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CCW, 0x0100);
								temp_motor_status = 2;
								temp_motor_counts = 0;
							}
						}
						else if(temp_motor_status == 2)
						{
							if(temp_motor_counts < 112)
							{
								MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CCW, temp_motor_speed);
								temp_motor_counts++;
							}
							else
							{
								MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CCW, temp_motor_speed);
								temp_motor_status = 3;
								temp_motor_counts = 0;
							}
						}
						else if(temp_motor_status == 3)
						{
							if(temp_motor_counts < 6)
							{
								MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
								temp_motor_counts++;
							}
							else
							{
								MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
								temp_motor_status = 0;
								temp_motor_counts = 0;
							}
						}
						osDelay(1000);
					}
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_HEATER);
					MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
					temp_control_queue_flag = 0;
					printf("Temperature control end.\n");
				}break;
				
				//预热
				case END_BY_OVEN_TEMPERATURE:
				{
					temp_end_flag = 0;
					osDelay(200);
					printf("Control temperatrue is: %f\n", temp_ovenTemperatureHandle.controlTemperature);
					osDelay(200);
					printf("End by oven teamperature,start controling...\n");
					osDelay(200);
					//打开风机
					MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0700);
					osDelay(3000);
					//喷水
					GPIO_OUTPUT_OPEN(OUTPUT_OVEN_SPRAY_VALVE);
					osDelay(1000);
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
					osDelay(1000);
					//开始加热烤箱
					while(temp_end_flag == 0)
					{
						MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0700);
						if(spi_temperature[TEMP_OVEN_MAIN] >= temp_ovenTemperatureHandle.controlTemperature - 30.0)
						{
							osDelay(50);
							printf("1Current temperature is: %f.\n", spi_temperature[TEMP_OVEN_MAIN]);
							GPIO_OUTPUT_OPEN(OUTPUT_OVEN_HEATER);
							osDelay(950);
							if(spi_temperature[TEMP_OVEN_MAIN] >= temp_ovenTemperatureHandle.controlTemperature - 30.0)
							{
								osDelay(50);
								printf("2Current temperature is: %f.\n", spi_temperature[TEMP_OVEN_MAIN]);
								GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_HEATER);
								temp_end_flag = 1;
							}
						}
						else
						{
							osDelay(50);
							printf("Current temperature is: %f.\n", spi_temperature[TEMP_OVEN_MAIN]);
							GPIO_OUTPUT_OPEN(OUTPUT_OVEN_HEATER);
							osDelay(950);
						}
					}
					osDelay(50);
					printf("Temperature control end.\n");
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_HEATER);
					//开始加热水箱
					osDelay(200);
					printf("Preheat water,start controling...\n");
					osDelay(200);
					while(spi_temperature[TEMP_STEAM_TANK] < WATER_TEMP)
					{
						printf("Current water temperature is: %f.\n", spi_temperature[TEMP_STEAM_TANK]);
						MOTOR_CONTROL(MOTOR_ENABLE, MOTOR_DIR_CW, 0x0700);
						GPIO_OUTPUT_OPEN(OUTPUT_WATER_HEATER);
						osDelay(1000);
					}
					osDelay(200);
					printf("Preheat water control end.\n");
					
					//预热结束
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_HEATER);
					GPIO_OUTPUT_CLOSE(OUTPUT_WATER_HEATER);
					MOTOR_CONTROL(MOTOR_DISABLE, MOTOR_DIR_CW, 0x0100);
					temp_control_queue_flag = 0;
				}break;
				
			}
			
		}
    osDelay(1);
  }
  /* USER CODE END TempTask */
}

/* USER CODE BEGIN Header_HumiTask */
/**
* @brief Function implementing the humiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HumiTask */
void HumiTask(void const * argument)
{
  /* USER CODE BEGIN HumiTask */
	
	uint32_t humi_time_counts;
	uint8_t humi_control_queue_flag = 0;
	uint8_t humi_pwm_count = 0;
	OVEN_HumiHandleTypeDef humi_ovenHumidityHandle;
	
  /* Infinite loop */
  for(;;)
  {
		
		if(humiQueueHandle != NULL)
		{
      if( xQueueReceive( humiQueueHandle,
                         &( humi_ovenHumidityHandle ),
                         ( TickType_t ) 10 ) == pdPASS )
      {
				humi_control_queue_flag = 1;
      }
		}

		if(humi_control_queue_flag == 1)
		{
			humi_time_counts = 0;
			
			switch (humi_ovenHumidityHandle.controlMode)
			{
				
				case STEAM_MODE:
				{
					osDelay(200);
					printf("steam mode,start controling...\n");
					osDelay(200);
					for(humi_time_counts = 0; humi_time_counts < humi_ovenHumidityHandle.endTimeSecond; humi_time_counts++)
					{
						if(spi_temperature[TEMP_STEAM_TANK] < WATER_TEMP)
						{
							GPIO_OUTPUT_OPEN(OUTPUT_WATER_HEATER);
						}
						else
						{
							GPIO_OUTPUT_CLOSE(OUTPUT_WATER_HEATER);
						}
						printf("Current steam temperature is: %f.\n", spi_temperature[TEMP_STEAM_TANK]);
						osDelay(1000);
					}
					printf("Humidity control end.\n");
					GPIO_OUTPUT_CLOSE(OUTPUT_WATER_HEATER);
					humi_control_queue_flag = 0;
				}break;
				
				case SPRAY_MODE:
				{
					osDelay(200);
					printf("Control humidity is: %d\n", humi_ovenHumidityHandle.controlHumidity);
					osDelay(200);
					printf("spray mode,start controling...\n");
					osDelay(200);
					humi_pwm_count = 20 - humi_ovenHumidityHandle.controlHumidity;
					if(humi_pwm_count == 0) humi_pwm_count = 1;
					for(humi_time_counts = 0; humi_time_counts < humi_ovenHumidityHandle.endTimeSecond; humi_time_counts++)
					{
						//若湿度为10（100%），则每隔10秒喷0.75秒水；湿度为1（10%），则每隔19秒喷0.75秒水
						if(humi_time_counts % humi_pwm_count == 0)
						{
							GPIO_OUTPUT_OPEN(OUTPUT_OVEN_SPRAY_VALVE);
							osDelay(750);
							GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
							osDelay(250);
						}
						else
						{
							GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
							osDelay(1000);
						}
					}
					printf("Humidity control end.\n");
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
					humi_control_queue_flag = 0;
				}break;
				
				case STEAM_SPRAY_MODE:
				{
					osDelay(200);
					printf("Control humidity is: %d\n", humi_ovenHumidityHandle.controlHumidity);
					osDelay(200);
					printf("steam spray mode,start controling...\n");
					osDelay(200);
					humi_control_queue_flag = 0;
					for(humi_time_counts = 0; humi_time_counts < humi_ovenHumidityHandle.endTimeSecond; humi_time_counts++)
					{
						//加热蒸汽水箱
						if(spi_temperature[TEMP_STEAM_TANK] < WATER_TEMP)
						{
							GPIO_OUTPUT_OPEN(OUTPUT_WATER_HEATER);
						}
						else
						{
							GPIO_OUTPUT_CLOSE(OUTPUT_WATER_HEATER);
						}
						printf("Current steam temperature is: %f.\n", spi_temperature[TEMP_STEAM_TANK]);
						
						//烤箱内喷水
						humi_pwm_count = 0;
						//若湿度为10（100%），则每隔10秒喷0.5秒水；湿度为1（10%），则每隔19秒喷0.5秒水
						if(humi_pwm_count < 20 - humi_ovenHumidityHandle.controlHumidity)
						{
							GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
							humi_pwm_count ++;
							osDelay(1000);
						}
						else
						{
							GPIO_OUTPUT_OPEN(OUTPUT_OVEN_SPRAY_VALVE);
							osDelay(500);
							GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
							osDelay(500);
							humi_pwm_count = 0;
						}
					}
					printf("Humidity control end.\n");
					GPIO_OUTPUT_CLOSE(OUTPUT_WATER_HEATER);
					GPIO_OUTPUT_CLOSE(OUTPUT_OVEN_SPRAY_VALVE);
					humi_control_queue_flag = 0;
				}break;
			}
		}
    osDelay(1);
  }
  /* USER CODE END HumiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
