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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LED1Handle;
osThreadId myTask03Handle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osMessageQId myQueue04Handle;
osMessageQId myQueue05Handle;
osMessageQId myQueue06Handle;
osMessageQId myQueue07Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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
	OLED_Init();
	MPU_Init();
	mpu_dmp_init();
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
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
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 1, float);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 1, float);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

  /* definition and creation of myQueue03 */
  osMessageQDef(myQueue03, 1, float);
  myQueue03Handle = osMessageCreate(osMessageQ(myQueue03), NULL);

  /* definition and creation of myQueue04 */
  osMessageQDef(myQueue04, 1, short);
  myQueue04Handle = osMessageCreate(osMessageQ(myQueue04), NULL);

  /* definition and creation of myQueue05 */
  osMessageQDef(myQueue05, 1, short);
  myQueue05Handle = osMessageCreate(osMessageQ(myQueue05), NULL);

  /* definition and creation of myQueue06 */
  osMessageQDef(myQueue06, 1, short);
  myQueue06Handle = osMessageCreate(osMessageQ(myQueue06), NULL);

  /* definition and creation of myQueue07 */
  osMessageQDef(myQueue07, 1, int8_t);
  myQueue07Handle = osMessageCreate(osMessageQ(myQueue07), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED1 */
  osThreadDef(LED1, StartTask02, osPriorityRealtime, 0, 200);
  LED1Handle = osThreadCreate(osThread(LED1), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityBelowNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

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
	float pitch,roll,yaw; 		//欧拉角
	short gyrox,gyroy,gyroz;											 //陀螺仪原始数据
//	int16_t kk;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(myQueue01Handle, &pitch, pdMS_TO_TICKS(50));
		xQueueReceive(myQueue02Handle, &roll, pdMS_TO_TICKS(50));
		xQueueReceive(myQueue03Handle, &yaw, pdMS_TO_TICKS(50));
		xQueueReceive(myQueue04Handle, &gyrox, pdMS_TO_TICKS(50));
		xQueueReceive(myQueue05Handle, &gyrox, pdMS_TO_TICKS(50));
		xQueueReceive(myQueue06Handle, &gyroz, pdMS_TO_TICKS(50));
//		xQueueReceive(myQueue07Handle, &kk, pdMS_TO_TICKS(50));
	  
	  OLED_ShowSignedNum(2,2,pitch,4);
	  OLED_ShowSignedNum(3,2,roll,4);
	  OLED_ShowSignedNum(4,2,yaw,4);
	  OLED_ShowSignedNum(2,8,gyrox,4);
	  OLED_ShowSignedNum(3,8,gyroy,4);
	  OLED_ShowSignedNum(4,8,gyroz,4);
//	  OLED_ShowSignedNum(1,1,kk,4);
//		if(strcmp((char*)rx_buffer,"aaaa")==0){//若收到“hello”，发送“yes” 
//			HAL_UART_Transmit_DMA(&huart1, "yes", 3);	
//			OLED_ShowString(1,1,(char*)rx_buffer);
	  
//			OLED_ShowChar(1,5,rx_buffer[0]);
//			OLED_ShowChar(1,7,rx_buffer[1]);
//			OLED_ShowChar(1,9,rx_buffer[2]);
//			OLED_ShowChar(1,11,rx_buffer[3]);
//			OLED_ShowChar(1,13,rx_buffer[4]);
			rx_len = 0;//清除计数
			recv_end_flag = 0;//清除接收结束标志位
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收	
//		}
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the LED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	float pitch,roll,yaw; 		//欧拉角
	short gyrox,gyroy,gyroz;					 //陀螺仪原始数据
	int8_t Speed = 0;
	TickType_t pxPreviousWakeTime = xTaskGetTickCount();
	
	PID_TypeDef PID1 = {0};
	PID1.Kp = 318;//530//318
	PID1.Kd = 1.32;//2.2
	PID_TypeDef PID2 = {0};
	PID2.Kp = -0.2;//
	PID2.Ki = -0.001;//
	
	int16_t g_encoder_left,g_encoder_tight;
//	BaseType_t err1,err2,err3;
  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive(myQueue07Handle, &Speed, pdMS_TO_TICKS(1)) == errQUEUE_EMPTY)
			Speed = 0;
		if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
		{
			xQueueOverwrite(myQueue01Handle,&pitch);
			xQueueOverwrite(myQueue02Handle,&roll);
			xQueueOverwrite(myQueue03Handle,&yaw);
//			if((err1 || err2 || err3 ) == errQUEUE_FULL)
//				xQueueReset(myQueue01Handle);
//			vTaskDelay(500);
		}
		if(MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz) == 0)
		{
			xQueueOverwrite(myQueue04Handle,&gyrox);
			xQueueOverwrite(myQueue05Handle,&gyroy);
			xQueueOverwrite(myQueue06Handle,&gyroz);
//			if((err1 || err2 || err3 ) == errQUEUE_FULL)
//				xQueueReset(myQueue02Handle);
//			vTaskDelay(500);
		}
		Get_encoder(&g_encoder_left,&g_encoder_tight);
//			xQueueOverwrite(myQueue07Handle,&g_encoder_left);
//			xQueueOverwrite(myQueue08Handle,&gyroy);
		Set_PID_Speed(&PID2,Speed,g_encoder_left,g_encoder_tight);
		Set_PID_UP(&PID1,PID2.OUT+5,pitch,gyroy);
		Limit(&PID1.OUT);
		if(pitch > 40 | pitch < -40)
			PID1.OUT = 0;
		Load(PID1.OUT,PID1.OUT);
//			xQueueOverwrite(myQueue07Handle,&PID1.Kp);
    vTaskDelayUntil(&pxPreviousWakeTime,pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  	int8_t Speed = 0;
  /* Infinite loop */
  for(;;)
  {
//	if(rx_buffer[0] == 0x00)
//	{
//		Speed = 0;
//	}
//	if(rx_buffer[0] == 0x01)
//	{
//		Speed = 2;
//	}
//	if(rx_buffer[0] == 0x02)
//	{
//		Speed = -2;
//	}
//	xQueueOverwrite(myQueue07Handle,&Speed);
//    vTaskDelay(pdMS_TO_TICKS(100));
  }

  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

