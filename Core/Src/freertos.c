/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart6;

extern rc_info_t rc;
char buf[200];
 
extern pid_struct_t motor_pid2_6020; //6020旋转角度PID
extern pid_struct_t motor_pid4_6020; //6020旋转角度PID
extern pid_struct_t motor_pid1_6623; //6623旋转角度PID
extern pid_struct_t motor_pid2_6623; //6623旋转角度PID

float target_angle_yaw_6020   = 0.0f; //云台角度
float target_angle_pitch_6020 = 0.0f;
float target_angle_yaw_6623   = 200.0f; //云台角度
float target_angle_pitch_6623 = 180.0f;

extern moto_info_6623 motor_info_6623_1;
extern moto_info_6623 motor_info_6623_2;
extern moto_info_6020 motor_info_6020_4;
extern moto_info_6020 motor_info_6020_2;


extern float roll_angle_gyroscope;
extern float pitch_angle_gyroscope;
extern float yaw_angle_gyroscope;

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
osThreadId my2060Task1Handle;
osThreadId my2060Task2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start2060Task1(void const * argument);
void Start2060Task2(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of my2060Task1 */
  osThreadDef(my2060Task1, Start2060Task1, osPriorityIdle, 0, 128);
  my2060Task1Handle = osThreadCreate(osThread(my2060Task1), NULL);

  /* definition and creation of my2060Task2 */
  osThreadDef(my2060Task2, Start2060Task2, osPriorityIdle, 0, 128);
  my2060Task2Handle = osThreadCreate(osThread(my2060Task2), NULL);

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
	  sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  SW1: %1d  SW2: %1d \r\n", rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.sw1, rc.sw2);
	  HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf) - 1), 55);
	  
//	  printf("R:%4.2f  P:%4.2f  Y:%4.2f\r\n", roll_angle_gyroscope,  pitch_angle_gyroscope,  yaw_angle_gyroscope);
	  
	  osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start2060Task1 */
/**
* @brief Function implementing the my2060Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start2060Task1 */
void Start2060Task1(void const * argument)
{
  /* USER CODE BEGIN Start2060Task1 */
  /* Infinite loop */
  for(;;)
  {
	  
	  if(rc.sw1 == 1 && rc.sw2 == 1){
		  if(yaw_angle_gyroscope < 0){
			  yaw_angle_gyroscope = 360 + yaw_angle_gyroscope;
		  }
		  yaw_angle_gyroscope = yaw_angle_gyroscope>300 ? 300 : (yaw_angle_gyroscope<220 ? 220: yaw_angle_gyroscope);
		  motor_info_6623_1.set_current = pid_calc(&motor_pid1_6623, yaw_angle_gyroscope, motor_info_6623_1.rotor_angle);
		  set_motor_voltage(-motor_info_6623_1.set_current,0x0000,0x0000,0x0000);
	  }
	  
	  
	  /* 6623云台控制程序 */
//	  if(rc.sw1 == 1 && rc.sw2 == 1){
//		  
//		  target_angle_yaw_6623 += rc.ch1 * 0.0005;
//		  target_angle_yaw_6623 = target_angle_yaw_6623>300 ? 300 : (target_angle_yaw_6623<220 ? 220: target_angle_yaw_6623);
//		  motor_info_6623_1.set_current = pid_calc(&motor_pid1_6623, target_angle_yaw_6623, motor_info_6623_1.rotor_angle);
//		  
//		  target_angle_pitch_6623 += rc.ch4 *0.0005;
//		  target_angle_pitch_6623 = target_angle_pitch_6623>200 ? 200 :(target_angle_pitch_6623<160 ? 160 :target_angle_pitch_6623);
//		  motor_info_6623_2.set_current = pid_calc(&motor_pid2_6623, target_angle_pitch_6623, motor_info_6623_2.rotor_angle);
//		  
//		  //set_motor_voltage(0x0000,-motor_info_6623_2.set_current,0x0000,0x0000);
//		  set_motor_voltage(-motor_info_6623_1.set_current,-motor_info_6623_2.set_current,0x0000,0x0000);
//		  
//	  }
	  
	  osDelay(5);
  }
  /* USER CODE END Start2060Task1 */
}

/* USER CODE BEGIN Header_Start2060Task2 */
/**
* @brief Function implementing the my2060Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start2060Task2 */
void Start2060Task2(void const * argument)
{
  /* USER CODE BEGIN Start2060Task2 */
  /* Infinite loop */
  for(;;)
  {
	/* 6020云台控制程序 */  
//	  if(rc.sw1 == 1 && rc.sw2 == 1){
//		  
//		  target_angle_yaw_6020 += rc.ch1 * 0.005;
//		  target_angle_yaw_6020 = target_angle_yaw_6020>180 ? 180 : (target_angle_yaw_6020<0 ? 0 : target_angle_yaw_6020);
//		  motor_info_6020_4.set_voltage = pid_calc(&motor_rotate4_6020, target_angle_yaw_6020, motor_info_6020_4.rotor_angle);
//		  
//		  target_angle_pitch_6020 += rc.ch4 * 0.005;
//		  target_angle_pitch_6020 = target_angle_pitch_6020>250 ? 250 : (target_angle_pitch_6020<180 ? 180: target_angle_pitch_6020);
//		  motor_info_6020_2.set_voltage = pid_calc(&motor_rotate2_6020, target_angle_pitch_6020, motor_info_6020_2.rotor_angle);
//		  
//		  set_motor_voltage(0x0000,motor_info_6020_2.set_voltage,0x0000,motor_info_6020_4.set_voltage);
//		  
//	  }
	  osDelay(5);
	  
  }
  /* USER CODE END Start2060Task2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
