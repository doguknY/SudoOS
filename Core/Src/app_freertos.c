/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
#include "control.h"
#include "checking.h"
#include "usart.h"
#include "adc.h"
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t stringWithAllData[169];
float_to_u8 converter32;
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
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for runCommand */
osThreadId_t runCommandHandle;
const osThreadAttr_t runCommand_attributes = {
  .name = "runCommand",
  .priority = (osPriority_t) osPriorityRealtime1,
  .stack_size = 256 * 4
};
/* Definitions for sendToGround */
osThreadId_t sendToGroundHandle;
const osThreadAttr_t sendToGround_attributes = {
  .name = "sendToGround",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for checkState */
osThreadId_t checkStateHandle;
const osThreadAttr_t checkState_attributes = {
  .name = "checkState",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for someLoop */
osThreadId_t someLoopHandle;
const osThreadAttr_t someLoop_attributes = {
  .name = "someLoop",
  .priority = (osPriority_t) osPriorityRealtime5,
  .stack_size = 512 * 4
};
/* Definitions for sd_IO */
osThreadId_t sd_IOHandle;
const osThreadAttr_t sd_IO_attributes = {
  .name = "sd_IO",
  .priority = (osPriority_t) osPriorityRealtime6,
  .stack_size = 1024 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void motorControlEntry(void *argument);
void runCommandEntry(void *argument);
void sendToGroundEntry(void *argument);
void checkStateEntry(void *argument);
void someLoopEntry(void *argument);
void sd_IOEntry(void *argument);

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
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorControl */
  motorControlHandle = osThreadNew(motorControlEntry, NULL, &motorControl_attributes);

  /* creation of runCommand */
  runCommandHandle = osThreadNew(runCommandEntry, NULL, &runCommand_attributes);

  /* creation of sendToGround */
  sendToGroundHandle = osThreadNew(sendToGroundEntry, NULL, &sendToGround_attributes);

  /* creation of checkState */
  checkStateHandle = osThreadNew(checkStateEntry, NULL, &checkState_attributes);

  /* creation of someLoop */
  someLoopHandle = osThreadNew(someLoopEntry, NULL, &someLoop_attributes);

  /* creation of sd_IO */
  sd_IOHandle = osThreadNew(sd_IOEntry, NULL, &sd_IO_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_motorControlEntry */
/**
 * @brief  Function implementing the motorControl thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_motorControlEntry */
void motorControlEntry(void *argument)
{
  /* USER CODE BEGIN motorControlEntry */

  /* Infinite loop */
  for (;;)
  {

    switch (flightState){

      case DEBUG_PROFILE_1:
        motor_control_debug();
        break;

      case START:
        // motor_control_debug();
        break;

      case AFTER_SEPARATION:
        motor_control(1, -7, -1);
        break;

      case STEADY:
        motor_control(1, -7, -1);
        break;

      case AFTER_200M:
        motor_control(1, -5, -1);
        break;

      case AFTER_2M:
        motor_control(1, -1, -1);
        break;
        
      case LANDED:
        motor_control(-1, 0, -1);
        break;

      default:
        // rip();
        break;
    }
    osDelay(1);
  }
  /* USER CODE END motorControlEntry */
}

/* USER CODE BEGIN Header_runCommandEntry */
/**
 * @brief Function implementing the runCommand thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runCommandEntry */
void runCommandEntry(void *argument)
{
  /* USER CODE BEGIN runCommandEntry */
  /* Infinite loop */
  for (;;)
  {
    /* Read data from buffer that came frome ground*/

    switch (loraReceiveBuffer[0]){
      case  BUZZER_ON:
        buzzer(1);
        break;

      case BUZZER_OFF:
        buzzer(0);
        break;

      default:
        break;
    }

    switch (loraReceiveBuffer[1]) {

      case SEPARATE:
        servo_separate();
        break;

      case REVERT:
        servo_revert();
        break;

      default:
        break;

    }

    switch (loraReceiveBuffer[2]) {

      case MOTOR_STOP:
        controlDebugState = STOP;

        pid_roll.Kp = 0;
        pid_roll.Ki = 0;
        pid_roll.Kd = 0;

        pid_pitch.Kp = 0;
        pid_pitch.Ki = 0;
        pid_pitch.Kd = 0;

        pid_roll.integrator = 0;
        pid_roll.prevError = 0;
        pid_roll.differentiator = 0;
        pid_roll.prevMeasurement = 0;
        pid_roll.out = 0;

        pid_pitch.integrator = 0;
        pid_pitch.prevError = 0;
        pid_pitch.differentiator = 0;
        pid_pitch.prevMeasurement = 0;
        pid_pitch.out = 0;

        break;

      case MOTOR_START_TEST:
        controlDebugState = START_TEST;
        break;

      case MOTOR_PID_TEST:
      
        controlDebugState = PID_TEST;

        switch (loraReceiveBuffer[3])
        {
        case 1:

          converter32.u8[0] = loraReceiveBuffer[4];
          converter32.u8[1] = loraReceiveBuffer[5];
          converter32.u8[2] = loraReceiveBuffer[6];
          converter32.u8[3] = loraReceiveBuffer[7];

          if (converter32.u32 > 0) {
            pid_roll.Kp = converter32.u32;
            pid_pitch.Kp = converter32.u32;
          }
          

          break;
        
        case 2:
          
          converter32.u8[0] = loraReceiveBuffer[4];
          converter32.u8[1] = loraReceiveBuffer[5];
          converter32.u8[2] = loraReceiveBuffer[6];
          converter32.u8[3] = loraReceiveBuffer[7];

          if (converter32.u32 > 0) {
            pid_roll.Ki = converter32.u32;
            pid_pitch.Ki = converter32.u32;
          }

          break;

        case 3:
          
          converter32.u8[0] = loraReceiveBuffer[4];
          converter32.u8[1] = loraReceiveBuffer[5];
          converter32.u8[2] = loraReceiveBuffer[6];
          converter32.u8[3] = loraReceiveBuffer[7];

          if (converter32.u32 > 0) {
            pid_roll.Kd = converter32.u32;
            pid_pitch.Kd = converter32.u32;
          }

          break;

        case 4:

          switch (loraReceiveBuffer[4])
          {

          case 1:
            if (loraReceiveBuffer[5] > 0) set_point_pitch = loraReceiveBuffer[5];
            break;

          case 2:
            if (loraReceiveBuffer[5] > 0) set_point_roll = loraReceiveBuffer[5];
            break;
          }

          break;

        }
        break;

      case MOTOR_QR_TEST:
        controlDebugState = QR_TEST;
        break;

      case MOTOR_INPUT_TEST:
        controlDebugState = INPUT_TEST;
        receivedDuty = loraReceiveBuffer[3];
        if (receivedDuty > 0) inputDuty = receivedDuty;  
        break;

      default:
        break;

    }

    osDelay(100);
  }
  /* USER CODE END runCommandEntry */
}

/* USER CODE BEGIN Header_sendToGroundEntry */
/**
 * @brief Function implementing the sendToGround thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sendToGroundEntry */
void sendToGroundEntry(void *argument)
{
  /* USER CODE BEGIN sendToGroundEntry */
  initSD();

  /* Infinite loop */
  for (;;)
  {
    
    loraRecevice();
    create_packet2();
    writeSD(telemetryStrPacket);

    loraTransmit(groundTransmitPacket, sizeof(groundTransmitPacket));

    osDelay(800);

  }
  /* USER CODE END sendToGroundEntry */
}

/* USER CODE BEGIN Header_checkStateEntry */
/**
 * @brief Function implementing the checkState thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_checkStateEntry */
void checkStateEntry(void *argument)
{
  /* USER CODE BEGIN checkStateEntry */
  /* Infinite loop */
  for (;;)
  {
  switch (flightState){

    case START:
      if (checkLiftoff() == 1)
      {
        time.liftoffTime = time.current;
        flightState = AFTER_LIFTOFF;
      }
      break;

    case AFTER_LIFTOFF:
      if (checkApogee() == 1)
      {
        time.apogeeTime = time.current;
        flightState = AFTER_APOGEE;
      }
      break;

    case AFTER_APOGEE:
      if (checkSeparationAltitude() == 1)
      {
        servo_separate();
        osDelay(WAIT_AFTER_SEPARATION_TIME_SECONDS * 1000);
        flightState = AFTER_SEPARATION;
      }
      break;
    
    case AFTER_SEPARATION:
      if (checkSteadyAltitude() == 1)
      {
        flightState = STEADY;
      }
      break;
    
    case STEADY:
      osDelay(STEADY_WAITING_TIME_SECONDS * 1000);
      flightState = AFTER_200M;
      break;

    case AFTER_200M:
      if (checkBeforeLanding() == 1)
      {
        flightState = AFTER_2M;
      }
      break;
    
    case AFTER_2M:
      if (checkLanding() == 1)
      {
        flightState = LANDED;
      }
      break;

    case LANDED:
      if (time.landingTime == 0.0f){
        time.landingTime = time.current;
      }
      osDelay(10000); /* TODO: Something for saving power*/
      break;

    default:
      // DEBUG_PROFILE_X
      break;
  }

  osDelay(2);

  }
  /* USER CODE END checkStateEntry */
}

/* USER CODE BEGIN Header_someLoopEntry */
/**
 * @brief Function implementing the someLoop thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_someLoopEntry */
void someLoopEntry(void *argument)
{
  /* USER CODE BEGIN someLoopEntry */
  /* Infinite loop */
  for (;;)
  {

    HAL_UART_Receive_IT(&huart1, &received_uart_byte, 1);
    HAL_UART_Receive_IT(&huart2, &received_uart_byte, 1);

    led(1);
    osDelay(500);
    led(0);
    osDelay(500);
  }
  /* USER CODE END someLoopEntry */
}

/* USER CODE BEGIN Header_sd_IOEntry */
/**
* @brief Function implementing the sd_IO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sd_IOEntry */
void sd_IOEntry(void *argument)
{
  /* USER CODE BEGIN sd_IOEntry */
  // initSD();
   
  /* Infinite loop */
  for(;;)
  {
    // writeSD(telemetryStrPacket);
    osDelay(1);
  }
  /* USER CODE END sd_IOEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

