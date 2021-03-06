/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "cpustate.h"
#include "BSP.h"
#include "module_mpu9250.h"
#include "MPU9250.h"
#include "tim.h"
#include "quad_math.h"
#include "filter.h"
#include "usart.h"
#include "afei_bluetooth.h"
#include "adc.h"
#include "bmp280.h"
#include "control.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId ledTaskHandle;
osThreadId mathTaskHandle;
osThreadId obstacleTaskHandle;
osThreadId osStatTaskHandle;
osThreadId uartTaskHandle;
osThreadId startupTaskHandle;
osThreadId controlTaskHandle;

/* USER CODE BEGIN Variables */
u32 MathTaskCounter;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StateLEDTask(void const * argument);
void StartMathTask(void const * argument);
void StartObstacleTask(void const * argument);
void OSStatTask(void const * argument);
void UartTask(void const * argument);
void StartupTask(void const * argument);
void ControlTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */

    /* This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    taskENTER_CRITICAL(); //关中断
//  if( FreeHeapSpace > 100 )
//  {
//      /* By now, the kernel has allocated everything it is going to, so
//      if there is a lot of heap remaining unallocated then
//      the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
//      reduced accordingly. */
//  }
//
    OSIdleCtr++;//空闲计数器加一

    taskEXIT_CRITICAL();                     //开中断
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
    while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
    while(1);
}
/* USER CODE END 5 */

/* Init FreeRTOS */

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

  /* Create the thread(s) */
  /* definition and creation of ledTask */
  osThreadDef(ledTask, StateLEDTask, osPriorityBelowNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of mathTask */
  osThreadDef(mathTask, StartMathTask, osPriorityRealtime, 0, 2048);
  mathTaskHandle = osThreadCreate(osThread(mathTask), NULL);

  /* definition and creation of obstacleTask */
  osThreadDef(obstacleTask, StartObstacleTask, osPriorityAboveNormal, 0, 256);
  obstacleTaskHandle = osThreadCreate(osThread(obstacleTask), NULL);

  /* definition and creation of osStatTask */
  osThreadDef(osStatTask, OSStatTask, osPriorityLow, 0, 128);
  osStatTaskHandle = osThreadCreate(osThread(osStatTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, UartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of startupTask */
  osThreadDef(startupTask, StartupTask, osPriorityRealtime, 0, 256);
  startupTaskHandle = osThreadCreate(osThread(startupTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, ControlTask, osPriorityHigh, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StateLEDTask function */
void StateLEDTask(void const * argument)
{

  /* USER CODE BEGIN StateLEDTask */
  s32 LEDPeriod=0;
  u32 LEDState=1;
  u32 LEDResumeTime=0;
  u32 raisetime =   5;
  u32 BatteryLedStateCunter=0;
  u32 droptime  =   15;
  HAL_GPIO_WritePin(BatLED_GPIO,BatLED_PIN,GPIO_PIN_SET);    
  /* Infinite loop */
  for(;;)
  {
    if (BatteryPercentage<15)/*电量不足*/
    {
        if(BatteryLedStateCunter<HAL_GetTick())
        {
            HAL_GPIO_TogglePin(BatLED_GPIO,BatLED_PIN);
            BatteryLedStateCunter+=300;
        }        
    }
    else if (BatteryPercentage>=99)/*电量充足*/
    {
        HAL_GPIO_WritePin(BatLED_GPIO,BatLED_PIN,GPIO_PIN_RESET);
    }
    else
        HAL_GPIO_WritePin(BatLED_GPIO,BatLED_PIN,GPIO_PIN_SET); 
    if(FlyMode == STANDBY)/*待机呼吸灯*/
    {
        if(LEDResumeTime<HAL_GetTick())
        {
            if (LEDState)
            {
                if(++LEDPeriod>80)
                    LEDState=0;
                osDelay(raisetime); 
            }
            else
            {
                if(--LEDPeriod==0)
                    LEDState=1;
                osDelay(droptime); 
            }
            LEDPWM(0,LEDPeriod);
            LEDPWM(1,LEDPeriod);
            LEDPWM(2,LEDPeriod);
            LEDPWM(3,LEDPeriod);
            if(LEDPeriod==0)
            {
                LEDResumeTime=HAL_GetTick()+2000; 
         //       vTaskDelete(NULL);
            }
        }
        else
        {
            osDelay(100);
        }
    }
    else
    {
        if(LEDResumeTime<HAL_GetTick())
        {
            LEDPWM(0,255);
            LEDPWM(1,255);
            LEDPWM(2,255);
            LEDPWM(3,255);
            osDelay(50);
            LEDPWM(0,0);
            LEDPWM(1,0);
            LEDPWM(2,0);
            LEDPWM(3,0);
            osDelay(50);
            LEDPWM(0,255);
            LEDPWM(1,255);
            LEDPWM(2,255);
            LEDPWM(3,255);
            osDelay(50);
            LEDPWM(0,0);
            LEDPWM(1,0);
            LEDPWM(2,0);
            LEDPWM(3,0);
            LEDResumeTime=HAL_GetTick()+2000; 
        }   
            
    }
  }
  /* USER CODE END StateLEDTask */
}

/* StartMathTask function */
void StartMathTask(void const * argument)
{
  /* USER CODE BEGIN StartMathTask */
    TickType_t xLastWakeTime;
    //Gyro_SPIAdj();/*陀螺仪初始化校准*/
   
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount ();
    
  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil( &xLastWakeTime, Math_PERIOD);
    MathTaskCounter++;
    IMUSO3Thread();/*姿态解算666*/
    BMP280_Get_Height();
    Control();
  }
  /* USER CODE END StartMathTask */
}

/* StartObstacleTask function */
void StartObstacleTask(void const * argument)
{
  /* USER CODE BEGIN StartObstacleTask */
   TickType_t xLastWakeTime;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount ();

  /* Infinite loop */
  for(;;)
  {
      vTaskDelayUntil( &xLastWakeTime, 40);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);/*LED发射管打开*/
      osDelay(10);
      ADC_Callback1(&hadc1); 
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);/*LED发射管关闭省电*/
      osDelay(25);
      ADC_Callback2(&hadc1); 
      
  }
  /* USER CODE END StartObstacleTask */
}

/* OSStatTask function */
void OSStatTask(void const * argument)
{
  /* USER CODE BEGIN OSStatTask */
  /* Infinite loop */
  for(;;)
  {
    OS_TaskStat();
       
  }
  /* USER CODE END OSStatTask */
}

/* UartTask function */
void UartTask(void const * argument)
{
  /* USER CODE BEGIN UartTask */
    //u32 baud=9600;
    u32 usart_counter;
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
    usart_counter++;
     
//      switch (usart_counter)
//      {
//          case 0:
//              break;
//          case 1:
//              Send_Data_Old(imu.roll, Send_Roll);
//              break;
//          case 2:
//              Send_Data_Old(imu.pitch, Send_Pitch);
//              break;
//          case 3:
//              Send_Data_Old(imu.yaw, Send_Yaw);
//              break;
//          case 4:
//              Send_Data_Old(BatteryPercentage, Send_Number1);
//              break;
//          case 5:
//              Send_Data_Old(BatteryVoltage, Send_Number2);
//              break;
//          case 6:
//              Send_Data_Old(MotorSpeed[0], Send_Number3);
//              break;
//          case 7:
//              Send_Data_Old(MotorSpeed[1], Send_Number4);
//              break;
//          case 8:
//              Send_Data_Old(MotorSpeed[2], Send_Number5);
//              break;
//          case 9:
//              Send_Data_Old(MotorSpeed[3], Send_Number6);
//              break;
//          default:
//              usart_counter=0;
//      }
    //Send_Data(Roll , Send_Roll);
    //Send_Data(Yaw , Send_Yaw);
    /*for 4.0双模*/
//      const u8 BLUETOOTH_SET_BAUD_CODE[8]="AT+BAUD6";
//      const u8 BLUETOOTH_SET_PINE_CODE[11]="AT+PINE9527";
//      const u8 BLUETOOTH_SET_PINB_CODE[11]="AT+PINB9527";
//      const u8 BLUETOOTH_SET_NAME_CODE[17]="AT+NAMEQuadrotorE";
//      const u8 BLUETOOTH_SET_NAMB_CODE[17]="AT+NAMBQuadrotorB";
//      const u8 BLUETOOTH_SET_SPEED_CODE[8]="AT+HIGH1";      
    /*for 2.0*/
//      const u8 BLUETOOTH_SET_BAUD_CODE[8]="AT+BAUD8";
//      const u8 BLUETOOTH_SET_PIN_CODE[10]="AT+PIN9527";
//      const u8 BLUETOOTH_SET_NAME_CODE[19]="AT+NAMEQuadrotor1.1"; 
//      HAL_UART_Transmit(&huart1, (u8 *)BLUETOOTH_SET_PIN_CODE, 10, 100);
//      osDelay(3000);
//      HAL_UART_Transmit(&huart1, (u8 *)BLUETOOTH_SET_NAME_CODE, 19, 100);
//      osDelay(3000);
//      HAL_UART_Transmit(&huart1, (u8 *)BLUETOOTH_SET_BAUD_CODE, 8, 100);
//      osDelay(3000);
//      while(1)
//      {
//          extern u32 usart_counter;
//          
//          baud+=9600;
//          huart1.Instance = USART1;
//          huart1.Init.BaudRate = baud;
//          huart1.Init.WordLength = UART_WORDLENGTH_8B;
//          huart1.Init.StopBits = UART_STOPBITS_1;
//          huart1.Init.Parity = UART_PARITY_NONE;
//          huart1.Init.Mode = UART_MODE_TX_RX;
//          huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//          huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//          HAL_UART_Init(&huart1);
//          HAL_UART_Transmit(&huart1, b, 8, 100);
//          osDelay(1000);          
//          if(usart_counter)
//          {
//              usart_counter=0;
//          }

//      }
  }
  /* USER CODE END UartTask */
}

/* StartupTask function */
void StartupTask(void const * argument)
{
  /* USER CODE BEGIN StartupTask */
  
  /* Infinite loop */
  for(;;)
  {  
    vTaskSuspend(mathTaskHandle);  
    vTaskSuspend(ledTaskHandle);
    vTaskSuspend(obstacleTaskHandle);
    vTaskSuspend(osStatTaskHandle);
    vTaskSuspend(uartTaskHandle);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
    //freq = ClkFreq();
    taskENTER_CRITICAL();                        //关中断
    OSIdleCtr =0;                       // Clear idle counter                                 */
    taskEXIT_CRITICAL();
    vTaskDelay(CULCULATE_PERIOD);      /* Determine MAX. idle counter value for CULCULATE_PERIOD ms ,延时 */
    taskENTER_CRITICAL();
    OSIdleCtrMax = OSIdleCtr;      /* Store maximum idle counter count in CULCULATE_PERIOD ms，获取CULCULATE_PERIODms内OSIdleCtrMax加到的最大值    */
    taskEXIT_CRITICAL();         //开中断
    vTaskDelay(3000);
    HAL_ADC_Start_DMA(&hadc1, (u32 *)ADC_ConvertedValue, ADC_SAMPLE_NUM * ADC_FINAL_ENUM);
      /*WARNING: calibration*/
    LEDPWM(0,255);
    LEDPWM(1,255);
    LEDPWM(2,255);
    LEDPWM(3,255);
//    attitude_init();
    HAL_UART_Receive_IT(&huart1,rcv_data,1);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//启动PWM
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//启动PWM
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//启动PWM
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//启动PWM
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//启动PWM
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//启动PWM
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//启动PWM
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//启动PWM
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//启动PWM
    /*红外开关*/
        

    BMP280_Init();/*初始化气压计*/  
    MPU9250_Init();
    BMP280_Height_Calibration();
    BatteryVoltage = (float)ADC_ConvertedValue[0][ADC_BATTERY]*2.f/4096.f*3.3f;
    BatteryPercentage = (BatteryVoltage - CHARGE_VOLTAGE)/(CHG_CPLT_VLTGf-CHARGE_VOLTAGE)*100;
    ControlInit();

    vTaskResume(ledTaskHandle);
    vTaskResume(mathTaskHandle);
    vTaskResume(obstacleTaskHandle);
    vTaskResume(osStatTaskHandle);
    vTaskResume(uartTaskHandle);
    vTaskResume(startupTaskHandle);
      
    vTaskDelete(NULL);
  }
  /* USER CODE END StartupTask */
}

/* ControlTask function */
void ControlTask(void const * argument)
{
  /* USER CODE BEGIN ControlTask */
    TickType_t xLastWakeTime;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount ();
  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil( &xLastWakeTime, 50);
    switch((u8)FlyMode)
    {
        case STANDBY:
            if(UsartFramRcvTime+200>HAL_GetTick())
            {
                FlyMode=TAKEOFFING;
            }
            break;
        case FLYING:
            MotorBase=3000;
            if(UsartFramRcvTime+200<HAL_GetTick())
            {
                FlyMode=STANDBY;
            }
            break;
        case TAKEOFFING:
            CtrlTakeOff();
            break;
        case LANDING:
            break;
    }
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
