/**
  ******************************************************************************
  * @file    LED.c
  * @author  Johnny Sun
  * @version V1.0
  * @date    2015/12/6
  * @brief   
  ******************************************************************************
**/
#include "led.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "control.h"
#include "tim.h"
u32 StatusLedOn=1;
s32 LEDPeriod=0;
u32 LEDState=1;
u32 LEDResumeTime=0;
u32 raisetime =   5;
u32 BatteryLedStateCunter=0;
u32 droptime  =   15;
void BigLed(u32 i)
{
    if(i<2)
    {
    (i)?
    (TIM4->CCR4=20):(TIM4->CCR4=0);
    }
}
void LedHandle(void)
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
    if(!imu.ready)/*校准常量*/
    {
            LEDPWM(0,128);
            LEDPWM(1,128);
            LEDPWM(2,128);
            LEDPWM(3,128);
    }
    else if((FlyMode == STANDBY)&&(StatusLedOn))/*待机呼吸灯*/
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
    else if(StatusLedOn)
    {
        osDelay(100);
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
            osDelay(70);
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
    else
    {
            LEDPWM(0,0);
            LEDPWM(1,0);
            LEDPWM(2,0);
            LEDPWM(3,0);
    }
}
