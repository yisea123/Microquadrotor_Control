/**
  ******************************************************************************
  * @file    Obstacles.c
  * @author  Johnny Sun
  * @version V1.0
  * @date    2015/12/6
  * @brief   
  ******************************************************************************
**/
#include "Obstacles.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "control.h"
u32 ADC_Distance_Calib_Flag;
float PitchObstacle,RollObstacle;
u32 ObstacleOn;
void ObstaclesHandler(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);/*LED发射管打开*/
    osDelay(5);
    ADC_Callback1(&hadc1); 
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);/*LED发射管关闭省电*/
    osDelay(15);
    ADC_Callback2(&hadc1); 
    if(ObstacleOn)
    {
        if(ADC_Distance_Calib_Flag)
        {
            ADC_Distance_Calib();
        }
        else
        {
            PitchObstacle=Comp_Distance[ADC_FRONT_LED]
                        -Comp_Distance[ADC_BACK_LED];
            RollObstacle=-Comp_Distance[ADC_LEFT_LED]
                        +Comp_Distance[ADC_RIGHT_LED];
        }
    }
    else
    {
        PitchObstacle=0;
        RollObstacle=0;
    }
}
