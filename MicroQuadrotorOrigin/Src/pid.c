/*********************************************************************************
 *@FileName   pid.c
 *@Version    V1
 *@Author     TracyW
 *@Date       2015/04/09
 *@Brief      PID得输出量、包含速度电流双闭环
 *********************************************************************************/
#include "bsp.h"
#include "stm32f4xx_hal.h"
#include <arm_math.h>
#include "filter.h"
#include "tim.h"
#include "pid.h"
//初始化PID


void PID_Init(PidTypeDef * pid)
{
    memset(pid, 0, sizeof(PidTypeDef));
}

//设置参数
/*
beta
dif_prior
kaff加速度前馈
kvff速度前馈
*/
void PID_SetParam(PidTypeDef * pid, float p, float i, float d, float beta,
                  float dif_prior, float kaff, float kvff)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->beta = beta;
    pid->dif_prior = dif_prior;
    pid->Kaff = kaff;
    pid->Kvff = kvff;
}
const float INTEGRAL_MAX=2000.f;
//PID计算
void PID_Calc(PidTypeDef * pid,  float set_val, float rel_val,int32_t dertT)
{
    float p = 0,
          i = 0,
          d = 0,
          kvff,
          kaff,
          Deriv,
          dt= dertT/1000.0;
    pid->s[2] = set_val;
    pid->e[2] = set_val - rel_val; //获取当前误差
    pid->r[2] = rel_val; //获取当前值

    if (dt==0) return;
    /*计算增量*/
    p = pid->Kp * pid->e[2]; //位置式p
    //p = pid->Kp * (pid->e[2] - pid->e[1]); //增量式P
    
    /*积分*/
     
    pid->integral += pid->e[2] * pid->Ki * dt;
    
    if (pid->integral > INTEGRAL_MAX)
    {
        pid->integral=INTEGRAL_MAX;
    }
    else if (pid->integral < -INTEGRAL_MAX)
    {
        pid->integral=-INTEGRAL_MAX;
    }
    /*积分分离*/
    if (pid->beta == 0)
    {
        i = pid->integral;
    }
    else
    {
        if (pid->e[2] <= pid->beta && pid->e[2] >= -pid->beta)
            i = pid->integral;
        else
            i = 0;
    }
    if(i==0) 
    {
        pid->integral=0;
        i=0;
    }
    
    /*微分先行*/
    
    if (pid->dif_prior)
    {
        Deriv=(pid->r[2] - pid->r[1])/dt;
        d = pid->Kd * Deriv;
    }
    else
    {
        Deriv=(pid->e[2] - pid->e[1])/dt;
        d = pid->Kd * Deriv;
    }

    kvff = (pid->s[2] - pid->s[1])/dt * pid->Kvff;//速度前馈
    kaff = (pid->s[2] - 2 * pid->s[1] + pid->s[0])/dt * pid->Kaff;//加速度前馈

//    pid->delta_U = p + i + d + kvff + kaff; //增量式PID
    pid->U = p + i + d + kvff + kaff;//pid->last_U + pid->delta_U; //位置式PID
//    if (pid->U > PWM_MAX_PERIOD)
//    {
//        pid->U = PWM_MAX_PERIOD;
//    }
//    else if (pid->U < 0)
//    {
//        pid->U = 0;
//    }

    /*记录上一次输出*/
    pid->last_U = pid->U;
    /*迭代设定值*/
    pid->s[0] = pid->s[1];
    pid->s[1] = pid->s[2];
    /*迭代误差*/
    pid->e[0] = pid->e[1];
    pid->e[1] = pid->e[2];
    /*迭代实际值*/
    pid->r[0] = pid->r[1];
    pid->r[1] = pid->r[2];
}


/**************************End of this file***************************************



                                          Copyright @TracyW */
