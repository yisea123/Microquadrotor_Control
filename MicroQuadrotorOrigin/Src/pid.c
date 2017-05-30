/*********************************************************************************
 *@FileName   pid.c
 *@Version    V1
 *@Author     TracyW
 *@Date       2015/04/09
 *@Brief      PID��������������ٶȵ���˫�ջ�
 *********************************************************************************/
#include "bsp.h"
#include "stm32f4xx_hal.h"
#include <arm_math.h>
#include "filter.h"
#include "tim.h"
#include "pid.h"
//��ʼ��PID


void PID_Init(PidTypeDef * pid)
{
    memset(pid, 0, sizeof(PidTypeDef));
}

//���ò���
/*
beta
dif_prior
kaff���ٶ�ǰ��
kvff�ٶ�ǰ��
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
//PID����
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
    pid->e[2] = set_val - rel_val; //��ȡ��ǰ���
    pid->r[2] = rel_val; //��ȡ��ǰֵ

    if (dt==0) return;
    /*��������*/
    p = pid->Kp * pid->e[2]; //λ��ʽp
    //p = pid->Kp * (pid->e[2] - pid->e[1]); //����ʽP
    
    /*����*/
     
    pid->integral += pid->e[2] * pid->Ki * dt;
    
    if (pid->integral > INTEGRAL_MAX)
    {
        pid->integral=INTEGRAL_MAX;
    }
    else if (pid->integral < -INTEGRAL_MAX)
    {
        pid->integral=-INTEGRAL_MAX;
    }
    /*���ַ���*/
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
    
    /*΢������*/
    
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

    kvff = (pid->s[2] - pid->s[1])/dt * pid->Kvff;//�ٶ�ǰ��
    kaff = (pid->s[2] - 2 * pid->s[1] + pid->s[0])/dt * pid->Kaff;//���ٶ�ǰ��

//    pid->delta_U = p + i + d + kvff + kaff; //����ʽPID
    pid->U = p + i + d + kvff + kaff;//pid->last_U + pid->delta_U; //λ��ʽPID
//    if (pid->U > PWM_MAX_PERIOD)
//    {
//        pid->U = PWM_MAX_PERIOD;
//    }
//    else if (pid->U < 0)
//    {
//        pid->U = 0;
//    }

    /*��¼��һ�����*/
    pid->last_U = pid->U;
    /*�����趨ֵ*/
    pid->s[0] = pid->s[1];
    pid->s[1] = pid->s[2];
    /*�������*/
    pid->e[0] = pid->e[1];
    pid->e[1] = pid->e[2];
    /*����ʵ��ֵ*/
    pid->r[0] = pid->r[1];
    pid->r[1] = pid->r[2];
}


/**************************End of this file***************************************



                                          Copyright @TracyW */
