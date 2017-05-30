/**
  ******************************************************************************
  * @file    Control.c
  * @author  Johnny Sun
  * @version V1.0
  * @date    2015/11/20
  * @brief   
  ******************************************************************************
**/
#include "control.h"
#include "tim.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
float  PitchSet,PitchError =0.0f;
float  RollSet,RollError  =0.0f;
float  YawSet,YawError   =0.0f;
u32 MotorOutput=0;
s32  MotorSpeed[4],MotorBase=100;
PidTypeDef  PidPitchRate,   PidPitchAngle,
            PidYawRate,     PidYawAngle,
            PidRollRate,    PidRollAngle,
            PidHeightRate,  PidHeight;
FlyModeType FlyMode= STANDBY;
void CtrlTakeOff()
{
    FlyMode = TAKEOFFING;
    
    PidRollAngle.integral=0;
    PidRollRate.integral=0;
    PidPitchAngle.integral=0;
    PidPitchRate.integral=0;
    PidYawAngle.integral=0;
    PidYawRate.integral=0;
    PidHeight.integral=0;
    PidHeightRate.integral=0;
    MotorOutput=1;
    
    MotorPWM(0,9999);
    MotorPWM(1,9999);
    MotorPWM(2,9999);
    MotorPWM(3,9999);
    osDelay(50);
    MotorPWM(0,0);
    MotorPWM(1,0);
    MotorPWM(2,0);
    MotorPWM(3,0);
    osDelay(300);
    MotorPWM(0,5000);
    MotorPWM(1,5000);
    MotorPWM(2,5000);
    MotorPWM(3,5000);
    osDelay(50);
    MotorPWM(0,0);
    MotorPWM(1,0);
    MotorPWM(2,0);
    MotorPWM(3,0);
    MotorBase=300;
    FlyMode = FLYING;
}
void CtrlLand()
{
    FlyMode = LANDING;
//    MotorOutput=1;
    PidRollAngle.integral=0;
    PidRollRate.integral=0;
    PidPitchAngle.integral=0;
    PidPitchRate.integral=0;
    PidYawAngle.integral=0;
    PidYawRate.integral=0;
    PidHeight.integral=0;
    PidHeightRate.integral=0;
}
/*
函数名：CtrlAttiAng(void)
描述：对飞行器姿态控制（pitch，roll，yaw）控制中，串级PID中的角度环控制
*/
void CtrlAttiAng(void)
{
//		static uint32_t tPrev=0;
		float angTarget[3]={0};
//		float dt=0,t=0;
//		t=micros();
//		dt=(tPrev>0)?(t-tPrev):0;
//		tPrev=t;
//		if (t==0) return;
//		if(altCtrlMode==MANUAL){
//			angTarget[ROLL]=(float)(RC_DATA.ROOL);
//			angTarget[PITCH]=(float)(RC_DATA.PITCH);
//		}else{
			angTarget[ROLL]=0;//rollSp;
			angTarget[PITCH]=0;//pitchSp;
//		}

//		if(headFreeMode){
//			#ifdef YAW_CORRECT
//        float radDiff = -(imu.yaw - headHold) * M_PI_F / 180.0f; 
//			#else
//				float radDiff = (imu.yaw - headHold) * M_PI_F / 180.0f; 
//			#endif
//        float cosDiff = cosf(radDiff);
//        float sinDiff = sinf(radDiff);
//        float tarPitFree = angTarget[PITCH] * cosDiff + angTarget[ROLL] * sinDiff;
//        angTarget[ROLL] = angTarget[ROLL] * cosDiff - angTarget[PITCH] * sinDiff;
//        angTarget[PITCH] = tarPitFree;
//		}
 
		PID_Calc(&PidPitchAngle,angTarget[PITCH]+PitchSet,imu.pitch,Math_PERIOD);
		PID_Calc(&PidRollAngle,angTarget[ROLL]+RollSet,imu.roll,Math_PERIOD);	 
}



/*
函数名：CtrlAttiRate(void)
描述：对飞行器姿态控制（pitch，roll，yaw）控制中，串级PID中的角速度环控制
*/
void CtrlAttiRate(void)
{
 	float yawRateTarget=0;
//	static uint32_t tPrev=0; 

//	float dt=0,t=0;
//	t=micros();
//	dt=(tPrev>0)?(t-tPrev):0;
//	tPrev=t;
//			if (t==0) return;
	//yawRateTarget=-(float)RC_DATA.YAW;
	
	//注意，原来的pid参数，对应的是 ad值,故转之

	PID_Calc(&PidPitchRate,PidPitchAngle.U,imu.gyro[PITCH]*180.0f/PI,Math_PERIOD);	
	PID_Calc(&PidRollRate,PidRollAngle.U,imu.gyro[ROLL]*180.0f/PI,Math_PERIOD);//gyroxGloble
	PID_Calc(&PidYawRate,PidYawAngle.U,imu.gyro[YAW]*180.0f/PI,Math_PERIOD);//DMP_DATA.GYROz
}

void ControlInit()
{
    PID_Init(&PidPitchAngle);
    PID_Init(&PidRollAngle);
    PID_Init(&PidYawAngle);
    PID_Init(&PidHeight);
    
    PID_Init(&PidPitchRate);
    PID_Init(&PidRollRate);
    PID_Init(&PidYawRate);
    PID_Init(&PidHeightRate);
    /*do not ask why. I get these param by intuition :)*/
                 /*p, i, d, beta, dif_prior, kaff, kvff*/
    PID_SetParam(&PidPitchAngle, 3.5,0,0,0,0,0,0);
    PID_SetParam(&PidRollAngle, 3.5,0,0,0,0,0,0);
    PID_SetParam(&PidYawAngle, 0,0,0,0,0,0,0);
    PID_SetParam(&PidHeight, 0,0,0,0,0,0,0);
    
    PID_SetParam(&PidPitchRate, 7,0,0.3,0,0,0,0);
    PID_SetParam(&PidRollRate, 7,0,0,3,0,0,0);
    PID_SetParam(&PidYawRate, 0,0,0,0,0,0,0);
    PID_SetParam(&PidHeightRate, 0,0,0,0,0,0,0);
}
void Control()
{
    CtrlAttiAng();
    CtrlAttiRate();
    
    MotorSpeed[0] = MotorBase + PidPitchRate.U - PidRollRate.U - PidYawRate.U + PidHeight.U;
    MotorSpeed[1] = MotorBase - PidPitchRate.U - PidRollRate.U + PidYawRate.U + PidHeight.U;
    MotorSpeed[2] = MotorBase - PidPitchRate.U + PidRollRate.U - PidYawRate.U + PidHeight.U;
    MotorSpeed[3] = MotorBase + PidPitchRate.U + PidRollRate.U + PidYawRate.U + PidHeight.U;
    if((MotorOutput)&&(FlyMode == FLYING))
    {
    MotorPWM(0,MotorSpeed[0]);
    MotorPWM(1,MotorSpeed[1]);
    MotorPWM(2,MotorSpeed[2]);
    MotorPWM(3,MotorSpeed[3]);
    }
    else if (FlyMode == STANDBY)
    {
        MotorPWM(0,0);
        MotorPWM(1,0);
        MotorPWM(2,0);
        MotorPWM(3,0);
    }
//    PID_Calc(&PidHeight,HeightSet);
}
