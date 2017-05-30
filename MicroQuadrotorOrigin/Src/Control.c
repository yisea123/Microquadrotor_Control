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
#include "obstacles.h"
#include "quad_math.h"
float  PitchSet,PitchError =0.0f;
float  RollSet,RollError  =0.0f;
float  YawSet,YawError   =0.0f;
float rollSp =0,pitchSp =0;		//根据动力分配重新计算得到的期望roll pitch
float  HeightSet=0.35;
float MotorBaseK=500; /*基准速度系数，与电池电压共同作用*/
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

float ANGLE_SET_RANGE=20.f;
float CtrlValAdj(float angle)
{
    (angle<0)?
    (angle = -angle * angle / ANGLE_SET_RANGE):
    (angle = angle * angle / ANGLE_SET_RANGE);
    
    if(angle>ANGLE_SET_RANGE)
    {
        angle=ANGLE_SET_RANGE;
    }
    else if(angle<-ANGLE_SET_RANGE)
    {
        angle=-ANGLE_SET_RANGE;
    }
    return angle;
}
/*
函数名：CtrlAttiAng(void)
描述：对飞行器姿态控制（pitch，roll，yaw）控制中，串级PID中的角度环控制
*/float angTarget[3]={0};
void CtrlAttiAng(void)
{
		static uint32_t tPrev=0;
		
		float dt=0,t=0;
		t=micros();
		dt=(tPrev>0)?(t-tPrev):0;
		tPrev=t;
//		if (t==0) return;
//		if(altCtrlMode==MANUAL){
//			angTarget[ROLL]=(float)(RC_DATA.ROOL);
//			angTarget[PITCH]=(float)(RC_DATA.PITCH);
//		}else{
			angTarget[ROLL]=CtrlValAdj(RollSet/2.0f);//rollSp;
			angTarget[PITCH]=CtrlValAdj(PitchSet/2.0f);//pitchSp;
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
 
		PID_Calc(&PidPitchAngle,angTarget[PITCH]+PitchObstacle+PitchError,imu.pitch,dt);
		PID_Calc(&PidRollAngle,angTarget[ROLL]+RollObstacle+RollError,imu.roll,dt);
       PID_Calc(&PidYawAngle,0,imu.yaw,dt);//DMP_DATA.GYROz
        PID_Calc(&PidHeight,HeightSet,-z_est[0],dt);
        
}



/*
函数名：CtrlAttiRate(void)
描述：对飞行器姿态控制（pitch，roll，yaw）控制中，串级PID中的角速度环控制
*/
void CtrlAttiRate(void)
{
// 	float yawRateTarget=0;
	static uint32_t tPrev=0; 

	float dt=0,t=0;
	t=micros();
	dt=(tPrev>0)?(t-tPrev):0;
	tPrev=t;
//			if (t==0) return;
	//yawRateTarget=-(float)RC_DATA.YAW;
	
	//注意，原来的pid参数，对应的是 ad值,故转之

	PID_Calc(&PidPitchRate,PidPitchAngle.U,imu.gyro[PITCH]*180.0f/PI,dt);	
	PID_Calc(&PidRollRate,PidRollAngle.U,imu.gyro[ROLL]*180.0f/PI,dt);//gyroxGloble
	PID_Calc(&PidYawRate,PidYawAngle.U,imu.gyro[YAW]*180.0f/PI,dt);//DMP_DATA.GYROz
    PID_Calc(&PidHeightRate,PidHeight.U,-z_est[1],dt);
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
    /*I get these param by intuition :)*/
                 /*p, i, d, beta, dif_prior, kaff, kvff*/
//    PID_SetParam(&PidPitchAngle, 3.5,0,0,0,0,0,0);
//    PID_SetParam(&PidRollAngle, 3.5,0,0,0,0,0,0);
//    PID_SetParam(&PidYawAngle, 0,0,0,0,0,0,0);
//    PID_SetParam(&PidHeight, 0,0,0,0,0,0,0);
//    
//    PID_SetParam(&PidPitchRate, 7,0,0.3,10,0,0,0);
//    PID_SetParam(&PidRollRate, 7,0,0.3,10,0,0,0);
//    PID_SetParam(&PidYawRate, 5,0,0.3,0,0,0,0);
//    PID_SetParam(&PidHeightRate, 0,0,0,0,0,0,0);
    PID_SetParam(&PidPitchAngle, 3,0,0,0,0,0,0);
    PID_SetParam(&PidRollAngle, 3,0,0,0,0,0,0);
    PID_SetParam(&PidYawAngle, 1,0,0,0,0,0,0);
    PID_SetParam(&PidHeight, 10,0,0,0,0,0,0);
    
    PID_SetParam(&PidPitchRate, 7,0.0,200,10,0,0,0);
    PID_SetParam(&PidRollRate, 7,0.0,200,10,0,0,0);
    PID_SetParam(&PidYawRate, 20,0,3000,0,0,0,0);
    PID_SetParam(&PidHeightRate, 10,0,30000,0,0,0,0);
}
void Control()
{
    static u32 i;
    i++;
    /*角度环次数与速度环次数比例*/
    if(i & 0x00000001)
    {
        //CtrlAlti();
        CtrlAttiAng();
    }
    CtrlAttiRate();
    
    MotorSpeed[0] = MotorBase + PidPitchRate.U - PidRollRate.U - PidYawRate.U + PidHeightRate.U;
    MotorSpeed[1] = MotorBase - PidPitchRate.U - PidRollRate.U + PidYawRate.U + PidHeightRate.U;
    MotorSpeed[2] = MotorBase - PidPitchRate.U + PidRollRate.U - PidYawRate.U + PidHeightRate.U;
    MotorSpeed[3] = MotorBase + PidPitchRate.U + PidRollRate.U + PidYawRate.U + PidHeightRate.U;
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

//cut deadband, move linear
float dbScaleLinear(float x, float x_end, float deadband)
{
	if (x > deadband) {
		return (x - deadband) / (x_end - deadband);

	} else if (x < -deadband) {
		return (x + deadband) / (x_end - deadband);

	} else {
		return 0.0f;
	}
}


float thrInit;

#define SLOW_THRO 200
//定义飞机最大倾斜角度
#define  Angle_Max  40.0
#define  YAW_RATE_MAX  180.0f/PI		//deg/s  
//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡
//#define  Rool_error_init   7      //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
//#define  Pitch_error_init  -5      //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝后偏，Pitch_error_init朝正向增大修改
//定高部分
#define LAND_SPEED						1.2f		//m/s^2
#define ALT_VEL_MAX 					4.0f
#define THR_MIN								0.38f		//min thrust ，根据机重和最小降速而定，用于下降速度过大时，油门过小，导致失衡。再增加fuzzy control ，在油门小时用更大的姿态参数

#define ALT_FEED_FORWARD  		0.5f
#define THR_MAX								1.0f		//max thrust
#define TILT_MAX 					(Angle_Max * PI / 180.0f )
const float ALT_CTRL_Z_DB = 0.1f;	//
float spZMoveRate;

float hoverThrust=0;
uint8_t zIntReset=1;	//integral reset at first . when change manual mode to climb rate mode
float thrustZInt=0, thrustZSp=0;
float thrustXYSp[2]={0,0};	//roll pitch
uint8_t recAltFlag=0;
float holdAlt=0;
uint8_t satZ=0,satXY=0;	//是否过饱和


#define ALT_LIMIT							2.0f		//限高 3.5
uint8_t isAltLimit=0;
float altLand;



//函数名：estimateHoverThru()
//输入：无
//输出: 预估得到的悬停油门基准值
//描述：预估悬停油门基准值，直接影响到该飞行器的z轴悬停
//悬停油门值相关因素有：电池电压
//Get a estimated value for hold throttle.It will have a direct affection on hover
//Battery voltage
float estimateHoverThru(void){
	float hoverHru = 0.55f;
	
	if(BatteryPercentageTemp > 4.05f){
		hoverHru = -0.35f;
	}else if(BatteryPercentageTemp > 3.90f){
		hoverHru = -0.40f;
	}else if(BatteryPercentageTemp > 3.80f){
		hoverHru = -0.45f;
	}else if(BatteryPercentageTemp > 3.70f){
		hoverHru = -0.50f;
	}else{
		hoverHru = -0.55f;
	}
	
	
//	if(Battery.BatteryVal > 4.05){
//		hoverHru = -0.05f;
//	}else if(Battery.BatteryVal > 3.90){
//		hoverHru = -0.10f;
//	}else if(Battery.BatteryVal > 3.80){
//		hoverHru = -0.15f;
//	}else if(Battery.BatteryVal > 3.70){
//		hoverHru = -0.20f;
//	}else{
//		hoverHru = -0.25f;
//	}
	
	return hoverHru;
}

//函数名：CtrlAlti()
//输入：无
//输出: 最终结果输出到全局变量thrustZSp
//描述：控制高度，也就是高度悬停控制函数
//only in climb rate mode and landind mode. now we don't work on manual mode

	float manThr=0,alt=0,velZ=0;
	float altSp=0;
	float posZVelSp=0;
	float altSpOffset,altSpOffsetMax=0;
	float dt=0,t=0;
	static float tPrev=0,velZPrev=0;
	float posZErr=0,velZErr=0,valZErrD=0;
	float thrustXYSpLen=0,thrustSpLen=0;
	float thrustXYMax=0;
void CtrlAlti(void)
{
	float manThr=0,alt=0,velZ=0;
	float altSp=0;
	float posZVelSp=0;
	float altSpOffset,altSpOffsetMax=0;
	u32 dt=0,t=0;
	float posZErr=0,velZErr=0,valZErrD=0;
	float thrustXYSpLen=0,thrustSpLen=0;
	float thrustXYMax=0;
	//get dt		
	//保证dt运算不能被打断，保持更新，否则dt过大，积分爆满。
	if(tPrev==0){
			tPrev=micros();
			return;
	}else{
			t=micros();
			dt=(t-tPrev) /1000000.0f;
			tPrev=t;
	}
	
	//only in climb rate mode and landind mode. now we don't work on manual mode
	//手动模式不使用该高度控制算法
//	if(MANUAL == altCtrlMode || !FLY_ENABLE){
//		return;
//	}
	
	//--------------pos z ctrol---------------//
	//get current alt 
	alt=-nav.z;
	//get desired move rate from stick
	manThr=0 / 1000.0f;
	spZMoveRate= -dbScaleLinear(manThr-0.5f,0.5f,ALT_CTRL_Z_DB);	// scale to -1~1 . NED frame
	spZMoveRate = spZMoveRate * ALT_VEL_MAX;	// scale to vel min max

	//get alt setpoint in CLIMB rate mode
	altSp 	=-nav.z;						//only alt is not in ned frame.
	altSp  -= spZMoveRate * dt;	 
	//limit alt setpoint
	altSpOffsetMax=ALT_VEL_MAX / PidHeight.Kp * 2.0f;
	altSpOffset = altSp-alt; 
	if( altSpOffset > altSpOffsetMax){
		altSp=alt +  altSpOffsetMax;
	}else if( altSpOffset < -altSpOffsetMax){
		altSp=alt - altSpOffsetMax;
	}

	//限高
	if(isAltLimit)
	{
		if(altSp - altLand > ALT_LIMIT)
		{
				altSp=altLand+ALT_LIMIT;
				spZMoveRate=0;
		}
	}
	
	// pid and feedforward control . in ned frame
	posZErr= -(altSp - alt);
	posZVelSp = posZErr * PidHeight.Kp + spZMoveRate * ALT_FEED_FORWARD;
//	//consider landing mode
//	if(altCtrlMode==LANDING)
//		posZVelSp = LAND_SPEED;
//	
	//获取一个预估的Z轴悬停基准值，相关因素有电池电压
	//get hold throttle. give it a estimated value
	if(zIntReset){
		thrustZInt = estimateHoverThru();
		zIntReset = 0;
	}
	
	velZ=nav.vz;	
	velZErr = posZVelSp - velZ;
	valZErrD = (spZMoveRate - velZ) * PidHeight.Kp - (velZ - velZPrev) / dt;	//spZMoveRate is from manual stick vel control
	velZPrev=velZ;
	
	thrustZSp= velZErr * PidHeightRate.Kp + valZErrD * PidHeightRate.Kd + thrustZInt;	//in ned frame. thrustZInt contains hover thrust
	
	//limit thrust min !!
//	if(altCtrlMode!=LANDING){
		if (-thrustZSp < THR_MIN){
			thrustZSp = -THR_MIN; 
		} 
//	}
	
	//与动力分配相关	testing
	satXY=0;
	satZ=0;
	thrustXYSp[0]= sinf(RollSet * PI /180.0f) ;//目标角度转加速度
	thrustXYSp[1]= sinf(PitchSet * PI /180.0f) ; 	//归一化
	thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
	//limit tilt max
	if(thrustXYSpLen >0.01f )
	{
		thrustXYMax=-thrustZSp * tanf(TILT_MAX);
		if(thrustXYSpLen > thrustXYMax)
		{
				float k=thrustXYMax / thrustXYSpLen;
				thrustXYSp[1] *= k;
				thrustXYSp[0] *= k;
				satXY=1;
				thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
		}
		
	}
	//limit max thrust!! 
	thrustSpLen=sqrtf(thrustXYSpLen * thrustXYSpLen + thrustZSp * thrustZSp);
	if(thrustSpLen > THR_MAX)
	{
			if(thrustZSp < 0.0f)	//going up
			{
						if (-thrustZSp > THR_MAX) 
						{
								/* thrust Z component is too large, limit it */
								thrustXYSp[0] = 0.0f;
								thrustXYSp[1] = 0.0f;
								thrustZSp = -THR_MAX;
								satXY = 1;
								satZ = 1;

							} 
							else {
								float k = 0;
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								thrustXYMax = sqrtf(THR_MAX * THR_MAX- thrustZSp * thrustZSp);
								k=thrustXYMax / thrustXYSpLen;
								thrustXYSp[1] *=k;
								thrustXYSp[0] *= k;
								satXY=1;
							}
			}
			else {		//going down
							/* Z component is negative, going down, simply limit thrust vector */
							float k = THR_MAX / thrustSpLen;
							thrustZSp *= k;
							thrustXYSp[1] *=k;
							thrustXYSp[0] *= k;
							satXY = 1;
							satZ = 1;
						}
		
	} 
	rollSp= asinf(thrustXYSp[0]) * 180.0f /PI;
	pitchSp = asinf(thrustXYSp[1]) * 180.0f /PI;				
	
	
	// if saturation ,don't integral
	if(!satZ )//&& fabs(thrustZSp)<THR_MAX
	{
			thrustZInt += velZErr * PidHeightRate.Ki * dt;
			if (thrustZInt > 0.0f)
							thrustZInt = 0.0f;
	}
}
