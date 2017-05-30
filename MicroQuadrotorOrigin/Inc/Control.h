#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "bsp.h"
#include "pid.h"
#include "quad_math.h"

extern float  PitchSet,PitchError;
extern float  RollSet,RollError;
extern float  YawSet,YawError;
extern float MotorBaseK;
extern u32 MotorOutput;
extern s32  MotorSpeed[],MotorBase;
extern u32 ObstacleOn;
extern PidTypeDef   PidPitchRate,   PidPitchAngle,
                    PidYawRate,     PidYawAngle,
                    PidRollRate,    PidRollAngle,
                    PidHeightRate,  PidHeight;
extern FlyModeType FlyMode;
void CtrlTakeOff(void);
void ControlInit(void);
void Control(void);
void CtrlAlti(void);
#endif