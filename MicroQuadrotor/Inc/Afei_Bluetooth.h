#ifndef __AFEI_BLUETOOTH_H
#define __AFEI_BLUETOOTH_H
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
/* Private breif   所有上位机显示小数点的数据接收时都要除以1000---------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/*___________________________发送数据**_____________________________*/
#define Send_Speed 10
#define Send_Position_X 11
#define Send_Position_Y 12
#define Send_Yaw 13  
#define Send_Pitch 14
#define Send_Roll 15     
#define Send_Number1 16                     /*Number1~Number6 备用数据*/
#define Send_Number2 17
#define Send_Number3 18
#define Send_Number4 19
#define Send_Number5 20
#define Send_Number6 21
#define Send_CPUProcesse 22                 /*CPU占用率*/
#define Send_FreeHeap 23                    /*芯片存储余量 不装逼可忽略*/
#define Send_FailNum 24                     /*失败的次数 闲的没事你可以算算*/
#define Send_CatchTheTopSuccess 25          /*哨兵成功登顶*/
#define Send_CatchTheTopFail 26             /*跪了 但至少努力过*/
/*____________________________接收数据________________________________*/

/**************PID数据****************/
#define Rec_PID1_P 0x20                      /*收到P值标志*/
#define Rec_PID1_I 0x21                      /*收到I值标志*/
#define Rec_PID1_D 0x22                      /*收到D值标志*/
#define Rec_PID1_PIDRange_Max 0x29           /*收到PID最大限幅标志*/
#define Rec_PID1_PIDRange_Min 0x2A           /*收到PID最小限幅标志*/

#define Rec_PID2_P 0x26
#define Rec_PID2_I 0x27
#define Rec_PID2_D 0x28
#define Rec_PID2_PIDRange_Max 0x2B
#define Rec_PID2_PIDRange_Min 0x2C

#define Rec_PID3_P 0x23
#define Rec_PID3_I 0x24
#define Rec_PID3_D 0x25
#define Rec_PID3_PIDRange_Max 0x2D
#define Rec_PID3_PIDRange_Min 0x2E

#define Rec_PID4_P 0xF0
#define Rec_PID4_I 0x60
#define Rec_PID4_D 0x61
#define Rec_PID4_PIDRange_Max 0xFC
#define Rec_PID4_PIDRange_Min 0xFD

#define Rec_PID5_P 0xF3
#define Rec_PID5_I 0xF4
#define Rec_PID5_D 0xF5
#define Rec_PID5_PIDRange_Max 0xFE
#define Rec_PID5_PIDRange_Min 0x45

#define Rec_PID6_P 0xF6
#define Rec_PID6_I 0xF7
#define Rec_PID6_D 0xF8
#define Rec_PID6_PIDRange_Max 0x46
#define Rec_PID6_PIDRange_Min 0x47

#define Rec_PID7_P 0xF9
#define Rec_PID7_I 0xFA
#define Rec_PID7_D 0xFB
#define Rec_PID7_PIDRange_Max 0x48
#define Rec_PID7_PIDRange_Min 0x49
/*******************方向等基本操作****************/
#define Rec_Go 0x35                     /*启动标志 上位机操控时可使用空格键操作*/
#define Rec_Break 0x36                  /*刹车标志 上位机操控时可使用空格键操作*/   
#define Rec_Up 0xE0                     /*键盘方向键  用于操控逼哥装逼OLED UI界面*/
#define Rec_Down 0xE1           
#define Rec_Left 0xE2           
#define Rec_Right 0xE3           

#define Rec_GoForward 0x37              /*键盘方向键 用于车体的遥控*/
#define Rec_GoBack 0x38
#define Rec_GoRight 0x3A
#define Rec_GoLeft 0x39
/*********************云台数据********************/
#define Rec_MaxCurrent1 0xE5            /*电机1最大电流*/
#define Rec_MinCurrent1 0xE6            /*电机1最小电流*/
#define Rec_MaxPitch1 0xE7              /*云台最大俯仰角*/
#define Rec_MinPitch1 0xE8              /*云台最小俯仰角*/
#define Rec_MaxYaw1 0xE9                /*云台最大偏航角*/
#define Rec_MinYaw1 0xEA                /*云台最小偏航角*/

#define Rec_MaxCurrent2 0xF1 
#define Rec_MinCurrent2 0xF2
#define Rec_MaxPitch2 0xEB
#define Rec_MinPitch2 0xEC
#define Rec_MaxYaw2 0xED
#define Rec_MinYaw2 0xEE
/********************底盘数据*******************/
#define Rec_MWSpeed1 0x4A              /*四个麦克纳姆轮的分速度*/
#define Rec_MWSpeed2 0x4B
#define Rec_MWSpeed3 0x4C
#define Rec_MWSpeed4 0x4D
#define Rec_Set_Anlge 0x4E             /*与当前车体正方向所成运动夹角*/
#define Rec_Set_Speed 0x4F             /*以设定夹角运行的速度*/
/********************哨兵数据********************/
#define Rec_GuardPitch 0x50            /*四个自由度数据*/
#define Rec_GuardRoll 0x51
#define Rec_GuardYaw 0x52
#define Rec_GuardHeight 0x53

#define Rec_AdvanceSpeed 0x54          /*哨兵启动向杆行进速度*/
#define Rec_SideSpeed 0x55             /*哨兵左右移动速度*/
#define Rec_VerticalSpeed 0x56         /*哨兵起降速度*/
#define Rec_AcceleratorRange 0x57      /*涵道油门范围*/
#define Rec_Accelerator 0x58           /*当前油门值*/

#define Rec_GuardSafe 0x59             /*逼哥人身保险*/
#define Rec_Reset 0x64                 /*复位*/
/***********************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
typedef enum
{
    PITCH_SET,
    YAW_SET,
    ROLL_SET,
    HEIGHT_SET,
    
    FIRMWARE_VERSION,
    BATTERY_PERCENTAGE,
    HEIGHT_PRESENT,
    PITCH_PRESENT,
    ROLL_PRESENT,
    YAW_PRESENT,
    BOARD_TEMPERTURE,
    
    PITCH_P_SET,
    PITCH_I_SET,
    PITCH_D_SET, 
    ROLL_P_SET,
    ROLL_I_SET,
    ROLL_D_SET,
    YAW_P_SET,
    YAW_I_SET,
    YAW_D_SET,
    HEIGHT_P_SET,
    HEIGHT_I_SET,
    HEIGHT_D_SET,
    
    RESET_COMMAND,
    SAVE_COMMAND,
    CONNECTED_QUEST,
    CONNECTED_ACK,
}SetupType;

/*自定义预留端口名字*/
union float_trans
{
	float float_form;
    s32     int_form;
	char char_form[4];
};

/* Defines for the UI parameter */
typedef enum    
{
    VALUE_TYPE_INTEGER,
    VALUE_TYPE_FLOAT,
}ValueTypeType;

typedef struct
{
    union float_trans Num;
    const union float_trans MinValue;
    const union float_trans MaxValue;
    const ValueTypeType ValueType;
    const SetupType SetupType;
}ValueStruct;

void Send_Array_Old(uint8_t * Array, uint16_t Len);
void Send_Data_Old(float num,uint8_t NUM_Type);
s32 BlueToothInit(void);
extern u8 USART_Old[],USART_New[],rcv_data[];
extern u32 UsartFramRcvTime;
void Send_Data_New(ValueStruct* num);

void Send_Current(void);
void Send_Driver_Info_Usart(void);
void Send_Encoder_Direct(void);
void Send_Motor_Direct(void);
void Send_Over_Current_Value(void);
void Send_Extreme_Current_Value(void);
void Send_Recovery_Current_Value(void);
void HAL_UART_RxCpltCallback2(u8 data);
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#endif //Rsing_Periph_Handle.h

