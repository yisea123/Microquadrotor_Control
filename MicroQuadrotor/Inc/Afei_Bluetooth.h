#ifndef __AFEI_BLUETOOTH_H
#define __AFEI_BLUETOOTH_H
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
/* Private breif   ������λ����ʾС��������ݽ���ʱ��Ҫ����1000---------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/*___________________________��������**_____________________________*/
#define Send_Speed 10
#define Send_Position_X 11
#define Send_Position_Y 12
#define Send_Yaw 13  
#define Send_Pitch 14
#define Send_Roll 15     
#define Send_Number1 16                     /*Number1~Number6 ��������*/
#define Send_Number2 17
#define Send_Number3 18
#define Send_Number4 19
#define Send_Number5 20
#define Send_Number6 21
#define Send_CPUProcesse 22                 /*CPUռ����*/
#define Send_FreeHeap 23                    /*оƬ�洢���� ��װ�ƿɺ���*/
#define Send_FailNum 24                     /*ʧ�ܵĴ��� �е�û�����������*/
#define Send_CatchTheTopSuccess 25          /*�ڱ��ɹ��Ƕ�*/
#define Send_CatchTheTopFail 26             /*���� ������Ŭ����*/
/*____________________________��������________________________________*/

/**************PID����****************/
#define Rec_PID1_P 0x20                      /*�յ�Pֵ��־*/
#define Rec_PID1_I 0x21                      /*�յ�Iֵ��־*/
#define Rec_PID1_D 0x22                      /*�յ�Dֵ��־*/
#define Rec_PID1_PIDRange_Max 0x29           /*�յ�PID����޷���־*/
#define Rec_PID1_PIDRange_Min 0x2A           /*�յ�PID��С�޷���־*/

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
/*******************����Ȼ�������****************/
#define Rec_Go 0x35                     /*������־ ��λ���ٿ�ʱ��ʹ�ÿո������*/
#define Rec_Break 0x36                  /*ɲ����־ ��λ���ٿ�ʱ��ʹ�ÿո������*/   
#define Rec_Up 0xE0                     /*���̷����  ���ڲٿرƸ�װ��OLED UI����*/
#define Rec_Down 0xE1           
#define Rec_Left 0xE2           
#define Rec_Right 0xE3           

#define Rec_GoForward 0x37              /*���̷���� ���ڳ����ң��*/
#define Rec_GoBack 0x38
#define Rec_GoRight 0x3A
#define Rec_GoLeft 0x39
/*********************��̨����********************/
#define Rec_MaxCurrent1 0xE5            /*���1������*/
#define Rec_MinCurrent1 0xE6            /*���1��С����*/
#define Rec_MaxPitch1 0xE7              /*��̨�������*/
#define Rec_MinPitch1 0xE8              /*��̨��С������*/
#define Rec_MaxYaw1 0xE9                /*��̨���ƫ����*/
#define Rec_MinYaw1 0xEA                /*��̨��Сƫ����*/

#define Rec_MaxCurrent2 0xF1 
#define Rec_MinCurrent2 0xF2
#define Rec_MaxPitch2 0xEB
#define Rec_MinPitch2 0xEC
#define Rec_MaxYaw2 0xED
#define Rec_MinYaw2 0xEE
/********************��������*******************/
#define Rec_MWSpeed1 0x4A              /*�ĸ������ķ�ֵķ��ٶ�*/
#define Rec_MWSpeed2 0x4B
#define Rec_MWSpeed3 0x4C
#define Rec_MWSpeed4 0x4D
#define Rec_Set_Anlge 0x4E             /*�뵱ǰ���������������˶��н�*/
#define Rec_Set_Speed 0x4F             /*���趨�н����е��ٶ�*/
/********************�ڱ�����********************/
#define Rec_GuardPitch 0x50            /*�ĸ����ɶ�����*/
#define Rec_GuardRoll 0x51
#define Rec_GuardYaw 0x52
#define Rec_GuardHeight 0x53

#define Rec_AdvanceSpeed 0x54          /*�ڱ���������н��ٶ�*/
#define Rec_SideSpeed 0x55             /*�ڱ������ƶ��ٶ�*/
#define Rec_VerticalSpeed 0x56         /*�ڱ����ٶ�*/
#define Rec_AcceleratorRange 0x57      /*�������ŷ�Χ*/
#define Rec_Accelerator 0x58           /*��ǰ����ֵ*/

#define Rec_GuardSafe 0x59             /*�Ƹ�������*/
#define Rec_Reset 0x64                 /*��λ*/
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

/*�Զ���Ԥ���˿�����*/
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

