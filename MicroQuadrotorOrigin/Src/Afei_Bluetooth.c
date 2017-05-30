/**
  ******************************************************************************
  * @file    BlueTooth.c
  * @author  Johnny Sun
  * @version V1.0
  * @date    2015/7/5
  * @brief   串口蓝牙参数调试   配RobotMasters 底盘电机驱动
  * @Functions:   void BlueTooth_Init(void)   蓝牙初始化操作
  *
  *               void Send_Data(float num, SetupTypeStruct Num)  发送数据
  *                                                            数据统一为float格式
  *
  *               void USART3_IRQHandler(void)  接收来自上位机数据,内部可自行添
  *                                             加收到相应数据后的操作。
  ******************************************************************************
**/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"  
#include "cpustate.h"
#include "BSP.h"
#include "usart.h"
#include "Afei_Bluetooth.h"
#include "Control.h"
#include "bmp280.h"
#include "adc.h"
#include "LED.h"
/****-------------------------------------------------------------------------------***/
u8 SEND_BUF_OLD[9] = {0xc0, 0xfb, 0xe2, 0, 0, 0, 0, 0, 0xdd};
u8 rcv_data[2];
/**
  * @brief  发参数  内部调用
  * @param  None
  * @retval None
  */
void Send_Array_Old(u8 * Array, u16 Len)
{
    HAL_UART_Transmit(&huart1, Array, Len, 100);
}
/******************************************************************************
* @fn Send_Data
*
* @param float num, uint8_t NUM_Type
*
* @return None.
*
* @note  向上位机发送参数   float num 数值    uint8_t NUM_Type  所发数据类型
*/
void Send_Data_Old(float num, uint8_t NUM_Type)
{
    int32_t Num;
    Num = num * 1024;
    if (Num < 0)
    {
        Num = 0 - Num;
        SEND_BUF_OLD[3] = 0x00;
    }
    else
    {
        SEND_BUF_OLD[3] = 0x01;
    }
    switch (NUM_Type)
    {
        case Send_Speed:
            SEND_BUF_OLD[4] = 0xC0;
            break;
        case Send_Position_X:
            SEND_BUF_OLD[4] = 0xC1;
            break;
        case Send_Position_Y:
            SEND_BUF_OLD[4] = 0xC2;
            break;
        case Send_Pitch:
            SEND_BUF_OLD[4] = 0xC3;
            break;
        case Send_Roll:
            SEND_BUF_OLD[4] = 0xC4;
            break;
        case Send_CPUProcesse:
            SEND_BUF_OLD[4] = 0xD5;
            break;
        case Send_FreeHeap:
            SEND_BUF_OLD[4] = 0xD6;
            break;
        case Send_Yaw:
            SEND_BUF_OLD[4] = 0xC5;
            break;
        case Send_Number1:
            SEND_BUF_OLD[4] = 0xC6;
            break;
        case Send_Number2:
            SEND_BUF_OLD[4] = 0xC7;
            break;
        case Send_Number3:
            SEND_BUF_OLD[4] = 0xC8;
            break;
        case Send_Number4:
            SEND_BUF_OLD[4] = 0xC9;
            break;
        case Send_Number5:
            SEND_BUF_OLD[4] = 0xCA;
            break;
        case Send_Number6:
            SEND_BUF_OLD[4] = 0xCB;
            break;
        case Send_FailNum:
            SEND_BUF_OLD[4] = 0xD7;
            break;
        case Send_CatchTheTopSuccess:
            SEND_BUF_OLD[4] = 0xD8;
            break;
        case Send_CatchTheTopFail:
            SEND_BUF_OLD[4] = 0xD9;
            break;
    }
    SEND_BUF_OLD[7] = Num & 0xff;
    SEND_BUF_OLD[6] = (Num >> 8)&0xff;
    SEND_BUF_OLD[5] = (Num >> 16)&0xff;

    Send_Array_Old(SEND_BUF_OLD, 9);
}
/******************************************************************************
* @fn USART3_IRQHandler
*
* @brief 蓝牙中断
*
* @return None.
*
* @note 接收上位机接收数据
*/
u8 USART_Old[9];
/***********测试数据  自行替换**********/
float p1, i1, d1, p2, i2, d2, p3, i3, d3;
float p4, i4, d4, p5, i5, d5, p6, i6, d6;
float p7, i7, d7;
float a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12;
float m1, n1, m2, n2, m3, n3, m4, n4, m5, n5, m6, n6, m7, n7;
float MW1, MW2, MW3, MW4, SA, SS;
float GP, GY, GH, AS, SS1, VS, AR, A, GR;
int go = 0;

/*****************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static u8 i = 3;
    s32 usart_temp;
        if (i == 3)
        {
            USART_Old[0] = USART_Old[1];
            USART_Old[1] = USART_Old[2];
            USART_Old[2] = USART_Old[3];
        }
         *(USART_Old + i) = USART1->DR & (uint16_t)0x00FF;
        HAL_UART_RxCpltCallback2(USART_Old[i]);/*同时兼容第二套协议*/
        if (USART_Old[0] == 0x01 && USART_Old[1] == 0x02 && USART_Old[2] == 0x03
            && USART_Old[3] == 0x04)
        {
            i++;
        }
        if (i == 9)
        {
            switch (USART_Old[4])
            {
                case 0x01:
                    usart_temp = (USART_Old[5] << 16) + (USART_Old[6] << 8) + USART_Old[7];
                    break;
                case 0x00:
                    usart_temp = -((USART_Old[5] << 16) + (USART_Old[6] << 8) + USART_Old[7]);
                    break;
            }
            switch (USART_Old[8])
            {
				/*************复位***************/
				case Rec_Reset:
					NVIC_SystemReset();
					break;
                /**************PID**************/
                case Rec_PID1_P:
                    PidPitchRate.Kp = (float)usart_temp / 1024;
                PidRollRate.Kp = (float)usart_temp / 1024;
                    break;

                case Rec_PID1_I:
                    PidPitchRate.Ki = (float)usart_temp / 1024;
                PidRollRate.Ki = (float)usart_temp / 1024;
                    break;

                case Rec_PID1_D:
                    PidPitchRate.Kd = (float)usart_temp / 1024;
                PidRollRate.Kd = (float)usart_temp / 1024;
                    break;

                case Rec_PID2_P:
                    PidPitchAngle.Kp = (float)usart_temp / 1024;
                PidRollAngle.Kp = (float)usart_temp / 1024;
                    break;

                case Rec_PID2_I:
                    PidPitchAngle.Ki = (float)usart_temp / 1024;
                PidRollAngle.Ki = (float)usart_temp / 1024;
                    break;

                case Rec_PID2_D:
                    PidPitchAngle.Kd = (float)usart_temp / 1024;
                PidRollAngle.Kd = (float)usart_temp / 1024;
                    break;

                case Rec_PID3_P:
                    PidYawRate.Kp = (float)usart_temp / 1024;
                    break;

                case Rec_PID3_I:
                    PidYawRate.Ki = (float)usart_temp / 1024;
                    break;

                case Rec_PID3_D:
                    PidYawRate.Kd = (float)usart_temp / 1024;
                    break;

                case Rec_PID4_P:
                    PidHeight.Kp = (float)usart_temp / 1024;
                    break;

                case Rec_PID4_I:
                    PidHeight.Ki = (float)usart_temp / 1024;
                    break;

                case Rec_PID4_D:
                    PidHeight.Kd = (float)usart_temp / 1024;
                    break;

                case Rec_PID5_P:
                    p5 = (float)usart_temp / 1024;
                    break;

                case Rec_PID5_I:
                    i5 = (float)usart_temp / 1024;
                    break;

                case Rec_PID5_D:
                    d5 = (float)usart_temp / 1024;
                    break;

                case Rec_PID6_P:
                    p6 = (float)usart_temp / 1024;
                    break;

                case Rec_PID6_I:
                    i6 = (float)usart_temp / 1024;
                    break;

                case Rec_PID6_D:
                    d6 = (float)usart_temp / 1024;
                    break;

                case Rec_PID7_P:
                    p7 = (float)usart_temp / 1024;
                    break;

                case Rec_PID7_I:
                    i7 = (float)usart_temp / 1024;
                    break;

                case Rec_PID7_D:
                    d7 = (float)usart_temp / 1024;
                    break;
                /***************PID Range**************/
                case Rec_PID1_PIDRange_Max:

                    m1 = usart_temp;
                    break;

                case Rec_PID1_PIDRange_Min:
                    n1 = usart_temp;
                    break;

                case Rec_PID2_PIDRange_Max:
                    m2 = usart_temp;
                    break;

                case Rec_PID2_PIDRange_Min:
                    n2 = usart_temp;
                    break;

                case Rec_PID3_PIDRange_Max:
                    m3 = usart_temp;
                    break;

                case Rec_PID3_PIDRange_Min:
                    n3 = usart_temp;
                    break;

                case Rec_PID4_PIDRange_Max:
                    m4 = usart_temp;
                    break;

                case Rec_PID4_PIDRange_Min:
                    n4 = usart_temp;
                    break;

                case Rec_PID5_PIDRange_Max:
                    m5 = usart_temp;
                    break;

                case Rec_PID5_PIDRange_Min:
                    n5 = usart_temp;
                    break;

                case Rec_PID6_PIDRange_Max:
                    m6 = usart_temp;
                    break;

                case Rec_PID6_PIDRange_Min:
                    n6 = usart_temp;
                    break;

                case Rec_PID7_PIDRange_Max:
                    m7 = usart_temp;
                    break;

                case Rec_PID7_PIDRange_Min:
                    n7 = usart_temp;
                    break;
                /***************刹车 启动****************/
                case Rec_Go:
                    FlyMode = TAKEOFFING;
                    break;

                case Rec_Break:
                    MotorOutput = 0;
                    FlyMode = STANDBY;
                    break;
                /**************云台****************/
                case Rec_MaxCurrent1:
                    a1 = (float)usart_temp/1024;
                    break;

                case Rec_MinCurrent1:
                    a2 = (float)usart_temp/1024;
                    break;

                case Rec_MaxCurrent2:
                    a3 = (float)usart_temp/1024;
                    break;

                case Rec_MinCurrent2:
                    a4 = (float)usart_temp/1024;
                    break;

                case Rec_MaxPitch1:
                    a5 = (float)usart_temp/1024;
                    break;

                case Rec_MinPitch1:
                    a6 = (float)usart_temp/1024;
                    break;

                case Rec_MaxYaw1:
                    a7 = usart_temp;
                    break;

                case Rec_MinYaw1:
                    a8 = usart_temp;
                    break;

                case Rec_MaxPitch2:
                    a9 = usart_temp;
                    break;

                case Rec_MinPitch2:
                    a10 = usart_temp;
                    break;

                case Rec_MaxYaw2:
                    a11 = usart_temp;
                    break;

                case Rec_MinYaw2:
                    w_z_baro=(float)usart_temp/1024;
                    w_z_acc=10/w_z_baro;
                    break;

                /***************方向键设zhi 上下左右 Oled***********/
                case Rec_Up:
                    //KeyUpCounter++;
                    break;

                case Rec_Down:
                    //KeyDownCounter++;
                    break;

                case Rec_Left:
                    //KeyLeftCounter++;
                    break;

                case Rec_Right:
                    //KeyRightCounter++;
                    break;
                /********************遥控 前后左右********************/
                case Rec_GoForward:

                    break;

                case Rec_GoBack:

                    break;

                case Rec_GoRight:

                    break;

                case Rec_GoLeft:

                    break;

                /**************底盘数据***************/

                case Rec_MWSpeed1:
                    MW1 = (float)usart_temp / 1024;
                    break;

                case Rec_MWSpeed2:
                    MW2 = (float)usart_temp / 1024;
                    break;

                case Rec_MWSpeed3:
                    MW3 = (float)usart_temp / 1024;
                    break;

                case Rec_MWSpeed4:
                    MW4 = (float)usart_temp / 1024;
                    break;

                case Rec_Set_Anlge:
                    SA = (float)usart_temp / 1024;
                    break;

                case Rec_Set_Speed:
                    SS = (float)usart_temp / 1024;
                    break;

                /**************自动车数据***************/

                case Rec_GuardPitch :
                    PitchError = (float)usart_temp / 1024;
                    break;

                case Rec_GuardRoll :
                    RollError = (float)usart_temp / 1024;
                    break;

                case Rec_GuardYaw :
                    YawError = (float)usart_temp / 1024;
                    break;

                case Rec_GuardHeight :
                    GH = (float)usart_temp / 1024;
                    break;

                case Rec_AdvanceSpeed :
                    AS = (float)usart_temp / 1024;
                    break;

                case Rec_SideSpeed:
                    SS1 = (float)usart_temp / 1024;
                    break;

                case Rec_VerticalSpeed :
                    VS = (float)usart_temp / 1024;
                    break;

                case Rec_AcceleratorRange:
                    MotorBase = usart_temp;
                    break;

                case Rec_Accelerator :
                    MotorBase = usart_temp;
					//Shoot_Speed=usart_temp;
                    break;
				
				/*如果接收到此指令则追踪指令 200ms接收一次*/
				case Rec_GuardSafe:
				//	k++;
					break;
                    
            }
            i = 3;
        }
}
u8 SEND_BUF_NEW[8] = {0xAA, 0x85, 0, 0, 0, 0, 0, 0};
/**
  * @brief  发参数  内部调用
  * @param  None
  * @retval None
  */
u32 Send_num = 0;
extern osTimerId TxTimerHandle;
extern osMessageQId UsartQueueHandle;
/******************************************************************************
* @fn Send_Data
*
* @param float num, uint8_t NUM_Type
*
* @return None.
*
* @note  向上位机发送参数   float num 数值    uint8_t NUM_Type  所发数据类型
*/
void Send_Data_New(TransmitValueStruct* num)
{
    s32 Num=num->Num.int_form;
    if (Num < 0)
    {      
        num->Num.int_form=-num->Num.int_form;
        SEND_BUF_NEW[3] = 0x00;
    }
    else
    {
        SEND_BUF_NEW[3] = 0x01;
    }
    SEND_BUF_NEW[2]=(u8)num->TransmitDataType;/*数据类型*/
//    SEND_BUF_NEW[3]=(u8)num->ValueType;/*整形or浮点*/
    SEND_BUF_NEW[4] = num->Num.char_form[3] ;
    SEND_BUF_NEW[5] = num->Num.char_form[2] ;
    SEND_BUF_NEW[6] = num->Num.char_form[1] ;
    SEND_BUF_NEW[7] = num->Num.char_form[0] ;

//    HAL_UART_Transmit(&huart1, SEND_BUF_NEW, 8, 100);

    if ( UsartQueueHandle != 0 )
    {
        // Send a pointer to a struct CANMessageDef object.  Don't block if the
        // queue is already full.
        xQueueSend( UsartQueueHandle, (void *)SEND_BUF_NEW, ( TickType_t ) 0 );
        return;// pdTRUE;
    }
    else
        return;// pdFALSE;
}
void SendBattery(void)
{
    TransmitValueStruct send_buf={0,0,100,VALUE_TYPE_FLOAT,BATTERY_PERCENTAGE};
    send_buf.Num.int_form= BatteryPercentage*1000;
    Send_Data_New(&send_buf);
}
void SendHeight(void)
{
    TransmitValueStruct send_buf={0,-10,10,VALUE_TYPE_FLOAT,HEIGHT_PRESENT};
    send_buf.Num.int_form = -nav.z*1000;
    Send_Data_New(&send_buf);
}
void SendTemperature(void)
{
    TransmitValueStruct send_buf={0,-20,80,VALUE_TYPE_FLOAT,BOARD_TEMPERTURE};
    send_buf.Num.int_form = bmp280_output.actual_temp_s32*10;
    Send_Data_New(&send_buf);
}
void SendPitch(void)
{
    TransmitValueStruct send_buf={0,-9999,9999,VALUE_TYPE_FLOAT,PITCH_PRESENT};
    send_buf.Num.int_form = imu.pitch*1000;
    Send_Data_New(&send_buf);
}

void SendRoll(void)
{
    TransmitValueStruct send_buf={0,-9999,9999,VALUE_TYPE_FLOAT,ROLL_PRESENT};
    send_buf.Num.int_form = imu.yaw*1000;
    Send_Data_New(&send_buf);
}
void SendYaw(void)
{
    TransmitValueStruct send_buf={0,-9999,9999,VALUE_TYPE_FLOAT,YAW_PRESENT};
    send_buf.Num.int_form = imu.roll*1000;
    Send_Data_New(&send_buf);
}
void SendFirmwareVersion(void)
{
    TransmitValueStruct send_buf={0,0,99,VALUE_TYPE_FLOAT,FIRMWARE_VERSION_PRESENT};
    send_buf.Num.int_form = FIRMWARE_VERSION*10;
    Send_Data_New(&send_buf);
}
//void Send_Driver_Info_Usart(void)
//{
//    ValueStruct send_buf={0,0,0,VALUE_TYPE_INTEGER,DRIVE_ERROR};
//    send_buf.Num.char_form[0] = (u8)ErrorStatusFlag;
//    send_buf.Num.char_form[1] = (u8)CanStatusFlag;
//    send_buf.Num.char_form[2] = (u8)SelfCanID;
//    send_buf.Num.char_form[3] = (u8)OverCurrentStatus;
//    Send_Data_New(&send_buf);
//}



/******************************************************************************
* @fn USART3_IRQHandler
*
* @brief 蓝牙中断
*
* @return None.
*
* @note 接收上位机接收数据
*/
u8 USART_New[8];
u32 usart_counter,usart_frame_counter,UsartFramRcvTime;
ReceiveDataType RcvDataType;
/*****************************/
void HAL_UART_RxCpltCallback2(u8 data)
{
    static u8 i = 2;
    union float_trans usart_temp;
    usart_counter++;
//        USART_ClearFlag(USART2, USART_FLAG_RXNE);
        if (i == 2)
        {
            USART_New[0] = USART_New[1];
            USART_New[1] = USART_New[2];
        }
        *(USART_New + i) = data;
        //HAL_UART_Receive_IT(&huart1,rcv_data,1);//开启下一次接收中断
        if (USART_New[0] == 0xAA && USART_New[1] == 0x85)
        {
            i++;
        }
        if (i == 8)
        {
            usart_frame_counter++;
            
            usart_temp.char_form[3]=USART_New[4];
            usart_temp.char_form[2]=USART_New[5];
            usart_temp.char_form[1]=USART_New[6];
            usart_temp.char_form[0]=USART_New[7];            
            if (USART_New[3]==(u8)VALUE_TYPE_INTEGER)
            {
                RcvDataType=(ReceiveDataType)USART_New[2];
                switch (USART_New[2])
                {
                    case FLY_CHECK:
                        imu.ready=0;
                        break;
                    case UP:
                        
                        MotorBaseK-=15;
                        break;
                    case DOWN:
                        MotorBaseK+=15;
                        break;
                    case STATUS_LED_SW:
                        StatusLedOn=usart_temp.int_form;
                        break;
                    case BIG_LED_SW:
                        BigLed(usart_temp.int_form);
                        break;
                    case OBSTACKLE_SW:
                        ObstacleOn=usart_temp.int_form;
                        break;
                    case MODE1:
                        ;
                        break;
                    case MODE2:
                        ;
                        break;
                    case MODE3:
                        ;
                        break;
                    case RESET_COMMAND:
                        NVIC_SystemReset();
                        break;
                    case SAVE_COMMAND:
                        break;
                }
            }
            else if (USART_New[3]==(u8)VALUE_TYPE_FLOAT)
            {
                RcvDataType=(ReceiveDataType)USART_New[2];
                switch (RcvDataType)
                {
                    case PITCH_SET:
                        
                        PitchSet = (float)usart_temp.int_form/1000;
                        UsartFramRcvTime=HAL_GetTick();
                        break;
                    case YAW_SET:
                        
                        YawSet = -(float)usart_temp.int_form/1000;
                        UsartFramRcvTime=HAL_GetTick();
                        break;
                    case ROLL_SET:
                        
                        RollSet = (float)usart_temp.int_form/1000;
                        UsartFramRcvTime=HAL_GetTick();
                        break;
                }
            }

            i = 2;
        }
}


