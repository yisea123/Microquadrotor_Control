
#include "bsp.h"
#include "module_mpu9250.h"
#include "mpu9250.h"
#include "stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi2;
/*====================================================================================================*/

u8 SPI_RW( SPI_TypeDef* SPI, u8 WriteByte )
{
    while ((SPIx->SR & SPI_FLAG_TXE) == (u16)RESET);
    SPIx->DR = WriteByte;
    while ((SPIx->SR & SPI_FLAG_RXNE) == (u16)RESET);

    return SPIx->DR;
}

/*====================================================================================================*
**函數 : MPU9250_ReadReg
**功能 : 讀暫存器
**輸入 : ReadAddr
**輸出 : ReadData
**使用 : MPU9250_ReadReg(ReadAddr, &DeviceID);
**====================================================================================================*/
static void MPU9250_ReadReg( u8 ReadAddr, u8 *ReadData )
{
    u8 data;
    data = 0x80 | ReadAddr;
    IMU_CSM = 0;
    HAL_SPI_Transmit(&hspi2,&data,1,1000);
    HAL_SPI_Receive(&hspi2,ReadData,1,1000);
//    HAL_SPI_TransmitReceive(&hspi2, &data, ReadData, 1, 1000);
    IMU_CSM = 1;
//    IMU_CSM = 0;
//    SPI_RW(SPIx, 0x80 | ReadAddr);
//    *ReadData = SPI_RW(SPIx, 0xFF);
//    IMU_CSM = 1;

}

/*====================================================================================================*
**函數 : MPU9250_WriteReg
**功能 : 寫暫存器
**輸入 : WriteAddr, WriteData
**輸出 : None
**使用 : MPU9250_WriteReg(WriteAddr, WriteData);
**====================================================================================================*/

static void MPU9250_WriteReg( u8 WriteAddr, u8 WriteData )
{
    u8 data[2];
    data[0] = WriteAddr;
    data[1] = WriteData;
    IMU_CSM = 0;
//    HAL_SPI_Transmit(&hspi2, &WriteAddr, 1, 1000);
    HAL_SPI_Transmit(&hspi2, data, 2, 1000);
    IMU_CSM = 1;
//    IMU_CSM = 0;
//    SPI_RW(SPIx, WriteAddr);
//    SPI_RW(SPIx, WriteData);
//    IMU_CSM = 1;
}

/*=====================================================================================================*
**函數 : MPU9250_ReadRegs
**功能 : 連續讀暫存器
**輸入 : ReadAddr, *ReadBuf, Bytes
**輸出 : None
**使用 : MPU9250_ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, 14);
**=====================================================================================================*/

static void MPU9250_ReadRegs( u8 ReadAddr, u8 *ReadBuf, u8 Bytes )
{
    u8 data;
    data = 0x80 | ReadAddr;
    IMU_CSM = 0;
    HAL_SPI_Transmit(&hspi2, &data, 1, 1000);
    HAL_SPI_Receive(&hspi2, ReadBuf, Bytes, 1000);
    IMU_CSM = 1;
//    u8 i = 0;
//    IMU_CSM = 0;
//    SPI_RW(SPIx, 0x80 | ReadAddr);
//    for (i = 0; i < Bytes; i++)
//        ReadBuf[i] = SPI_RW(SPIx, 0xFF);
//    IMU_CSM = 1;
}

void MPU9250_Mag_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
  uint8_t  status = 0;
  uint32_t timeout = 256;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
  vTaskDelay(10);

  do {
			MPU9250_ReadReg(MPU6500_I2C_MST_STATUS,&status);
    vTaskDelay(10);
  } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
}
void MPU9250_Mag_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  uint8_t  status = 0;
  uint32_t timeout = 256;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  vTaskDelay(10);
  for(uint8_t i = 0; i < lens; i++) {
    MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr + i);
    vTaskDelay(10);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData[i]);
    vTaskDelay(10);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    vTaskDelay(10);
    do {
      MPU9250_ReadReg(MPU6500_I2C_MST_STATUS,&status);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
  }
}
uint8_t MPU9250_Mag_ReadReg( uint8_t readAddr )
{
  uint8_t status = 0;
  uint8_t readData = 0;
  uint32_t timeout = 256;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
  vTaskDelay(10);;

  do {
    MPU9250_ReadReg(MPU6500_I2C_MST_STATUS,&status);
    vTaskDelay(10);;
  } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

   MPU9250_ReadReg(MPU6500_I2C_SLV4_DI,&readData);

  return readData;
}
void MPU9250_Mag_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t status = 0;
  uint32_t timeout = 256;

  MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  vTaskDelay(10);
  for(uint8_t i = 0; i< lens; i++) {
    MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr + i);
    vTaskDelay(10);
    MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    vTaskDelay(10);

    do {
      MPU9250_ReadReg(MPU6500_I2C_MST_STATUS,&status);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

    MPU9250_ReadReg(MPU6500_I2C_SLV4_DI,readData+i);
    vTaskDelay(10);
  }
}
/*====================================================================================================*
**函數 : MPU9250_Init
**功能 : 初始化 MPU9250
**輸入 : None
**輸出 : None
**使用 : MPU9250_Init();
**====================================================================================================*/
HAL_StatusTypeDef res;
#define MPU9250_InitRegNum 11
void MPU9250_Init( void )
{
    u8 i = 0;
    u8 MPU6500_Init_Data[MPU9250_InitRegNum][2] =
    {
        {0x80, MPU6500_PWR_MGMT_1},     // Reset Device
        {0x01, MPU6500_PWR_MGMT_1},     // Clock Source
        {0x00, MPU6500_PWR_MGMT_2},     // Enable Acc & Gyro
        {MPU_LPS_42Hz, MPU6500_CONFIG},         // 44hz滤波
        {MPU_GyrFS, MPU6500_GYRO_CONFIG},    // +-2000dps
        {MPU_AccFS, MPU6500_ACCEL_CONFIG},   // +-4G
        {0x00, MPU6500_ACCEL_CONFIG_2}, // Set Acc Data Rates
        {0x30, MPU6500_INT_PIN_CFG},    //
//			{0x01, MPU6500_INT_ENABLE},     // Set RAW_RDY_EN
        {0x5D, MPU6500_I2C_MST_CTRL},   // I2C Speed 500 kHz
        {0x30, MPU6500_USER_CTRL},      // Enable AUX
        //Enable the MAG:
//        {0x8C,MPU6500_I2C_SLV0_ADDR},     //七位地址b000_1100
//        {AK8963_I2C_ADDR>>1,MPU6500_I2C_SLV0_ADDR},     //七位地址b000_1100
//        {AK8963_WIA,MPU6500_I2C_SLV0_REG},//读取AK8963的WIA
//        {0x81,MPU6500_I2C_SLV0_CTRL},     //开启从I2C读取
//        {0x01,MPU6500_I2C_MST_DELAY_CTRL}
    };

    for (i = 0; i < MPU9250_InitRegNum; i++)
    {

        //res=HAL_SPI_Transmit(&hspi2,&(MPU6500_Init_Data[i][0]),2,1000);
        MPU9250_WriteReg(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
        vTaskDelay(10);
    }
    compass_config();
    vTaskDelay(100);
    MPU9250_Check();
}

/****************************************************************************
函数：check_compass
功能：测试compass
****************************************************************************/

void check_compass(void)
{
    u8 data = 0;
    MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);
    MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_WIA);
    MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);

    MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00, &data);
//printf("\r\nthe id is %x\r\n",data);
//  if(data == AK8963_Device_ID)
//      printf("\r\nAK8963 init success\r\n");
//  else
//      printf("\r\nAK8963 init failed\r\n");
}
u8 ASA[3];
void compass_config(void)
{
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
//    vTaskDelay(1);
////  MPU9250_WriteReg(MPU6500_I2C_SLV0_REG,AK8963_WIA);
////    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL2);
//    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, 0x01); //reset compass
//    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x86);
//    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);
//    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, 0x12); //continue read
//    vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x86);
//    vTaskDelay(1);
//    

  MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);       // Reset Device
  vTaskDelay(10);
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
  vTaskDelay(10);
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x1F);       // Fuse ROM access mode
  vTaskDelay(50);
	ASA[0]=MPU9250_Mag_ReadReg(AK8963_ASAX);        //
	ASA[1]=MPU9250_Mag_ReadReg(AK8963_ASAY);
	ASA[2]=MPU9250_Mag_ReadReg(AK8963_ASAZ);
		
  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
  vTaskDelay(10); 
  MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);		
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);  
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
  vTaskDelay(10);
  MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);
  vTaskDelay(10);

  MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // Continuous measurement mode 2
  vTaskDelay(10);

	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09);
	vTaskDelay(10);
	MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81);
	vTaskDelay(10);
}
//void read_compass_data(u8 *ReadBuf)
//{

//    MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
//    //  vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_HXL);
//    //  vTaskDelay(1);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x86);
//    //  vTaskDelay(1);
//    MPU9250_ReadRegs(MPU6500_EXT_SENS_DATA_00, ReadBuf, 6);
//    //  vTaskDelay(1);

//}
//void compass_data_adjustment(u8 *AdjustVaule)   //,u8 *CompassData
//{
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ASAX);
//    MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x83);
//    MPU9250_ReadRegs(MPU6500_EXT_SENS_DATA_00, AdjustVaule, 3);
////  printf("\r\nasax = %d,asay = %d,asaz = %d\r\n",AdjustVaule[0],AdjustVaule[1],AdjustVaule[2]);
//}

/*====================================================================================================*
**函數 : MPU9250_Check
**功能 : MPU9250 Check
**輸入 : None
**輸出 : Status
**使用 : Status = MPU9250_Check();
**====================================================================================================*/

u8 MPU9250_Check( void )
{
    u8 DeviceID;
    u8 res=SUCCESS;
    /* MPU6500 */
    DeviceID = 0x00;
    MPU9250_ReadReg(MPU6500_WHO_AM_I, &DeviceID);
    if (DeviceID != MPU6500_Device_ID)
    {
        res= ERROR;
    }
    DeviceID=MPU9250_Mag_ReadReg(AK8963_WIA);
    if (DeviceID != AK8963_Device_ID)
    {
        res= ERROR;
    }
    return res;
}

/*====================================================================================================*
**函數 : SPI9250_Init
**功能 : SPI9250 Init
**輸入 : None
**輸出 : Status
**使用 : Status = MPU9250_Check();
**====================================================================================================*/
int SPI9250_Init(void)
{

    MPU9250_Init();
    vTaskDelay(100);
    if (MPU9250_Check() == SUCCESS)
        return 1;
    else
        return 0;

}
/*====================================================================================================*
**函數 : MPU9250_Read
**功能 : 讀取感測器資料
**輸入 : *ReadBuf
**輸出 : None
**使用 : MPU9250_Read(ReadBuf);
**====================================================================================================*/

u16 g_fTempe_offset;
void MPU9250_Read(void)
{

    u8 ReadBuf[22];
    MPU9250_ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, 22);
//    read_compass_data(ReadBuf + 14);
    g_Raw_Data.Accel_X  = (s16)((u16)ReadBuf[0] << 8 | (u16)ReadBuf[1]);
    g_Raw_Data.Accel_Y  = (s16)((u16)ReadBuf[2] << 8 | (u16)ReadBuf[3]);
    g_Raw_Data.Accel_Z  = (s16)((u16)ReadBuf[4] << 8 | (u16)ReadBuf[5]);
    g_Raw_Data.Temp     = ((s16)((u16)ReadBuf[6] << 8 | (u16)ReadBuf[7]) -g_fTempe_offset);
    g_Raw_Data.Gyro_X   = (s16)((u16)ReadBuf[8] << 8 | (u16)ReadBuf[9]);
    g_Raw_Data.Gyro_Y   = (s16)((u16)ReadBuf[10] << 8 | (u16)ReadBuf[11]);
    g_Raw_Data.Gyro_Z   = (s16)((u16)ReadBuf[12] << 8 | (u16)ReadBuf[13]);
//	if(!(ReadBuf[14] & AK8963_STATUS_DRDY) || (ReadBuf[14] & AK8963_STATUS_DOR) || (ReadBuf[21] & AK8963_STATUS_HOFL))    //Data is Ready
//    return;
	g_Raw_Data.Mag_X =((s16)((u16)ReadBuf[16] << 8 | (u16)ReadBuf[15]))*((ASA[0]-128)*0.5f/128+1);              //Adjusted Measurement Data
	g_Raw_Data.Mag_Y=((s16)((u16)ReadBuf[18] << 8 | (u16)ReadBuf[17]))*((ASA[1]-128)*0.5f/128+1);
	g_Raw_Data.Mag_Z=((s16)((u16)ReadBuf[20] << 8 | (u16)ReadBuf[19]))*((ASA[2]-128)*0.5f/128+1);

//    g_Raw_Data.Mag_X    = ((s16)((u16)ReadBuf[14] << 8 | (u16)ReadBuf[15]));
//    g_Raw_Data.Mag_X    = ((s16)((u16)ReadBuf[16] << 8 | (u16)ReadBuf[17]));
//    g_Raw_Data.Mag_X    = ((s16)((u16)ReadBuf[18] << 8 | (u16)ReadBuf[19]));

//    g_Rea_Data.Accel_X  = g_Raw_Data.Accel_X * MPU_AccFS_N;//-g_fAccel_offset[0];
//    g_Rea_Data.Accel_Y  = g_Raw_Data.Accel_Y * MPU_AccFS_N;//-g_fAccel_offset[1];
//    g_Rea_Data.Accel_Z  = g_Raw_Data.Accel_Z * MPU_AccFS_N;//-g_fAccel_offset[2];
//    g_Rea_Data.Temp     = g_Raw_Data.Temp * MPU9250T_85degC + 21;
//    g_Rea_Data.Gyro_X   = g_Raw_Data.Gyro_X  * MPU_GyrFS_N;
//    g_Rea_Data.Gyro_Y   = g_Raw_Data.Gyro_Y * MPU_GyrFS_N;
//    g_Rea_Data.Gyro_Z   = g_Raw_Data.Gyro_Z * MPU_GyrFS_N;
//    g_Rea_Data.Mag_X    = g_Raw_Data.Mag_X * MPU_MegFS_N;
//    g_Rea_Data.Mag_X    = g_Raw_Data.Mag_X  * MPU_MegFS_N;
//    g_Rea_Data.Mag_X    = g_Raw_Data.Mag_X  * MPU_MegFS_N;
}

//void Gyro_SPIAdj(void)
//{
//    double temple_gyro[3];
//    double temple_accel[3];
//    double temple_Tempera;
//    u32 i = 1000;
//    while (i--)
//    {
//        MPU9250_Read();
//        temple_accel[0] += g_Raw_Data.Accel_X;
//        temple_accel[1] += g_Raw_Data.Accel_Y;
//        temple_accel[2] += g_Raw_Data.Accel_Z;
//        temple_Tempera += g_Raw_Data.Temp;
//        temple_gyro[0] += g_Raw_Data.Gyro_X;
//        temple_gyro[1] += g_Raw_Data.Gyro_Y;
//        temple_gyro[2] += g_Raw_Data.Gyro_Z;
//    }
//    temple_accel[0] = temple_accel[0] / 1000.0f;
//    temple_accel[1] = temple_accel[1] / 1000.0f;
//    temple_accel[2] = temple_accel[2] / 1000.0f;
//    temple_Tempera = temple_Tempera / 1000.0f;
//    temple_gyro[0] = temple_gyro[0] / 1000.0f;
//    temple_gyro[1] = temple_gyro[1] / 1000.0f;
//    temple_gyro[2] = temple_gyro[2] / 1000.0f;

//    g_fAccel_offset[0] = temple_accel[0];
//    g_fAccel_offset[1] = temple_accel[1];
//    g_fAccel_offset[2] = temple_accel[2];
//    g_fTempe_offset   = temple_Tempera;
//    g_fGyro_offset[0] = temple_gyro[0];
//    g_fGyro_offset[1] = temple_gyro[1];
//    g_fGyro_offset[2] = temple_gyro[2];
//}
