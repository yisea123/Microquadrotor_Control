 /*
****************************************************************************
* Copyright (C) 2014 - 2015 Bosch Sensortec GmbH
*
* bmp280_support.c
* Date: 2015/03/27
* Revision: 1.0.5
*
* Usage: Sensor Driver support file for BMP280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmp280.h"
#include "bsp.h"
#include "module_mpu9250.h"
#include "mpu9250.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "quad_math.h"
#define BMP_CSM   PBO(12)//12
#define BMP280_ONE_U8X  (1u)
#define BMP280_TWO_U8X  (2u)
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMP280_API

float AltGroundPa=0; //���ʱ��Ӧ����ѹֵ
float BMP280_Height=0;
float MPU9250_Height=0;
float MPU9250_Height_Speed=0;
float Height;
void BMP280_Height_Calibration(void)
{
    u32 i;
    u32 PA_OFFSET_INIT_NUM=30;
    double pa_int = 0; 
    
    for(i=0;i < PA_OFFSET_INIT_NUM;i++)
    {
        bmp280_data_readout_template();/*��ȡ�������*/
        pa_int += bmp280_output.actual_press_u32; 
        osDelay(30);
    }
    AltGroundPa = pa_int / PA_OFFSET_INIT_NUM;
    BMP280_Height = 0; //�߶� Ϊ 0
}
float acc_z,acc_g=9.6;
void  BMP280_Get_Height(void)
{
    float BMP280_Height_temp;
    
    bmp280_data_readout_template();/*��ȡ�������*/
	//����������ϵ�ʱ��λ�õĸ߶�ֵ ����λΪm
	BMP280_Height_temp = 4433000.0f * (1 - pow((bmp280_output.actual_press_u32 / AltGroundPa), 0.1903f))*0.01f;
    BMP280_Height=0.9f*BMP280_Height+0.1f*BMP280_Height_temp;
    //	return BMP280_Height; 
    acc_z=imu.accRaw[2]/cos(imu.pitchRad)/cos(imu.rollRad);
    MPU9250_Height_Speed+=(acc_z-acc_g)*0.005f;
    MPU9250_Height+=MPU9250_Height_Speed*0.005f;
    Height=BMP280_Height*0.9+0.1*MPU9250_Height;

}


/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 I2C_routine(void);
s8 SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP280_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp280_data_readout_template(void);
/*----------------------------------------------------------------------------*
 *  struct bmp280_t parameters can be accessed by using bmp280
 *	bmp280_t having the following parameters
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmp280_t bmp280;
struct bmp280_res bmp280_output;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
void BMP280_Init(void)
{
	/* The variable used to assign the standby time*/
	u8 v_standby_time_u8 = BMP280_INIT_VALUE;
	
	/* result of communication results*/
	s32 com_rslt = bmpERROR;
/*********************** START INITIALIZATION ************************/
  /*	Based on the user need configure I2C or SPI interface.
   *	It is example code to explain how to use the bma2x2 API*/
   #ifdef BMP280
	/*I2C_routine();*/
	SPI_routine(); 
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
*-------------------------------------------------------------------------*/
	
    com_rslt = bmp280_init(&bmp280);
    osDelay(100);
	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
    osDelay(100);
	/*	For reading the pressure and temperature data it is required to
	 *	set the work mode
	 *	The measurement period in the Normal mode is depends on the setting of
	 *	over sampling setting of pressure, temperature and standby time
	 *
	 *	OSS				pressure OSS	temperature OSS
	 *	ultra low power			x1			x1
	 *	low power				x2			x1
	 *	standard resolution		x4			x1
	 *	high resolution			x8			x2
	 *	ultra high resolution	x16			x2
	 */
	/* The oversampling settings are set by using the following API*/
    
	com_rslt += bmp280_set_work_mode(BMP280_ULTRA_HIGH_RESOLUTION_MODE);
    osDelay(100);
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	 *	value have to be given*/
	 /*	Normal mode comprises an automated perpetual cycling between an (active)
	 *	Measurement period and an (inactive) standby period.
	 *	The standby time is determined by the contents of the register t_sb.
	 *	Standby time can be set using BMP280_STANDBYTIME_125_MS.
	 *	Usage Hint : BMP280_set_standbydur(BMP280_STANDBYTIME_125_MS)*/

	com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
    osDelay(100);
//	/* This API used to read back the written value of standby time*/
//	com_rslt += bmp280_get_standby_durn(&v_standby_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/
}
s32 bmp280_data_readout_template(void)
{

	s32 com_rslt = bmpERROR;
/*------------------------------------------------------------------*
************ START READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*---------------------------------------------------------------------*/
	/* API is used to read the uncompensated temperature*/
//	com_rslt += bmp280_read_uncomp_temperature(&bmp280_output.ncomp_tem_s32);

//	/* API is used to read the uncompensated pressure*/
//	com_rslt += bmp280_read_uncomp_pressure(&bmp280_output.uncomp_pres_s32);

//	/* API is used to read the uncompensated temperature and pressure*/
//	com_rslt += bmp280_read_uncomp_pressure_temperature(&bmp280_output.actual_press_s32,
//	&bmp280_output.actual_temp_s32);
/*--------------------------------------------------------------------*
************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------*
************ START READ TRUE PRESSURE AND TEMPERATURE********
*---------------------------------------------------------------------*/
//	/* API is used to read the true temperature*/
//	/* Input value as uncompensated temperature*/
//	com_rslt += bmp280_compensate_temperature_int32(bmp280_output.actual_temp_s32);

//	/* API is used to read the true pressure*/
//	/* Input value as uncompensated pressure*/
//	com_rslt += bmp280_compensate_pressure_int32(bmp280_output.actual_press_u32);

	/* API is used to read the true temperature and pressure*/
	/* Input value as uncompensated pressure and temperature*/
	com_rslt += bmp280_read_pressure_temperature(&bmp280_output.actual_press_u32,
	&bmp280_output.actual_temp_s32);
/*--------------------------------------------------------------------*
************ END READ TRUE PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/


/************************* START DE-INITIALIZATION ***********************/

	/*	For de-initialization it is required to set the mode of
	 *	the sensor as "SLEEP"
	 *	the device reaches the lowest power consumption only
	 *	In SLEEP mode no measurements are performed
	 *	All registers are accessible
	 *	by using the below API able to set the power mode as SLEEP*/
	 /* Set the power mode as SLEEP*/
//	com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);

   return com_rslt;
/************************* END DE-INITIALIZATION **********************/
}

#ifdef BMP280_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
//	bmp280.bus_write = BMP280_I2C_bus_write;
//	bmp280.bus_read = BMP280_I2C_bus_read;
//	bmp280.dev_addr = BMP280_I2C_ADDRESS2;
//	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}

/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bmp280_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bmp280.bus_write = BMP280_SPI_bus_write;
	bmp280.bus_read = BMP280_SPI_bus_read;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 24
#define BUFFER_LENGTH	0xFF
#define	MASK_DATA	0x80
#define REGISTER_MASK	0x7F

/*=====================================================================================================*
**���� : BMP280_ReadRegs
**���� : �B�m�x������
**ݔ�� : ReadAddr, *ReadBuf, Bytes
**ݔ�� : None
**ʹ�� : BMP280_ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, 14);
**=====================================================================================================*/

static s8 BMP280_ReadRegs( u8 ReadAddr, u8 *ReadBuf, u8 Bytes )
{
//    u8 i = 0;
    u8 data;
    data = 0x80 | (ReadAddr);
    BMP_CSM = 0;
    HAL_SPI_Transmit(&hspi2, &data, 1, 100);
    HAL_SPI_Receive(&hspi2, ReadBuf, Bytes, 100);
    BMP_CSM = 1;
//    u8 i = 0;
//    BMP_CSM = 0;
//    SPI_RW(SPIx, 0x80 | ReadAddr);
//    for (i = 0; i < Bytes; i++)
//        ReadBuf[i] = SPI_RW(SPIx, 0xFF);
//    BMP_CSM = 1;
    return 0;
}

s8 BMP280_WriteRegs(u8 WriteAddr, u8 Bytes, u8 *WriteData)
{
    BMP_CSM = 0;
    HAL_SPI_Transmit(&hspi2, &WriteAddr, 1, 100);
    HAL_SPI_Transmit(&hspi2, WriteData, Bytes, 100);
    BMP_CSM = 1;
    return 0;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data to be read
 */
s8  BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError=BMP280_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN]={BUFFER_LENGTH};
//	u8 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as BMP280_INIT_VALUE)*/
//	array[BMP280_INIT_VALUE] = reg_addr|MASK_DATA;/*read routine is initiated register address is mask with 0x80*/
	/*
	* Please take the below function as your reference for
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
	* add your SPI read function here
	* iError is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMP280_INIT_VALUE
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the SPI read
	* and write string function
	* For more information please refer data sheet SPI communication:
	*/
    iError = BMP280_ReadRegs(reg_addr, reg_data, cnt);
    //iError = BMP280_ReadRegs( array[BMP280_INIT_VALUE], array+1, cnt);
//	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
//		*(reg_data + stringpos) = array[stringpos+BMP280_ONE_U8X];
//	}
    
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8  BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
//	u8 array[SPI_BUFFER_LEN * BMP280_TWO_U8X];
//	u8 stringpos = BMP280_INIT_VALUE;
    u8 addr;
//	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
//		/* the operation of (reg_addr++)&0x7F done: because it ensure the
//		   BMP280_INIT_VALUE and 1 of the given value
//		   It is done only for 8bit operation*/
////		array[stringpos * BMP280_TWO_U8X] = (reg_addr++) & REGISTER_MASK;
////		array[stringpos * BMP280_TWO_U8X + BMP280_ONE_U8X] = *(reg_data + stringpos);
        addr=reg_addr & REGISTER_MASK;
//        HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
//        reg_addr++;
//        error = HAL_SPI_Transmit(&hspi2, reg_data++, 1, 100);
//        
//	}
	/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as BMP280_INIT_VALUE
     * and FAILURE defined as -1
	 */
    iError = BMP280_WriteRegs(addr,cnt,reg_data);
    
	return (s8)iError;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void  BMP280_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
    osDelay(msek);
}


#endif