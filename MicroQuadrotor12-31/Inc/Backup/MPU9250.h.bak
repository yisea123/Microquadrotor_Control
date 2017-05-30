
/**
  ******************************************************************************
  * @file    includes.h
  * @author  
  * @version V1.0
  * @date    2015.4.2
  * @note
  * @history    V1.0 2015.4.2
  *                 include file of main.
  ******************************************************************************
  */
#ifndef __MPU9250_H
#define __MPU9250_H
/*driver incluede*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

			//定义MPU6050内部地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样率 典型值 0X07 125Hz
#define	CONFIG			0x1A	//低通滤波频率 典型值 0x00 
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围                 典型值 0x18 不自检 2000deg/s
#define	ACCEL_CONFIG	0x1C	//加速度计自检及测量范围及高通滤波频率 典型值 0x01 不自检 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    //只读


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C

#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	

#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理 典型值 0x00 正常启用
#define	WHO_AM_I		0x75	//只读  默认读出应该是 MPU6050_ID = 0x68


#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //由于数据存放地址是连续的，所以一并读出


/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       (0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       (0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       (0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      (0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   (0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   (0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  (0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  (0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   (0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   (0.002995177763f) // 0.002995177763 degC/LSB

/* ---- MPU6500 Reg In MPU9250 ---------------------------------------------- */
#define MPU6500_I2C_SLVx_EN         ((uint8_t)0x80)
#define MPU6500_I2C_SLV4_DONE       ((uint8_t)0x40)
#define MPU6500_I2C_SLV4_NACK       ((uint8_t)0x10)

#define MPU6500_I2C_ADDR            ((u8)0xD0)
#define MPU6500_Device_ID           ((u8)0x71)  // In MPU9250

#define MPU6500_SELF_TEST_XG        ((u8)0x00)
#define MPU6500_SELF_TEST_YG        ((u8)0x01)
#define MPU6500_SELF_TEST_ZG        ((u8)0x02)
#define MPU6500_SELF_TEST_XA        ((u8)0x0D)
#define MPU6500_SELF_TEST_YA        ((u8)0x0E)
#define MPU6500_SELF_TEST_ZA        ((u8)0x0F)
#define MPU6500_XG_OFFSET_H         ((u8)0x13)
#define MPU6500_XG_OFFSET_L         ((u8)0x14)
#define MPU6500_YG_OFFSET_H         ((u8)0x15)
#define MPU6500_YG_OFFSET_L         ((u8)0x16)
#define MPU6500_ZG_OFFSET_H         ((u8)0x17)
#define MPU6500_ZG_OFFSET_L         ((u8)0x18)
#define MPU6500_SMPLRT_DIV          ((u8)0x19)
#define MPU6500_CONFIG              ((u8)0x1A)
#define MPU6500_GYRO_CONFIG         ((u8)0x1B)
#define MPU6500_ACCEL_CONFIG        ((u8)0x1C)
#define MPU6500_ACCEL_CONFIG_2      ((u8)0x1D)
#define MPU6500_LP_ACCEL_ODR        ((u8)0x1E)
#define MPU6500_MOT_THR             ((u8)0x1F)
#define MPU6500_FIFO_EN             ((u8)0x23)
#define MPU6500_I2C_MST_CTRL        ((u8)0x24)
#define MPU6500_I2C_SLV0_ADDR       ((u8)0x25)
#define MPU6500_I2C_SLV0_REG        ((u8)0x26)
#define MPU6500_I2C_SLV0_CTRL       ((u8)0x27)
#define MPU6500_I2C_SLV1_ADDR       ((u8)0x28)
#define MPU6500_I2C_SLV1_REG        ((u8)0x29)
#define MPU6500_I2C_SLV1_CTRL       ((u8)0x2A)
#define MPU6500_I2C_SLV2_ADDR       ((u8)0x2B)
#define MPU6500_I2C_SLV2_REG        ((u8)0x2C)
#define MPU6500_I2C_SLV2_CTRL       ((u8)0x2D)
#define MPU6500_I2C_SLV3_ADDR       ((u8)0x2E)
#define MPU6500_I2C_SLV3_REG        ((u8)0x2F)
#define MPU6500_I2C_SLV3_CTRL       ((u8)0x30)
#define MPU6500_I2C_SLV4_ADDR       ((u8)0x31)
#define MPU6500_I2C_SLV4_REG        ((u8)0x32)
#define MPU6500_I2C_SLV4_DO         ((u8)0x33)
#define MPU6500_I2C_SLV4_CTRL       ((u8)0x34)
#define MPU6500_I2C_SLV4_DI         ((u8)0x35)
#define MPU6500_I2C_MST_STATUS      ((u8)0x36)
#define MPU6500_INT_PIN_CFG         ((u8)0x37)
#define MPU6500_INT_ENABLE          ((u8)0x38)
#define MPU6500_INT_STATUS          ((u8)0x3A)
#define MPU6500_ACCEL_XOUT_H        ((u8)0x3B)
#define MPU6500_ACCEL_XOUT_L        ((u8)0x3C)
#define MPU6500_ACCEL_YOUT_H        ((u8)0x3D)
#define MPU6500_ACCEL_YOUT_L        ((u8)0x3E)
#define MPU6500_ACCEL_ZOUT_H        ((u8)0x3F)
#define MPU6500_ACCEL_ZOUT_L        ((u8)0x40)
#define MPU6500_TEMP_OUT_H          ((u8)0x41)
#define MPU6500_TEMP_OUT_L          ((u8)0x42)
#define MPU6500_GYRO_XOUT_H         ((u8)0x43)
#define MPU6500_GYRO_XOUT_L         ((u8)0x44)
#define MPU6500_GYRO_YOUT_H         ((u8)0x45)
#define MPU6500_GYRO_YOUT_L         ((u8)0x46)
#define MPU6500_GYRO_ZOUT_H         ((u8)0x47)
#define MPU6500_GYRO_ZOUT_L         ((u8)0x48)
#define MPU6500_EXT_SENS_DATA_00    ((u8)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((u8)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((u8)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((u8)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((u8)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((u8)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((u8)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((u8)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((u8)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((u8)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((u8)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((u8)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((u8)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((u8)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((u8)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((u8)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((u8)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((u8)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((u8)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((u8)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((u8)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((u8)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((u8)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((u8)0x60)
#define MPU6500_I2C_SLV0_DO         ((u8)0x63)
#define MPU6500_I2C_SLV1_DO         ((u8)0x64)
#define MPU6500_I2C_SLV2_DO         ((u8)0x65)
#define MPU6500_I2C_SLV3_DO         ((u8)0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  ((u8)0x67)
#define MPU6500_SIGNAL_PATH_RESET   ((u8)0x68)
#define MPU6500_MOT_DETECT_CTRL     ((u8)0x69)
#define MPU6500_USER_CTRL           ((u8)0x6A)
#define MPU6500_PWR_MGMT_1          ((u8)0x6B)
#define MPU6500_PWR_MGMT_2          ((u8)0x6C)
#define MPU6500_FIFO_COUNTH         ((u8)0x72)
#define MPU6500_FIFO_COUNTL         ((u8)0x73)
#define MPU6500_FIFO_R_W            ((u8)0x74)
#define MPU6500_WHO_AM_I            ((u8)0x75)	// ID = 0x71 In MPU9250
#define MPU6500_XA_OFFSET_H         ((u8)0x77)
#define MPU6500_XA_OFFSET_L         ((u8)0x78)
#define MPU6500_YA_OFFSET_H         ((u8)0x7A)
#define MPU6500_YA_OFFSET_L         ((u8)0x7B)
#define MPU6500_ZA_OFFSET_H         ((u8)0x7D)
#define MPU6500_ZA_OFFSET_L         ((u8)0x7E)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((u8)0x18)
#define AK8963_Device_ID            ((u8)0x48)

// Read-only Reg
#define AK8963_WIA                  ((u8)0x00)
#define AK8963_INFO                 ((u8)0x01)
#define AK8963_ST1                  ((u8)0x02)
#define AK8963_HXL                  ((u8)0x03)
#define AK8963_HXH                  ((u8)0x04)
#define AK8963_HYL                  ((u8)0x05)
#define AK8963_HYH                  ((u8)0x06)
#define AK8963_HZL                  ((u8)0x07)
#define AK8963_HZH                  ((u8)0x08)
#define AK8963_ST2                  ((u8)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((u8)0x0A)
#define AK8963_CNTL2                ((u8)0x0B)
#define AK8963_ASTC                 ((u8)0x0C)
#define AK8963_TS1                  ((u8)0x0D)
#define AK8963_TS2                  ((u8)0x0E)
#define AK8963_I2CDIS               ((u8)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((u8)0x10)
#define AK8963_ASAY                 ((u8)0x11)
#define AK8963_ASAZ                 ((u8)0x12)
#define AK8963_STATUS_DRDY          ((uint8_t)0x01)
#define AK8963_STATUS_DOR           ((uint8_t)0x02)
#define AK8963_STATUS_HOFL          ((uint8_t)0x08)
typedef struct MPU9250_Rea_Data
{
    float Accel_X;  //寄存器原生X轴加速度表示值
    float Accel_Y;  //寄存器原生Y轴加速度表示值
    float Accel_Z;  //寄存器原生Z轴加速度表示值
    float Temp;     //寄存器原生温度表示值
    float Gyro_X;   //寄存器原生X轴陀螺仪表示值
    float Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    float Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	float Mag_X;
	float Mag_Y;
	float Mag_Z;
}MPU9250_Rea_Data;

typedef struct MPU9250_Raw_Data
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	short Mag_X;
	short Mag_Y;
	short Mag_Z;
}MPU9250_Raw_Data;

typedef struct kal_fir
{
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/
	float Q;
	float R;
		
	float K;						/*卡尔曼增益，列矩阵*/
	float X;						/*最优估计输出矩阵，列矩阵*/
	float C;						/*最优估计协方差矩阵C(k|k)*/
         
	float input;				    /*量测值，即Z(k)*/
}kal_fir;

typedef struct First_order_filter
{
	float a;								//x[n]系数
	float current_output;		            //y[n]
	float last_output;			            //y[n-1]
}
first_filter;

extern MPU9250_Raw_Data  g_Raw_Data;

//kal_fir g_Kal_Fir;
//first_filter LowCroyaw_filter,LowCropitch_filter;
//float kalman_fir(float input,kal_fir* fir_param);
//float first_order_filter(first_filter* first_struct, float input);
#endif  //__MPU9250_H

