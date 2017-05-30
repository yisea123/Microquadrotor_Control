/**
  ******************************************************************************
  * @file       spi.h
  * @author          @UESTC
  * @version    V1.0
  * @date       2015.11.2
  * @note       参考正点原子
  * @history    V1.0 2015.11.2
  *                
  ******************************************************************************
 **/

#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"

#define  uint8_t    u8;                  
#define  uint16_t   u16;                               
#define  uint32_t   u32; 
 	
 	    													  
void SPI2_Init(void);			 //初始化SPI2
void SPI2_SetSpeed(u8 SpeedSet);  //设置SPI2速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2总线读写一个字节
		 
#endif

