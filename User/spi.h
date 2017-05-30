/**
  ******************************************************************************
  * @file       spi.h
  * @author          @UESTC
  * @version    V1.0
  * @date       2015.11.2
  * @note       �ο�����ԭ��
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
 	
 	    													  
void SPI2_Init(void);			 //��ʼ��SPI2
void SPI2_SetSpeed(u8 SpeedSet);  //����SPI2�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ�
		 
#endif

