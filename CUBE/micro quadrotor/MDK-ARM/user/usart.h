/**
  ******************************************************************************
  * @file       MX_Usart.h
  * @author     TracyW     @UESTC
  * @version    V1.0
  * @date       2015.11.9
  * @note       
  * @history    V1.0 2015.11.9
  *                
  ******************************************************************************
 **/
 
#ifndef __MX_USART_H
#define __MX_USART_H

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];


#endif

/************************ (C) COPYRIGHT @TracyW *****************END OF FILE****/
