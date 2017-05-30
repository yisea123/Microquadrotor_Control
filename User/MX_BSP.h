/**
  ******************************************************************************
  * @file       MX_BSP.h
  * @author     TracyW     @UESTC
  * @version    V1.0
  * @date       2015.11.5
  * @note       
  * @history    V1.0 2015.11.5
  *                
  ******************************************************************************
 **/


#ifndef __MX_BSP_H
#define __MX_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/*############################### LED #######################################*/
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
} Led_TypeDef;

typedef enum 
{  
  BUTTON_KEY = 0,
} Button_TypeDef;

//typedef enum 
//{  
//  BUTTON_MODE_GPIO = 0,
//  BUTTON_MODE_EXTI = 1
//} ButtonMode_TypeDef;     



#define LEDn                             4

#define LED4_PIN                         GPIO_PIN_3
#define LED4_GPIO_PORT                   GPIOA
#define LED4_GPIO_CLK_ENABLE()           __GPIOA_CLK_ENABLE()  
#define LED4_GPIO_CLK_DISABLE()          __GPIOA_CLK_DISABLE()  

#define LED3_PIN                         GPIO_PIN_2
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()  
#define LED3_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()  
  
#define LED2_PIN                         GPIO_PIN_1
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()  

#define LED1_PIN                         GPIO_PIN_0
#define LED1_GPIO_PORT                   GPIOA
#define LED1_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 2) LED3_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 3) LED4_GPIO_CLK_ENABLE(); \
                                           }while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 2) LED3_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 3) LED4_GPIO_CLK_DISABLE(); \
                                            }while(0)

/*
#define BUTTONn                          1 


#define KEY_BUTTON_PIN                  GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT            GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()   __GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn            EXTI0_IRQn 

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                }while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
*/

/*############################### SPI2 #######################################*/
#define DISCOVERY_SPIx                              SPI2
#define DISCOVERY_SPIx_CLK_ENABLE()                 __SPI2_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT                    GPIOB                      /* GPIOB */
#define DISCOVERY_SPIx_AF                           GPIO_AF5_SPI2
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()            __GPIOB_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()           __GPIOB_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                      GPIO_PIN_13                 /* PB.13 */
#define DISCOVERY_SPIx_MISO_PIN                     GPIO_PIN_14                 /* PB.14 */
#define DISCOVERY_SPIx_MOSI_PIN                     GPIO_PIN_15                 /* PB.15 */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define SPIx_TIMEOUT_MAX                            0x1000 /*<! The value of the maximal timeout for BUS waiting loops */



///*############################# I2C1 #########################################*/
///* I2C clock speed configuration (in Hz) */
//#ifndef BSP_I2C_SPEED
// #define BSP_I2C_SPEED                            100000
//#endif /* BSP_I2C_SPEED */

///* I2C peripheral configuration defines (control interface of the audio codec) */
//#define DISCOVERY_I2Cx                            I2C1
//#define DISCOVERY_I2Cx_CLK_ENABLE()               __I2C1_CLK_ENABLE()
//#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
//#define DISCOVERY_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
//#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT          GPIOB
//#define DISCOVERY_I2Cx_SCL_PIN                    GPIO_PIN_6
//#define DISCOVERY_I2Cx_SDA_PIN                    GPIO_PIN_9

//#define DISCOVERY_I2Cx_FORCE_RESET()              __I2C1_FORCE_RESET()
//#define DISCOVERY_I2Cx_RELEASE_RESET()            __I2C1_RELEASE_RESET()

///* I2C interrupt requests */                  
//#define DISCOVERY_I2Cx_EV_IRQn                    I2C1_EV_IRQn
//#define DISCOVERY_I2Cx_ER_IRQn                    I2C1_ER_IRQn

///* Maximum Timeout values for flags waiting loops. These timeouts are not based
//   on accurate values, they just guarantee that the application will not remain
//   stuck if the SPI communication is corrupted.
//   You may modify these timeout values depending on CPU frequency and application
//   conditions (interrupts routines ...). */   
//#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */


/*############################# ACCELEROMETER ################################*/
/* Read/Write command */
#define READWRITE_CMD                     ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD                  ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                        ((uint8_t)0x00)

/* Chip Select macro definition */
#define ACCELERO_CS_LOW()       HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_RESET)
#define ACCELERO_CS_HIGH()      HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  ACCELEROMETER Interface pins
  */
#define ACCELERO_CS_PIN                        GPIO_PIN_3                 /* PE.03 */
#define ACCELERO_CS_GPIO_PORT                  GPIOE                      /* GPIOE */
#define ACCELERO_CS_GPIO_CLK_ENABLE()          __GPIOE_CLK_ENABLE()
#define ACCELERO_CS_GPIO_CLK_DISABLE()         __GPIOE_CLK_DISABLE()
#define ACCELERO_INT_GPIO_PORT                 GPIOE                      /* GPIOE */
#define ACCELERO_INT_GPIO_CLK_ENABLE()         __GPIOE_CLK_ENABLE()
#define ACCELERO_INT_GPIO_CLK_DISABLE()        __GPIOE_CLK_DISABLE()
#define ACCELERO_INT1_PIN                      GPIO_PIN_0                 /* PE.00 */
#define ACCELERO_INT1_EXTI_IRQn                EXTI0_IRQn 
#define ACCELERO_INT2_PIN                      GPIO_PIN_1                 /* PE.01 */
#define ACCELERO_INT2_EXTI_IRQn                EXTI1_IRQn 



uint32_t BSP_GetVersion(void);
void     BSP_LED_Init(Led_TypeDef Led);
void     BSP_LED_On(Led_TypeDef Led);
void     BSP_LED_Off(Led_TypeDef Led);
void     BSP_LED_Toggle(Led_TypeDef Led);
//void     BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode);
//uint32_t BSP_PB_GetState(Button_TypeDef Button);


  
#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT @TracyW *****************END OF FILE****/
