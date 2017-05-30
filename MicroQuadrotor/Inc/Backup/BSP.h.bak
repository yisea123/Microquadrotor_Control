
/**
  ******************************************************************************
  * @file    BSP.h
  * @author  Johnny Sun
  * @version V1.0
  * @date    2015.4.21
  * @note
  * @history    V1.0 2015.3.31
  *                 其他未包含在模块中的初始化配置
  *             V2.0 2015.4.21
  *                 Compatible with other mainboard
  ******************************************************************************
  */
  

#ifndef __BSP_H__
#define __BSP_H__
#include "stm32f4xx_hal.h"
/*定义当前板子类型*/
#define BOARD_V100      /*第一版双层板*/

#define Math_PERIOD (5)
#define BMP280_API
#define BMP280
#define BatLED_GPIO GPIOB
#define BatLED_PIN  GPIO_PIN_8

typedef enum
{
    STANDBY,
    TAKEOFFING,
    LANDING,
    FLYING,
}FlyModeType;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */
#endif /* __BSP_H__ */
