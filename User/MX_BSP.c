/**
  ******************************************************************************
  * @file       MX_BSP.c
  * @author     TracyW     @UESTC
  * @version    V1.0
  * @date       2015.11.5
  * @note       This file provides set of firmware functions to manage Leds and push-button
  * @history    V1.0 2015.11.5
  *                
  ******************************************************************************
 **/

/* Includes ------------------------------------------------------------------*/
#include "MX_BSP.h"




GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, 
                                 LED2_GPIO_PORT, 
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, 
                                 LED2_PIN, 
                                 LED3_PIN,
                                 LED4_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT}; 
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN}; 
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */ 
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;    /*<! Value of Timeout when SPI communication fails */

static SPI_HandleTypeDef    SpiHandle;
static I2C_HandleTypeDef    I2cHandle;


static void     I2Cx_Init(void);
static void     I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
static void     I2Cx_MspInit(void);
static void     I2Cx_Error(uint8_t Addr);

static void     SPIx_Init(void);
static void     SPIx_MspInit(void);
static uint8_t  SPIx_WriteRead(uint8_t Byte);
static void    SPIx_Error(void);

/* Link functions for Accelerometer peripheral */
void            ACCELERO_IO_Init(void);
void            ACCELERO_IO_ITConfig(void);
void            ACCELERO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void            ACCELERO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/* Link functions for Audio peripheral */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_DeInit(void);
void            AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);




void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}


void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}


void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}


void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}



void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if (Mode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  
  if (Mode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}


uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* SPI Routines *********************************/

static void SPIx_Init(void)
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI configuration -----------------------------------------------------*/
    SpiHandle.Instance = DISCOVERY_SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    SPIx_MspInit();
    HAL_SPI_Init(&SpiHandle);
  }
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte: Byte send.
  * @retval The received byte value
  */
static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

/**
  * @brief  SPIx error treatment function.
  * @param  None
  * @retval None
  */
static void SPIx_Error(void)
{
  /* De-initialize the SPI communication bus */
  HAL_SPI_DeInit(&SpiHandle);
  
  /* Re-Initialize the SPI communication bus */
  SPIx_Init();
}

/**
  * @brief  SPI MSP Init.
  * @param  hspi: SPI handle
  * @retval None
  */
static void SPIx_MspInit(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable the SPI peripheral */
  DISCOVERY_SPIx_CLK_ENABLE();
  
  /* Enable SCK, MOSI and MISO GPIO clocks */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();
  
  /* SPI SCK, MOSI, MISO pin configuration */
  GPIO_InitStructure.Pin = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MISO_PIN | DISCOVERY_SPIx_MOSI_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
}

/******************************* I2C Routines**********************************/
/**
  * @brief  Configures I2C interface.
  * @param  None
  * @retval None
  */
static void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
  {
    /* DISCOVERY_I2Cx peripheral configuration */
    I2cHandle.Init.ClockSpeed = BSP_I2C_SPEED;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2cHandle.Init.OwnAddress1 = 0x33;
    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Instance = DISCOVERY_I2Cx;
      
    /* Init the I2C */
    I2Cx_MspInit();
    HAL_I2C_Init(&I2cHandle);
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written 
  * @retval HAL status
  */
static void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS  
  * @param  Reg: The target register address to read
  * @retval HAL status
  */
static uint8_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1,I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
  return value;
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(&I2cHandle);
  
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init();
}

/**
  * @brief I2C MSP Initialization
  * @param None
  * @retval None
  */
static void I2Cx_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* DISCOVERY_I2Cx SCL and SDA pins configuration ---------------------------*/
  GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = DISCOVERY_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);     

  /* Enable the DISCOVERY_I2Cx peripheral clock */
  DISCOVERY_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DISCOVERY_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DISCOVERY_I2Cx_RELEASE_RESET();

  /* Enable and set I2Cx Interrupt to the highest priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

  /* Enable and set I2Cx Interrupt to the highest priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn); 
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/***************************** LINK ACCELEROMETER *****************************/

/**
  * @brief  Configures the Accelerometer SPI interface.
  * @param  None
  * @retval None
  */
void ACCELERO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure the Accelerometer Control pins --------------------------------*/
  /* Enable CS GPIO clock and configure GPIO pin for Accelerometer Chip select */  
  ACCELERO_CS_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PIN for LIS Chip select */
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);
  
  /* Deselect: Chip Select high */
  ACCELERO_CS_HIGH();
  
  SPIx_Init();
}

/**
  * @brief  Configures the Accelerometer INT2.
  *         EXTI0 is already used by user button so INT1 is not configured here.
  * @param  None
  * @retval None
  */
void ACCELERO_IO_ITConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable INT2 GPIO clock and configure GPIO PINs to detect Interrupts */
  ACCELERO_INT_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.Pin = ACCELERO_INT2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACCELERO_INT_GPIO_PORT, &GPIO_InitStructure);
  
  /* Enable and set Accelerometer INT2 to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)ACCELERO_INT2_EXTI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)ACCELERO_INT2_EXTI_IRQn);
}

/**
  * @brief  Writes one byte to the Accelerometer.
  * @param  pBuffer: pointer to the buffer containing the data to be written to the Accelerometer.
  * @param  WriteAddr: Accelerometer's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void ACCELERO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  ACCELERO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  ACCELERO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the Accelerometer.
  * @param  pBuffer: pointer to the buffer that receives the data read from the Accelerometer.
  * @param  ReadAddr: Accelerometer's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the Accelerometer.
  * @retval None
  */
void ACCELERO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  ACCELERO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to ACCELEROMETER (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  ACCELERO_CS_HIGH();
}

/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initializes Audio low level.
  * @param  None
  * @retval None
  */
void AUDIO_IO_Init(void) 
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable Reset GPIO Clock */
  AUDIO_RESET_GPIO_CLK_ENABLE();
  
  /* Audio reset pin configuration */
  GPIO_InitStruct.Pin = AUDIO_RESET_PIN; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);    
  
  I2Cx_Init();
  
  /* Power Down the codec */
  HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);
  
  /* Wait for a delay to insure registers erasing */
  HAL_Delay(5); 
  
  /* Power on the codec */
  HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET);
  
  /* Wait for a delay to insure registers erasing */
  HAL_Delay(5); 
}

/**
  * @brief  DeInitializes Audio low level.
  * @param  None
  * @retval None
  */
void AUDIO_IO_DeInit(void)
{
  
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, Value);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg);
}


/************************ (C) COPYRIGHT @Tracy *****END OF FILE****/
