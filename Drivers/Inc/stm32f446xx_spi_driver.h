/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 24, 2020
 *      Author: Pranesh
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for SPIx peripherals
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;		/*!< possible values from @SPI_DEVICE_MODES */
	uint8_t SPI_BusConfig;		/*!< possible values from @SPI_BUS_CONFIG	*/
	uint8_t SPI_SclkSpeed;		/*!< possible values from @SPI_SCLK_SPEEDS	*/
	uint8_t SPI_DFF;			/*!< possible values form @SPI_DFF			*/			
	uint8_t SPI_CPOL;			/*!< possible values from @SPI_CPOL			*/
	uint8_t SPI_CPHA;			/*!< possible values from @SPI_CPHA			*/
	uint8_t SPI_SSM;			/*!< possible values from @SPI_SSM			*/
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t		*pSPIx;		/*!< Base address of the SPI peripheral	*/
	SPI_Config_t		SPIConfig;	/*!< SPI pin configuration settings	*/
	uint8_t				*pTxBuffer;	/*!< Application Tx buffer address */
	uint8_t 			*pRxBuffer; /*!< Application Rx buffer address */
	uint32_t			TxLen;		/*!< Tx length */
	uint32_t			RxLen;		/*!< Rx length */
	uint8_t				TxState;	/*!< Tx state, possible values from @SPI_APP_STATES */
	uint8_t 			RxState;	/*!< Rx state, possible values from @SPI_APP_STATES */
}SPI_Handle_t;

/*
 * @SPI_APP_STATES
 * SPI application states
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * @SPI_APP_EVENTS
 * SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

/*
 * @SPI_DEVICE_MODES
 * SPI device mode
 */
#define SPI_DEVICE_MODE_SLAVE	0
#define SPI_DEVICE_MODE_MASTER	1

/*
 * @SPI_BUS_CONFIG
 * SPI bus configs
 */
#define SPI_BUS_CFG_FULL_DUPLEX		1
#define SPI_BUS_CFG_HALF_DUPLEX		2
#define SPI_BUS_CFG_SIMPLEX_RX_ONLY	3

 /*
  * @SPI_SCLK_SPEEDS
  * SPI serial clock speeds 
  */
 #define SPI_SCLK_SPEED_DIV2	0
 #define SPI_SCLK_SPEED_DIV4	1
 #define SPI_SCLK_SPEED_DIV8	2
 #define SPI_SCLK_SPEED_DIV16	3
 #define SPI_SCLK_SPEED_DIV32	4
 #define SPI_SCLK_SPEED_DIV64	5
 #define SPI_SCLK_SPEED_DIV128	6
 #define SPI_SCLK_SPEED_DIV256	7

 /*
  * @SPI_DFF
  * SPI data frame format
  */
 #define SPI_DFF_8_BITS	0
 #define SPI_DFF_16_BITS	1

  /*
   * @SPI_CPOL
   * SPI serial clock polarity
   */
 #define SPI_CPOL_LOW		0
 #define SPI_CPOL_HIGH		1

  /*
   * @SPI_CPHA
   * SPI serial clock phase
   */  
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_SSM
 * SPI software slave management
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/******************************************************************************
 * 							SPI bit position macros
 ******************************************************************************/
/*
 * Bit position macros for CR1
 */ 
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*
 * Bit position macros for CR2
 */
 #define SPI_CR2_RXDMAEN	0
 #define SPI_CR2_TXDMAEN	1
 #define SPI_CR2_SSOE		2
 #define SPI_CR2_ERRIE		5
 #define SPI_CR2_RXNEIE		6
 #define SPI_CR2_TXEIE		7
 
/*
 * Bit position macros for SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*
 * SPI related status flag definition
 */
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG 		(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG	(1 << SPI_SR_CRC_ERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/****************************************************************************
 * 						APIs supported by this Driver
 * 		For more information about the APIs check the function definitions
 ****************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configutaion and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent);

/*
 * Peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Helper functions
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagPosition);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
