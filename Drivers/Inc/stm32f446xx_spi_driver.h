/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 24, 2020
 *      Author: Pranesh
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

/*
 * Configuration structure for SPIx peripherals
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t		*pSPIx;		/*!< This holds the base address of the SPI peripheral	*/
	SPI_PinConfig_t		SPIConfig;	/*!< This holds GPIO pin configuration settings	*/
}SPI_Handle_t;
#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
