/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 24, 2020
 *      Author: Pranesh
 */

#include "stm32f446xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}

	if (EnOrDi == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function Initializes the SPI pin
 *
 * @param[in]         - SPI pin handler structure
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Device mode
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FULL_DUPLEX)
	{
		// Clear bidi mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_HALF_DUPLEX)
	{
		// Set bidi mode
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RX_ONLY)
	{
		// Clear bidi mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);

		// Set RXONLY bit
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RX_ONLY);
	}

	// Serial clock speed (baud rate)
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// Data frame format
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Serial clock polarity
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Serial clock phase
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Software slave management
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function De-Initializes a SPI pin
 *
 * @param[in]         - base address of the SPI  peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data via SPI
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]		  - pointer to data buffer to be transmitted
 * @param[in]		  - length of the data
 *
 * @return            - value read from the pin
 *
 * @Note              - This is a blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
		{
			// Wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

			// Check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF

				// Load the data in to the DR
				pSPIx->DR =   *((uint16_t*)pTxBuffer);
				Len-= 2;

				(uint16_t*)pTxBuffer++;
			}
			else
			{
				//8 bit DFF

				pSPIx->DR =   *pTxBuffer;
				Len--;

				pTxBuffer++;
			}
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}




/*********************************************************************
 * @fn      		  - SPI_ToggleOutputPin
 *
 * @brief             - This function writes to a SPI output port
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]		  - pin number to toogle the output for
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_ToggleOutputPin(SPI_RegDef_t *pSPIx, uint8_t PinNumber)
{
	/*
	pSPIx->ODR ^= (1 << PinNumber);
	*/
}

/*********************************************************************
 * @fn      		  - SPI_IRQConfig
 *
 * @brief             - This funtion configures an IRQ
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]		  - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER_0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)	// 32 to 63
		{
			// program ISER1 register
			*NVIC_ISER_1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)	// 64 to 95
		{
			// program ISER2 register
			*NVIC_ISER_2 |= (1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 96 && IRQNumber < 128) // 96 to 127
		{
			// program ISER3 register
			*NVIC_ISER_3 |= (1 << (IRQNumber % 96));
		}
	}

	if (EnorDi == DISABLE)
	{
		if (IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER_0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)	// 32 to 63
		{
			// program ICER1 register
			*NVIC_ICER_1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)	// 64 to 95
		{
			// program ICER2 register
			*NVIC_ICER_2 |= (1 << (IRQNumber % 64));
		}
		else if (IRQNumber >= 96 && IRQNumber < 128) // 96 to 127
		{
			// program ICER3 register
			*NVIC_ICER_3 |= (1 << (IRQNumber % 64));
		}
	}
	*/
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This funtion configures the priority an Interrupt
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]		  - priority of the interrupt
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	/*
	// Find our the IPRx register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
	*/
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function is used for handling of the interrupt
 *
 * @param[in]         - pin number of the interrupt
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_RegDef_t *pHandle)
{
	/*
	// Clear the EXTI Priority Register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		// Clear
		EXTI->PR |= (1 << PinNumber);
	}
	*/
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function is used to get the status of the SPI peripheral flag
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - name of the flag
 *
 * @return            - none
 *
 * @Note              - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagPosition)
{
	if(pSPIx->SR & FlagPosition)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}

	return FLAG_RESET;
}

