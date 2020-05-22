/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 24, 2020
 *      Author: Pranesh
 */

#include "stm32f446xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data via SPI
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]		  - pointer to the receive data buffer
 * @param[in]		  - length of the data
 *
 * @return            - value read from the pin
 *
 * @Note              - This is a blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
		{
			// Wait until RXE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET );

			// Check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF

				// Load the data in to the DR
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len-= 2;

				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 bit DFF

				*pRxBuffer = pSPIx->DR;
				Len--;

				pRxBuffer++;
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function sends data via SPI with an interrupt
 *
 * @param[in]         - pointer to SPI handle structure
 * @param[in]		  - pointer to the transmission data buffer
 * @param[in]		  - length of the data
 *
 * @return            - State of SPI transmission
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		// Save the Tx buffer address and Length info
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral till transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data transmission will be handled by ISR code
	}

	return pSPIHandle->TxState;

}


/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function receives data via SPI with an interrupt
 *
 * @param[in]         - pointer to SPI handle structure
 * @param[in]		  - pointer to the receive data buffer
 * @param[in]		  - length of the data
 *
 * @return            - state of SPI reception
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	if (pSPIHandle->RxState != SPI_BUSY_IN_TX)
	{
		// Save the Rx buffer address and Length info
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral till reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data transmission will be handled by ISR code
	}

	return pSPIHandle->RxState;
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
	// Find our the IPRx register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function is used for handling of the interrupt
 *
 * @param[in]         - SPI Handle structure
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t status_register, control_register;

	//first lets check for TXE
	status_register = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	control_register = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if (status_register && control_register)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	status_register = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	control_register = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if (status_register && control_register)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for ovr flag
	status_register = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	control_register = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if (status_register && control_register)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*********************************************************************
 * @fn      		  - SPI_ApplicationEventCallback
 *
 * @brief             - This function is used to callback application after SPI interrupt
 *
 * @param[in]         - SPI Handle structure
 * @param[in]         - SPI APP EVENTS macro
 *
 * @return            -
 *
 * @Note              -
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{

	//This is a weak implementation . the user application may override this function.
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function is used to enable/disable peripheral control
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function is used to enable/disable SSI
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            -
 *
 * @Note              -
 */
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}


}


/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - This function is used to enable/disable SSOE
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            -
 *
 * @Note              -
 */
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


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

// Helper functions
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}
	else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. clear the ovr flag by reading from register
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		uint8_t temp;

		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

		(void)temp;
	}

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}



