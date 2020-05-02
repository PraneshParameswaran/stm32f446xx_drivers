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
	/*
	// Pin Mode
	if (pSPIHandle->SPI_PinConfig.SPI_pinMode <= SPI_MODE_ANALOG)	// Non-interrupt mode
	{

		pSPIHandle->pSPIx->MODER &= ~(0x3 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);	// Clearing
		pSPIHandle->pSPIx->MODER |= pSPIHandle->SPI_PinConfig.SPI_pinMode << (2 * pSPIHandle->SPI_PinConfig.SPI_pinNumber);
	}
	else	// Interrupt mode
	{
		if (pSPIHandle->SPI_PinConfig.SPI_pinMode == SPI_MODE_FT)
		{
			// Configure the FTSR
			EXTI->FTSR	|= (1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);

			// Clear the corresponding RTSRv
			EXTI->RTSR &= ~(1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);
		}

		if (pSPIHandle->SPI_PinConfig.SPI_pinMode == SPI_MODE_RT)
		{
			// Configure the RTSR
			EXTI->RTSR	|= (1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);

			// Clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);
		}

		if (pSPIHandle->SPI_PinConfig.SPI_pinMode == SPI_MODE_RFT)
		{
			// Configure the FTSR and RTSR
			EXTI->FTSR	|= (1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);
			EXTI->RTSR	|= (1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);
		}

		// Configure the SPI port selection in SYSCFG_EXTICR
		uint8_t reg_index = pSPIHandle->SPI_PinConfig.SPI_pinNumber / 4;
		uint8_t shift_pos = pSPIHandle->SPI_PinConfig.SPI_pinNumber % 4;

		uint8_t port_code = SPI_BASEADDR_TO_CODE(pSPIHandle->pSPIx);

		SYSCFG_PCLK_EN();	// Enable the peripheral clock

		SYSCFG->EXTICR[reg_index] = (port_code << (shift_pos * 4));

		// Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);
	}

	// Output type
	pSPIHandle->pSPIx->OTYPER &= ~(0x1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);	// Clearing
	pSPIHandle->pSPIx->OTYPER |= pSPIHandle->SPI_PinConfig.SPI_pinOPType << pSPIHandle->SPI_PinConfig.SPI_pinNumber;

	// Output speed
	pSPIHandle->pSPIx->OSPEEDER &= ~(0x1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);	// Clearing
	pSPIHandle->pSPIx->OSPEEDER |= pSPIHandle->SPI_PinConfig.SPI_pinSpeed << (2 * pSPIHandle->SPI_PinConfig.SPI_pinNumber);

	// Pull-up/Pull-down control
	pSPIHandle->pSPIx->PUPDR &= ~(0x1 << pSPIHandle->SPI_PinConfig.SPI_pinNumber);	// Clearing
	pSPIHandle->pSPIx->PUPDR |= pSPIHandle->SPI_PinConfig.SPI_pinPuPdControl << (2 * pSPIHandle->SPI_PinConfig.SPI_pinNumber);

	// Alternate functionality
	if (pSPIHandle->pSPIx->MODER == SPI_MODE_ATL_FN)
	{
		uint8_t reg_index, shift_pos;

		reg_index = pSPIHandle->SPI_PinConfig.SPI_pinNumber / 8;
		shift_pos = pSPIHandle->SPI_PinConfig.SPI_pinNumber % 8;

		pSPIHandle->pSPIx->AFR[reg_index] &= ~(0xF << pSPIHandle->SPI_PinConfig.SPI_pinNumber);	// Clearing
		pSPIHandle->pSPIx->AFR[reg_index] |= pSPIHandle->SPI_PinConfig.SPI_PinAltFunMode << (4 * shift_pos);
	}
*/
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
 * @fn      		  - SPI_ReadFromInputPin
 *
 * @brief             - This function reads input from a SPI pin
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]		  - input pin number to read from
 *
 * @return            - value read from the pin
 *
 * @Note              - none
 */
uint8_t SPI_ReadFromInputPin(SPI_RegDef_t *pSPIx, uint8_t PinNumber)
{
	/*
	return (uint8_t)((pSPIx->IDR >> PinNumber) & (0x00000001));
	*/
}

/*********************************************************************
 * @fn      		  - SPI_ReadFromInputPort
 *
 * @brief             - This function reads input from a SPI port
 *
 * @param[in]         - base address of the SPI  peripheral
 *
 * @return            - value read from the port
 *
 * @Note              - none
 */
uint16_t SPI_ReadFromInputPort(SPI_RegDef_t *pSPIx)
{
	/*
	return (uint16_t)(pSPIx->IDR);
	*/
}

/*********************************************************************
 * @fn      		  - SPI_WriteToOutputPin
 *
 * @brief             - This function writes to a SPI output pin
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]         - pin number to write to
 * @param[in]		  - value to be written to the pin
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_WriteToOutputPin(SPI_RegDef_t *pSPIx, uint8_t PinNumber, uint8_t Value)
{
	/*
	if (Value == SPI_PIN_SET)
	{
		pSPIx->ODR |= (1 << PinNumber);
	}

	if (Value == SPI_PIN_RESET)
	{
		pSPIx->ODR &= ~(1 << PinNumber);
	}
	*/
}

/*********************************************************************
 * @fn      		  - SPI_WriteToOutputPort
 *
 * @brief             - This function writes to a SPI output port
 *
 * @param[in]         - base address of the SPI  peripheral
 * @param[in]		  - value to be written to the port
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_WriteToOutputPort(SPI_RegDef_t *pSPIx, uint16_t Value)
{
	/*
	pSPIx->ODR = Value;
	*/
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

