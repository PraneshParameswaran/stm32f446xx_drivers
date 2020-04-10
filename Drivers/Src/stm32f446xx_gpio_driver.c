/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Apr 3, 2020
 *      Author: Pranesh Parameswaran
 */
#include "stm32f446xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO  peripheral
 * @param[in]         - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}

	if (EnOrDi == DISABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function Initializes the GPIO pin
 *
 * @param[in]         - GPIO pin handler structure
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Pin Mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_pinMode <= GPIO_MODE_ANALOG)	// Non-interrupt mode
	{

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);	// Clearing
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->GPIO_PinConfig.GPIO_pinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
	}
	else	// Interrupt mode
	{

	}

	// Output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);	// Clearing
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->GPIO_PinConfig.GPIO_pinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber;

	// Output speed
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);	// Clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= pGPIOHandle->GPIO_PinConfig.GPIO_pinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);

	// Pull-up/Pull-down control
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);	// Clearing
	pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);

	// Alternate functionality
	if (pGPIOHandle->pGPIOx->MODER == GPIO_MODE_ATL_FN)
	{
		uint8_t reg_index, shift_pos;

		reg_index = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber / 8;
		shift_pos = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[reg_index] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);	// Clearing
		pGPIOHandle->pGPIOx->AFR[reg_index] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * shift_pos);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function De-Initializes a GPIO pin
 *
 * @param[in]         - base address of the GPIO  peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads input from a GPIO pin
 *
 * @param[in]         - base address of the GPIO  peripheral
 * @param[in]		  - input pin number to read from
 *
 * @return            - value read from the pin
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads input from a GPIO port
 *
 * @param[in]         - base address of the GPIO  peripheral
 *
 * @return            - value read from the port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to a GPIO output pin
 *
 * @param[in]         - base address of the GPIO  peripheral
 * @param[in]         - pin number to write to
 * @param[in]		  - value to be written to the pin
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes to a GPIO output port
 *
 * @param[in]         - base address of the GPIO  peripheral
 * @param[in]		  - value to be written to the port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function writes to a GPIO output port
 *
 * @param[in]         - base address of the GPIO  peripheral
 * @param[in]		  - pin number to toogle the output for
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This funtion configures an IRQ
 *
 * @param[in]         - IRQ number of the interrupt
 * @param[in]		  - priority of the interrupt
 * @param[in]		  - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function is used for handling of the interrupt
 *
 * @param[in]         - pin number of the interrupt
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

