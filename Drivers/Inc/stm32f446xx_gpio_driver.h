/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Apr 3, 2020
 *      Author: Pranesh Parameswaran
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
	uint32_t	GPIO_pinNumber;			/*!< possible values from @GPIO_PIN_NUMS */
	uint32_t	GPIO_pinMode;			/*!< possible values from @GPIO_PIN_MODES */
	uint32_t	GPIO_pinSpeed;			/*!< possible values from @GPIO_PIN_OUT_SPEEDS */
	uint32_t	GPIO_pinPuPdControl;	/*!< possible values from @GPIO_PIN_PU_PD_CONFIG */
	uint32_t	GPIO_pinOPType;			/*!< possible values from @GPIO_PIN_OUT_TYPES */
	uint32_t	GPIO_PinAltFunMode;		/*!< possible values from @GPIO_ALT_FN */
}GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t		*pGPIOx;			/*!< This holds the base address of the GPIO port to which the pin belongs	*/
	GPIO_PinConfig_t	GPIO_PinConfig;		/*!< This holds GPIO pin configuration settings	*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ATL_FN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_FT		4
#define GPIO_MODE_RT		5
#define GPIO_MODE_RFT		6

/*
 * @GPIO_PIN_OUT_SPEEDS
 * GPIO pin output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * @GPIO_PIN_PU_PD_CONFIG
 * GPIO pin pull-up and pull-down configurations
 */
#define GPIO_NO_PUPD	0
#define	GPIO_PIN_PU		1
#define GPIO_PIN_PD		2

/*
 * @GPIO_PIN_OUT_TYPES
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_ALT_FN
 * GPIO alternate functionality modes
 */
#define GPIO_AF0	0
#define GPIO_AF1	1
#define GPIO_AF2	2
#define GPIO_AF3	3
#define GPIO_AF4	4
#define GPIO_AF5	5
#define GPIO_AF6	6
#define GPIO_AF7	7
#define GPIO_AF8	8
#define GPIO_AF9	9
#define GPIO_AF10	10
#define GPIO_AF11	11
#define GPIO_AF12	12
#define GPIO_AF13	13
#define GPIO_AF14	14
#define GPIO_AF15	15


/****************************************************************************
 * 						APIs supported by this Driver
 * 		For more information about the APIs check the function definitions
 ****************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Date read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configutaion and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
