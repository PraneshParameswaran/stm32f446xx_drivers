/*
 * stm32f446xx.h
 *
 *  Created on: Apr 2, 2020
 *      Author: Pranesh Parameswaran
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Flash,RAM and ROM base addresses
 */
#define FLASH_BASEADDR			0x08000000U		/* Base address of Flash */
#define SRAM1_BASEADDR			0X20000000U		/* Base address of SRAM1 (112kB) */
#define SRAM2_BASEADDR			0X2001C000U		/* Base address of SRAM2 (16kB) */
#define ROM_BASEADDR			0x1FFF0000U		/* Base address of System Memory */
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR				0X40000000U			/* Base address of peripherals */
#define APB1_PERIPH_BASEADDR		PERIPH_BASEADDR		/* Base address of APB1 peripherals */
#define APB2_PERIPH_BASEADDR		0X40010000U			/* Base address of APB2 peripherals */
#define AHB1_PERIPH_BASEADDR		0X40020000U			/* Base address of AHB1 peripherals */
#define AHB2_PERIPH_BASEADDR		0X50000000U			/* Base address of AHB2 peripherals */

/*
 * AHB1 Bus Peripheral base addresses
 * TODO: Complete for all other peripherals
 */
#define GPIOA_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X0000U)	/* Base address of GPIOA port */
#define GPIOB_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X0400U)	/* Base address of GPIOB port */
#define GPIOC_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X0800U)	/* Base address of GPIOC port */
#define GPIOD_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X0C00U)	/* Base address of GPIOD port */
#define GPIOE_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X1000U)	/* Base address of GPIOE port */
#define GPIOF_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X1400U)	/* Base address of GPIOF port */
#define GPIOG_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X1800U)	/* Base address of GPIOG port */
#define GPIOH_BASEADDR				(AHB1_PERIPH_BASEADDR + 0X1C00U)	/* Base address of GPIOH port */

#define RCC_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x3800U)	/* Base address of RCC port	*/

/*
 * APB1 Bus Peripheral base addresses
 * TODO: Complete for all other peripherals
 */
#define I2C1_BASEADDR				(APB1_PERIPH_BASEADDR + 0X5400U)	/* Base address of I2C1 port */
#define I2C2_BASEADDR				(APB1_PERIPH_BASEADDR + 0X5800U)	/* Base address of I2C2 port */
#define I2C3_BASEADDR				(APB1_PERIPH_BASEADDR + 0X5C00U)	/* Base address of I2C3 port */

#define SPI2_BASEADDR				(APB1_PERIPH_BASEADDR + 0X3800U)	/* Base address of SPI2 port */
#define SPI3_BASEADDR				(APB1_PERIPH_BASEADDR + 0X3C00U)	/* Base address of SPI3 port */

#define USART2_BASEADDR				(APB1_PERIPH_BASEADDR + 0X4400U)	/* Base address of USART2 port */
#define USART3_BASEADDR				(APB1_PERIPH_BASEADDR + 0X4800U)	/* Base address of USART3 port */

#define UART4_BASEADDR				(APB1_PERIPH_BASEADDR + 0X4C00U)	/* Base address of UART4 port */
#define UART5_BASEADDR				(APB1_PERIPH_BASEADDR + 0X5000U)	/* Base address of UART5 port */

/*
 * APB2 Bus Peripheral base addresses
 * TODO: Complete for all other peripherals
 */
#define EXTI_BASEADDR				(APB2_PERIPH_BASEADDR + 0X3C00U)	/* Base address of EXTI port */

#define SPI1_BASEADDR				(APB2_PERIPH_BASEADDR + 0X2000U)	/* Base address of SPI1 port */

#define SYSCFG_BASEADDR				(APB2_PERIPH_BASEADDR + 0X3800U)	/* Base address of SYSCFG port */

#define USART1_BASEADDR				(APB2_PERIPH_BASEADDR + 0X1000U)	/* Base address of USART1 port */
#define USART6_BASEADDR				(APB2_PERIPH_BASEADDR + 0X1400U)	/* Base address of USART5 port */

/******************************* Peripheral register definition structures *******************************/
typedef struct
{
	__vo uint32_t MODER;		/*!< GPIO port mode register, 					Address offset: 0x00	*/
	__vo uint32_t OTYPER;		/*!< GPIO port output type register,			Address offset: 0x04	*/
	__vo uint32_t OSPEEDER;		/*!< GPIO port output speed register,			Address offset: 0x08	*/
	__vo uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register,		Address offset: 0x0C	*/
	__vo uint32_t IDR;			/*!< GPIO port input data register,				Address offset: 0x10	*/
	__vo uint32_t ODR;			/*!< GPIO port output data register,			Address offset: 0x14	*/
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register,			Address offset: 0x18	*/
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register,		Address offset: 0x1C	*/
	__vo uint32_t AFR[2];		/*!< AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register, 	Address offset: 0x20 - 0x24 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/*!< RCC clock control register,									Address offset: 0x00	*/
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register, 								Address offset: 0x04	*/
	__vo uint32_t CFGR;			/*!< RCC clock configuration register, 								Address offset: 0x08	*/
	__vo uint32_t CIR;			/*!< RCC clock interrupt register, 									Address offset: 0x0C	*/
	__vo uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register, 							Address offset: 0x10	*/
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register, 							Address offset: 0x14	*/
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register, 							Address offset: 0x18	*/
		 uint32_t RESERVED0;	/*!< Reserved,														Address offset: 0x1C	*/
	__vo uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register, 							Address offset: 0x20	*/
	__vo uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register, 							Address offset: 0x24	*/
	     uint32_t RESERVED1[2];	/*!< Reserved,														Address offset: 0x28 - 0x2C	*/
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register, 					Address offset: 0x30	*/
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register, 					Address offset: 0x34	*/
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register, 					Address offset: 0x38	*/
	     uint32_t RESERVED2;	/*!< Reserved,														Address offset: 0x3C	*/
	__vo uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register,						Address offset: 0x40	*/
	__vo uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register, 					Address offset: 0x44	*/
	     uint32_t RESERVED3[2];	/*!< Reserved,														Address offset: 0x48 - 0x4C	*/
	__vo uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50	*/
	__vo uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register, 	Address offset: 0x54	*/
	__vo uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register, 	Address offset: 0x58	*/
	     uint32_t RESERVED4;	/*!< Reserved, 														Address offset: 0x5C	*/
	__vo uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register, 	Address offset: 0x60	*/
	__vo uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enabled in low power mode register, 	Address offset: 0x64	*/
		 uint32_t RESERVED5[2];	/*!< Reserved,														Address offset: 0x68 - 0x6C	*/
	__vo uint32_t BDCR;			/*!< RCC Backup domain control register, 							Address offset: 0x70	*/
	__vo uint32_t CSR;			/*!< RCC clock control & status register, 							Address offset: 0x74	*/
	     uint32_t RESERVED6[2];	/*!< Reserved,														Address offset: 0x78 - 0x7C	*/
	__vo uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register, 				Address offset: 0x80	*/
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register, 							Address offset: 0x84	*/
	__vo uint32_t PLLSAICFGR;	/*!< RCC PLL configuration register, 								Address offset: 0x88	*/
	__vo uint32_t DCKCFGR;		/*!< RCC Dedicated Clock Configuration Register, 					Address offset: 0x8C	*/
	__vo uint32_t CKGATENR;		/*!< RCC clocks gated enable register, 								Address offset: 0x90	*/
	__vo uint32_t DCKCFGR2;		/*!< RCC dedicated clocks configuration register 2, 				Address offset: 0x94	*/
}RCC_RegDef_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN	( RCC->AHB1ENR |= (1 << 7) )

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()	( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()	( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()	( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock enable macros for USARx peripherals
 */
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock enable macros for UARx peripherals
 */
#define UART4_PCLK_EN()	( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()	( RCC->APB1ENR |= (1 << 20) )

/*
 * Clock enable macros for SYSCFGx peripherals
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock disable macros for USARx peripherals
 */
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock disable macros for UARx peripherals
 */
#define UART4_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 20) )

/*
 * Clock disable macros for SYSCFGx peripherals
 */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )

#endif /* INC_STM32F446XX_H_ */
