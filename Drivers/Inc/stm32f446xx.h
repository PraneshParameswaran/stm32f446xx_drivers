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

/******************************* Processor Specific Details *******************************/

/*
 * ARM Cortex Mx processor ISERx registers
 */
#define NVIC_ISER_0		((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER_1		((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER_2		((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER_3		((__vo uint32_t*) 0xE000E10C)
#define NVIC_ISER_4		((__vo uint32_t*) 0xE000E110)
#define NVIC_ISER_5		((__vo uint32_t*) 0xE000E114)
#define NVIC_ISER_6		((__vo uint32_t*) 0xE000E118)
#define NVIC_ISER_7		((__vo uint32_t*) 0xE000E11C)

/*
 * ARM Cortex Mx processor ICERx registers
 */
#define NVIC_ICER_0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_1		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_2		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_3		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_4		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_5		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_6		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER_7		((__vo uint32_t*)0xE000E180)

/*
 * ARM Cortex Mx processor IPRx Base Address
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

/*
 * Number of priority bits implemented in IPRx registers
 */
#define NO_PR_BITS_IMPLEMENTED		4

/*********************************************************************************************/

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

#define SPI4_BASEADDR				(APB2_PERIPH_BASEADDR + 0X3400U)
#define SPI1_BASEADDR				(APB2_PERIPH_BASEADDR + 0X3000U)	/* Base address of SPI1 port */

#define SYSCFG_BASEADDR				(APB2_PERIPH_BASEADDR + 0X3800U)	/* Base address of SYSCFG port */

#define USART1_BASEADDR				(APB2_PERIPH_BASEADDR + 0X1000U)	/* Base address of USART1 port */
#define USART6_BASEADDR				(APB2_PERIPH_BASEADDR + 0X1400U)	/* Base address of USART5 port */

/*
 * IRQ (Interrupt Request) numbers of STM32F446xx MCU
 */
#define IRQ_WWDG					0
#define IRQ_PVD						1
#define IRQ_TAMP_STAMP				2
#define IRQ_RTC_WKUP				3
#define IRQ_FLASH					4
#define IRQ_RCC						5
#define IRQ_EXTI0					6
#define IRQ_EXTI1					7
#define IRQ_EXTI2					8
#define IRQ_EXTI3					9
#define IRQ_EXTI4					10
#define IRQ_DMA1_STREAM0			11
#define IRQ_DMA1_STREAM1			12
#define IRQ_DMA1_STREAM2			13
#define IRQ_DMA1_STREAM3			14
#define IRQ_DMA1_STREAM4			15
#define IRQ_DMA1_STREAM5			16
#define IRQ_DMA1_STREAM6			17
#define IRQ_ADC						18
#define IRQ_CAN1_TX					19
#define IRQ_CAN_RX0					20
#define IRQ_CAN_RX1					21
#define IRQ_CAN_SCE					22
#define IRQ_EXTI_9_5				23
#define IRQ_TIM1_BRK_TIM9			24
#define IRQ_TIM1_UP_TIM10			25
#define IRQ_TIM1_TRG_COM_TIM11		26
#define IRQ_TIM1_CC					27
#define IRQ_TIM2					28
#define IRQ_TIM3					29
#define IRQ_TIM4					30
#define IRQ_I2C1_EV					31
#define IRQ_I2C1_ER					32
#define IRQ_I2C2_EV					33
#define IRQ_I2C2_ER					34
#define IRQ_SPI1					35
#define IRQ_SPI2					36
#define IRQ_USART1					37
#define IRQ_USART2					38
#define IRQ_USART3					39
#define IRQ_EXTI_15_10				40
#define IRQ_RTC_ALARM				41
#define IRQ_OTG_FS_WKUP				42
#define IRQ_TIM8_BRK_TIM12			43
#define IRQ_TIM8_UP_TIM13			44
#define IRQ_TIM8_TRG_COM_TIM14		45
#define IRQ_TIM8_CC					46
#define IRQ_DMA1_STREAM7			47
#define IRQ_FMC						48
#define IRQ_SDIO					49
#define IRQ_TIM5					50
#define IRQ_SPI3					51
#define IRQ_USART4					52
#define IRQ_UART5					53
#define IRQ_TIM6_DAC				54
#define IRQ_TIM7					55
#define IRQ_DMA2_STREAM0			56
#define IRQ_DMA2_STREAM1			57
#define IRQ_DMA2_STREAM2			58
#define IRQ_DMA2_STREAM3			59
#define IRQ_DMA2_STREAM4			60
#define IRQ_CAN2_TX					63
#define IRQ_CAN2_RX00				64
#define IRQ_CAN2_RX1				65
#define IRQ_CAN2_SCE				66
#define IRQ_OTG_FS					67
#define IRQ_DMA2_STREAM5			68
#define IRQ_DMA2_STREAM6			69
#define IRQ_DMA2_STREAM7			70
#define IRQ_USART6					71
#define IRQ_I2C3_EV					72
#define IRQ_I2C3_ER					73
#define IRQ_OTG_HS_EP1_OUT			74
#define IRQ_OTG_HS_EP1_IN			75
#define IRQ_OTG_HS_WKUP				76
#define IRQ_OTG_HS					77
#define IRQ_DCMI					78
#define IRQ_FPU						81
#define IRQ_SPI4					84
#define IRQ_SAI1					87
#define IRQ_SAI2					91
#define IRQ_QUAD_SPI				92
#define IRQ_HDMI_CEC				93
#define IRQ_SPDI_RX					94
#define IRQ_FMPI2C1					95
#define IRQ_FMPI2C1_ERROR			96

/*
 * IRQ Priority levels
 */
#define IRQ_PRI0		0
#define IRQ_PRI1		1
#define IRQ_PRI2		2
#define IRQ_PRI3		3
#define IRQ_PRI4		4
#define IRQ_PRI5		5
#define IRQ_PRI6		6
#define IRQ_PRI7		7
#define IRQ_PRI8		8
#define IRQ_PRI9		9
#define IRQ_PRI10		10
#define IRQ_PRI11		11
#define IRQ_PRI12		12
#define IRQ_PRI13		13
#define IRQ_PRI14		14
#define IRQ_PRI15		15


/******************************* Peripheral register definition structures *******************************/
/*
 * Peripheral register definition structure for GPIO
 */
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

/*
 * Peripheral register definition structure for RCC
 */
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
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;		/*!< Interrupt mask register,					Address offset: 0x00	*/
	__vo uint32_t EMR;		/*!< Event mask register,						Address offset: 0x04	*/
	__vo uint32_t RTSR;		/*!< Rising trigger selection register,			Address offset: 0x08	*/
	__vo uint32_t FTSR;		/*!< Falling trigger selection register,		Address offset: 0x0C	*/
	__vo uint32_t SWIER;	/*!< Software interrupt event register,			Address offset: 0x10	*/
	__vo uint32_t PR;		/*!< Pending register,							Address offset: 0x14	*/
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*!< SYSCFG memory remap register,							Address offset: 0x00		*/
	__vo uint32_t PMC;			/*!< SYSCFG peripheral mode configuration register, 		Address offset: 0x04		*/
	__vo uint32_t EXTICR[4];	/*!< SYSCFG external interrupt configuration register, 		Address offset: 0x08 - 0x14	*/
	__vo uint32_t RESERVED1[2];	/*!< Reserved,											 	Address offset: 0x18 - 0x1C	*/
	__vo uint32_t CMPCR;		/*!< Compensation cell control register, 					Address offset: 0x20		*/
	__vo uint32_t RESERVED2[2];	/*!< Reserved,												Address offset: 0x24 - 0x28	*/
	__vo uint32_t CFGR;			/*!< SYSCFG configuration register,							Address offset: 0x2C		*/
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;		/*!< SPI control register 1, 			Address offset: 0x00	*/
	__vo uint32_t CR2;		/*!< SPI control register 2, 			Address offset: 0x04	*/
	__vo uint32_t SR;		/*!< SPI status register,    			Address offset: 0x08	*/
	__vo uint32_t DR;		/*!< SPI data register,		 			Address offset: 0x0C	*/
	__vo uint32_t CRCPR;	/*!< SPI CRC polynomial register,		Address offset: 0x10	*/
	__vo uint32_t RXCRCR;	/*!< SPI RX CRC register,				Address offset: 0x14	*/
	__vo uint32_t TXCRCR;	/*!< SPI TX CRC register,				Address offset: 0x18 */
	__vo uint32_t I2SCFGR;	/*!< SPI_I2S configuration register,	Address offset: 0x1C	*/
	__vo uint32_t I2SPR;	/*!< SPI_I2S prescalar register, 		Address offset: 0x20	*/
}SPI_RegDef_t;


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

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7) )

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
#define SPI4_PCLK_EN()  ( RCC->APB2ENR |= (1 << 13) )

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
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )

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
#define SPI4_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 13) )

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

/*
 * Reset macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Returns port code for (0 - 7) for a given GPIO base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0 : \
									 (x == GPIOB)? 1 : \
									 (x == GPIOC)? 2 : \
									 (x == GPIOD)? 3 : \
									 (x == GPIOE)? 4 : \
									 (x == GPIOF)? 5 : \
									 (x == GPIOG)? 6 : \
									 (x == GPIOH)? 7 : 0)

/******************************************************************************
 * 								Generic Macros
 ******************************************************************************/

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F446XX_H_ */
