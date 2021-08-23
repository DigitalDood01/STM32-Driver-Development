/*
 * STM32G070.h
 *
 *  Created on: 23-Jul-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G070_H_
#define INC_STM32G070_H_
#include <stdint.h>

/***********************************************************PROCESSOR SPECIFIC DETAILS*********************************************************************/
/* ARM CORTEX M0+ NVIC  base address  */

#define NVIC_BASE_ADDR						0xE000E100U


#define NO_OF_BITS_IN_PR_IMPLEMENTED		2


/*Base addresses of flash and SRAM memory*/

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U			/* The value is unsigned*/
#define ROM_BASEADDR 						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR


/*Bus peripherals base addresses*/
#define PERIPH_BASE 						0x40000000U
#define APB1PERIPH_BASE 					PERIPH_BASE
#define APB2PERIPH_BASE 					0x40010000U
#define AHB1PERIPH_BASE 					0x40020000U
#define IOPORT_BASE							0x50000000U

/* Base addresses of IO port peripherals*/
#define GPIOA_BASEADDR						(IOPORT_BASE + 0x0000)
#define GPIOB_BASEADDR						(IOPORT_BASE + 0x0400)
#define GPIOC_BASEADDR						(IOPORT_BASE + 0x0800)
#define GPIOD_BASEADDR						(IOPORT_BASE + 0x0C00)
#define GPIOE_BASEADDR						(IOPORT_BASE + 0x1000)
#define GPIOF_BASEADDR						(IOPORT_BASE + 0x1400)

/* Base addresses on peripherals hanging on AHB1 bus */
#define DMA1_BASE_ADDR 						(AHB1PERIPH_BASE + 0x0000)
#define DMA2_BASE_ADDR 						(AHB1PERIPH_BASE + 0x0400)
#define RCC_BASE_ADDR 						(AHB1PERIPH_BASE + 0x1000)
#define EXTI_BASE_ADDR						(AHB1PERIPH_BASE + 0x1800)
#define FLASH_BASE_ADDR						(AHB1PERIPH_BASE + 0x2000)
#define CRC_BASE_ADDR 						(AHB1PERIPH_BASE + 0x3000)

/* Base addresses on peripherals hanging on APB1 bus */
#define TIMER3_BASE_ADDR					(APB1PERIPH_BASE + 0x0400)
#define TIMER4_BASE_ADDR					(APB1PERIPH_BASE + 0x0800)
#define TIMER6_BASE_ADDR					(APB1PERIPH_BASE + 0x1000)
#define TIMER7_BASE_ADDR					(APB1PERIPH_BASE + 0x1400)
#define RTC_BASE_ADDR						(APB1PERIPH_BASE + 0x2800)
#define SPI2_BASE_ADDR						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE_ADDR						(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE_ADDR					(APB1PERIPH_BASE + 0x4400)
#define USART3_BASE_ADDR					(APB1PERIPH_BASE + 0x4800)
#define USART4_BASE_ADDR					(APB1PERIPH_BASE + 0x4C00)
#define USART5_BASE_ADDR					(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE_ADDR						(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDR						(APB1PERIPH_BASE + 0x5800)
#define USB_BASE_ADDR						(APB1PERIPH_BASE + 0x5C00)
#define I2C3_BASE_ADDR						(APB1PERIPH_BASE + 0x8800)


/* Base addresses on peripherals hanging on APB2 bus */
#define SYSCFG_BASE_ADDR					(APB2PERIPH_BASE + 0x0000)
#define ADC_BASE_ADDR						(APB2PERIPH_BASE + 0x2400)
#define TIM1_BASE_ADDR						(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE_ADDR						(APB2PERIPH_BASE + 0x3000)
#define USART1_BASE_ADDR 					(APB2PERIPH_BASE + Ox3800)
#define USART6_BASE_ADDR					(APB2PERIPH_BASE + Ox3C00)
#define TIMER15_BASE_ADDR					(APB2PERIPH_BASE + Ox4000)
#define TIMER16_BASE_ADDR					(APB2PERIPH_BASE + Ox4400)
#define TIMER17_BASE_ADDR					(APB2PERIPH_BASE + Ox4800)


/**************************************Peripheral register Definition structures *****************************************************************/

/* Peripheral register definition structure for GPIO*/
typedef struct
{
	volatile uint32_t MODER; 				/* volatile type qualifier is used to make variables compliant from reading from processor everytime*/
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;
}GPIO_RegDef_t;

/* Register definition structure for SPI */
typedef struct
{
	volatile uint32_t SPIx_CR1;
	volatile uint32_t SPIx_CR2;
	volatile uint32_t SPIx_SR;
	volatile uint32_t SPIx_DR;
	volatile uint32_t SPIx_CRCPR;
	volatile uint32_t SPIx_RXCRCR;
	volatile uint32_t SPIx_TXCRCR;
	volatile uint32_t SPIx_I2SCFGR;
	volatile uint32_t SPIx_I2SPR;
}SPI_RegDef_t;

/* Register definition structure for I2C */
typedef struct
{
	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_TIMINGR;
	volatile uint32_t I2C_TIMEOUTR;
	volatile uint32_t I2C_ISR;
	volatile uint32_t I2C_ICR;
	volatile uint32_t I2C_PECR;
	volatile uint32_t I2C_RXDR;
	volatile uint32_t I2C_TXDR;
}I2C_RegDef_t;

/* Register definition structure for RCC(Reset Clock Control)*/
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t ICSCR;
	volatile uint32_t CFGR;
	volatile uint32_t PLL_CFGR;
	uint32_t reserved1[2];
	volatile uint32_t CIER;
	volatile uint32_t CIFR;
	volatile uint32_t CICR;
	volatile uint32_t IOPRSTR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t APBRSTR1;
	volatile uint32_t APBRSTR2;
	volatile uint32_t IOPENR;
	volatile uint32_t AHBENR;
	volatile uint32_t APBENR1;
	volatile uint32_t APBENR2;
	volatile uint32_t IPOSMENR;
	volatile uint32_t AHBSMENR;
	volatile uint32_t APBSMENR1;
	volatile uint32_t APBSMENR2;
	volatile uint32_t CCIPR;
	volatile uint32_t CCIPR2;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
}RCC_RegDef_t;

/* EXTI peripheral definition structure */

typedef struct
{
	volatile uint32_t EXTI_RTSR1;
	volatile uint32_t EXTI_FTSR1;
	volatile uint32_t EXTI_SWIER1;
	volatile uint32_t EXTI_RPR1;
	volatile uint32_t EXTI_FPR1;
	 uint32_t reserved1[19];
	volatile uint32_t EXTI_EXTICR[4];
	 uint32_t reserved2[4];
	volatile uint32_t EXTI_IMR1;
	volatile uint32_t EXTI_EMR1;
}EXTI_RegDef_t;

/* SYSCFG structure definition */

typedef struct
{
	volatile uint32_t SYSCFG_CFGR1;
	uint32_t reserved1[5];
	volatile uint32_t SYSCFG_CFGR2;
	uint32_t reserved2[38];
	volatile uint32_t SYSCFG_ITLINE0;
	uint32_t reserved3;
	volatile uint32_t SYSCFG_ITLINE2;
	volatile uint32_t SYSCFG_ITLINE3;
	volatile uint32_t SYSCFG_ITLINE4;
	volatile uint32_t SYSCFG_ITLINE5;
	volatile uint32_t SYSCFG_ITLINE6;
	volatile uint32_t SYSCFG_ITLINE7;
	volatile uint32_t SYSCFG_ITLINE8;
	volatile uint32_t SYSCFG_ITLINE9;
	volatile uint32_t SYSCFG_ITLINE10;
	volatile uint32_t SYSCFG_ITLINE11;
	volatile uint32_t SYSCFG_ITLINE12;
	volatile uint32_t SYSCFG_ITLINE13;
	volatile uint32_t SYSCFG_ITLINE14;
	uint32_t reserved4;
	volatile uint32_t SYSCFG_ITLINE16;
	volatile uint32_t SYSCFG_ITLINE17;
	volatile uint32_t SYSCFG_ITLINE18;
	volatile uint32_t SYSCFG_ITLINE19;
	volatile uint32_t SYSCFG_ITLINE20;
	volatile uint32_t SYSCFG_ITLINE21;
	volatile uint32_t SYSCFG_ITLINE22;
	volatile uint32_t SYSCFG_ITLINE23;
	volatile uint32_t SYSCFG_ITLINE24;
	volatile uint32_t SYSCFG_ITLINE25;
	volatile uint32_t SYSCFG_ITLINE26;
	volatile uint32_t SYSCFG_ITLINE27;
	volatile uint32_t SYSCFG_ITLINE28;
	volatile uint32_t SYSCFG_ITLINE29;
}SYSCFG_RegDef_t;

/* Structure for elements in NVIC region */
typedef struct
{
	volatile uint32_t ISER;
	uint32_t reserved1[31];
	volatile uint32_t ICER;
	uint32_t reserved2[31];
	volatile uint32_t ISPR;
	uint32_t reserved3[31];
	volatile uint32_t ICPR;
	uint32_t reserved4[95];
	volatile uint32_t IPR[8];
}NVIC_RegDef_t;

/* peripheral definitions(peripheral base addresses typecasted to xxx_RegDef_t) */

#define GPIOA 								((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 								((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI 								((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG								((SYSCFG_RegDef_t)SYSCFG_BASE_ADDR)

/* Macro definition for base address for NVIC */
#define NVIC 								((NVIC_RegDef_t*)NVIC_BASE_ADDR)

/* Macro definition for base addresses for SPIx peripherals */
#define SPI1								((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2								((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3								((SPI_RegDef_t*)SPI3_BASE_ADDR)

/* Clock enable Macros for GPIO peripherals */
#define GPIOA_PCLOCK_EN()					(RCC->IOPENR |= (1<<0))
#define GPIOB_PCLOCK_EN()					(RCC->IOPENR |= (1<<1))
#define GPIOC_PCLOCK_EN()					(RCC->IOPENR |= (1<<2))
#define GPIOD_PCLOCK_EN()					(RCC->IOPENR |= (1<<3))
#define GPIOE_PCLOCK_EN()					(RCC->IOPENR |= (1<<4))
#define GPIOF_PCLOCK_EN()					(RCC->IOPENR |= (1<<5))


/* Clock enable Macros for I2C peripherals */
#define I2C1_PCLOCK_EN()					(RCC->APBENR1 |= (1<<21))
#define I2C2_PCLOCK_EN()					(RCC->APBENR1 |= (1<<22))
#define I2C3_PCLOCK_EN()					(RCC->APBENR1 |= (1<<23))

/* Clock enable Macros for SPI peripherals */
#define SPI1_PCLOCK_EN()					(RCC->APBENR2 |= (1<<12))
#define SPI2_PCLOCK_EN()					(RCC->APBENR1 |= (1<<14))
#define SPI3_PCLOCK_EN()					(RCC->APBENR1 |= (1<<15))

/* Clock enable Macros for USART peripherals */
#define USART1_PCLOCK_EN()					(RCC->APBENR2 |= (1<<14))
#define USART2_PCLOCK_EN()					(RCC->APBENR1 |= (1<<17))
#define USART3_PCLOCK_EN()					(RCC->APBENR1 |= (1<<18))
#define USART4_PCLOCK_EN()					(RCC->APBENR1 |= (1<<19))
#define USART5_PCLOCK_EN()					(RCC->APBENR1 |= (1<<8))
#define USART6_PCLOCK_EN()					(RCC->APBENR1 |= (1<<9))

/* Clock enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLOCK_EN()					(RCC->APBENR2 |= (1<<0))

/* Clock disable Macros for GPIO peripherals */
#define GPIOA_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<0))
#define GPIOB_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<1))
#define GPIOC_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<2))
#define GPIOD_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<3))
#define GPIOE_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<4))
#define GPIOF_PCLOCK_DI()					(RCC->IOPENR &= ~(1<<5))


/* Clock disable Macros for I2C peripherals */
#define I2C1_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<21))
#define I2C2_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<22))
#define I2C3_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<23))

/* Clock disable Macros for SPI peripherals */
#define SPI1_PCLOCK_DI()					(RCC->APBENR2 &= ~(1<<12))
#define SPI2_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<14))
#define SPI3_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<15))

/* Clock disable Macros for USART peripherals */
#define USART1_PCLOCK_DI()					(RCC->APBENR2 &= ~(1<<14))
#define USART2_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<17))
#define USART3_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<18))
#define USART4_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<19))
#define USART5_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<8))
#define USART6_PCLOCK_DI()					(RCC->APBENR1 &= ~(1<<9))

/* Clock disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLOCK_DI()					(RCC->APBENR2 &= ~(1<<0))

/* Macros to reset the GPIO peripherals */
#define GPIOA_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<0));	(RCC->IOPRSTR &= ~(1<<0)); } while(0)
#define GPIOB_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<1));	(RCC->IOPRSTR &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<2));	(RCC->IOPRSTR &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<3));	(RCC->IOPRSTR &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<4));	(RCC->IOPRSTR &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET()					do{ (RCC->IOPRSTR |= (1<<5));	(RCC->IOPRSTR &= ~(1<<5)); } while(0)

/* Macros to reset the SPI peripherals */

#define SPI1_REG_RESET()					do{ (RCC->APBENR2 |= (1<<12));  (RCC->APBENR2 &= ~(1<<12)); } while(0)
#define SPI2_REG_RESET()					do{ (RCC->APBENR1 |= (1<<14));  (RCC->APBENR1 &= ~(1<<14)); } while(0)
#define SPI3_REG_RESET()					do{ (RCC->APBENR1 |= (1<<15));  (RCC->APBENR1 &= ~(1<<15)); } while(0)

#define GPIO_BASE_ADDR_TO_CODE(x)			((x == GPIOA) ? 00 :\
											(x == GPIOB) ? 01 :\
											(x == GPIOC) ? 02 :\
											(x == GPIOD) ? 03 :\
											(x == GPIOE) ? 04 :\
											(x == GPIOF) ? 05 : 0 )

/* Macros for IRQ number */
/* For GPIO*/

#define IRQ_EXTI0_1							5
#define IRQ_EXTI2_3							6
#define IRQ_EXTI4_15						7

/* For SPI */
#define IRQ_SPI1 							25
#define IRQ_SPI2_3							26


/* Macros for IRQ priority */

#define NVIC_IRQ_PRIORITY0 					0
#define NVIC_IRQ_PRIORITY1 					1
#define NVIC_IRQ_PRIORITY2 					2
#define NVIC_IRQ_PRIORITY3 					3
#define NVIC_IRQ_PRIORITY4 					4
#define NVIC_IRQ_PRIORITY5 					5
#define NVIC_IRQ_PRIORITY6 					6
#define NVIC_IRQ_PRIORITY7 					7
#define NVIC_IRQ_PRIORITY8 					8
#define NVIC_IRQ_PRIORITY9 					9
#define NVIC_IRQ_PRIORITY10 				10
#define NVIC_IRQ_PRIORITY11 				11
#define NVIC_IRQ_PRIORITY12 				12
#define NVIC_IRQ_PRIORITY13 				13
#define NVIC_IRQ_PRIORITY14 				14
#define NVIC_IRQ_PRIORITY15 				15


/* Some generic Macros*/

#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET 								DISABLE
#define FLAG_RESET							RESET
#define FLAG_SET							SET


/**************************************************** Bit Position macros for SPI registers*********************************************************************/

/* for SPI_CR1 register */

#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BAUDRATE					3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSB_FIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_CRCL						11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15

/* for SPI_CR2 register */

#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_NSSP						3
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7
#define SPI_CR2_DATA_SIZE					8
#define SPI_CR2_FRXTH						12
#define SPI_CR2_LDMA_RX						13
#define SPI_CR2_LDMA_TX						14

/* for SPI_SR register */

#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8
#define SPI_SR_FRVL							9
#define SPI_SR_FTVL							11

/*****************************************************************************************************************************************************************/

/**************************************************** Bit Position macros for I2C registers*********************************************************************/

/* For I2C_CR1 Register */

#define I2C_CR1_PE							0
#define I2C_CR1_TXIE						1
#define I2C_CR1_RXIE						2
#define I2C_CR1_ADDRIE						3
#define I2C_CR1_NACKIE						4
#define I2C_CR1_STOPIE						5
#define I2C_CR1_TCIE						6
#define I2C_CR1_ERRIE						7
#define I2C_CR1_DNF							8
#define I2C_CR1_ANFOFF						12
#define I2C_CR1_TXDMAEN						14
#define I2C_CR1_RXDMAEN						15
#define I2C_CR1_SBC							16
#define I2C_CR1_NOSTRETCH					17
#define I2C_CR1_WUPEN						18
#define I2C_CR1_GCEN						19
#define I2C_CR1_SMBHEN						20
#define I2C_CR1_SMBDEN						21
#define I2C_CR1_ALERTEN						22
#define I2C_CR1_PECEN						23

/* For I2C_CR1 Register */

#define I2C_CR2_SADD						0
#define I2C_CR2_RD_WRN						10
#define I2C_CR2_ADD10						11
#define I2C_CR2_HEAD10R						12
#define I2C_CR2_START						13
#define I2C_CR2_STOP						14
#define I2C_CR2_NACK						15
#define I2C_CR2_NBYTES						23
#define I2C_CR2_RELOAD						24
#define I2C_CR2_AUTOEND						25
#define I2C_CR2_PECBYTE						26

/* For I2C_ISR Register */

#define I2C_ISR_TXE							0
#define I2C_ISR_TXIS						1
#define I2C_ISR_RXNE						2
#define I2C_ISR_ADDR						3
#define I2C_ISR_NACKF						4
#define I2C_ISR_STOPF						5
#define I2C_ISR_TC							6
#define I2C_ISR_TCR							7
#define I2C_ISR_BERR						8
#define I2C_ISR_ARLO						9
#define I2C_ISR_OVR							10
#define I2C_ISR_PECRR						11
#define I2C_ISR_TIMEOUT						12
#define I2C_ISR_ALERT						13
#define I2C_ISR_BUSY						15
#define I2C_ISR_DIR							16
#define I2C_ISR_ADDCODE						17

/* For I2C_ICR Register */

#define I2C_ICR_ADDRCF						3
#define I2C_ICR_NACKCF						4
#define I2C_ICR_STOPCF						5
#define I2C_ICR_BERRCF						8
#define I2C_ICR_ARLOCF						9
#define I2C_ICR_OVRCF						10
#define I2C_ICR_PECCF						11
#define I2C_ICR_TIMOUTCF					12
#define I2C_ICR_ALERTCF						13

/* For I2C_TIMEOUTR Register */

#define I2C_TIMEOUTR_TIMEOUTA				0
#define I2C_TIMEOUTR_TIDLE					12
#define I2C_TIMEOUTR_TIMOUTEN				15
#define I2C_TIMEOUTR_TIMEOUTB				16
#define I2C_TIMEOUTR_TEXTEN					31

/* For I2C_TIMINGR Register */

#define I2C_TIMINGR_SCLL 					0
#define I2C_TIMINGR_SCLH 					8
#define I2C_TIMINGR_SDADEL 					16
#define I2C_TIMINGR_SCLDEL 					20
#define I2C_TIMINGR_PRESC 					28



#endif /* INC_STM32G070_H_ */
