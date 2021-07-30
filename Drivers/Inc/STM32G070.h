/*
 * STM32G070.h
 *
 *  Created on: 23-Jul-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G070_H_
#define INC_STM32G070_H_
#include <stdint.h>
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
	 uint32_t reserved1[12];
	volatile uint32_t EXTI_EXTICR[4];
	 uint32_t reserved2[3];
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

#define GPIO_BASE_ADDR_TO_CODE(x)			((x == GPIOA) ? 00 :\
											(x == GPIOB) ? 01 :\
											(x == GPIOC) ? 02 :\
											(x == GPIOD) ? 03 :\
											(x == GPIOE) ? 04 :\
											(x == GPIOF) ? 05 : 0 )

/* Macros for IRQ number */
#define IRQ_EXTI0_1							12
#define IRQ_EXTI2_3							13
#define IRQ_EXTI4_15						14

/* Some generic Macros*/

#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET 								DISABLE
#endif /* INC_STM32G070_H_ */
