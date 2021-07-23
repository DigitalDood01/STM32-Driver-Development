/*
 * STM32G070.h
 *
 *  Created on: 23-Jul-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G070_H_
#define INC_STM32G070_H_

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
#define DMA1_BASE_ADDR 						(AHB1PERIPH_BASE + Ox0000)
#define DMA2_BASE_ADDR 						(AHB1PERIPH_BASE + Ox0400)
#define RCC_BASE_ADDR 						(AHB1PERIPH_BASE + Ox1000)
#define EXTI_BASE_ADDR						(AHB1PERIPH_BASE + Ox1800)
#define FLASH_BASE_ADDR						(AHB1PERIPH_BASE + Ox2000)
#define CRC_BASE_ADDR 						(AHB1PERIPH_BASE + Ox3000)

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
#define I2C3_BASE_ADDR						(APB1PERIPH_BASE + 0x5C00)
#define SYSCFG_BASE_ADDR					(APB1PERIPH_BASE + 0x5C00)
#define ADC_BASE_ADDR						(APB1PERIPH_BASE + 0x5C00)
#define TIM1_BASE_ADDR						(APB1PERIPH_BASE + 0x5C00)
#define SPI1_BASE_ADDR						(APB1PERIPH_BASE + 0x5C00)

/* Base addresses on peripherals hanging on APB2 bus */
#define USART1_BASE_ADDR 					(APB2PERIPH_BASE + Ox3800)
#define USART6_BASE_ADDR					(APB2PERIPH_BASE + Ox3C00)
#define TIMER15_BASE_ADDR					(APB2PERIPH_BASE + Ox4000)
#define TIMER16_BASE_ADDR					(APB2PERIPH_BASE + Ox4400)
#define TIMER17_BASE_ADDR					(APB2PERIPH_BASE + Ox4800)
#endif /* INC_STM32G070_H_ */
