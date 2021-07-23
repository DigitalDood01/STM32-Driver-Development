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

#endif /* INC_STM32G070_H_ */
