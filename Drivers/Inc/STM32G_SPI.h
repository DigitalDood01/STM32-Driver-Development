/*
 * STM32G_SPI.h
 *
 *  Created on: Aug 17, 2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G_SPI_H_
#define INC_STM32G_SPI_H_

#include "STM32G070.h"

/* Configuration structure for SPIx peripherals */

typedef struct
{
	uint8_t SPI_DeviceMode;			/* Master or slave mode */
	uint8_t SPI_BusConfig;			/* simplex/full-duplex/half-duplex */
	uint8_t SPI_CRCL;				/* data frame format 8-bit or 16-bit */
	uint8_t SPI_CPHA;				/* clock phase */
	uint8_t SPI_CPOL;				/* clock polarity */
	uint8_t SPI_SSM;				/* software or hardware slave management */
	uint8_t SPI_Speed;				/* Configuring the baud rate for transmission */

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;

/******************************************* Macros for accessing the elements in SPI_Config_t structure**********************************************************/

/* Macros for device mode configuration */

#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE 			0

/* Macros for SPI bus configuration  */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*Macros for configuring the serial clock speed */

#define SPI_SCLK_SPEED_DIV2 			0
#define SPI_SCLK_SPEED_DIV4 			1
#define SPI_SCLK_SPEED_DIV8 			2
#define SPI_SCLK_SPEED_DIV16 			3
#define SPI_SCLK_SPEED_DIV32 			4
#define SPI_SCLK_SPEED_DIV64 			5
#define SPI_SCLK_SPEED_DIV128 			6
#define SPI_SCLK_SPEED_DIV256 			7

/* Macros for configuring the data frame format */

#define SPI_CRCL_8BITS 					0					/* DEFAULT MODE */
#define SPI_CRCL_16BITS 				1

/* Macros for configuring the clock polarity */

#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW 					0

/* Macros for configuring the clock phase */

#define SPI_CPHA_HIGH 					1
#define SPI_CPHA_LOW 					0

/* Macros for configuring the SLAVE SELECT MANAGEMENT */

#define SPI_SSM_ENABLE					0
#define SPI_SSM_DISABLE					1

/**********************************************************APIs Supported by this Driver***************************************************************************/

/* Peripheral Clock setup */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and Deint
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);							/* Resetting the SPI registers to the default values*/


/*
 * Data Send and Receive
 */
void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 */


void SPI_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle); 							/*this function handles the interrupt */

#endif /* INC_STM32G_SPI_H_ */
