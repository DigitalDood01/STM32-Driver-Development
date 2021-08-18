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
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;


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
