/*
 * STM32G_USART.h
 *
 *  Created on: 19-Oct-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G_USART_H_
#define INC_STM32G_USART_H_

#include "STM32G070.h"

/* Configuration structure for USART peripheral */

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowCOntrol;
}USART_Config_t;


/*
 * Handle structure for USARTs peripheral
 */

typedef struct
{
	USART_Config_t USART_Config;
	USART_RegDef_t *pUSARTx;
}USART_Handle_t;



/**********************************************************APIs Supported by this Driver***************************************************************************/

/* Peripheral Clock setup */
void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and Deint
 */
void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_RegDef_t *pUSARTx);							/* Resetting the USART registers to the default values*/

/*
 * Send and Receive data
 */
void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len);

/*
 * Send and Receive data using interrupt
 */
uint8_t USART_SendDataInterrupt(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataInterrupt(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len);

/* Get the flag the status */
uint8_t USART_Get_Flag_Status(USART_RegDef_t *pUSARTx, uint32_t Flagname);

/*
 * IRQ Configuration and ISR Handling
 */


void USART_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQ_Handling(USART_Handle_t *pUSART_Handle);

/* Other peripheral control APIs */

void USART_Peripheral_Control(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_Manage_NACKing(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_GenerateStopCondition(USART_RegDef_t *pUSARTx);


/* Application callback */

void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t Application_Event);

#endif /* INC_STM32G_USART_H_ */
