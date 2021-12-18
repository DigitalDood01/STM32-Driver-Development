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
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTs peripheral
 */

typedef struct
{
	USART_Config_t USART_Config;
	USART_RegDef_t *pUSARTx;
}USART_Handle_t;

/* Possible Modes in USART */
#define USART_MODE_ONLY_TX 					0
#define USART_MODE_ONLY_RX 					1
#define USART_MODE_TXRX 					2

/* Possible Baud Rate options in USART */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					1200
#define USART_STD_BAUD_9600					1200
#define USART_STD_BAUD_19200				19200
#define USART_STD_BAUD_38400				38400
#define USART_STD_BAUD_57600				57600
#define USART_STD_BAUD_115200				115200
#define USART_STD_BAUD_230400				230400
#define USART_STD_BAUD_460800				460800
#define USART_STD_BAUD_921600				921600
#define USART_STD_BAUD_2M					2000000
#define USART_STD_BAUD_3M					3000000

/* Possible PARITY control options in USART */
#define USART_PARITY_EN_ODD 				1
#define USART_PARITY_EN_EVEN 				2
#define USART_PARITY_DISABLE 				0

/* Possible word length options in USART */
#define USART_WORD_LENGTH_8BITS				0
#define USART_WORD_LENGTH_9BITS				1

/* Possible stop bits options in USART */
#define USART_STOPBITS_1					0
#define USART_STOPBITS_0_5					1
#define USART_STOPBITS_2					2
#define USART_STOPBITS_1_5					3

/* Possible Hardware flow control options in USART */
#define USART_HW_FLOW_CTRL_NONE				0
#define USART_HW_FLOW_CTRL_CTS				1
#define USART_HW_FLOW_CTRL_RTS				2
#define USART_HW_FLOW_CTRL_CTS_RTS			3


/* I2C related status flags definitions */

#define USART_PARITY_ERROR_FLAG					(1 << USART_ISR_PE)
#define USART_FRAMING_ERROR_FLAG				(1 << USART_ISR_FE)
#define USART_NOISE_DETECTION_FLAG				(1 << USART_ISR_NE)
#define USART_OVERRUN_ERROR_FLAG				(1 << USART_ISR_ORE)
#define USART_IDLE_FLAG							(1 << USART_ISR_IDLE)
#define USART_RXFNE_FLAG						(1 << USART_ISR_RXFNE)
#define USART_TC_FLAG							(1 << USART_ISR_TC)
#define USART_TXFNF_FLAG						(1 << USART_ISR_TXFNF)
#define USART_LINE_BREAK_FLAG					(1 << USART_ISR_LBDF)
#define USART_NOISE_DETECTION_FLAG				(1 << USART_ISR_NE)
#define USART_CTS_FLAG							(1 << USART_ISR_CTS)
#define USART_BAUD_ERROR_FLAG					(1 << USART_ISR_ABRE)
#define USART_BUSY_FLAG							(1 << USART_ISR_BUSY)
#define USART_CHARACTER_MATCH_FLAG				(1 << USART_ISR_CMF)
#define USART_TXFIFO_FLAG						(1 << USART_ISR_TXFIFO)
#define USART_RXFIFO_FLAG						(1 << USART_ISR_RXFF)

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
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);



/* Application callback */

void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t Application_Event);

#endif /* INC_STM32G_USART_H_ */
