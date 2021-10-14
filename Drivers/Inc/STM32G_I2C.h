/*
 * STM32G_I2C.h
 *
 *  Created on: 23-Aug-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G_I2C_H_
#define INC_STM32G_I2C_H_

#include "STM32G070.h"
#include <stddef.h>

/* Configuration structure for I2C peripheral */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_NACKControl;
	uint16_t I2C_FM_Duty_Cycle;

}I2C_Config_t;

/* Handle structure for I2Cx peripheral */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 	 *pTxBuffer;
	uint8_t 	 *pRxBuffer;
	uint32_t 	  RxLen;
	uint32_t 	  TxLen;
	uint8_t 	  TxRxStatus;
	uint8_t 	  DevAddr;
	uint32_t 	  RxSize;
	uint8_t 	  Sr;
}I2C_Handle_t;

/******************************************* Macros for accessing the elements in I2C_Config_t structure***********************************************************/

/* I2C Clock speed Configuration */

#define I2C_SCL_SPEED_STD_MODE 				100000
#define I2C_SCL_SPEED_FAST_MODE4K 			400000
#define I2C_SCL_SPEED_FAST_MODE2K 			200000

/* I2C ACKControl Configuration */

#define I2C_NACK_ENABLE 					1
#define I2C_NACK_DISABLE					0

/* I2C fast mode duty cycle */

#define I2C_FM_DUTY_2
#define I2C_FM_DUTY_16_9


/* I2C related status flags definitions */

#define I2C_TXE_FLAG					(1 << I2C_ISR_TXE)
#define I2C_RXNE_FLAG					(1 << I2C_ISR_RXNE)
#define I2C_BUSY_FLAG					(1 << I2C_ISR_BUSY)
#define I2C_TIMEOUT_FLAG				(1 << I2C_ISR_TIMEOUT)
#define I2C_OVR_FLAG				    (1 << I2C_ISR_OVR)
#define I2C_ARBITRATION_LOST_FLAG		(1 << I2C_ISR_ARLO)
#define I2C_BUSERROR_FLAG				(1 << I2C_ISR_BERR)
#define I2C_TCR_FLAG					(1 << I2C_ISR_TCR)
#define I2C_TC_FLAG						(1 << I2C_ISR_TC)
#define I2C_STOP_FLAG					(1 << I2C_ISR_STOPF)
#define I2C_NACK_FLAG					(1 << I2C_ISR_NACKF)
#define I2C_ADDRESS_MATCH_FLAG			(1 << I2C_ISR_ADDR)

#define I2C_DISABLE_SR 					RESET
#define I2C_ENABLE_SR					SET

/* I2C Application states */
#define I2C_READY 						0
#define I2C_BUSY_IN_RX 					1
#define I2C_BUSY_IN_TX 					2

/* Possible I2C application events */

#define I2C_EVENT_TX_COMPLETE 			1
#define I2C_EVENT_RX_COMPLETE 			2
#define I2C_EVENT_OVR_ERR	 			3
/**********************************************************APIs Supported by this Driver***************************************************************************/

/* Peripheral Clock setup */
void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and Deint
 */
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);							/* Resetting the I2C registers to the default values*/

/*
 * Send and Receive data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);

/*
 * Send and Receive data using interrupt
 */
uint8_t I2C_MasterSendDataInterrupt(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataInterrupt(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr);

/* Get the flag the status */
uint8_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t Flagname);


/*
 * IRQ Configuration and ISR Handling
 */


void I2C_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_IRQ_Handling(I2C_Handle_t *pI2C_Handle);

/* Other peripheral control APIs */

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_Manage_NACKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


/* Application callback */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Application_Event);
#endif /* INC_STM32G_I2C_H_ */
