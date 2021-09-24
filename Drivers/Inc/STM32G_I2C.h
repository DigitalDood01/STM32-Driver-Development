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
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr);

/* Get the flag the status */
uint8_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t Flagname);


/*
 * IRQ Configuration and ISR Handling
 */


void I2C_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


/* Other peripheral control APIs */

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/* Application callback */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Application_Event);
#endif /* INC_STM32G_I2C_H_ */
