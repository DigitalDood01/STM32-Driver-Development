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
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FM_Duty_Cycle;

}I2C_Config_t;

/* Handle structure for I2Cx peripheral */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;



#endif /* INC_STM32G_I2C_H_ */
