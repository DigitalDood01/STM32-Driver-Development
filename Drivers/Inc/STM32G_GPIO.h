/*
 * STM32G_GPIO.h
 *
 *  Created on: 23-Jul-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G_GPIO_H_
#define INC_STM32G_GPIO_H_

#include "STM32G070.h"

/*Configuration structure for GPIO Pin*/
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAlfFunMode;

}GPIO_PinConfig_t;
/* handle structure for GPIO pin*/

typedef struct
{
	/* pointer to hold the base address of the GPIO peripheral*/
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;		/* THis hold the GPIO pin configuration settings*/

}GPIO_Handle_t;
#endif /* INC_STM32G_GPIO_H_ */
