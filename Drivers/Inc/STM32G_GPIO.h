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

/***************************************************APIs supported by this driver******************************************************************/

/* Peripheral clock setup */

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init and DeInit*/

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);							/* Resetting the GPIO register to the default values*/

/* Data read and Write*/

uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber);
uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGIOx);							/* Port is of 16 pins*/
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber);

/* IRQ handling and Configuration*/

void GPIOIRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void GPIOIRQ_Handling(uint8_t PinNumber); 							/*this function handles the interrupt of the GPIO pin number */


#endif /* INC_STM32G_GPIO_H_ */
