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
	uint8_t GPIO_PinAltFunMode;
	uint8_t GPIO_PinInterruptMode;

}GPIO_PinConfig_t;
/* handle structure for GPIO pin*/

typedef struct
{
	/* pointer to hold the base address of the GPIO peripheral*/
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;		/* THis hold the GPIO pin configuration settings*/

}GPIO_Handle_t;

/* Macros for pin numbers of GPIO port */

#define GPIO_PIN_NO0 				0
#define GPIO_PIN_NO1 				1
#define GPIO_PIN_NO2 				2
#define GPIO_PIN_NO3 				3
#define GPIO_PIN_NO4 				4
#define GPIO_PIN_NO5 				5
#define GPIO_PIN_NO6 				6
#define GPIO_PIN_NO7 				7
#define GPIO_PIN_NO8 				8
#define GPIO_PIN_NO9 				9
#define GPIO_PIN_NO10 				10
#define GPIO_PIN_NO11 				11
#define GPIO_PIN_NO12 				12
#define GPIO_PIN_NO13 				13
#define GPIO_PIN_NO14 				14
#define GPIO_PIN_NO15 				15

/* Macros for the different possible  modes in GPIO peripherals*/

#define GPIO_Mode_Input 			0
#define GPIO_Mode_Output 			1
#define GPIO_Mode_Alt_Fun			2
#define GPIO_Mode_Analog 			3

/* Macros for different Interrupt modes in GPIO Peripherals */
#define GPIO_No_Interrupt			0
#define GPIO_Interrupt_FT 			1 				/* IP_FT = Input falling edge  */
#define GPIO_Interrupt_RT 			2 				/* IP_RT = Input raising edge */
#define GPIO_Interrupt_RFT 			3 				/* IP_RFT = Input falling/raising edge */

/* Macros for possible GPIO output types */

#define GPIO_OP_TYPE_PUSHPULL		0
#define GPIO_OP_TYPE_OPENDRAIN		1

/* Macros for different speed modes in GPIO */

#define GPIO_SPEED_VERYLOW			0
#define GPIO_SPEED_LOW				1
#define GPIO_SPEED_HIGH				2
#define GPIO_SPEED_VERYHIGH			3

/* Macros for GPIO pullup/pulldown configuration */

#define GPIO_NO_PUPD				0
#define GPIO_PULLUP					1
#define GPIO_PULLDOWN				2


/***************************************************APIs supported by this driver******************************************************************/

/* Peripheral clock setup */

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init and DeInit*/

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);							/* Resetting the GPIO register to the default values*/

/* Data read and Write*/

uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx);							/* Port is of 16 pins*/
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ handling and Configuration*/

void GPIOIRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi);/* This function enables the interrupt, setting up the IRQ number*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIOIRQ_Handling(uint8_t PinNumber); 							/*this function handles the interrupt of the GPIO pin number */


#endif /* INC_STM32G_GPIO_H_ */
