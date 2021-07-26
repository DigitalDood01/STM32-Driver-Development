/*
 * STM32G_GPIO.c
 *
 *  Created on: 23-Jul-2021
 *      Author: romanandhan l
 */

#include "STM32G_GPIO.h"
#include <stdint.h>

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_Init
 *
 * Brief 						- This function initializes the pin configuration of the given GPIO port
 *
 * Param1						- handle configuration for GPIO port
 * Param2						-
 * Param3 						-
 *
 * Return 						-  None
 *
 * Note 						-
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_PeriClkCtrl
 *
 * Brief 						- This function enables or disables the peripheral clock of the given GPIO port
 *
 * Param1						- Base address of the GPIO port
 * Param2						- Macro for enable or disable
 * Param3 						-
 *
 * Return 						-  None
 *
 * Note 						-
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_DeInit
 *
 * Brief 						- This function resets the GPIO register to the default values
 *
 * Param1						- Base address of the GPIO port
 * Param2						-
 * Param3 						-
 *
 * Return 						-  None
 *
 * Note 						-
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
															/* Resetting the GPIO register to the default values*/
}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_ReadfromInputPin
 *
 * Brief 						- This function reads the value from the given GPIO port and Pin
 *
 * Param1						- Base address of the GPIO port
 * Param2						- Pin number from which the value should be read
 * Param3 						-
 *
 * Return 						- Read value from the given PIN
 *
 * Note 						-
 */


uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber)
{
	return 0;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_ReadfromInputPort
 *
 * Brief 						- This function reads the value from the given GPIO port
 *
 * Param1						- Base address of the GPIO port
 * Param2						-
 * Param3 						-
 *
 * Return 						- Read value from the given port
 *
 * Note 						- the output read is of uint16_t type because there are 16 pins
 */

uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGIOx)
{
	/* Port is of 16 pins*/
	return 0;
}


/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_WritetoOutputPin
 *
 * Brief 						- This function writes the value in the given pin of the GPIO port
 *
 * Param1						- Base address of the GPIO port
 * Param2						- Pin number to which the value to be written
 * Param3 						- Value to be written
 *
 * Return 						- None
 *
 * Note 						-
 */

void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber, uint8_t value)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_WritetoOutputPort
 *
 * Brief 						- This function writes the value in the given GPIO port
 *
 * Param1						- Base address of the GPIO port
 * Param2						- Pin number to which the value to be written
 * Param3 						- Value to be written
 *
 * Return 						- None
 *
 * Note 						- the output written is of uint16_t type because there are 16 pins
 */

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGIOx, uint16_t value)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIO_ToggleOutputPin
 *
 * Brief 						- This function toggles the value in the given GPIO pin
 *
 * Param1						- Base address of the GPIO port
 * Param2						- Pin number to which the value need to be toggled
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGIOx, uint8_t PinNumber)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIOIRQ_Config
 *
 * Brief 						- This function configures the Interrupt Request(IRQ)
 *
 * Param1						- IRQ number
 * Param2						- Priority
 * Param3 						- enable or disable macro
 *
 * Return 						- None
 *
 * Note 						-
 */

void GPIOIRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
	/* This function enables the interrupt, setting up the IRQ number*/
}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIOIRQ_Handling
 *
 * Brief 						- This function defines how to handle the Interrupt
 *
 * Param1						- Pin number
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 */
void GPIOIRQ_Handling(uint8_t PinNumber)
{
	/*this function handles the interrupt of the GPIO pin number */
}

