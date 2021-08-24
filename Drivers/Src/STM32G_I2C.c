/*
 * STM32G_I2C.c
 *
 *  Created on: 23-Aug-2021
 *      Author: romanandhan l
 */


#include "STM32G_I2C.h"
#include <stdint.h>

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_PeriClkCtrl
 *
 * Brief 						- It enables or disables the clock for I2C in RCC registers
 *
 * Param1						- Address of I2Cx peripheral
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLOCK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLOCK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLOCK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLOCK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLOCK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLOCK_DI();
		}
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Init
 *
 * Brief 						- This API initializes the I2C_CR1 register based on the programmer's input
 *
 * Param1						- pointer for the I2C handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_Init(I2C_Handle_t *pI2C_Handle)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_DeInit
 *
 * Brief 						-  This API resets the I2C registers to its default values
 *
 * Param1						- base address of I2Cx port
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_DeInit(I2C_RegDef_t *pI2Cx)							/* Resetting the I2C registers to the default values*/
{
	/* Resetting the I2C registers to the default values*/
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}
/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Get_Flag_Status
 *
 * Brief 						- It enables or disables the clock for I2C in RCC registers
 *
 * Param1						- Address of I2Cx peripheral
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- flag is set or reset
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{

}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_IRQ_Config
 *
 * Brief 						- This function enables the interrupt, setting up the IRQ number
 *
 * Param1						-
 * Param2						-
 * Param3 						-
 *
 * Return 						-
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi)/* This function enables the interrupt, setting up the IRQ number*/
{
	/* This function enables the interrupt, setting up the IRQ number*/
	/* This involves configuring ISER register in Processor side */

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31) 				/* 0 to 31 */
		{
			//program ISER0 register
			NVIC->ISER |= (1 << IRQNumber);
		}
	}
	else
	{
		if(IRQNumber <= 31) 				/* 0 to 31 */
		{
			//program ICER0 register
			NVIC->ICER |= (1 << IRQNumber);
		}
	}

}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_IRQPriorityConfig
 *
 * Brief 						-
 *
 * Param1						-
 * Param2						-
 * Param3 						-
 *
 * Return 						-
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/* 1. Find out the IPR register	 */
	uint8_t iprx = IRQNumber / 4; 				/* There are 8 IPR registers so divide by 4 */
	uint8_t ipr_section = IRQNumber % 4;		/* Each IPR register accomodates 4 IRQ numbers, so take modulo of 4 */

	uint8_t shift_amount = ((ipr_section*8) +(8 - NO_OF_BITS_IN_PR_IMPLEMENTED));
	NVIC->IPR[iprx] |= (IRQPriority << shift_amount);
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Peripheral_Control
 *
 * Brief 						- This API enables the I2C in I2C_CR1 register
 *
 * Param1						- Base address of I2C port
 * Param2						- Enable or Disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- This API should be called after initializing all the configurations for I2C
 ************************************************************************************************************************************/

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

}
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Application_Event)
{

}
