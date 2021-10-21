/*
 * STM32G_USART.c
 *
 *  Created on: 19-Oct-2021
 *      Author: romanandhan l
 */


#include "STM32G_USART.h"
#include <stdint.h>

/*********************************************************************************************************************************
 *
 * Function Name 				- USART_PeriClkCtrl
 *
 * Brief 						- It enables or disables the clock for USART in RCC registers
 *
 * Param1						- Address of USARTx peripheral
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLOCK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLOCK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLOCK_EN();
		}
		else if(pUSARTx == USART4)
		{
			USART4_PCLOCK_EN();
		}
		else if(pUSARTx == USART5)
		{
			USART5_PCLOCK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLOCK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLOCK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLOCK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLOCK_DI();
		}
		else if(pUSARTx == USART4)
		{
			USART4_PCLOCK_DI();
		}
		else if(pUSARTx == USART5)
		{
			USART5_PCLOCK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLOCK_DI();
		}
	}
}
/*********************************************************************************************************************************
 *
 * Function Name 				- USART_Peripheral_Control
 *
 * Brief 						- This API enables the USART in USART_CR1 register
 *
 * Param1						- Base address of USART port
 * Param2						- Enable or Disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- This API should be called after initializing all the configurations for USART
 ************************************************************************************************************************************/
void USART_Peripheral_Control(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- USART_Get_Flag_Status
 *
 * Brief 						- It returns the state of the flags in USART peripheral
 *
 * Param1						- Address of USARTx peripheral
 * Param2						- Macro of the flag name
 * Param3 						-
 *
 * Return 						- flag is set or reset
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t USART_Get_Flag_Status(USART_RegDef_t *pUSARTx, uint32_t Flagname)
{
	if(pUSARTx->USART_ISR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- USART_IRQ_Config
 *
 * Brief 						- This function enables the interrupt, setting up the IRQ number
 *
 * Param1						- IRQ number
 * Param2						- Enable or disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void USART_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi)/* This function enables the interrupt, setting up the IRQ number*/
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
 * Function Name 				- USART_IRQPriorityConfig
 *
 * Brief 						- This API configures the interrupt priority
 *
 * Param1						- IrQ number
 * Param2						- IRQ priority
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/* 1. Find out the IPR register	 */
	uint8_t iprx = IRQNumber / 4; 				/* There are 8 IPR registers so divide by 4 */
	uint8_t ipr_section = IRQNumber % 4;		/* Each IPR register accomodates 4 IRQ numbers, so take modulo of 4 */

	uint8_t shift_amount = ((ipr_section*8) +(8 - NO_OF_BITS_IN_PR_IMPLEMENTED));
	NVIC->IPR[iprx] |= (IRQPriority << shift_amount);
}
