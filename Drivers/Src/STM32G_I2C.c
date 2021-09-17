/*
 * STM32G_I2C.c
 *
 *  Created on: 23-Aug-2021
 *      Author: romanandhan l
 */

#include "STM32G_I2C.h"
#include <stdint.h>

uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 32, 64, 128, 256};
uint16_t APB1_PreScalar[4] = {2,4,8,16};
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

uint32_t RCC_GetPLLOutput_Clock(void)
{
	return 0;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- RCC_GetPCLK1Value
 *
 * Brief 						- Returns the APB1 clock frequency in MHz
 *
 * Param1						- None
 * Param2						- None
 * Param3 						-
 *
 * Return 						- Frequency in MHz
 *
 * Note 						-
 ************************************************************************************************************************************/

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, System_Clk,temp_AHB_PRE,temp_APB1_PRE;

	uint8_t clock_source,AHBPRE, APB1PRE;

	/* 1. Find out the Clock source */

	clock_source = (RCC->CFGR >> 3) & 0x7; /* Bring the clock information indicating bits to LSB */

	if(clock_source == 0)
	{
		/* It is HSI */

		System_Clk = 16000000U;
	}

	else if(clock_source == 1)
	{
		/* It is HSE */

		System_Clk = 8000000U; /* Based on the Crystal frequency in the Development board */
	}

	else if(clock_source == 2)
	{
		/* It is PLL */

		System_Clk = RCC_GetPLLOutput_Clock(); /* This function is not implemented currently */
	}

	/* 2. Find out the AHB prescalar Value */

	temp_AHB_PRE = (RCC->CFGR >> 8) & (0xFF); /* Bring the AHB Prescalar information indicating bits to LSB */

	if(temp_AHB_PRE < 8)
	{
		AHBPRE = 1;
	}

	else
	{
		AHBPRE = AHB_PreScalar[temp_AHB_PRE - 8];
	}

	/* 3. Find out the APB prescalar Value */

	temp_APB1_PRE = (RCC->CFGR >> 12) & (0xFFF); /* Bring the APB1 Prescalar information indicating bits to LSB */

	if(temp_APB1_PRE < 4)
	{
		APB1PRE = 1;
	}

	else
	{
		APB1PRE = APB1_PreScalar[temp_APB1_PRE - 4];
	}

	/* 4. Find out the Perpheral clock frequency */

	pclk1 = ((System_Clk / AHBPRE) / APB1PRE);

	return pclk1;
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
	uint32_t tempreg = 0;

	/* 1. Configure the NACK Control */
	tempreg |= pI2C_Handle->I2C_Config.I2C_NACKControl << I2C_CR2_NACK;
	pI2C_Handle->pI2Cx->I2C_CR2 = tempreg;
	tempreg = 0;
	/* 2. Inorder to Configure the Clock for I2C get the system clock information from the RCC section */

	/* output of RCC_GetPCLK1Value() is in MHz. so divide it by 1Mhz */
	tempreg |= ((RCC_GetPCLK1Value() / 1000000U) << I2C_TIMINGR_PRESC);
	pI2C_Handle->pI2Cx->I2C_TIMINGR = tempreg;
	tempreg = 0;

	/* 3. Disable Own address in I2C_OAR1 Register */
	tempreg &= ~(1<<15);
	pI2C_Handle->pI2Cx->I2C_OAR1 = tempreg;

	/* 4. Program the Device Own address */
	tempreg |= pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1; /* Left shifted by one as the address is 7 bit */
	pI2C_Handle->pI2Cx->I2C_OAR1 = tempreg;
	tempreg = 0;


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
 * Function Name 				- I2C_MasterSendData
 *
 * Brief 						-
 *
 * Param1						-
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	/* 1. Generate the start condition */
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	/* 2. Send address of slave with R/W set as 0 */
	I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, SlaveAddr);

	/* 3. Send the data until len becomes 0 */

	while(len > 0)
	{
	while(!I2C_Get_Flag_Status(pI2C_Handle->pI2Cx, I2C_TXE_FLAG)); /* Wait until TXE is set */
	pI2C_Handle->pI2Cx->I2C_TXDR = *pTxBuffer;
	pTxBuffer = pTxBuffer + sizeof(uint8_t);
	len--;
	}

	/* 4.Wait until TXE is set after transmission */
	while(!I2C_Get_Flag_Status(pI2C_Handle->pI2Cx, I2C_TXE_FLAG));

	/* 5. Generate Stop Condition */
	I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
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
	return 0;
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
