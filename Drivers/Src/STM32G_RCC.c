/*
 * STM32G_RCC.c
 *
 *  Created on: 16-Dec-2021
 *      Author: romanandhan l
 */

#include "STM32G_RCC.h"
#include <stdint.h>


uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 32, 64, 128, 256};
uint16_t APB1_PreScalar[4] = {2,4,8,16};

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

uint32_t RCC_GetPLLOutput_Clock(void)
{
	return 0;
}
