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

/* Some private APIs */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_Close_Receive_Data(I2C_Handle_t *pI2C_Handle);
static void I2C_Close_Send_Data(I2C_Handle_t *pI2C_Handle);

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

	/* Enable the clock for I2C peripheral*/
	I2C_PeriClkCtrl(pI2C_Handle->pI2Cx, ENABLE);

	/* 2. Inorder to Configure the Clock for I2C get the system clock information from the RCC section */

	/* output of RCC_GetPCLK1Value() is in MHz. so divide it by 1Mhz */
	//tempreg |= ((RCC_GetPCLK1Value() / 1000000U) << I2C_TIMINGR_PRESC);
	tempreg = (RCC_GetPCLK1Value() / 1000000U);
	tempreg = tempreg - sizeof(uint8_t);
	tempreg = (tempreg << I2C_TIMINGR_PRESC);


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
 * Brief 						- This API sends data from master using blocking method
 *
 * Param1						- Address of I2C handle
 * Param2						- Address of Txbuffer
 * Param3 						- length of the Txbuffer
 * Param4						- Slave address
 * Param5 						- Enable disable macro for repeated start condition
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	/* 1. Generate the start condition */
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	/* 2. Send address of slave with R/W set as 0 */
	I2C_ExecuteAddressPhaseWrite(pI2C_Handle->pI2Cx, SlaveAddr);

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

	if(Sr == I2C_DISABLE_SR)
	{
		/* 5. Generate Stop Condition */
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_MasterReceiveData
 *
 * Brief 						- This API receives data from slave using blocking method
 *
 * Param1						- Address of I2C handle
 * Param2						- Address of Txbuffer
 * Param3 						- length of the Txbuffer
 * Param4						- Slave address
 * Param5 						- Enable disable macro for repeated start condition
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr)
{
	/*1.  Generate the Start condition */
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	/* 2. Send address of slave with R/W set as 1 */
	I2C_ExecuteAddressPhaseRead(pI2C_Handle->pI2Cx, SlaveAddr);

	/* 3. Confirm that the address phase is completed by checking the ADDR bit in ISR register */
	while(!I2C_Get_Flag_Status(pI2C_Handle->pI2Cx, I2C_ADDRESS_MATCH_FLAG));

	/* Procedure to read only 1 byte from slave */
	if(len ==1)
	{
		/* Disable acking */
		I2C_Manage_NACKing(pI2C_Handle->pI2Cx, I2C_NACK_ENABLE);

		/* Clear the Addr flag */
		I2C_ClearAddrFlag(pI2C_Handle);

		/* wait until the RXNE flag is set to 1 */
		while(!I2C_Get_Flag_Status(pI2C_Handle->pI2Cx, I2C_RXNE_FLAG));

		if(Sr == I2C_DISABLE_SR)
		{
			/* Generate the stop condition */
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}

		/* read data from the buffer */
		*pRxBuffer = pI2C_Handle->pI2Cx->I2C_RXDR;
	}

	/* Procedure to read more than 1 byte of data */
	if(len > 1)
	{
		/* Clear the Addr flag */
		I2C_ClearAddrFlag(pI2C_Handle);

		for(uint32_t i = len; i > 0; i--)
		{
			/* wait until the RXNE flag is set to 1 */
			while(!I2C_Get_Flag_Status(pI2C_Handle->pI2Cx, I2C_RXNE_FLAG));

			if(i == 2) /* if only last 2 bytes are remaining */
			{
				/* Disable acking */
				I2C_Manage_NACKing(pI2C_Handle->pI2Cx, I2C_NACK_ENABLE);

				if(Sr == I2C_DISABLE_SR)
				{
					/* Generate the stop condition */
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}
			}
			/* read data from the buffer */
			*pRxBuffer = pI2C_Handle->pI2Cx->I2C_RXDR;

			/* Increment the buffer address */
			pRxBuffer = pRxBuffer + sizeof(uint8_t);
		}
	}
	/* Re-enable the ACKing */
	I2C_Manage_NACKing(pI2C_Handle->pI2Cx, I2C_NACK_ENABLE);
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_MasterSendDataInterrupt
 *
 * Brief 						- This API sends data from master using interrupt mode
 *
 * Param1						- Address of I2C handle
 * Param2						- Address of Txbuffer
 * Param3 						- length of the Txbuffer
 * Param4						- Slave address
 * Param5 						- Enable disable macro for repeated start condition
 * Return 						- status of Tx
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t I2C_MasterSendDataInterrupt(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2C_Handle->TxRxStatus;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2C_Handle->pTxBuffer = pTxBuffer;
		pI2C_Handle->TxLen = len;
		pI2C_Handle->TxRxStatus = I2C_BUSY_IN_TX;
		pI2C_Handle->DevAddr = SlaveAddr;
		pI2C_Handle->Sr = Sr;

		/*1.  Generate the Start condition */
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		/*2 Enable the TXIE bit in I2C_CR1 register*/
		pI2C_Handle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_TXIE);
	}
	return busystate;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_MasterReceiveDataInterrupt
 *
 * Brief 						- This API receives data from slave using interrupt method
 *
 * Param1						- Address of I2C handle
 * Param2						- Address of Txbuffer
 * Param3 						- length of the Txbuffer
 * Param4						- Slave address
 * Param5 						- Enable disable macro for repeated start condition
 * Return 						- status of Rx
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t I2C_MasterReceiveDataInterrupt(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2C_Handle->TxRxStatus;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2C_Handle->pRxBuffer = pRxBuffer;
		pI2C_Handle->RxLen = len;
		pI2C_Handle->TxRxStatus = I2C_BUSY_IN_RX;
		pI2C_Handle->DevAddr = SlaveAddr;
		pI2C_Handle->Sr = Sr;

		/*1.  Generate the Start condition */
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		/*2 Enable the RXIE bit in I2C_CR1 register*/
		pI2C_Handle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_RXIE);
	}
	return busystate;
}
/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Get_Flag_Status
 *
 * Brief 						- It returns the state of the flags in I2C peripheral
 *
 * Param1						- Address of I2Cx peripheral
 * Param2						- Macro of the flag name
 * Param3 						-
 *
 * Return 						- flag is set or reset
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{
	if(pI2Cx->I2C_ISR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_IRQ_Config
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
 * Brief 						- This API configures the interrupt priority
 *
 * Param1						- IEQ number
 * Param2						- IRQ priority
 * Param3 						-
 *
 * Return 						- None
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
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Application_Event)
{

}


/**************************************Private Functions for I2C***********************************************************************************************/

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_GenerateStartCondition
 *
 * Brief 						- This API enables the START bit in I2C_CR2 register
 *
 * Param1						- Base address of I2C port
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	uint32_t tempreg = 0;
	tempreg |= (1 << I2C_CR2_START);
	pI2Cx->I2C_CR2 |= tempreg;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_ExecuteAddressPhaseWrite
 *
 * Brief 						- This API loads the slave address to the TxDR register(R/W = 0)
 *
 * Param1						- Base address of I2C port
 * Param2						- Address of the slave
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/* shift the slave address by one bit, making space for the R/W bit */
	SlaveAddr = SlaveAddr <<1;
	/* Clear the 0th bit as write operation is need to be performed */
	SlaveAddr &= ~(1);
	/* Load the slave address in DR register */
	pI2Cx->I2C_TXDR = SlaveAddr;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_ExecuteAddressPhaseWrite
 *
 * Brief 						- This API loads the slave address to the TxDR register(R/W = 1)
 *
 * Param1						- Base address of I2C port
 * Param2						- Address of the slave
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/* shift the slave address by one bit, making space for the R/W bit */
	SlaveAddr = SlaveAddr <<1;
	/* Set the 0th bit as read operation is need to be performed */
	SlaveAddr |= 1;
	/* Load the slave address in DR register */
	pI2Cx->I2C_TXDR = SlaveAddr;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_GenerateStopCondition
 *
 * Brief 						- This API enables the stop bit  in I2C_CR2 register
 *
 * Param1						- Base address of I2C port
 * Param2						- Address of the slave
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	uint32_t tempreg = 0;
	tempreg |= (1 << I2C_CR2_STOP);
	pI2Cx->I2C_CR2 |= tempreg;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_ClearAddrFlag
 *
 * Brief 						- This API clears the Addr flag   in I2C_ICR register
 *
 * Param1						- Base address of I2C port
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- This flag is set by the hardware. In order to clear the flag ADDRCF bit in ISR register should be set to 1
 ******************************************************************************************************************************************/
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle)
{
	if(pI2C_Handle->TxRxStatus == I2C_BUSY_IN_RX)
	{
	if(pI2C_Handle->RxLen ==1)
	{
		/* manage the NACK */
		I2C_Manage_NACKing(pI2C_Handle->pI2Cx, ENABLE);

		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_ADDRCF);
	}
	pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_ADDRCF);
	}
	else
	{
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_ADDRCF);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Manage_NACKing
 *
 * Brief 						- This API enables or diables the NACK bit in I2C_CR2 register
 *
 * Param1						- Base address of I2C port
 * Param2						- Enable or diable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void I2C_Manage_NACKing(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_NACK_DISABLE)
	{
		/* Enable the ack */
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_NACK);
	}
	else
	{
		/* Enable the Nack */
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_NACK);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Close_Receive_Data
 *
 * Brief 						- This API is for closing the I2C reception process with interrupt
 *
 * Param1						- I2C handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

static void I2C_Close_Receive_Data(I2C_Handle_t *pI2C_Handle)
{

	pI2C_Handle->TxRxStatus = I2C_READY;
	pI2C_Handle->pRxBuffer = NULL;
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->RxSize = 0;
}
static void I2C_Close_Send_Data(I2C_Handle_t *pI2C_Handle)
{
	pI2C_Handle->TxRxStatus = I2C_READY;
	pI2C_Handle->pTxBuffer = NULL;
	pI2C_Handle->TxLen = 0;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_Close_Receive_Data
 *
 * Brief 						- This API is for closing the I2C transmission process with interrupt
 *
 * Param1						- I2C handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

/*********************************************************************************************************************************
 *
 * Function Name 				- I2C_IRQ_Handling
 *
 * Brief 						- This API is for handling the interrupt when Rx/Tx flag is affected or due to any error condictions
 *
 * Param1						- I2C handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void I2C_IRQ_Handling(I2C_Handle_t *pI2C_Handle)
{
	/* Interrupt handling for both master and slave mode of a device */

	uint32_t temp1, temp2;
	/*1. Interrupt handler for TXE event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_TXIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_TXE);

	if(temp1 && temp2)
	{
		/* handle the TXe event */
		/* The TXE flag is set */
		/* do the data transmission */

		if(pI2C_Handle->TxRxStatus == I2C_BUSY_IN_TX)
		{
			if(pI2C_Handle->TxLen == 0)
			{
				/* Load the data into the Data register */
				pI2C_Handle->pI2Cx->I2C_TXDR = *(pI2C_Handle->pTxBuffer);

				/* Decrement the Txlen */
				pI2C_Handle->TxLen--;
				/* Increment the TxBuffer */
				pI2C_Handle->pTxBuffer++;
			}
		}
		I2C_Close_Send_Data(pI2C_Handle);
	}


	/*2. Interrupt handler for RXe event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_RXIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_RXNE);

	if(temp1 && temp2)
	{
		/* handle the RXe event */
		/* The RXNE flag is set */
		/* Do the data reception*/

		if(pI2C_Handle->TxRxStatus == I2C_BUSY_IN_RX)
		{
			if(pI2C_Handle->RxLen == 1)
			{
				*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->I2C_RXDR;

				pI2C_Handle->RxLen--;

			}

			if(pI2C_Handle->RxSize > 1)
			{
				if(pI2C_Handle->RxLen ==2)
				{
					/* manage the NACK */
					I2C_Manage_NACKing(pI2C_Handle->pI2Cx, ENABLE);
				}

				/* Read the RxData Register */
				*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->I2C_RXDR;

				pI2C_Handle->RxLen--;

				pI2C_Handle->pRxBuffer++;
			}

			if(pI2C_Handle->RxLen == 0)
			{
				/* Close the I2C data reception and notify the application*/

				/*1. Generate the stop condition*/
				if(pI2C_Handle->Sr == I2C_DISABLE_SR)
				{
					/* Generate the stop condition */
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}

				I2C_Close_Receive_Data(pI2C_Handle);

				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_RX_COMPLETE);
			}
		}

	}

	/*3.Interrupt handler for STOP event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_STOPIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_STOPF);

	if(temp1 && temp2)
	{
		/* handle the STOP event */
		/* The stop flag is set */
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_STOPCF);

		/* Notify the Application that the STOP is detected */
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_COMPLETE);
	}

	/*4.Interrupt handler for Transfer Complete event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_TCIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_TC);

	if(temp1 && temp2)
	{
		/* handle the Transfer Complete event */
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_STOP);
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_COMPLETE);
	}

	/*5.Interrupt handler for Address Match event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_ADDRIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_ADDR);

	if(temp1 && temp2)
	{
		/* handle the Address Match event */
		/* Clear the ADDR flag */
		I2C_ClearAddrFlag(pI2C_Handle);
	}

	/*6.Interrupt handler for NACK Reception event */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_NACKIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_NACKF);

	if(temp1 && temp2)
	{
		/* handle the NACK Reception event */
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_NACKCF);
	}

	/**************************************************************************Interrupt Handling for Errors in I2C********************************************/

	temp1 = 0;
	temp2 = 0;

	/*1. Interrupt handler for bus error */
	temp1 = pI2C_Handle->pI2Cx->I2C_CR1 & (1 << I2C_CR1_ERRIE);
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_BERR);

	if(temp1 && temp2)
	{
		/* Handle the bus error */
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_BERRCF);
	}

	/*2. Interrupt handler for Arbitration loss */
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_ARLO);

	if(temp1 && temp2)
	{
		/* Handle the Arbitration loss */
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_ARLOCF);
	}

	/*3. Interrupt handler for Overrun/Underrun error */
	temp2 = pI2C_Handle->pI2Cx->I2C_ISR & (1 << I2C_ISR_OVR);

	if(temp1 && temp2)
	{
		/* Handle the Overrun/Underrun error */
		pI2C_Handle->pI2Cx->I2C_ICR |= (1 << I2C_ICR_OVRCF);
	}


}
