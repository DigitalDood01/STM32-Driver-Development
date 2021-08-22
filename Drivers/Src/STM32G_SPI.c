/*
 * STM32G_SPI.c
 *
 *  Created on: Aug 17, 2021
 *      Author: romanandhan l
 */


#include "STM32G_SPI.h"
#include <stdint.h>


/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_Init
 *
 * Brief 						- This API initializes the SPI_CR1 register based on the programmer's input
 *
 * Param1						- pointer for the SPI handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/

void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	/* Enable the peripheral clock control */

	SPI_PeriClkCtrl(pSPI_Handle->pSPIx, ENABLE);

	/* Configure the SPI1_CR1 Register */

	/* 1. Configure the device mode */
	uint32_t tempreg = 0;
	tempreg |= pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	/* 2 Configure the bus configuration */
	if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* BIDI bit in SPI_CR1 Register should be cleared */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* BIDI bit in SPI_CR1 Register should be set */
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/* BIDI bit in SPI_CR1 Register should be cleared */

		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		/* RXONLY bit in SPI_CR1 Register should be set */
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* 3. Configure the data length format(whether it is 8 bit or 16 bit) */
	tempreg |= pSPI_Handle->SPI_Config.SPI_CRCL << SPI_CR1_CRCL;

	/* 4. Configure thhe clock polarity(idle state of the clock pulse) for SPI communication */
	tempreg |= pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* 5. Configure thhe clock phase(data capture at which transition of clock pulse) for SPI communication */
	tempreg |= pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* 6 Configure the baud rate(clock speed) for SPI transmission */
	tempreg |= pSPI_Handle->SPI_Config.SPI_Speed << SPI_CR1_BAUDRATE;

	/* 7. Configure whether the slave select management is done by hardware or software */
	tempreg |= pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	/* copy the contents in tempreg variable to SPI_CR1 register */
	pSPI_Handle->pSPIx->SPIx_CR1 = tempreg;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_Get_Flag_Status
 *
 * Brief 						- It enables or disables the clock for SPI in RCC registers
 *
 * Param1						- Address of SPIx peripheral
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- flag is set or reset
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t SPI_Get_Flag_Status(SPI_RegDef_t *pSPIx, uint32_t Flagname)
{
	if(pSPIx->SPIx_SR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_PeriClkCtrl
 *
 * Brief 						- It enables or disables the clock for SPI in RCC registers
 *
 * Param1						- Address of SPIx peripheral
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLOCK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLOCK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLOCK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLOCK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLOCK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLOCK_DI();
		}
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_DeInit
 *
 * Brief 						-  This API resets the SPI registers to its default values
 *
 * Param1						- base address of SPIx port
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	/* Resetting the SPI registers to the default values*/
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI1_REG_RESET();
	}
}


/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_sendData
 *
 * Brief 						- This API is to transmit data via SPI communication
 *
 * Param1						- Base address of SPI port
 * Param2						- address of Tx buffer
 * Param3 						- length of the data to be transmitted
 *
 * Return 						-
 *
 * Note 						- This is a blocking call
 ************************************************************************************************************************************/

void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len >0)
	{
		/* 1 Wait until the TXE flag is set  */

		while( SPI_Get_Flag_Status(pSPIx, SPI_TXE_FLAG) != FLAG_SET );

		/* 2. Check the CRCL bit in SPI_CR1 register(8-bit or 16-bit data) */

		if(pSPIx->SPIx_CR1 & (1 << SPI_CR1_CRCL))
		{
			/* 16-bit data format */

			/* 3. Load the data into the Data Register */

			pSPIx->SPIx_DR = (*(uint16_t *)pTxBuffer);
			len--;
			len--;
			/* Increment the Txbuffer so that it points to the next data item */
			pTxBuffer += 2;

		}
		else
		{
			/* 8-bit data format */

			/* 3. Load the data into the Data Register */

			pSPIx->SPIx_DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_ReceiveData
 *
 * Brief 						- This API is to receive data via SPI communication
 *
 * Param1						- Base address of SPI port
 * Param2						- address of Rx buffer
 * Param3 						- length of the data to be transmitted
 *
 * Return 						-
 *
 * Note 						-
 ************************************************************************************************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len >0)
	{
		/* 1 Wait until the RXNE flag is set  */

		while( SPI_Get_Flag_Status(pSPIx, SPI_RXNE_FLAG) == FLAG_SET );

		/* 2. Check the CRCL bit in SPI_CR1 register(8-bit or 16-bit data) */

		if(pSPIx->SPIx_CR1 & (1 << SPI_CR1_CRCL))
		{
			/* 16-bit data format */

			/* 3. Read the data from the Data Register */

			(*(uint16_t *)pRxBuffer) = pSPIx->SPIx_DR;
			len--;
			len--;
			/* Increment the Txbuffer so that it points to the next data item */
			pRxBuffer += 2;

		}
		else
		{
			/* 8-bit data format */

			/* 3. Read the data from the Data Register */

			(*pRxBuffer) = pSPIx->SPIx_DR;
			len--;
			pRxBuffer++;
		}
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_sendData_Interrupt
 *
 * Brief 						- This API is to receive data via SPI communication
 *
 * Param1						- address of SPI handle structure
 * Param2						- address of Tx buffer
 * Param3 						- length of the data to be transmitted
 *
 * Return 						-
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t SPI_sendData_Interrupt(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	/* 1. Save the Tx buffer address and length information in global variables */

	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = len;

	/* 2. Mark SPI state as busy in transmission so that, no other code can take over same SPI peripheral until transmission is over */

	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	/* 3. Enable TXEIE control bit in SPI_CR2 register to get interrupt whenever TXE flag is set */

	pSPIHandle->pSPIx->SPIx_CR2 |= (1<<SPI_CR2_TXEIE);

	/* 4. Data transmission will be handled by the ISR code */
	}

	return state;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_ReceiveData_Interrupt
 *
 * Brief 						- This API is to receive data via SPI communication
 *
 * Param1						- address of SPI handle structure
 * Param2						- address of Rx buffer
 * Param3 						- length of the data to be transmitted
 *
 * Return 						-
 *
 * Note 						-
 ************************************************************************************************************************************/

uint8_t SPI_ReceiveData_Interrupt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
	/* 1. Save the Tx buffer address and length information in global variables */

	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = len;

	/* 2. Mark SPI state as busy in transmission so that, no other code can take over same SPI peripheral until transmission is over */

	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	/* 3. Enable TXEIE control bit in SPI_CR2 register to get interrupt whenever TXE flag is set */

	pSPIHandle->pSPIx->SPIx_CR2 |= (1<<SPI_CR2_RXNEIE);

	/* 4. Data transmission will be handled by the ISR code */
	}

	return state;
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_Peripheral_Control
 *
 * Brief 						- This API enables the SPI in SPI_CR1 register
 *
 * Param1						- Base address of SPI port
 * Param2						- Enable or Disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- This API should be called after initializing all the configurations for SPI
 ************************************************************************************************************************************/
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPIx_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPIx_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_SSI_Config
 *
 * Brief 						- This API enables the SSI bit in SPI_CR1 register
 *
 * Param1						- Base address of SPI port
 * Param2						- Enable or Disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- SSI should be set in non multi master mode when SSM bit in SPI_CR1 register is set
 ************************************************************************************************************************************/
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPIx_CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPIx_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_SSIOE_Config
 *
 * Brief 						- This API enables the SSOE bit in SPI_CR2 register
 *
 * Param1						- Base address of SPI port
 * Param2						- Enable or Disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						- SSOE bit should be set when hardware slave select management is selected.
 ************************************************************************************************************************************/
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPIx_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPIx_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_IRQ_Config
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

void SPI_IRQ_Config(uint8_t IRQNumber,  uint8_t EnorDi)
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
 * Function Name 				- SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/* 1. Find out the IPR register	 */
	uint8_t iprx = IRQNumber / 4; 				/* There are 8 IPR registers so divide by 4 */
	uint8_t ipr_section = IRQNumber % 4;		/* Each IPR register accomodates 4 IRQ numbers, so take modulo of 4 */

	uint8_t shift_amount = ((ipr_section*8) +(8 - NO_OF_BITS_IN_PR_IMPLEMENTED));
	NVIC->IPR[iprx] |= (IRQPriority << shift_amount);
}


/*********************************************************************************************************************************
 *
 * Function Name 				- SPI_IRQ_Handling
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

void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle) 							/*this function handles the interrupt */
{
	uint8_t temp1, temp2;

	/* Lets check for the occurrence of TXE interrupt */
	/* Store the state of TXE flag in temp1 local variable */

	temp1 = pSPI_Handle->pSPIx->SPIx_SR & (1 << SPI_SR_TXE);

	/* Store the state of TXEIE flag in temp1 local variable */

	temp2 = pSPI_Handle->pSPIx->SPIx_CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		/* Handle TXE */
		SPI_TXE_Interrupt_Handle(pSPI_Handle);
	}

	/* Lets check for the occurrence of RXNE interrupt */
	/* Store the state of RXNE flag in temp1 local variable */

	temp1 = pSPI_Handle->pSPIx->SPIx_SR & (1 << SPI_SR_RXNE);

	/* Store the state of RXNEIE flag in temp1 local variable */

	temp2 = pSPI_Handle->pSPIx->SPIx_CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		/* Handle RXNE */
		SPI_RXNE_Interrupt_Handle(pSPI_Handle);
	}

	/*  Lets check for the occurrence of OVR(overrun) interrupt */
	/* Store the state of OVR flag in temp1 local variable */

	temp1 = pSPI_Handle->pSPIx->SPIx_SR & (1 << SPI_SR_OVR);

	/* Store the state of ERRIE flag in temp1 local variable */

	temp2 = pSPI_Handle->pSPIx->SPIx_CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		/* Handle OVR */
		SPI_OVR_ERRIE_Interrupt_Handle(pSPI_Handle);
	}
}

