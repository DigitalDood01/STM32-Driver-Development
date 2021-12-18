/*
 * STM32G_USART.c
 *
 *  Created on: 19-Oct-2021
 *      Author: romanandhan l
 */


#include "STM32G_USART.h"
#include "STM32G_RCC.h"

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
 * Function Name 				- USART_Init
 *
 * Brief 						- This API initializes the USART_CR1, CR2, CR3 registers based on the programmer's input
 *
 * Param1						- pointer for the USART handle structure
 * Param2						-
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 ************************************************************************************************************************************/
void USART_Init(USART_Handle_t *pUSART_Handle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClkCtrl(pUSART_Handle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}
	else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}
	else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= (pUSART_Handle->USART_Config.USART_WordLength <<USART_CR1_M1) |(pUSART_Handle->USART_Config.USART_WordLength <<USART_CR1_M0) ;


    //Configuration of parity control bit fields
	if ( pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 <<USART_CR1_PS);

	}

   //Program the CR1 register
	pUSART_Handle->pUSARTx->USART_CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSART_Handle->USART_Config.USART_NoOfStopBits <<USART_CR2_STOP ;

	//Program the CR2 register
	pUSART_Handle->pUSARTx->USART_CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_RTSE) | ( 1 << USART_CR3_CTSE);

	}


	pUSART_Handle->pUSARTx->USART_CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSART_Handle->pUSARTx, pUSART_Handle->USART_Config.USART_Baud);
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



/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while (!USART_Get_Flag_Status(pUSARTHandle->pUSARTx,USART_TC_FLAG));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->USART_TDR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while (USART_Get_Flag_Status(pUSARTHandle->pUSARTx,USART_RXFNE_FLAG));


		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LENGTH_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_RDR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_RDR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_RDR);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_RDR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK1Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((PCLKx * 25)/(4 * BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * M_part)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * M_part)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->USART_BRR = tempreg;
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


