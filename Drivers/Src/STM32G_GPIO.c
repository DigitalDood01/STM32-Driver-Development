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
 * Note 						-  Bit wise OR is used to set the respective registers so that only the required bit positions are set
 * 								   and leaves the other bit positions as untouched
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp = 0;
	/* 1. Configure the mode of GPIO pin */

	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<= GPIO_Mode_Analog)
	{
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); /* each pin takes 2 bit fields so it is multiplied by 2*/
		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 <<(2* pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));	/* Ensuring that the required bit positions are cleared before setting it */
		pGPIO_Handle->pGPIOx->MODER |= temp;  /* set the actual register based on the mode */

		/* Non interrupt mode */

	}
	else
	{
		/*to be programmed later(Interrupt mode) */
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_Mode_IP_FT)
		{
			/* 1. COnfigure the falling trigger selection register(FTSR) */
			EXTI->EXTI_FTSR1 |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber); /* COnfigure the FTSR reg by the corresponding GPIO pin number */

			/*  Clear the  corresponding RTSR bit for safety */
			EXTI->EXTI_RTSR1 &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_Mode_IP_RT)
		{
			/*1. COnfigure the Rising trigger selection register(RTSR) */
			EXTI->EXTI_RTSR1 |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber); /* COnfigure the RTSR reg by the corresponding GPIO pin number */

			/*  Clear the  corresponding FTSR bit for safety */
			EXTI->EXTI_FTSR1 &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_Mode_IP_RFT)
		{
			/*1. Configure both RTSR and FTSR */
			EXTI->EXTI_RTSR1 |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->EXTI_FTSR1 |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/*2. Configure the GPIO port selection in EXTICR	register	 */
		uint8_t temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4; 	/* There are 4 EXTICR registers. To find out which register we divide by 4 */
		uint8_t temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;	/* to find at which position value needs to be written modulo of 4 is taken */
		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIO_Handle->pGPIOx);

		EXTI->EXTI_EXTICR[temp1] =  portcode << (8*temp2);


		/*3. Enable the EXTI interrupt delivery using IMR(Interrupt mask register) */
		EXTI->EXTI_IMR1 |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	/* 2. Configure the speed */
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));	/* Ensuring that the required bit positions are cleared before setting it */
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/* 3. Configure the PullUp/PullDown setting */
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));	/* Ensuring that the required bit positions are cleared before setting it */
	pGPIO_Handle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/* 4. Configure the output type */
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);	/* Ensuring that the required bit positions are cleared before setting it */
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/* 5. Configure the alternate functionality */
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_Mode_Alt_Fun)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8; 	/* to find which AFR register divide pin number by 8 */
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;	/* to find position to be written in register take modulo of pin number by 8 */
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF <<(4* temp2));	/* Ensuring that the required bit positions are cleared before setting it */
		pGPIO_Handle->pGPIOx->AFR[temp1] |= pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4* temp2);
	}
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
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLOCK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLOCK_DI();
		}
	}
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
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
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
 * Return 						- Read value from the given PIN (0 or 1)
 *
 * Note 						-
 */


uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = ((uint8_t)(pGPIOx->IDR >> PinNumber) & (0x00000001)); 		/* To read data in given pin number and mask it to protect the data*/
	return value;
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

uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx)
{
	/* Port is of 16 pins*/
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

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

void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == SET)
	{
		/* write 1 to the output data register at the bit field corresponding to the pin number */
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		/* write 0 to the output data register at the bit field corresponding to the pin number */
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
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

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
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

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^=(1<<PinNumber); /* ExOR operator is used to toggle the pin*/
}

/*********************************************************************************************************************************
 *
 * Function Name 				- GPIOIRQ_Config
 *
 * Brief 						- This function configures the Interrupt Request(IRQ)
 *
 * Param1						- IRQ number
 * Param2						- enable or disable macro
 * Param3 						-
 *
 * Return 						- None
 *
 * Note 						-
 */

void GPIOIRQ_Config(uint8_t IRQNumber, uint8_t EnorDi)
{
	/* This function enables the interrupt, setting up the IRQ number*/
	/* This involves configuring some register in Processor side */
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31) 				/* 0 to 31 */
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		if(IRQNumber > 31 && IRQNumber <= 64) 	/* 32 to 63 */
		{
			/* program ISER1 register */
			*NVIC_ISER1 |= (1 << IRQNumber % 32); 					/* Take modulus to write properly the IRQ number in ISER1 register */
		}
	}
	else
	{
		if(IRQNumber <= 31) 				/* 0 to 31 */
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		if(IRQNumber > 31 && IRQNumber <= 64) 	/* 32 to 63 */
		{
			/* program ICER1 register */
			*NVIC_ICER1 |= (1 << IRQNumber % 32);					/* Take modulus to write properly the IRQ number in ICER1 register */
		}
	}
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

