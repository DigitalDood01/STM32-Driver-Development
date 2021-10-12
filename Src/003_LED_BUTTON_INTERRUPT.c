/*
 * 003_LED_BUTTON_INTERRUPT.c
 *
 *  Created on: 07-Aug-2021
 *      Author: romanandhan l
 */


#include "STM32G_GPIO.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 		    ENABLE
#define LOW			    DISABLE
#define BUTTON_PRESSED  HIGH
#define BUTTON_RELEASED LOW
void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}
int main(void)
{
	GPIO_Handle_t GPIO_LED, GPIO_BUTTON; 			/* Create a variable for GPIO LED and GPIO BUTTON*/

	/* Before configuring the elements of structure initialize it to zero as it is a local variable
	 * This is done using memset fuction*/

	memset(&GPIO_LED,0, sizeof(GPIO_LED));
	memset(&GPIO_BUTTON,0, sizeof(GPIO_BUTTON));

	/* Initializing the GPIO port for the LED */
	GPIO_LED.pGPIOx =GPIOA;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Output;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PUSHPULL;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_LED.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_No_Interrupt;

	/* First call the clock control API for LED*/
	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_LED);

	/* Initializing the GPIO port for the BUTTON */
	GPIO_BUTTON.pGPIOx = GPIOC;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO13;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Input;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_Interrupt_RFT;

	/* Call the clock control API for BUTTON*/
	GPIO_PeriClkCtrl(GPIOC, ENABLE);

	GPIO_Init(&GPIO_BUTTON);

	/* IRQ Configurations */
	GPIO_IRQ_Config(IRQ_EXTI4_15, ENABLE);

	GPIO_IRQPriorityConfig(IRQ_EXTI4_15, NVIC_IRQ_PRIORITY_EXTI_4_15);
	while(1);

}

/* Configure the ISR(Interrupt service routine */
void EXTI4_15_IRQHandler(void)
{
	delay();
	GPIO_IRQ_Handling(GPIO_Interrupt_RFT, GPIO_PIN_NO13); 		/* Clear the pending event from EXTI line */
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);
}


