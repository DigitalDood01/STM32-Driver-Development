/*
 * 003_LED_BUTTON_INTERRUPT.c
 *
 *  Created on: 07-Aug-2021
 *      Author: romanandhan l
 */


#include "STM32G_GPIO.h"
#include <stdint.h>

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

	/* Initializing the GPIO port for the LED */
	GPIO_LED.pGPIOx =GPIOA;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Output;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PUSHPULL;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* Initializing the GPIO port for the BUTTON */
	GPIO_BUTTON.pGPIOx = GPIOC;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO13;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_IP_RT;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;


	/* First call the clock control API for LED*/
	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	/* Call the clock control API for BUTTON*/
	GPIO_PeriClkCtrl(GPIOC, ENABLE);

	GPIO_Init(&GPIO_LED);

	GPIO_Init(&GPIO_BUTTON);

	/* IRQ Configurations */
	GPIO_IRQPriorityConfig(IRQ_EXTI0_1, NVIC_IRQ_PRIORITY15);

	GPIOIRQ_Config(IRQ_EXTI0_1, ENABLE);

	return 0;
	}

/* Configure the ISR(Interrupt service routine */
void EXTI0_1_IRQHandler(void)
{
	GPIOIRQ_Handling(GPIO_PIN_NO13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);
}
