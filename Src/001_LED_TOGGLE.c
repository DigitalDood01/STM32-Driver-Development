/*
 * 001_LED_TOGGLE.c
 *
 *  Created on: 28-Jul-2021
 *      Author: romanandhan l
 */
#include "STM32G_GPIO.h"
#include <stdint.h>


void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}
int main(void)
{
	GPIO_Handle_t GPIO_LED; 			/* Create a variable for GPIO LED*/

	/* Initializing the GPIO port for the application */
	GPIO_LED.pGPIOx =GPIOA;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO5;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Output;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PUSHPULL;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* First call the clock control API*/
	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_LED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);
		delay();
	}
	return 0;
}
