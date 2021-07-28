/*
 * 002_LED_BUTTON.c
 *
 *  Created on: 28-Jul-2021
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
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Input;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	/* First call the clock control API for LED*/
	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	/* Call the clock control API for BUTTON*/
	GPIO_PeriClkCtrl(GPIOC, ENABLE);

	GPIO_Init(&GPIO_LED);

	GPIO_Init(&GPIO_BUTTON);

	while(1)
	{
		if((GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13)) == BUTTON_RELEASED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO5);

		}


	}
	return 0;
}
