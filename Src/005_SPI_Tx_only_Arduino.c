/*
 * 005_SPI_Tx_only_Arduino.c
 *
 *  Created on: Aug 19, 2021
 *      Author: romanandhan l
 */


#include "STM32G_SPI.h"
#include "STM32G_GPIO.h"
#include <stdint.h>
#include <string.h>

/*
 * PA1 - SPI1_SCK
 * PA2 - SPI1_MOSI
 * PA4 - SPI1_NSS
 * PA6 - SPI1_MISO
 * Alternste Functionality mode = 0
 */

void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Alt_Fun;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PUSHPULL;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	SPIPins.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_No_Interrupt;

	/* SCLK */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO1;
	GPIO_Init(&SPIPins);

	/* MOSI */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO2;
	GPIO_Init(&SPIPins);

	/* MISO */
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
	//GPIO_Init(&SPIPins);

	/* NSS */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO4;
	GPIO_Init(&SPIPins);

}

void SPI1_Inits(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_CRCL = SPI_CRCL_8BITS;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;  					/* Software slave management is enabled for NSS pin */
	SPI1Handle.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV8; 			/* generates serial clock of 8MHz */

	/* Initialize the SPI1 peripheral */

	SPI_Init(&SPI1Handle);
}

void GPIO_Button_Init(void)
{
	GPIO_Handle_t GPIO_BUTTON;
	/* Initializing the GPIO port for the BUTTON */
		GPIO_BUTTON.pGPIOx = GPIOC;
		GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO13;
		GPIO_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Input;
		GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GPIO_BUTTON.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_No_Interrupt;

		GPIO_Init(&GPIO_BUTTON);
}
int main(void)
{
	char user_data[] = "HelloBrother";
	/* Initialize the GPIO pins for button */

	GPIO_Button_Init();
	/* This function is to initialize the GPIO pins to behave as SPI2 pins */

	SPI1_GPIOInits();

	/* This function is to initialize the SPI peripheral */

	SPI1_Inits();

	/* Making SSOE 1 does output enable
	 * here the NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0 */

	SPI_SSOE_Config(SPI1, ENABLE);

	while(1)
	{
	/* wait till the button is pressed */

	while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
	delay();
	/* Enable the SPI1 peripheral */

	SPI_Peripheral_Control(SPI1, ENABLE);

	/* First send the length information */

	//uint8_t datalen = strlen(user_data);
	//SPI_sendData(SPI1,&datalen, 1);

	/* Lets send the data */

		SPI_sendData(SPI1,  (uint8_t*)user_data, strlen(user_data));

	/* Lets confirm that SPI is not busy */

	while(SPI_Get_Flag_Status(SPI1, SPI_BUSY_FLAG));

	/* Disable the SPI1 peripheral */

	SPI_Peripheral_Control(SPI1, DISABLE);
	}

	return 0;
}
