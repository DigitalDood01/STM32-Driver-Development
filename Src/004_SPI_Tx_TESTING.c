/*
 * 004_SPI_Tx_TESTING.c
 *
 *  Created on: 18-Aug-2021
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

	/* SCLK */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO1;
	GPIO_Init(&SPIPins);

	/* MISO */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
	GPIO_Init(&SPIPins);

	/* MOSI */
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO2;
	GPIO_Init(&SPIPins);

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
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_ENABLE;  					/* Software slave management is enabled for NSS pin */
	SPI1Handle.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV2; 			/* generates serial clock of 8MHz */

	/* Initialize the SPI1 peripheral */

	SPI_Init(&SPI1Handle);
}
int main(void)
{
	char user_data[] = "Hello Bro";
	/* This function is to initialize the GPIO pins to behave as SPI2 pins */

	SPI1_GPIOInits();

	/* This function is to initialize the SPI peripheral */

	SPI1_Inits();

	/* Enable the SPI1 peripheral */

	SPI_Peripheral_Control(SPI1, ENABLE);

	/* Lets send the data */

	SPI_sendData(SPI1, (uint8_t *)user_data, strlen(user_data));

	while(1);

	return 0;
}
