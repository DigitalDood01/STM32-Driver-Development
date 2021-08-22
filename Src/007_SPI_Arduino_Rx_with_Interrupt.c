/*
 * 007_SPI_Arduino_Rx_with_Interrupt.c
 *
 *  Created on: 22-Aug-2021
 *      Author: romanandhan l
 */


#include "STM32G_SPI.h"
#include "STM32G_GPIO.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

SPI_Handle_t SPI1Handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;


volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;
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

void Slave_GPIO_InterruptPinInit()
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Output;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
	spiIntPin.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_Interrupt_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;
	spiIntPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PUSHPULL;

	GPIO_Init(&spiIntPin);

	GPIO_IRQ_Config(IRQ_EXTI4_15, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_EXTI4_15, NVIC_IRQ_PRIORITY14);

}
int main(void)
{
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	//This function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOE_Config(SPI1, ENABLE);

	SPI_IRQ_Config(IRQ_SPI1, ENABLE);

	while(1){

		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		GPIO_IRQ_Config(IRQ_EXTI4_15, DISABLE);

		//enable the SPI2 peripheral
		SPI_Peripheral_Control(SPI1,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_sendData_Interrupt(&SPI1Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveData_Interrupt(&SPI1Handle, (uint8_t)&ReadByte, 1) == SPI_BUSY_IN_RX );
		}

		// confirm SPI is not busy
		while( SPI_Get_Flag_Status(SPI1,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_Peripheral_Control(SPI1,DISABLE);

		dataAvailable = 0;

		GPIO_IRQ_Config(IRQ_EXTI4_15,ENABLE);
	}
	return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void)
{

	SPI_IRQ_Handling(&SPI1Handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t Application_Event)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(Application_Event == SPI_EVENT_RX_COMPLETE)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI4_15_IRQHandler(void)
{
	GPIO_IRQ_Handling(GPIO_Interrupt_FT, GPIO_PIN_NO6);
	dataAvailable = 1;
}

