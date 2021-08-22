/*
 * 006_SPI_Ardiuno_Tx_and_Rx.c
 *
 *  Created on: 21-Aug-2021
 *      Author: romanandhan l
 */


#include "STM32G_SPI.h"
#include "STM32G_GPIO.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*
 * PA1 - SPI1_SCK
 * PA2 - SPI1_MOSI
 * PA4 - SPI1_NSS
 * PA6 - SPI1_MISO
 * Alternste Functionality mode = 0
 */

/* Command Codes */

#define COMMAND_LED_CTRL 			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ 			0x52
#define COMMAND_PRINT	 			0x53
#define COMMAND_ID_READ 			0x54

#define LED_ON 						ENABLE
#define LED_OFF 					DISABLE

/* Ardiuno analog pins */
#define ANALOG_PIN0 				0
#define ANALOG_PIN1 				1
#define ANALOG_PIN2 				2
#define ANALOG_PIN3 				3
#define ANALOG_PIN4 				4

/* Ardiuno LED */
#define LED_PIN 					9


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;
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

uint8_t SPI_Verify_Response(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		/* ACK */
		return 1;
	}
	else
	{
		/* NACK */
		return 0;
	}
}
int main(void)
{
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

	/* 1. CMD_LED_CTRL 		<PIN_NO_1> 			<VALUE>	 */

	uint8_t command_code = COMMAND_LED_CTRL;
	uint8_t ackbyte = 0;
	uint8_t args[2];
	uint8_t dummy_read = 0;
	uint8_t dummy_write = 0xff;

	SPI_sendData(SPI1, &command_code, 1);

	/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
	SPI_ReceiveData(SPI1, &dummy_read, 1);

	/* send some dummy bytes to fetch the response of the slave */
	SPI_sendData(SPI1, &dummy_write, 1);

	/* Receive the ACK from the arduino */
	SPI_ReceiveData(SPI1, &ackbyte, 1);

	/* Check if the received msg is ACK or not */
	if(SPI_Verify_Response(ackbyte))
	{
		/* Send the arguments */
		args[0] = LED_PIN;
		args[1] = LED_ON;

		SPI_sendData(SPI1, args, 2);

		/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
		SPI_ReceiveData(SPI1, &dummy_read, 2);
	}


	/* End of CMD_LED_CTRL */

	/* 2. CMD_SENSOR_READ 		<analog pin No>
	 */
	/* wait till the button is pressed */

	while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
	delay();

	command_code = COMMAND_SENSOR_READ;

	/* Send the command */

	SPI_sendData(SPI1, &command_code, 1);

	/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		/* send some dummy bytes to fetch the response of the slave */
		SPI_sendData(SPI1, &dummy_write, 1);

		/* Receive the ACK from the arduino */
		SPI_ReceiveData(SPI1, &ackbyte, 1);

		/* Check if the received msg is ACK or not */
		if(SPI_Verify_Response(ackbyte))
		{
			/* Send the arguments */
			args[0] = ANALOG_PIN0;

			SPI_sendData(SPI1, args, 1);

			/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
			SPI_ReceiveData(SPI1, &dummy_read, 1);

			/* Insert some delay so that slave can be ready with the data */
			delay();

			/* send some dummy bytes to fetch the response of the slave */
			SPI_sendData(SPI1, &dummy_write, 1);

			/* fetch the sensor value */
			uint8_t analog_read;
			SPI_ReceiveData(SPI1, &analog_read, 1);

		}
		/* End of command sensor read */

		/* 3. CMD_LED_READ 			<PIN NO> */

		/* wait till the button is pressed */

		while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
		delay();

		command_code = 	COMMAND_LED_READ;

		/* Send the command */

		SPI_sendData(SPI1, &command_code, 1);

		/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		/* send some dummy bytes to fetch the response of the slave */
		SPI_sendData(SPI1, &dummy_write, 1);

		/* Receive the ACK from the arduino */
		SPI_ReceiveData(SPI1, &ackbyte, 1);

		/* Check if the received msg is ACK or not */
		if(SPI_Verify_Response(ackbyte))
		{
			args[0] = LED_PIN;

			SPI_sendData(SPI1, args, 1);

			/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
			SPI_ReceiveData(SPI1, &dummy_read, 1);

			/* Insert some delay so that slave can be ready with the data */
			delay();

			/* send some dummy bytes to fetch the response of the slave */
			SPI_sendData(SPI1, &dummy_write, 1);

			uint8_t led_read;
			SPI_ReceiveData(SPI1, &led_read, 1);

		}
		/* End of LED status read */

		/* 4. CMD_PRINT    		<LEN> 		<MSG> */

		/* wait till the button is pressed */

		while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
		delay();

		command_code = 	COMMAND_PRINT;
		uint8_t message[] = "Hello to everyone!";

		/* Send the command */

		SPI_sendData(SPI1, &command_code, 1);

		/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		/* send some dummy bytes to fetch the response of the slave */
		SPI_sendData(SPI1, &dummy_write, 1);

		/* Receive the ACK from the arduino */
		SPI_ReceiveData(SPI1, &ackbyte, 1);

		/* Check if the received msg is ACK or not */
		if(SPI_Verify_Response(ackbyte))
		{
			/* send the length of the message */
			args[0] = strlen((char*)message);

			SPI_sendData(SPI1, args, 1);

			/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
			SPI_ReceiveData(SPI1, &dummy_read, 1);

			/* Now send the message */

			for(uint8_t i = 0; i<args[0]; i++)
			{
				SPI_sendData(SPI1, &message[i], 1);

				/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
				SPI_ReceiveData(SPI1, &dummy_read, 1);
			}
		}
		/* End of CMD_PRINT */

		/* 5. CMD_ID_READ */

		/* wait till the button is pressed */

		while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
		delay();

		command_code = COMMAND_ID_READ;

		/* Send the command */

		SPI_sendData(SPI1, &command_code, 1);

		/* Do dummy read to clear of the RXNE flag as in SPI when 1 byte of data is sent, it also receives 1 byte of garbage value, we need to clear it in Rxbufer */
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		/* send some dummy bytes to fetch the response of the slave */
		SPI_sendData(SPI1, &dummy_write, 1);

		/* Receive the ACK from the arduino */
		SPI_ReceiveData(SPI1, &ackbyte, 1);

		uint8_t i = 0;
		uint8_t id[11];
		/* Check if the received msg is ACK or not */
		if(SPI_Verify_Response(ackbyte))
		{
			for(i=0; i<10; i++)
			{
				/* send some dummy bytes to fetch the response of the slave */
				SPI_sendData(SPI1, &dummy_write, 1);

				/* receive the ID from the slave */
				SPI_ReceiveData(SPI1, &id[i], 1);
			}
		}

	/* Check whether the Busy flag is set or not */

	while(SPI_Get_Flag_Status(SPI1, SPI_BUSY_FLAG));

	/* Disable the SPI1 peripheral */

	SPI_Peripheral_Control(SPI1, DISABLE);
	}

	return 0;
}
