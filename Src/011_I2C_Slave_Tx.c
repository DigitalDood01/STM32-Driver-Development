/*
 * 011_I2C_Slave_Tx.c
 *
 *  Created on: 15-Oct-2021
 *      Author: romanandhan l
 */




#include "STM32G_I2C.h"
#include "STM32G_GPIO.h"
#include <stdint.h>
#include <string.h>

/*
 * PA9 - I2C1_SCL
 * PA10 - I2C1_SDA
 * Alternate Functionality mode = 6
 */

I2C_Handle_t I2C1_Handle;

char tx_buff[32] = "STM32 slave mode testing";

#define SLAVE_ADDRESS  0x68U

void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOA;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_Mode_Alt_Fun;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
	I2CPins.GPIO_PinConfig.GPIO_PinInterruptMode = GPIO_No_Interrupt;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OPENDRAIN;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	/*Configure the SCL */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO9;
	GPIO_Init(&I2CPins);

	/*Configure the SDA */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO10;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{

	I2C1_Handle.pI2Cx = I2C1;
	//I2C1_Handle.I2C_Config.I2C_NACKControl = I2C_NACK_DISABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = 0xFF;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD_MODE;


	/* Initialize the I2C1 peripheral */

	I2C_Init(&I2C1_Handle);
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

	/* Initialize the GPIO button */
	GPIO_Button_Init();

	/* Configure the I2C_GPIO peripherals */
	I2C1_GPIOInits();

	/* I2C IRQ configuration */
	I2C_IRQ_Config(IRQ_I2C1, ENABLE);

	/* Initialize the I2C peripherals */
	I2C1_Inits();

	/* Enable the I2C peripheral */
	I2C_Peripheral_Control(I2C1, ENABLE);

	/* wait for button press */
	while(1);
}

void I2C1_IRQHandler(void)
{
	I2C_IRQ_Handling(&I2C1_Handle);
}

uint8_t command_code = 0;
uint8_t count = 0;
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Application_Event)
{

	if(Application_Event == I2C_EVENT_DATA_REQ)
	{
		/* Master wants data, slave has to send it */

		if(command_code == 0x51)
		{
			/* send the length information to the master*/
			I2C_SlaveSendData(pI2C_Handle->pI2Cx, strlen(tx_buff));
		}

		else if(command_code == 0x52)
		{
			/* send the contents of tx buffer */
			I2C_SlaveSendData(pI2C_Handle->pI2Cx, tx_buff[count]);
			count++;
		}
	}
	else if(Application_Event == I2C_EVENT_DATA_RCV)
	{
		/* data is waiting for the slave to read, slave has to read it*/
		command_code = I2C_SlaveReceiveData(pI2C_Handle->pI2Cx);
	}
}

