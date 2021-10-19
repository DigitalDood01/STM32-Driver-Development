/*
 * 009_I2C_Master_Rx_Testing.c
 *
 *  Created on: 27-Sep-2021
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

uint8_t rcv_buff[32];

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
	uint8_t commandcode;
	uint8_t length;
	/* Initialize the GPIO button */
	GPIO_Button_Init();

	/* Configure the I2C_GPIO peripherals */
	I2C1_GPIOInits();

	/* Initialize the I2C peripherals */
	I2C1_Inits();

	/* Enable the I2C peripheral */
	I2C_Peripheral_Control(I2C1, ENABLE);

	/* wait for button press */
	while(1)
	{
		/* wait till the button is pressed */
		while(GPIO_ReadfromInputPin(GPIOC, GPIO_PIN_NO13));
		delay();

		commandcode = 0x51;

		I2C_MasterSendData(&I2C1_Handle, &commandcode,1, SLAVE_ADDRESS, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1_Handle, &length,1, SLAVE_ADDRESS,I2C_ENABLE_SR);

		commandcode = 0x52;

		I2C_MasterSendData(&I2C1_Handle, &commandcode,1, SLAVE_ADDRESS,I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1_Handle, rcv_buff,length, SLAVE_ADDRESS,I2C_DISABLE_SR);


	}


}
