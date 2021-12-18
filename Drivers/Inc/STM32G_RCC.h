/*
 * STM32G_RCC.h
 *
 *  Created on: 16-Dec-2021
 *      Author: romanandhan l
 */

#ifndef INC_STM32G_RCC_H_
#define INC_STM32G_RCC_H_

#include "STM32G070.h"



uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutput_Clock(void);


#endif /* INC_STM32G_RCC_H_ */
