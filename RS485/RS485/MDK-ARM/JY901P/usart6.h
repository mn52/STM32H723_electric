#ifndef __USART6_H
#define __USART6_H

#include "stm32h7xx_hal.h"

// UART_HandleTypeDef 全局变量声明
extern UART_HandleTypeDef huart6;

// 函数声明
void Usart6Init(unsigned int uiBaud);
void Uart6Send(unsigned char *p_data, unsigned int uiSize);


void Error_Handler(void);



#endif 
