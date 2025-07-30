#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "stm32h7xx.h"

#define IIC_SCL_PIN        GPIO_PIN_6
#define IIC_SCL_PORT       GPIOB
#define IIC_SDA_PIN        GPIO_PIN_7
#define IIC_SDA_PORT       GPIOB


//把PB6/7初始化为高电平
void MyI2C_Init(void);

//开始
void MyI2C_Start(void);
//结束
void MyI2C_Stop(void);
//发送一个字节
void MyI2C_SendByte(uint8_t Byte);
//接受一个字节
uint8_t MyI2C_ReceiveByte(void);
 //发送应答
void MyI2C_SendACK(uint8_t ACKBit);
//接收应答
uint8_t MyI2C_ReceiveACK(void);


#endif