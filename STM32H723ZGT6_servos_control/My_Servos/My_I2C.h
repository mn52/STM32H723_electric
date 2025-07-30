#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H


#include "stm32h7xx_hal.h"

// 定义GPIO引脚和端口
#define IIC_SCL_PIN GPIO_PIN_6
#define IIC_SDA_PIN GPIO_PIN_7
#define IIC_SCL_PORT GPIOB
#define IIC_SDA_PORT GPIOB

// 延时函数声明
void iic_delay_PCA(void);

// I2C初始化函数声明
void IIC_Init_PCA(void);

// I2C启动信号函数声明
void IIC_Start_PCA(void);

// I2C停止信号函数声明
void IIC_Stop_PCA(void);

// I2C等待应答信号函数声明
int IIC_WaitAck_PCA(void);

// I2C发送ACK应答信号函数声明
void IIC_Ack_PCA(void);

// I2C发送NACK应答信号函数声明
void IIC_NAck_PCA(void);

// I2C发送一个字节函数声明
void IIC_SendByte_PCA(int data_t);

// I2C读取一个字节函数声明
int IIC_ReadByte_PCA(int ack);

#endif // MY_I2C_H


