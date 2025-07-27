#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H
#include "main.h"  // 包含HAL库头文件


#define MPU6050_IIC_GPIO                   GPIOB
#define MPU6050_IIC_SCL_PIN                GPIO_PIN_6
#define MPU6050_IIC_SDA_PIN                GPIO_PIN_7

// 使用HAL库的GPIO读写函数
#define MPU6050_IIC_SCL(state)             HAL_GPIO_WritePin(MPU6050_IIC_GPIO, MPU6050_IIC_SCL_PIN, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define MPU6050_IIC_SDA(state)             HAL_GPIO_WritePin(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_PIN, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define MPU6050_IIC_SDA_IN                 HAL_GPIO_ReadPin(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_PIN)

#define MPU6050_IIC_delay_4us()            delay_us(4)

// 函数声明保持不变
void MPU6050_IIC_IO_Init(void);
void MPU6050_IIC_SDA_IO_OUT(void);
void MPU6050_IIC_SDA_IO_IN(void);
void MPU6050_IIC_Start(void);
void MPU6050_IIC_Stop(void);
void MPU6050_IIC_Send_Byte(uint8_t txd);
uint8_t MPU6050_IIC_Read_Byte(uint8_t ack);
uint8_t MPU6050_IIC_Read_Ack(void);
void MPU6050_IIC_Send_Ack(uint8_t ack);

#endif
















