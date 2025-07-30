#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32h7xx.h"

#define PCA9685_ADDRESS 0x80

#define PCA9685_MODE1_REG       0x00
#define PCA9685_MODE2_REG       0x01
#define PCA9685_SUBADR1_REG     0x02
#define PCA9685_SUBADR2_REG     0x03
#define PCA9685_SUBADR3_REG     0x04
#define PCA9685_ALLCALL_REG     0x05
#define PCA9685_PRESCALE_REG    0xFE
#define PCA9685_LED0_REG        0x06

void PCA9685_Write(uint8_t reg, uint8_t data);
//指定地址读一个时序（主读）
uint8_t PCA9685_ReadReg(uint8_t RegAddress);
//设置pwm频率
void PCA9685_SetPWMFreq(float freq);
//设置pwm通道
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);
//舵机和pwm角度占空比换算
void PCA9685_SetServoAngle(uint8_t channel, float angle);
//pca9685初始化
void PCA9685_Init(void);

#endif