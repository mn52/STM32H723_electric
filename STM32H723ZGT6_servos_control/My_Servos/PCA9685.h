#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32h7xx.h"
#define PCA_Addr 0x80
#define PCA_Mode1 0x00
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA_Pre 0xFE

//void Servos_Enable(void);
void PCA_Write(uint8_t Addr,uint8_t Data);
uint8_t PCA_Read(uint8_t Addr);
void PCA_SetPWM(uint8_t Channel,uint32_t ON,uint32_t OFF);
void PCA_SetFreq(float freq);
void SetAngle(uint8_t Channel, uint16_t angel);
void PCA_Init(float hz);


#endif