#include "stm32h7xx.h"
#include "My_I2C.h"
#include "PCA9685.h"
#include <math.h>
//指定地址写一个字节（主写）
void PCA9685_Write(uint8_t reg, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(PCA9685_ADDRESS); // 设备地址 + 写模式
    MyI2C_ReceiveACK(); // 等待ACK
    MyI2C_SendByte(reg); // 寄存器地址
    MyI2C_ReceiveACK();
    MyI2C_SendByte(data); // 数据
    MyI2C_ReceiveACK();
    MyI2C_Stop();
}
//指定地址读一个时序（主读）
uint8_t PCA9685_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						
	MyI2C_SendByte(PCA9685_ADDRESS);	
	MyI2C_ReceiveACK();					
	MyI2C_SendByte(RegAddress);			
	MyI2C_ReceiveACK();					
	
	MyI2C_Start();						
	MyI2C_SendByte(PCA9685_ADDRESS | 0x01);	
	MyI2C_ReceiveACK();					
	Data = MyI2C_ReceiveByte();			
	MyI2C_SendACK(1);					
	MyI2C_Stop();	
	
	return Data;
}

//设置pwm频率
void PCA9685_SetPWMFreq(float freq) {
    // 计算预分频值 
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;          // 12位分辨率
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5);
    
    // 进入睡眠模式设置预分频
    uint8_t oldmode = PCA9685_ReadReg(PCA9685_MODE1_REG);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // 设置睡眠模式
    PCA9685_Write(PCA9685_MODE1_REG, newmode);
    PCA9685_Write(PCA9685_PRESCALE_REG, prescale);
    PCA9685_Write(PCA9685_MODE1_REG, oldmode);
    HAL_Delay(5);
    PCA9685_Write(PCA9685_MODE1_REG, oldmode | 0x80); // 重启
}
//设置pwm通道
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = PCA9685_LED0_REG + 4 * channel;
    
    MyI2C_Start();
    MyI2C_SendByte(PCA9685_ADDRESS);
    MyI2C_ReceiveACK();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveACK();
    
    MyI2C_SendByte(on & 0xFF);       
    MyI2C_ReceiveACK();
    MyI2C_SendByte(on >> 8);          
    MyI2C_ReceiveACK();
    MyI2C_SendByte(off & 0xFF);       
    MyI2C_ReceiveACK();
    MyI2C_SendByte(off >> 8);         
    MyI2C_ReceiveACK();
    
    MyI2C_Stop();
}
//舵机和pwm角度占空比换算
void PCA9685_SetServoAngle(uint8_t channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 角度转PWM (0.5ms-2.5ms对应0-180度)
    float pulse = 500.0 + (2000.0 * angle / 180.0);
    uint16_t value = (uint16_t)(pulse * 4096.0 / 20000.0); 
    
    PCA9685_SetPWM(channel, 0, value);
}
//pca9685初始化
void PCA9685_Init(void) {
    MyI2C_Init();
    HAL_Delay(100);
    
    // 复位PCA9685
    PCA9685_Write(PCA9685_MODE1_REG, 0x00);
    PCA9685_Write(PCA9685_MODE2_REG, 0x04); // 输出驱动模式
    
    // 设置PWM频率为50Hz 
    PCA9685_SetPWMFreq(50.0);
    
    // 唤醒芯片
    uint8_t mode1 = PCA9685_ReadReg(PCA9685_MODE1_REG);
    PCA9685_Write(PCA9685_MODE1_REG, mode1 & ~0x10); // 清除睡眠位
    HAL_Delay(5);
}
