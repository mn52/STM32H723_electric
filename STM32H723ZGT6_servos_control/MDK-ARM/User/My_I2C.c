#include "stm32h7xx.h"
#include "My_I2C.h"
#include "stm32h7xx_hal.h"  
void MyI2C_W_SCL(GPIO_PinState BitValue){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,BitValue);
}

void MyI2C_W_SDA(GPIO_PinState BitValue){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,BitValue);
}

uint8_t MyI2C_R_SDA(void){
    // 直接返回读取结果（0或1）
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1 : 0;
}
//将SDA设置为输出模式
static void SDA_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}

//设置SDA为输入模式
static void SDA_In(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}
//把PB6/7初始化为高电平
void MyI2C_Init(void){

	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	
}

//开始
void MyI2C_Start(void){
	SDA_Out();
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
	
}
//结束
void MyI2C_Stop(void){
	SDA_Out();
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
	
}
//发送一个字节
void MyI2C_SendByte(uint8_t Byte){
	SDA_Out();
	uint8_t i;
	for(i=0;i<8;i++){
		MyI2C_W_SDA(Byte & (0x80 >> i));
	    MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}
//接受一个字节
 uint8_t MyI2C_ReceiveByte(void){
	 uint8_t Byte=0x00;
	 MyI2C_W_SDA(1);
	 for(int i=0;i<8;i++){
	 MyI2C_W_SCL(1);
	 if (MyI2C_R_SDA() == 1){ Byte |= (0x80 >> i); }
	 MyI2C_W_SCL(0);
	 }
	 return Byte;
}
 //发送应答
void MyI2C_SendACK(uint8_t ACKBit){
	MyI2C_W_SDA(ACKBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}
//接收应答
uint8_t MyI2C_ReceiveACK(void){
	uint8_t ACKBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	ACKBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return ACKBit;
}
