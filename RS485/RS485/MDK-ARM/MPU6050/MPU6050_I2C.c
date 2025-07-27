#include "MPU6050_I2C.h"

// 实现微秒延时函数
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

void MPU6050_IIC_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 开漏输出 + 上拉
    GPIO_InitStruct.Pin = MPU6050_IIC_SCL_PIN | MPU6050_IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStruct);

    // 初始状态置高
    MPU6050_IIC_SCL(1);
    MPU6050_IIC_SDA(1);
}

void MPU6050_IIC_SDA_IO_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU6050_IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStruct);
}

void MPU6050_IIC_SDA_IO_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MPU6050_IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStruct);
}

void MPU6050_IIC_Start(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SDA(1);
    MPU6050_IIC_SCL(1);
    delay_us(4);
    MPU6050_IIC_SDA(0);
    delay_us(4);
    MPU6050_IIC_SCL(0);
}

void MPU6050_IIC_Stop(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL(0);
    MPU6050_IIC_SDA(0);
    delay_us(4);
    MPU6050_IIC_SCL(1);
    delay_us(4);
    MPU6050_IIC_SDA(1);
    delay_us(4);
}

uint8_t MPU6050_IIC_Read_Ack(void)
{
    uint8_t ucErrTime = 0;
    MPU6050_IIC_SDA_IO_IN();
    MPU6050_IIC_SDA(1); // 确保内部上拉
    delay_us(4);
    MPU6050_IIC_SCL(1);
    delay_us(4);
    
    // 正确检测低电平为ACK
    while (HAL_GPIO_ReadPin(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_PIN) == GPIO_PIN_SET)//改了
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            MPU6050_IIC_Stop();
            return 1;
        }
        delay_us(2);
    }
    MPU6050_IIC_SCL(0);
    return 0;
}

void MPU6050_IIC_Send_Ack(uint8_t ack)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL(0);
    MPU6050_IIC_SDA(ack ? 1 : 0); // 0 = ACK, 1 = NACK
    delay_us(4);
    MPU6050_IIC_SCL(1);
    delay_us(4);
    MPU6050_IIC_SCL(0);
}

void MPU6050_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    MPU6050_IIC_SDA_IO_OUT();
    MPU6050_IIC_SCL(0);
    
    for (t = 0; t < 8; t++)
    {
        MPU6050_IIC_SDA((txd & 0x80) ? 1 : 0);
        txd <<= 1;
        delay_us(4);
        MPU6050_IIC_SCL(1);
        delay_us(4);
        MPU6050_IIC_SCL(0);
        delay_us(4);
    }
    MPU6050_IIC_Read_Ack();
}

uint8_t MPU6050_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    MPU6050_IIC_SDA_IO_IN();
    
    for (i = 0; i < 8; i++)
    {
        receive <<= 1;
        MPU6050_IIC_SCL(0);
        delay_us(4);
        MPU6050_IIC_SCL(1);
        if (HAL_GPIO_ReadPin(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_PIN) == GPIO_PIN_SET)
            receive++;
        delay_us(4);
    }
    MPU6050_IIC_SCL(0);
    delay_us(4);
    MPU6050_IIC_Send_Ack(ack);
    return receive;
}


