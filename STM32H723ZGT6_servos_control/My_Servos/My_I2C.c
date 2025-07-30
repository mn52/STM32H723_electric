#include "stm32h7xx.h"
#include "My_I2C.h"
#include "stm32h7xx_hal.h"  



// 声明GPIO句柄
GPIO_InitTypeDef GPIO_InitStruct = {0};

void iic_delay_PCA(void) {
    /* 400KHz I2C speed on 480MHz system clock */
    for (volatile int i = 0; i < 50; i++) { // Modify this value to change I2C speed
       /* delay */
    }
}

/*************************************************************************
* 函数名称：void IIC_Init_PCA(void)
* 函数说明：模式IIC初始化
* 输入参数：
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：IIC_Init_PCA();  //模式IIC初始化 IIC命令库LQ_SOFTI2C.h文件声明
*************************************************************************/
void IIC_Init_PCA(void) {
    // 启用GPIOB时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 配置SCL引脚为开漏输出
    GPIO_InitStruct.Pin = IIC_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_PORT, &GPIO_InitStruct);

    // 配置SDA引脚为开漏输出
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    // 设置初始状态为高电平
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
}

/*************************************************************************
* 函数名称：void IIC_Start_PCA(void)
* 函数说明：模式IIC启动信号
* 输入参数：
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：IIC_Start_PCA();
*************************************************************************/
void IIC_Start_PCA(void) {
    // SDA输出模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET); // START: when CLK is high, DATA change from high to low
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET); // 结束I2C启动信号 正规规范写法
}

/*************************************************************************
* 函数名称：void IIC_Stop_PCA(void)
* 函数说明：模式IIC停止信号
* 输入参数：
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：IIC_Stop_PCA();
*************************************************************************/
void IIC_Stop_PCA(void) {
    // SDA输出模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET); // STOP: when CLK is high DATA change from low to high
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET); // 结束I2C停止信号
    iic_delay_PCA();
}

/*************************************************************************
* 函数名称：int IIC_WaitAck_PCA(void)
* 函数说明：模式IIC等待应答信号
* 输入参数：
* 返回值：1 表示应答失败    0 表示应答成功
* 修改时间：2023年6月7日
* 调用方法：内部调用  当有效应答时返回9个 SCL=0 时 SDA 仍保持不变,然后 SCL = 1时 SDA仍然为低
*************************************************************************/
int IIC_WaitAck_PCA(void) {
    int ucErrTime = 0;
    int a = 0;

    // SDA输入模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    a = HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN);
    while (a) {
        ucErrTime++;
        if (ucErrTime > 100) {
            IIC_Stop_PCA();
            return 1;
        }
        a = HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN);
    }
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET); // 时钟下降沿
    return 0;
}

/*************************************************************************
* 函数名称：void IIC_Ack_PCA(void)
* 函数说明：模式IIC发送ACK应答信号
* 输入参数：
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：内部调用  发送完一个字节数据后发送ACK通知从机继续传输下一个字节数据
*************************************************************************/
void IIC_Ack_PCA(void) {
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    // SDA输出模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
}

/*************************************************************************
* 函数名称：void IIC_NAck_PCA(void)
* 函数说明：模式IIC发送NACK应答信号
* 输入参数：
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：内部调用  发送完最后一个字节数据后发送NACK通知从机停止传输数据，并产生停止信号
*************************************************************************/
void IIC_NAck_PCA(void) {
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    // SDA输出模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    iic_delay_PCA();
    iic_delay_PCA();
    iic_delay_PCA();
    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
}

/*************************************************************************
* 函数名称：void IIC_SendByte_PCA(int data_t)
* 函数说明：模式IIC发送一个字节
* 输入参数：data   :  发送的字节
* 返回值：
* 修改时间：2023年6月7日
* 调用方法：IIC_SendByte_PCA(0x12);
*************************************************************************/
void IIC_SendByte_PCA(int data_t) {
    int t;
    // SDA输出模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET); // 拉低时钟线开始传送
    for (t = 0; t < 8; t++) {
        if (data_t & (0x80 >> t)) {
            HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET);
        }
        iic_delay_PCA();
        iic_delay_PCA();
        HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);

        iic_delay_PCA();
        iic_delay_PCA();
        HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
        iic_delay_PCA();
    }
    iic_delay_PCA();
}

/*************************************************************************
* 函数名称：int IIC_ReadByte_PCA(int ack)
* 函数说明：模式IIC读取一个字节
* 输入参数：ack=1 时发送ACK接收完数据 ack=0 时不发送ACK接收完全部数据
* 返回值：接收到的字节
* 修改时间：2023年6月7日
* 调用方法：IIC_ReadByte_PCA(0x12);
*************************************************************************/
int IIC_ReadByte_PCA(int ack) {
    int i, receive = 0;
    int a = 0;

    // SDA输入模式
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);

    for (i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
        iic_delay_PCA();
        iic_delay_PCA();
        iic_delay_PCA();
        HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
        receive <<= 1;
        a = HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN);
        if (a) {
            receive++; // 主机会发来的高电平
        }
        iic_delay_PCA();
    }
    if (ack)
        IIC_Ack_PCA(); // 发送ACK
    else
        IIC_NAck_PCA(); // 发送nACK
    return receive;
}
