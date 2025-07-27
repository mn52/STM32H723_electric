#include "stm32h7xx_hal.h"
#include "usart6.h"
#include "wit_c_sdk.h"

UART_HandleTypeDef huart6;

void Usart6Init(unsigned int uiBaud)
{
    // 1. 使能时钟
    //__HAL_RCC_GPIOC_CLK_ENABLE();
    //__HAL_RCC_USART6_CLK_ENABLE();
    
    // 2. 配置GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // TX引脚配置 (PC6)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;     // 复用功能选择USART6
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // RX引脚配置 (PC7)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    // 注意：RX引脚也使用AF_PP模式
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 3. 配置USART6
    huart6.Instance = USART6;
    huart6.Init.BaudRate = uiBaud;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        // 错误处理
        Error_Handler();
    }
    
    // 4. 配置中断
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 8);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    
    // 5. 使能接收中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

// USART6中断处理函数
void USART6_IRQHandler(void)
{
    // 处理接收中断
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)(huart6.Instance->RDR & 0xFF);
        WitSerialDataIn(data);
        __HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_NEF);
    }
    
    // 清除其他可能的中断标志
    __HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_OREF | UART_CLEAR_PEF | UART_CLEAR_FEF);
}

// USART6发送函数
void Uart6Send(unsigned char *p_data, unsigned int uiSize)
{
    HAL_UART_Transmit(&huart6, p_data, uiSize, 100);
}


