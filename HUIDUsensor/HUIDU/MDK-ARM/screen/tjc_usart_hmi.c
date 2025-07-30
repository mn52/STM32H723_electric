/**
 * 说明：
 *    1. 将 tjc_usart_hmi.c 和 tjc_usart_hmi.h 添加到项目中
 *    2. 使用前需要包含头文件 #include "tjc_usart_hmi.h"
 *    3. 使用前需要先调用 HAL_UART_Transmit_IT() 初始化串口发送
 */

#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include <stdio.h>
#include "tjc_usart_hmi.h"

// 定义环形缓冲区结构体
typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
} RingBuffer_t;

RingBuffer_t ringBuffer;        // 定义一个环形缓冲区实例
uint8_t RxBuffer[1];            // 用于接收单字节数据

/**
 * 函数名：intToStr
 * 创建时间：2024.09.18
 * 功能说明：将整数转换为字符串
 * 参数说明：
 *      num: 要转换的整数
 *      str: 转换后的字符串存储位置
 * 返回值：无
 */
void intToStr(int num, char* str)
{
    int i = 0;
    int isNegative = 0;

    // 处理负数
    if (num < 0)
    {
        isNegative = 1;
        num = -num;
    }

    // 提取每一位数字
    do
    {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    // 添加负号
    if (isNegative)
    {
        str[i++] = '-';
    }

    // 结束字符串
    str[i] = '\0';

    // 反转字符串
    int start = 0;
    int end = i - 1;
    while (start < end)
    {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

/**
 * 函数名：uart_send_char
 * 创建时间：2024.09.18
 * 功能说明：通过串口发送一个字符
 * 参数说明：
 *      ch: 要发送的字符
 * 返回值：无
 */
void uart_send_char(char ch)
{
    uint8_t ch2 = (uint8_t)ch;
    // 等待发送完成
    while (__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TC) == RESET);
    // 发送字符
    HAL_UART_Transmit_IT(&TJC_UART, &ch2, 1);
}

/**
 * 函数名：uart_send_string
 * 创建时间：2024.09.18
 * 功能说明：通过串口发送字符串
 * 参数说明：
 *      str: 要发送的字符串指针
 * 返回值：无
 */
void uart_send_string(char* str)
{
    while (*str != '\0' && str != NULL)
    {
        uart_send_char(*str++);
    }
}

/**
 * 函数名：tjc_send_string
 * 创建时间：2024.09.18
 * 功能说明：发送字符串命令到串口屏（带结束标志）
 * 参数说明：
 *      str: 要发送的字符串
 * 返回值：无
 */
void tjc_send_string(char* str)
{
    while (*str != '\0' && str != NULL)
    {
        uart_send_char(*str++);
    }
    uart_send_char(0xff);
    uart_send_char(0xff);
    uart_send_char(0xff);
}

/**
 * 函数名：tjc_send_txt
 * 创建时间：2024.09.18
 * 功能说明：发送文本设置命令到串口屏
 * 参数说明：
 *      objname: 对象名称，如 "t0"
 *      attribute: 属性名，如 "txt"
 *      txt: 要设置的文本内容
 * 返回值：无
 */
void tjc_send_txt(char* objname, char* attribute, char* txt)
{
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_string("=\"");
    uart_send_string(txt);
    uart_send_char('\"');
    uart_send_char(0xff);
    uart_send_char(0xff);
    uart_send_char(0xff);
}

/**
 * 函数名：tjc_send_val
 * 创建时间：2024.09.18
 * 功能说明：发送数值设置命令到串口屏
 * 参数说明：
 *      objname: 对象名称，如 "n0"
 *      attribute: 属性名，如 "val"
 *      val: 要设置的数值
 * 返回值：无
 */
void tjc_send_val(char* objname, char* attribute, int val)
{
    char txt[12] = "";
    intToStr(val, txt);

    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_char('=');
    uart_send_string(txt);

    uart_send_char(0xff);
    uart_send_char(0xff);
    uart_send_char(0xff);
}

/**
 * 函数名：tjc_send_nstring
 * 创建时间：2024.09.18
 * 功能说明：发送指定长度的字符串到串口屏
 * 参数说明：
 *      str: 要发送的字符串
 *      str_length: 字符串长度
 * 返回值：无
 */
void tjc_send_nstring(char* str, unsigned char str_length)
{
    for (int var = 0; var < str_length; ++var)
    {
        uart_send_char(*str++);
    }
    uart_send_char(0xff);
    uart_send_char(0xff);
    uart_send_char(0xff);
}

/**
 * 函数名：HAL_UART_RxCpltCallback
 * 创建时间：2022.10.08
 * 功能说明：串口接收完成回调函数
 * 参数说明：
 *      huart: UART句柄
 * 返回值：无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART.Instance)
    {
        write1ByteToRingBuffer(RxBuffer[0]);
        HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
    }
}

/**
 * 函数名：initRingBuffer
 * 创建时间：2022.10.08
 * 功能说明：初始化环形缓冲区
 * 参数说明：无
 * 返回值：无
 */
void initRingBuffer(void)
{
    ringBuffer.Head = 0;
    ringBuffer.Tail = 0;
    ringBuffer.Length = 0;
}

/**
 * 函数名：write1ByteToRingBuffer
 * 创建时间：2022.10.08
 * 功能说明：向环形缓冲区写入一个字节的数据
 * 参数说明：
 *      data: 要写入的数据
 * 返回值：无
 */
void write1ByteToRingBuffer(uint8_t data)
{
    if (ringBuffer.Length >= RINGBUFFER_LEN)
    {
        return; // 缓冲区已满
    }

    ringBuffer.Ring_data[ringBuffer.Tail] = data;
    ringBuffer.Tail = (ringBuffer.Tail + 1) % RINGBUFFER_LEN;
    ringBuffer.Length++;
}

/**
 * 函数名：deleteRingBuffer
 * 创建时间：2022.10.08
 * 功能说明：从环形缓冲区删除指定数量的数据
 * 参数说明：
 *      size: 要删除的数据长度
 * 返回值：无
 */
void deleteRingBuffer(uint16_t size)
{
    if (size >= ringBuffer.Length)
    {
        initRingBuffer();
        return;
    }

    for (int i = 0; i < size; i++)
    {
        ringBuffer.Head = (ringBuffer.Head + 1) % RINGBUFFER_LEN;
        ringBuffer.Length--;
    }
}

/**
 * 函数名：read1ByteFromRingBuffer
 * 创建时间：2022.10.08
 * 功能说明：从环形缓冲区读取一个字节的数据
 * 参数说明：
 *      position: 要读取的位置偏移
 * 返回值：读取到的数据
 */
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
    uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;
    return ringBuffer.Ring_data[realPosition];
}

/**
 * 函数名：getRingBufferLength
 * 创建时间：2022.10.08
 * 功能说明：获取环形缓冲区中当前数据长度
 * 参数说明：无
 * 返回值：数据长度
 */
uint16_t getRingBufferLength(void)
{
    return ringBuffer.Length;
}

/**
 * 函数名：isRingBufferOverflow
 * 创建时间：2022.10.08
 * 功能说明：判断环形缓冲区是否溢出
 * 参数说明：无
 * 返回值：
 *      0: 未溢出
 *      1: 已溢出
 */
uint8_t isRingBufferOverflow(void)
{
    return (ringBuffer.Length >= RINGBUFFER_LEN);
}
