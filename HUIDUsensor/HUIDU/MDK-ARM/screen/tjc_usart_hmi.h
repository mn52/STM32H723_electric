#ifndef __TJCUSARTHMI_H__
#define __TJCUSARTHMI_H__

#include <stdio.h>

/**
	ղӡսǁĻԮࠚ
*/



#define TJC_UART huart2
#define TJC_UART_INS USART2
extern UART_HandleTypeDef huart2;



void tjc_send_string(char* str);
void tjc_send_txt(char* objname, char* attribute, char* txt);
void tjc_send_val(char* objname, char* attribute, int val);
void tjc_send_nstring(char* str, unsigned char str_length);
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);




#define RINGBUFFER_LEN	(500)     //֨ӥخճޓ˕ؖޚ˽ 500

#define usize getRingBufferLength()
#define code_c() initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x) read1ByteFromRingBuffer(x)

extern uint8_t RxBuffer[1];
extern uint32_t msTicks;

#endif
