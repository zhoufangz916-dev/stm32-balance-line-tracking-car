#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "stm32f10x.h"

void USART1_Init(uint32_t BaudRate);
void Usart_SendByte(uint8_t Byte);
void Usart_SendBytes(uint8_t *Buf, uint16_t Len);

#endif
