#ifndef __XUNJI_H
#define __XUNJI_H

#include "stm32f10x.h"

/* 传感器的"串行时钟" -> PA11   （原来接丝印 1 的那根线）
   传感器的"串行数据" -> PA10   （原来接丝印 2 的那根线）
   按键端口           -> PA12   （校准用，原来就有） */
#define XUNJI_CLK_PORT		GPIOA
#define XUNJI_CLK_PIN		GPIO_Pin_11
#define XUNJI_DAT_PORT		GPIOA
#define XUNJI_DAT_PIN		GPIO_Pin_10
#define XUNJI_BTN_PORT		GPIOA
#define XUNJI_BTN_PIN		GPIO_Pin_12

void    Xunji_Init(void);
uint8_t Xunji_Read(void);				/* 读一帧，bit0~bit7，1 = 这一路压在黑线上 */
int16_t Xunji_Error(uint8_t Read);		/* 换算成偏差，负 = 线偏左，正 = 线偏右，全白返回 0 */
void    Xunji_Btn_Press(void);			/* 拉低按键端口，等于按住传感器上的校准键 */
void    Xunji_Btn_Release(void);		/* 松开按键端口 */

#endif
