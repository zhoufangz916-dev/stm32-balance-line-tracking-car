#ifndef __VOFA_USART_H
#define __VOFA_USART_H
#include "stm32f10x.h"

void Vofa_SendData(float Angle, float Gyro, float BalanceOut,
                   float SpeedTarget, float SpeedActual, float SpeedOut,
                   float Kp, float Kd);

#endif
