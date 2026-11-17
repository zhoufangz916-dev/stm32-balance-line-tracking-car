#include "vofa_usart.h"
#include "bsp_usart.h"

void Vofa_SendData(float Angle, float Gyro, float BalanceOut,
                   float SpeedTarget, float SpeedActual, float SpeedOut,
                   float Kp, float Kd)
{
    float Data[8];
    uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7F};

    Data[0] = Angle;
    Data[1] = Gyro;
    Data[2] = BalanceOut;
    Data[3] = SpeedTarget;
    Data[4] = SpeedActual;
    Data[5] = SpeedOut;
    Data[6] = Kp;
    Data[7] = Kd;

    Usart_SendBytes((uint8_t *)Data, sizeof(Data));
    Usart_SendBytes(Tail, 4);
}
