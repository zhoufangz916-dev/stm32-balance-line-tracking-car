#include "bsp_usart.h"

/* main.c 里的可调参数 */
extern float Balance_Kp;
extern float Balance_Kd;
extern float Speed_Kp;
extern float Speed_Ki;
extern float Speed_Target;
extern uint8_t Stop;

#define RX_BUFFER_SIZE 32
static char RxBuffer[RX_BUFFER_SIZE];
static uint8_t RxIndex = 0;

/* 简易浮点解析，避免在中断里调用 sscanf */
static float ParseFloat(char *Str)
{
    float Value = 0.0f;
    float Frac = 0.1f;
    uint8_t Neg = 0;

    while (*Str == ' ' || *Str == '=' || *Str == ':' || *Str == ',') Str++;

    if (*Str == '-') { Neg = 1; Str++; }
    else if (*Str == '+') { Str++; }

    while (*Str >= '0' && *Str <= '9')
    {
        Value = Value * 10.0f + (float)(*Str - '0');
        Str++;
    }

    if (*Str == '.')
    {
        Str++;
        while (*Str >= '0' && *Str <= '9')
        {
            Value += (float)(*Str - '0') * Frac;
            Frac *= 0.1f;
            Str++;
        }
    }

    if (Neg) Value = -Value;
    return Value;
}

void USART1_Init(uint32_t BaudRate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* USART1 默认在 PA9/PA10，但这两个脚要留给八路循迹传感器，
       所以用部分重映射把 TX/RX 挪到 PB6/PB7。外设本身仍是 USART1，
       收发函数和 USART1_IRQHandler 都不受影响。
       注意：重映射必须开 AFIO 时钟。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);     // USART1 → PB6/PB7

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          // PB6 = TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;          // PB7 = RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      // 上拉。没插 VOFA 时 RX 悬空会被噪声触发，
                                                       // 随机收到 'T'/'S' 之类的字符就把速度和启停改了
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    /* 打开接收中断，允许上位机发命令实时改参数 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void Usart_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Usart_SendBytes(uint8_t *Buf, uint16_t Len)
{
    uint16_t i;
    for (i = 0; i < Len; i++)
    {
        Usart_SendByte(Buf[i]);
    }
}

/* 上位机命令（VOFA+ 底部发送框，记得带回车/换行）：
   P3.5  -> Balance_Kp = 3.5
   D0.4  -> Balance_Kd = 0.4
   V0.5  -> Speed_Kp = 0.5
   I0.1  -> Speed_Ki = 0.1
   T20   -> Speed_Target = 20
   S     -> 启动/停止
*/
void USART1_IRQHandler(void)
{
    char ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (char)USART_ReceiveData(USART1);

        if (ch == '\n' || ch == '\r')
        {
            RxBuffer[RxIndex] = '\0';

            if (RxIndex > 0)
            {
                switch (RxBuffer[0])
                {
                    case 'P': case 'p':
                        if (RxIndex >= 2) Balance_Kp = ParseFloat(&RxBuffer[1]);
                        break;
                    case 'D': case 'd':
                        if (RxIndex >= 2) Balance_Kd = ParseFloat(&RxBuffer[1]);
                        break;
                    case 'V': case 'v':
                        if (RxIndex >= 2) Speed_Kp = ParseFloat(&RxBuffer[1]);
                        break;
                    case 'I': case 'i':
                        if (RxIndex >= 2) Speed_Ki = ParseFloat(&RxBuffer[1]);
                        break;
                    case 'T': case 't':
                        if (RxIndex >= 2) Speed_Target = ParseFloat(&RxBuffer[1]);
                        break;
                    case 'S': case 's':
                        Stop = !Stop;
                        break;
                    default: break;
                }
            }

            RxIndex = 0;
        }
        else if (RxIndex < RX_BUFFER_SIZE - 1)
        {
            RxBuffer[RxIndex++] = ch;
        }
        else
        {
            RxIndex = 0;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
