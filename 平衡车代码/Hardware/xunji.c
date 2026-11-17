#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "xunji.h"

void Xunji_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			//时钟脚：单片机自己输出
	GPIO_InitStructure.GPIO_Pin = XUNJI_CLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XUNJI_CLK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//数据脚：单片机读，上拉
	GPIO_InitStructure.GPIO_Pin = XUNJI_DAT_PIN;
	GPIO_Init(XUNJI_DAT_PORT, &GPIO_InitStructure);

	//按键端口：开漏输出。开漏只会拉低不会推高，跟传感器内部电源不打架
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = XUNJI_BTN_PIN;
	GPIO_Init(XUNJI_BTN_PORT, &GPIO_InitStructure);

	GPIO_SetBits(XUNJI_CLK_PORT, XUNJI_CLK_PIN);	//平时停在高电平，跟读完一帧之后的电平一致

	Xunji_Btn_Release();
}

//串行读一帧。规矩就一条：时钟拉低的时候读数据，读完拉高，高电平要撑够 5us
//时钟拉高的时候传感器才推下一位，所以读必须赶在拉低那一瞬间
//每帧之间 TIM4 隔了 5ms 没动时钟，传感器自己就归零了，不用额外管
uint8_t Xunji_Read(void)
{
	uint8_t Read = 0;
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		GPIO_ResetBits(XUNJI_CLK_PORT, XUNJI_CLK_PIN);

		if(GPIO_ReadInputDataBit(XUNJI_DAT_PORT, XUNJI_DAT_PIN) == 0)	//低电平 = 压在黑线上
		{
			Read |= (1<<i);
		}

		GPIO_SetBits(XUNJI_CLK_PORT, XUNJI_CLK_PIN);
		Delay_us(5);
	}

	return Read;
}

//把 8 位状态换算成偏差，负 = 线偏左，正 = 线偏右
//八路的位置：丝印 1 在最左边(-7)，丝印 8 在最右边(+7)
//压在线上那几路求个平均，压在中间就接近 0，压在边上就接近 ±7
int16_t Xunji_Error(uint8_t Read)
{
	const int8_t Pos[8] = {-7,-5,-3,-1,1,3,5,7};
	int16_t Sum = 0;
	uint8_t Cnt = 0;
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		if(Read & (1<<i))
		{
			Sum += Pos[i];
			Cnt++;
		}
	}

	if(Cnt == 0)			//八路全白，丢线了，就当没有偏差
	{
		return 0;
	}

	return Sum / Cnt;
}

//拉低按键端口，等于按住传感器上的校准键（别超过 15 秒，手册说会报故障）
void Xunji_Btn_Press(void)
{
	GPIO_ResetBits(XUNJI_BTN_PORT, XUNJI_BTN_PIN);
}

void Xunji_Btn_Release(void)
{
	GPIO_SetBits(XUNJI_BTN_PORT, XUNJI_BTN_PIN);
}
