#include "stm32f10x.h"                  // Device header

volatile int16_t EncoderCount;		//编码器累计脉冲数（软件计数，中断中修改）

/**
  * 函    数：编码器初始化（PA0/PA1，EXTI软件正交解码）
  * 参    数：无
  * 返 回 值：无
  * 注意事项：本模块使用外部中断实现编码器解码，不占用定时器
  *           A相PA0和B相PA1均配置为双边沿中断，分辨率为4倍频
  */
void Encoder_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//开启AFIO的时钟，外部中断需要
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA0 PA1 PA6 PA7引脚初始化为上拉输入

	/*AFIO选择中断引脚*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);	//指定EXTI0对应PA0
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);	//指定EXTI1对应PA1

	/*EXTI初始化*/
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;		//选择EXTI线0和1
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;					//使能中断线
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//选择中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//双边沿触发，获得4倍分辨率
	EXTI_Init(&EXTI_InitStructure);

	/*NVIC初始化*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级最高，避免丢脉冲
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级最高，避免丢脉冲
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
	
	/*输入捕获初始化*/
	TIM_ICInitTypeDef TIM_ICInitStructure;							//定义结构体变量
	TIM_ICStructInit(&TIM_ICInitStructure);							//结构体初始化，若结构体没有完整赋值
																	//则最好执行此函数，给结构体所有成员都赋一个默认值
																	//避免结构体初值不确定的问题
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				//选择配置定时器通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM3, &TIM_ICInitStructure);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;				//选择配置定时器通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM3, &TIM_ICInitStructure);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
	
	/*编码器接口配置*/
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
																	//配置编码器模式以及两个输入通道是否反相
																	//注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
																	//此函数必须在输入捕获初始化之后进行，否则输入捕获的配置会覆盖此函数的部分配置
	
	/*TIM使能*/
	TIM_Cmd(TIM3, ENABLE);			//使能TIM3，定时器开始运行
}

/**
  * 函    数：获取编码器的增量值
  * 参    数：无
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Encoder_Get_Left(void)
{
	/*使用Temp变量作为中继，目的是返回计数后将其清零*/
	int16_t Temp;
	Temp = EncoderCount;
	EncoderCount = 0;
	return Temp;
}

int16_t Encoder_Get_Right(void)
{
	int16_t Temp;
	Temp = (int16_t)(TIM3->CNT);	//读取TIM3的计数
	TIM_SetCounter(TIM3, 0);		//读完清零
	return Temp;
}

/**
  * 函    数：PA0边沿中断服务函数（编码器A相）
  * A相跳变时，与B相电平比较判断方向
  */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
		{
			EncoderCount++;			//A相与B相电平相同：正向
		}
		else
		{
			EncoderCount--;			//A相与B相电平不同：反向
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/**
  * 函    数：PA1边沿中断服务函数（编码器B相）
  * B相跳变时，与A相电平比较判断方向
  */
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
			EncoderCount--;			//B相与A相电平相同：反向
		}
		else
		{
			EncoderCount++;			//B相与A相电平不同：正向
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
