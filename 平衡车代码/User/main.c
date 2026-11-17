#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer1.h"
#include "Timer4.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "xunji.h"
#include "bsp_usart.h"
#include "vofa_usart.h"
#include <math.h>

uint8_t KeyNum;

float Speed_Target = 0;								//速度环变量
float Speed_Target_Ramp = 0;						//软启动：实际拿去算误差的目标值，一点点逼近 Speed_Target
float Speed_Actual,Speed_Out;
float Speed_Kp = 5.4,Speed_Ki = 0.027;
float Speed_Error,Speed_ErrorInt;
float Speed_ErrorLow,Speed_ErrorLow_Last;	//编码器误差一阶低通

int16_t AX, AY, AZ, GX, GY, GZ;
float Acc_Angle;        // 加速度计算出来的角度，单位：度
float Gyro_Rate;        // 陀螺仪算出来的角速度，单位：度/秒
float Gyro_Filtered;    // 低通滤波后的角速度，用于 D 项
float Angle;            // 滤波后的最终角度
float Gyro_Offset = 0;  // 陀螺仪静止零偏
float Angle_Zero = -2.81;   // 小车竖直时对应的角度零点

float AX_F = 0, AZ_F = 0, GY_F = 0;
float Balance_Out;
float Balance_Kp = 2.31, Balance_Kd = 0.41;					//位置环变量
float Balance_Error;
float Balance_ErrorInt = 0;

float GZ_Offset = 0;     // GZ 静止零偏（跟 Gyro_Offset 放一起）
float Turn_Kp = 0.4;      		 // 转向环变量0.553
float Turn_Out;          // 转向环输出 = 左右轮差速

/* ---------- 循迹相关 ---------- */
#define LINE_CROSS_FRAMES 30			// 连续 30 帧全黑（一帧 5ms，合 0.15s）才算到十字路口。
										// 单帧全黑大多是颠出来的假的；调大不容易误触发但停得晚
uint8_t Track_Mode = 0;					// 0 = 纯平衡   1 = 循迹
uint8_t Line_Raw = 0;					// 最近一次读到的 8 路状态，1 = 这一路压在黑线上
uint16_t Line_Black_Cnt = 0;			// 连续读到几帧全黑，认十字路口用
uint8_t Corner_Time = 0;				// 直角弯锁：>0 = 正在过弯，这期间全白也接着拐；每帧-1，数完交回循迹
int8_t  Corner_Dir  = 0;				// 直角弯方向：-1 = 往 1 路那侧拐，+1 = 往 8 路那侧拐
int16_t Line_Error = 0;					// 最近一次偏差，负 = 线偏左，正 = 线偏右
float   Line_Speed = 0.75f;				// 循迹前进速度（速度环的目标值）。正 = 前进

float Turn_P = 0.08;   // 误差最大±350，×0.08≈28；给太大会一直满舵打摆，拐弯抖就再调小
float turn_out = 0;

float Angle_Limit = 45.0f;		//倾角超过它就断电机（参考工程是 Pitch<-50 或 >40）

uint8_t Data_Ready = 0;		// TIM4 完成一次位置环后置 1

uint8_t Stop = 1;

uint16_t Oled_Count = 0;

void Calibrate_Sensor(void)
{
    float Sum = 0;
	float Sum_Z = 0;
    int i;
	
    for (i = 0; i < 200; i++)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		Sum   += GY;
		Sum_Z += GZ;
		Delay_us(50);
	}

	Gyro_Offset = Sum / 200.0;
	GZ_Offset   = Sum_Z / 200.0;

    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    Angle = Angle_Zero;
}

int main(void)
{
	uint8_t i;

	OLED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	MPU6050_Init();
	Xunji_Init();

	Calibrate_Sensor();
	
	Timer1_Init();
	Timer4_Init();
	USART1_Init(115200);
	
	while(1)
	{
		KeyNum = Key_GetNum();

		/* 按键1(PB5)：平衡 <-> 循迹 切换。
		   切到循迹就直接启动（Stop 清零 + 给定前进速度），一键就能跑；
		   切回平衡把速度归零，车原地保持平衡（平衡车不能断电机，否则直接摔）。 */
		if(KeyNum == 1)
		{
			Track_Mode = !Track_Mode;
			Stop = 0;
			Speed_Target = Track_Mode ? Line_Speed : 0;
		}

		/* 按键2(PB4)：启停（平衡的开关） */
		if(KeyNum == 2)
		{
			Stop = !Stop;
			if (Stop) Speed_Target = 0;
		}

		if (Data_Ready)
		{
			Oled_Count = 0;

//			OLED_Printf(0,0,OLED_8X16,"Ang:%+06.1f",Angle - Angle_Zero);
			OLED_Printf(0,0,OLED_8X16,"Ang:%+05.1f S%dM%d",Angle - Angle_Zero, Stop, Track_Mode);

			/* 八路原值。左边第一位 = 1 路，1 = 这一路压在黑线上。
			   校验循迹就靠这一行，不用接 VOFA：把车放线上看哪几位是 1，
			   再把车左右平移，看这个 1 是不是一路跟着挪过去。
			   显示的是 TIM4 里刚读到的 Line_Raw，不要在这儿再读一次：
			   同一帧被读两次的话，两次之间隔不到 1ms，传感器来不及归零，帧就错位了。 */
			OLED_ShowString(0,16,"T:",OLED_8X16);
			for(i = 0; i < 8; i++)
			{
				OLED_ShowNum(16 + i * 8, 16, (Line_Raw >> i) & 1, 1, OLED_8X16);
			}

			/* 偏差：负 = 线偏左，正 = 线偏右，最大 ±350 所以要 3 位 */
			OLED_ShowString(0,32,"E:",OLED_8X16);
			OLED_ShowSignedNum(16,32,Line_Error,3,OLED_8X16);

//			OLED_Printf(0,48,OLED_8X16,"Err:%+06.1f",Angle - Acc_Angle);
//			OLED_Printf(0,48,OLED_8X16,"Z:%+06.1f",Angle_Zero);
//			OLED_Printf(0,48,OLED_8X16,"Out:%+05.0f",Balance_Out);
//			OLED_Printf(0,48,OLED_8X16,"Spd:%+06.1f",Speed_Actual);
//			OLED_Printf(0,48,OLED_8X16,"v%+05.1f o%+05.0f",Speed_Actual,Balance_Out - Speed_Out);
			OLED_Printf(0,48,OLED_8X16,"v%+05.1f t%+05.0f",Speed_Actual,turn_out);

			OLED_Update();
	}
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count1;
	if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
	{
		Key_Tick();

		Count1++;
		if(Count1 >= 3)					//速度环
		{
			Count1 = 0;
			Speed_Actual = -(Encoder_Get_Left() + Encoder_Get_Right()) / 2.0;//速度增值范围是-300到300

			/* 软启动：目标值一步一步挪过去，6ms 最多变 0.1（爬到 2 要 120ms）。
			   不然按键1 一按，目标从 0 直接跳到 Line_Speed，速度环一下输出几十，
			   车被顶得站不住。步子 0.3 起步还是会冲一下，冲完传感器一颠就出假全黑，
			   所以放慢到 0.1。上面 Speed_Target 是按键/VOFA 写的原值，没改。 */
			if      (Speed_Target - Speed_Target_Ramp >  0.1f) Speed_Target_Ramp += 0.1f;
			else if (Speed_Target_Ramp - Speed_Target >  0.1f) Speed_Target_Ramp -= 0.1f;
			else                                               Speed_Target_Ramp  = Speed_Target;

			Speed_Error = Speed_Target_Ramp - Speed_Actual;

			/* 参考工程的做法：编码器误差先过一阶低通再进积分，P 项仍用原始误差 */
			Speed_ErrorLow = 0.2f * Speed_Error + 0.8f * Speed_ErrorLow_Last;
			Speed_ErrorLow_Last = Speed_ErrorLow;

			Speed_ErrorInt += Speed_ErrorLow;
			if (Speed_ErrorInt >= 300) {Speed_ErrorInt = 300;}
			if (Speed_ErrorInt <= -300) {Speed_ErrorInt = -300;}
			
			if ((Angle - Angle_Zero) > 30 || (Angle - Angle_Zero) < -30)
			{
				Speed_ErrorInt = 0;
			}

			Speed_Out = Speed_Kp * Speed_Error + Speed_Ki * Speed_ErrorInt;
			
			if(Speed_Out >= 100){Speed_Out = 100;}
			if(Speed_Out <= -100){Speed_Out = -100;}
		}

		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}  
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

		/* 本车 MPU 安装朝向使原始 GY 与 Angle 反号（2026-10-06 实测：Ang 增大时 Gyro 读负）。
		取负让 Gyro_Rate 与 dAngle/dt 同号。重新粘 MPU 后必须重测这条。 */
		Gyro_Rate = (Gyro_Offset - GY) / 16.4;

		/* 角速度低通：只给 OLED/VOFA 看，D 项不再过它（参考工程的 D 项直接用原始陀螺仪） */
		Gyro_Filtered = 0.9f * Gyro_Filtered + 0.1f * Gyro_Rate;

		Acc_Angle = atan2(AX, AZ) * 180.0 / 3.14159;
		Angle = 0.9 * (Angle + Gyro_Rate * 0.005) + 0.1 * Acc_Angle;

		Balance_Error = 0.0 - (Angle - Angle_Zero);

		/* 参考工程的 D 项 = Vertical_Kd(负值) * 原始陀螺仪，即负增益：
		   P 项把车往回推的那一刻，D 项要帮着往同一个方向推，才叫阻尼。
		   这里 Balance_Kd 取正值，所以前面用减号。不认同就把这个减号改回加号。 */
		Balance_Out = Balance_Kp * Balance_Error - Balance_Kd * Gyro_Rate;

		if (Balance_Out > 100.0f)  Balance_Out = 100.0f;
		if (Balance_Out < -100.0f) Balance_Out = -100.0f;
		
		u8 trace = Xunji_Read();   // 每帧只读一次传感器，给循迹和OLED用
		Line_Raw = trace;

		/* 十字路口检测：八路全黑（0xFF）= 到路口。单帧全黑大多是颠出来的假的，
		   要连续 LINE_CROSS_FRAMES 帧（一帧 5ms，20 = 0.1s）全黑才算数，中途读到非全黑就清零重数 */
		if (Line_Raw == 0xFF) { if (Line_Black_Cnt < LINE_CROSS_FRAMES) Line_Black_Cnt++; }
		else                  { Line_Black_Cnt = 0; }

		/* 到路口（终点）= 停止循迹、切回纯平衡：前进速度归零，车原地站住保持平衡 */
		if (Line_Black_Cnt == LINE_CROSS_FRAMES && Track_Mode == 1)
		{
			Track_Mode   = 0;
			Speed_Target = 0;
		}

		int sum_bit = 0;
		int pos = 0;
		for(int i=0;i<8;i++)
		{
			if( trace & (1<<i) ) //该位为1，踩到黑线（Xunji_Read 黑线读出是 bit=1）
			{
				pos += i;
				sum_bit++;
			}
		}
		int error = 0;
		if(sum_bit > 0)
		{
			//计算黑线中心位置，8路编号0~7，中间在3和4之间 = 3.5
			float center = (float)pos / sum_bit;
			error = (int)((center - 3.5f) * 100);
		}
		Line_Error = error;    //给OLED显示误差
		//正号是实测过的方向，别改回负号
		//低通：差速每帧只挪 10%，黑线在探头格间跳变时不会一步一跳（转弯抖就是它；嫌反应慢就改回 0.7/0.3）
		turn_out = 0.95f * turn_out + 0.05f * (error * Turn_P);

		/* 直角弯：黑线整排连续压到一侧，质心那点差速拐不过来，直接给固定大差速硬拐。
		   两组图案：从 1 路那侧压过来 = 0x07/0x0F/0x1F/0x3F（往 -1 那侧拐）；
		            从 8 路那侧压过来 = 0xE0/0xF0/0xF8/0xFC（往 +1 那侧拐，和质心同方向）。
		   上锁后规则只有一条：方向和时长都不再改，从检测到的那一帧起按记住的方向硬拐 30 帧，
		   数完自动放锁交回质心循迹——锁着的时候读到什么图案（包括全白）都不理。
		   为什么别的都不看（实测踩过的两个坑）：
		   1) 拐弯中途线会从探头底下扫出去，而且会先扫到反方向那几路
		      （右拐的时候，线是先往左边滑出去的）。见到反方向图案就反悔的话，车就在原地左右打摆；
		   2) 扫的过程中线还会经过中间，要是"线回中间"就松锁，等于没锁。
		   调法：锁数完了还没拐过来（又直着冲出去）→ 把 30 加大（比如 45）；
		        出弯转过头、画龙 → 把 30 改小（比如 20）。
		   差速力度就是下面那个 20.0f，和以前一样自己调。
		   必须放在低通后面，放前面会被它覆盖 */
		if (Corner_Time == 0)   // 只有锁是空的才认新弯；锁着的时候任何图案都不再改方向、不再续时间
		{
			if      (Line_Raw == 0x07 || Line_Raw == 0x0F || Line_Raw == 0x1F || Line_Raw == 0x3F)
			{
				Corner_Dir  = -1;
				Corner_Time = 45;
			}
			else if (Line_Raw == 0xE0 || Line_Raw == 0xF0 || Line_Raw == 0xF8 || Line_Raw == 0xFC)
			{
				Corner_Dir  = +1;
				Corner_Time = 45;
			}
		}

		if (Corner_Time > 0)
		{
			Corner_Time--;                     // 锁倒计时，数完就交回质心循迹
			turn_out = 48.0f * Corner_Dir;     // 锁住：中途全白、线扫到另一边，都按记住的方向继续拐
		}

		if (Track_Mode == 0 || Stop == 1) turn_out = 0;   // 差速只在循迹运行时给，纯平衡不拐

		/* 转向环。GZ 阻尼两种模式都有；turn_out 是循迹才有的差速（质心法），纯平衡时为 0 */
		Turn_Out = Turn_Kp * ((GZ - GZ_Offset) / 16.4);

		Turn_Out += turn_out;   // 循迹差速和 GZ 阻尼加在一起，共用 Turn_Out 这一路

		/* 防止 Turn_Kp 手滑给太大，把差速拉满跟直立环抢控制权 */
		if (Turn_Out >  50.0f) Turn_Out =  50.0f;
		if (Turn_Out < -50.0f) Turn_Out = -50.0f;

		/* 倒地保护：倾角太大直接断电机（参考工程 Turn_Off 的做法，那边是 +40/-50） */
		if ((Angle - Angle_Zero) > Angle_Limit || (Angle - Angle_Zero) < -Angle_Limit)
		{
			Motor_SetSpeed_Left(0);
			Motor_SetSpeed_Right(0);
		}

		else if (Stop == 0)
		{
			float Base = Balance_Out + Speed_Out;
			float v_L  = Base - Turn_Out;
			float v_R  = Base + Turn_Out;

			if (v_L >  100.0f) v_L =  100.0f;
			if (v_L < -100.0f) v_L = -100.0f;
			if (v_R >  100.0f) v_R =  100.0f;
			if (v_R < -100.0f) v_R = -100.0f;

			Motor_SetSpeed_Left(v_L);
			Motor_SetSpeed_Right(-v_R);
		}

		else
		{
			Motor_SetSpeed_Left(0);
			Motor_SetSpeed_Right(0);
		}

		Data_Ready = 1;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
