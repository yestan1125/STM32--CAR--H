#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12|GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	PWM_Init();
}

//void Motor_SetSpeedA(int8_t SpeedA)
//{
//	if (SpeedA >= 0)
//	{
//		GPIO_SetBits(GPIOA, GPIO_Pin_11);
//		GPIO_ResetBits(GPIOA, GPIO_Pin_12);
//		PWM_SetCompare1(SpeedA);
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
//		GPIO_SetBits(GPIOA, GPIO_Pin_12);
//		PWM_SetCompare1(-SpeedA);
//	}
//}
//void Motor_SetSpeedB(int8_t SpeedB)
//{
//	if (SpeedB >= 0)
//	{
//		GPIO_SetBits(GPIOA, GPIO_Pin_14);
//		GPIO_ResetBits(GPIOA, GPIO_Pin_15);
//		PWM_SetCompare2(SpeedB);
//	}
//	else
//	{
//		GPIO_SetBits(GPIOA, GPIO_Pin_14);
//		GPIO_ResetBits(GPIOA, GPIO_Pin_14);
//		PWM_SetCompare2(-SpeedB);
//	}
//}
/* 设置电机 A 的速度 */
void Motor_SetSpeedA(int8_t Speed)
{
    if (Speed >= 0) // 正转
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_11); // AIN1 高
        GPIO_ResetBits(GPIOA, GPIO_Pin_12); // AIN2 低
        PWM_SetCompare1(Speed); // CH1
    }
    else // 反转`q
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_11); // AIN2 高
        GPIO_ResetBits(GPIOA, GPIO_Pin_12); // AIN1 低
        PWM_SetCompare1(-Speed); // CH1
    }
}

/* 设置电机 B 的速度 */
void Motor_SetSpeedB(int8_t Speed)
{
    if (Speed >= 0) // 正转
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_15); // BIN1 高
        GPIO_ResetBits(GPIOA, GPIO_Pin_14); // BIN2 低
        PWM_SetCompare2(Speed); // CH3
    }
    else // 反转
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_14); // BIN2 高
        GPIO_ResetBits(GPIOA, GPIO_Pin_14); // BIN1 低
        PWM_SetCompare2(-Speed); // CH3
    }
}