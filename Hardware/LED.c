#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}

void LED1_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}

void LED1_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

void LED3_ON(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_15);
}

void LED3_OFF(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void LED4_ON(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void LED4_OFF(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	}
}

void LED2_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}

void LED2_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) == 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	}
}
