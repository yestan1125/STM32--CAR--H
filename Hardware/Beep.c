#include "stm32f10x.h"                  // Device header

void Beep_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//GPIO模式，赋值为推挽输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;				//GPIO引脚，赋值为第12号引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//GPIO速度，赋值为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOC,GPIO_Pin_14);
}
void Beep_on(void)
{
  GPIO_ResetBits(GPIOC,GPIO_Pin_14);
}
void Beep_off(void)
{
GPIO_SetBits(GPIOC,GPIO_Pin_14);
}