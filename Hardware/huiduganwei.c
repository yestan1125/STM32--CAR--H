#include "stm32f10x.h"                  // Device header
#include "pid.h"

//初始化每个io口
void Ganwei_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
uint8_t    GraySensor_Read(uint8_t *sensorState)
{
    sensorState[0] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);   
    sensorState[1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);  
    sensorState[2] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);  
    sensorState[3] = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);   
    sensorState[4] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);   
    sensorState[5] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);   
    sensorState[6] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);   
    sensorState[7] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);  

         uint8_t sensorValue = 0;
    for (int i = 0; i < 8; i++) {
        sensorValue |= sensorState[i] << (7 - i);  // 相互对应的
    }
    return sensorValue;
}