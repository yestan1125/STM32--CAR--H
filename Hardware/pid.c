#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "Motor.h"
#include "huiduganwei.h"
#include "OLED.h"
#include "Mpu6050.h"
#include "Madgwick.h"
#include "Timer.h"

extern int Flag1;
extern int Flag6;

//速度控制pid

 typedef struct{
    float kp;
	float ki;
	float kd;
	float error;
	float last_error;
	uint16_t sensor;
	float sumerror;
	float ideal_value;
}PID;
//初始化PID
void PID_Init(PID*pid, float kp,float ki,float kd,float ideal_value)
{
    pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->ideal_value =ideal_value;
	pid->sumerror =0;
	pid->sensor =0;
}
//计算pid
float PID_Updata1(PID*pid,uint16_t sensor)
{
	if(sensor>260){
		sensor = 360 - sensor;
	}
  
	  pid->sensor = sensor;
//	if(pid->sensor<231){
	pid->error = pid->ideal_value - pid->sensor;
	
	float a=0.9;//0.9
	pid->error= a*pid->error+(1-a)*pid->last_error;//采用一个一阶滤波系统
	
	
//	}else
//	{
//	pid->error = pid->sensor-pid->ideal_value;
//	}
	//kp
	float sum_p=pid->error*pid->kp;
//	//ki
	pid->sumerror+=pid->error;
	float sum_i = pid->ki*pid->sumerror;
	//积分限幅
	if(pid->sumerror>50){
	pid->sumerror=0;
	}
	//kd
	float sum_d=pid->kd*( pid->error-pid->last_error);
	pid->last_error=pid->error;
	return sum_d+sum_i+ sum_p;
}
float PID_Updata3(PID*pid,uint16_t sensor)
{
    pid->sensor = sensor;
//	if(pid->sensor<231){
	pid->error = pid->ideal_value - pid->sensor;
	
	float a=0.9;//0.9
	pid->error= a*pid->error+(1-a)*pid->last_error;//采用一个一阶滤波系统
	
	
//	}else
//	{
//	pid->error = pid->sensor-pid->ideal_value;
//	}
	//kp
	float sum_p=pid->error*pid->kp;
//	//ki
	pid->sumerror+=pid->error;
	float sum_i = pid->ki*pid->sumerror;
	//积分限幅
	if(pid->sumerror>50){
	pid->sumerror=0;
	}
	//kd
	float sum_d=pid->kd*( pid->error-pid->last_error);
	pid->last_error=pid->error;
	return sum_d+sum_i+ sum_p;
}
float PID_Updata4(PID*pid,uint16_t sensor)
{
	if(sensor<80)
	{
	pid->sensor = sensor+360;
	}else{
    pid->sensor = sensor;
	}
//	if(pid->sensor<231){
	pid->error = pid->ideal_value - pid->sensor;
	
	float a=0.9;//0.9
	pid->error= a*pid->error+(1-a)*pid->last_error;//采用一个一阶滤波系统
	
	
//	}else
//	{
//	pid->error = pid->sensor-pid->ideal_value;
//	}
	//kp
	float sum_p=pid->error*pid->kp;
//	//ki
	pid->sumerror+=pid->error;
	float sum_i = pid->ki*pid->sumerror;
	//积分限幅
	if(pid->sumerror>50){
	pid->sumerror=0;
	}
	//kd
	float sum_d=pid->kd*( pid->error-pid->last_error);
	pid->last_error=pid->error;
	return sum_d+sum_i+ sum_p;
}
float PID_Updata2(PID*pid,uint16_t sensor)
{
    pid->sensor = sensor*100;
	pid->error = pid->sensor -pid->ideal_value ;
	float a=0.9;//0.9
	pid->error= a*pid->error+(1-a)*pid->last_error;//采用一个一阶滤波系统
	//kp
	float sum_p=pid->error*pid->kp;
//	//ki
	pid->sumerror+=pid->error;
	float sum_i = pid->ki*pid->sumerror;
	//积分限幅
	if(pid->sumerror>100){
	pid->sumerror=0;
	}
	//kd
	float sum_d=pid->kd*( pid->error-pid->last_error);
	pid->last_error=pid->error;
	return sum_d+sum_i+ sum_p;
}
//开始计算
//void PID_PWM(void)
//{
//    uint16_t SpeedA =Encoder_GetA();
//    uint16_t SpeedB =Encoder_GetB();
//	float SpeedA1 =  PID_Updata(&motorA,SpeedA);
//	float SpeedB1 =  PID_Updata(&motorB,SpeedB);
//	Motor_SetSpeedA((uint8_t)SpeedA1+SpeedA);
//	Motor_SetSpeedB((uint8_t)SpeedB1+SpeedB);
//}
//位置环

float Position_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value)
{
	float Speederror;
    PID_Init(pid,kp,ki,kd,ideal_value);
	 uint8_t sensorState[8] = {0}; 
   uint8_t sensorValue=  GraySensor_Read(sensorState);  
	 Speederror =  PID_Updata1(pid,sensorValue);
	 return Speederror;
}
float Position_Cycle1(PID*pid, float kp,float ki,float kd,float ideal_value)
{
	float Speederror;
    PID_Init(pid,kp,ki,kd,ideal_value);
	 uint8_t sensorState[8] = {0}; 
   uint8_t sensorValue=  GraySensor_Read(sensorState);  
	 Speederror =  PID_Updata3(pid,sensorValue);
	 return Speederror;
}
PID pid1;
float yaw2;
float Yaw_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value)
{
    float Speederror;
    PID_Init(pid,kp,ki,kd,ideal_value); 
	yaw2 =  getYaw();
	Speederror =  PID_Updata1(pid,yaw2);
	 return Speederror;
}
float Yaw_Cycle1(PID*pid, float kp,float ki,float kd,float ideal_value)
{
    float Speederror;
    PID_Init(pid,kp,ki,kd,ideal_value); 
	yaw2 =  getYaw();
	Speederror =  PID_Updata4(pid,yaw2);
	 return Speederror;
}
//float Speed_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value)
//{
//  float  Speederror;
//	uint16_t AveSpeed;
//	AveSpeed = (SpeedA2+SpeedB2)/2;
//	PID_Init(pid,kp,ki,kd,ideal_value);
//	 Speederror= PID_Updata2(pid,AveSpeed);
//	return Speederror;
//}
//void TIM1_UP_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)		//判断是否是TIM2的更新事件触发的中断
//	{         //1ms
//        millis++;	
//        SpeedA2 = Encoder_GetA();
//		SpeedB2 = Encoder_GetB(); 		//每隔固定时间段读取一次编码器计数增量值，即为速度值
//		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			//清除TIM2更新事件的中断标志位
//  								            //中断标志位必须清除												//否则中断将连续不断地触发，导致主程序卡死
//	}
//}