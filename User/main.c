#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "huiduganwei.h"
#include "Serial.h"
#include "pid.h"
#include "Timer.h"
#include "Mpu6050.h"
#include "Madgwick.h"
#include "LED.h"
#include "Beep.h"
void Key_Scan(void);
void MODE_adjust (void);
void MODE_mode(void);
void Adjust(int a);
float Speed_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value);
uint16_t SpeedA2;
uint16_t SpeedB2;
uint64_t millis;
uint8_t i;
uint16_t sensorValue1;
uint8_t KeyNum;
uint8_t SpeedA;
uint8_t SpeedB;
 uint16_t SpeedA1;
 uint16_t SpeedB1;
uint16_t Flag;
uint16_t Flag1;
uint16_t Flag2;
uint16_t Flag3;
uint16_t Flag4;
uint16_t Flag5;
uint16_t Flag6;
uint16_t SA,SB;
 uint8_t sensorState1[8] = {0}; 
float Error;
float yaw1;
 float yaw4;
 float yaw3;
uint16_t Count;
uint16_t Count1;
uint16_t Count2;
uint16_t Count3;
uint16_t Count4;
float Distance1;
float Distance;
float Distance2;
float Speederror;
float Speederror1; 
 float Speederror2;
uint64_t millis;//millis为当前程序运行的时间，类似arduino里millis()函数
uint8_t ID;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量

MPU6050Params mpu6050 = {
    .MPU6050dt = 10,
    .preMillis = 0,
    .MPU6050ERROE = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

SensorMsg msg = {
	.A = {0.0f, 0.0f, 0.0f},
	.G = {0.0f, 0.0f, 0.0f}
};

int main(void)
{
	LED_Init();
	Beep_Init();
	OLED_Init();
	Motor_Init();
	Encoder_Init();
	Ganwei_Init();
	Serial_Init();
	Timer_Init();
	MPU6050_Init();
	begin(1000.0f / (float)mpu6050.MPU6050dt);
	Delay_s(3);
	dataGetERROR();
	Key_Init();
	while (1)
	{ 
		//姿态角	
		if(millis - mpu6050.preMillis >= mpu6050.MPU6050dt) {
			mpu6050.preMillis = millis;
			dataGetAndFilter();		                            //获取MPU6050的数据
			updateIMU(msg.G[0], msg.G[1], msg.G[2], msg.A[0], msg.A[1], msg.A[2]);
		}			
		Key_Scan();
	}
}

uint8_t key_set = 0;
uint8_t key_up =0;
uint8_t key_down = 0;
char *menu[12] = {"MODE", "ADJUST", "MODE1", "MODE2", "MODE3", "MODE4", "SPEED", "POSITION", "YAW", "P:", "I:", "D:"};

uint8_t set_state;    // 从主程序引入状态机变量
uint8_t blink_flag;   // 引入闪烁标志
uint8_t menu_state;		//菜单
uint8_t up_state;			//上
uint8_t down_state;		//下
uint8_t back_state;		//返回
uint8_t mode_state;		//mode canshu
uint8_t adjust_state;		//adjust canshu
#define KEY_SET     GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)  // 确认按键,接vcc
#define KEY_ADD     GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)  // 上按键,接GND
#define KEY_SUB     GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)  // 下按键,接vcc

int press_time = 0;


void Adjust(int a)
{
    switch (a)
    {
        case 1: //修改P
            break;
        case 2: //修改I
            break;
        case 3: //修改D
            break;
    }
}

void MODE_mode(void){
	if (key_up )
                {
                    mode_state-=1;
                }
                else if (KEY_SUB )
                {
                    mode_state+=1;
                }
								if(mode_state == 1 ){
									OLED_ShowString(1, 1, "->");
									OLED_ShowString(2, 1, "  ");
									OLED_ShowString(3, 1, "  ");
									OLED_ShowString(4, 1, "  ");
								}
								else if(mode_state == 2 ){
									OLED_ShowString(1, 1, "  ");
									OLED_ShowString(2, 1, "->");
									OLED_ShowString(3, 1, "  ");
									OLED_ShowString(4, 1, "  ");
								}
								else if(mode_state == 3 ){
									OLED_ShowString(1, 1, "  ");
									OLED_ShowString(2, 1, "  ");
									OLED_ShowString(3, 1, "->");
									OLED_ShowString(4, 1, "  ");
								}
								else {
									OLED_ShowString(1, 1, "  ");
									OLED_ShowString(2, 1, "  ");
									OLED_ShowString(3, 1, "  ");
									OLED_ShowString(4, 1, "->");
								}
               
 }

 void MODE_adjust (void){
	 if (key_up )
                {
                    adjust_state -=1;
                }
                else if (key_down )
                {
                    adjust_state +=1;
                }
								if(adjust_state == 1 ){
									OLED_ShowString(1, 1, "->");
									OLED_ShowString(2, 1, "  ");
									OLED_ShowString(3, 1, "  ");
									OLED_ShowString(4, 1, "  ");
								}
								else if(adjust_state == 2 ){
									OLED_ShowString(1, 1, "  ");
									OLED_ShowString(2, 1, "->");
									OLED_ShowString(3, 1, "  ");
									OLED_ShowString(4, 1, "  ");
								}
								else if(adjust_state == 3 ){
									OLED_ShowString(1, 1, "  ");
									OLED_ShowString(2, 1, "  ");
									OLED_ShowString(3, 1, "->");
									OLED_ShowString(4, 1, "  ");
								}
								
               
 }

void Key_Scan(void)
{
    static uint8_t key_menu_state = 1;
    PID pid;
	if(KEY_SET){
		 Delay_ms(300);
		key_set = 1;
	 }
	else{
		key_set = 0;
	}
	 if(KEY_SUB){
		 Delay_ms(300);
		 key_down = 1;
	 }
	 else{
		key_down = 0;
	 }
	 if(!KEY_ADD){
		 Delay_ms(300);
			key_up = 1;
	 }
	else {
			key_up = 0;
	}
    // 选择菜单
    if (key_set)
    {
        key_menu_state = 0;
        menu_state++;
        press_time++;
        if (menu_state > 4)
        {
            menu_state = 1;
			mode_state =0;
					OLED_Clear();
        }
        
			}
    switch (menu_state)
    {
        case 1:
            OLED_ShowString(1, 3, menu[0]);
            OLED_ShowString(2, 3, menu[1]);
            if (key_up)
            {
                OLED_ShowString(1, 1, "->");
                OLED_ShowString(2, 1, "  ");
                up_state = 3;
            }
            else if (key_down)
            {
                OLED_ShowString(1, 1, "  ");
                OLED_ShowString(2, 1, "->");
                up_state = 4;
            }
            break;

        case 2:
            // 清除指定行而不是整个屏幕
            //OLED_ClearLine(1);  // 清除第1行
              // 清除第2行
            
			OLED_Clear();	
            if (up_state == 3)
            {
                OLED_ShowString(1, 3, menu[2]);
                OLED_ShowString(2, 3, menu[3]);
                OLED_ShowString(3, 3, menu[4]);
                OLED_ShowString(4, 3, menu[5]);
                MODE_mode();
            }
            else if (up_state == 4)
            {
                OLED_ShowString(1, 3, menu[6]);
                OLED_ShowString(2, 3, menu[7]);
                OLED_ShowString(3, 3, menu[8]);
                int j = 1;
               MODE_adjust ( );
                
            }
            break;		
				case 3:
					OLED_Clear();
				 if (mode_state == 1 && up_state == 3)
                {
					OLED_ShowNum(4,1,yaw1,3);
		 if( sensorValue1<250){
			SpeedA = 0;
			 SpeedB = 0;
		 }else{
		Speederror = Yaw_Cycle(&pid,0.01,0,0,180);
		SpeedA = 30-Speederror;
		SpeedB = 30+ Speederror;
		 }
		Motor_SetSpeedA(-SpeedA);//原A
		Motor_SetSpeedB(-SpeedB);			// 任务1
                }
                else if (mode_state == 2 && up_state ==3)
                {
					OLED_ShowNum(2,1,yaw1,3);
					if(sensorValue1 < 251&&Flag3==0){
					Flag3++;
					}
					if(Flag3==1&&sensorValue1==251&&(yaw1>330||yaw1<10)){
					Flag3++;
					}
					if(sensorValue1 < 251&&Flag3==2){
					Flag3++;
					}
					if(sensorValue1 == 251&&Flag3==3&&yaw1<190){
			SpeedA = 0;
			SpeedB = 0;	
			Flag3=4;
					}
         switch(Flag3){
			 case 0:
				 OLED_ShowNum(1,1,1,1);
		Speederror = Yaw_Cycle(&pid,0.05,0,0,180);
		SpeedA = 30-Speederror;
		SpeedB = 30+ Speederror;
			 break;
			 case 1:
				 OLED_ShowNum(1,1,2,1);                                   
		 Speederror = -Position_Cycle(&pid,0.09,0.03,0.05,231);//0.5 0.1 0.2
     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
		  SpeedA = 30-Speederror; 
			 break;
			 case 2:
		OLED_ShowNum(1,1,3,1);
		Speederror = Yaw_Cycle1(&pid,0.1,0.00,0,338);
		SpeedA = 20-Speederror;
		SpeedB = 20+Speederror;
			 break;
			 case 3:
				 OLED_ShowNum(1,1,4,1);
		 Speederror = -Position_Cycle(&pid,0.09,0.01,0.05,231);//0.5 0.1 0.2
     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
		  SpeedA = 30-Speederror; 
			 break;
			
		 }
		 Motor_SetSpeedA(-SpeedA);//原A
		Motor_SetSpeedB(-SpeedB);
                }
                else if (mode_state == 3 && up_state ==3)
                {
       if(sensorValue1<250&&Flag2==0)
	  {
	  Flag1++;
	  }
	 if((yaw1<5||yaw1>350)&&Flag1==1)
	 {
	 Flag1++;
	 }
	 if(Flag1==3&&yaw1<250&&sensorValue1==251){
	SpeedA = 0;
	SpeedB =0;
	Flag1=4;
	 }
      switch(Flag1)
	  {       
		case 0:
		  Speederror1 = -Yaw_Cycle(&pid,0.2,0.025,0.1,145); 
		  SpeedB =20-Speederror1;
	  	  SpeedA =20+Speederror1; 
		  Flag2 =0;
	      OLED_ShowNum(1,1,1,1);
		  break;
		  case 1:
		  Speederror = -Position_Cycle(&pid,0.3,0.05,0.1,231);//0.5 0.1 0.2
     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
		  SpeedA = 30-Speederror; 
		  Flag2=1;
		  OLED_ShowNum(1,1,2,1);
		  break;
	      case 2:
		  Speederror1 = Yaw_Cycle(&pid,0.3,0.05,0.2,40);  
		  SpeedB = 20+Speederror1;
		  SpeedA = 20-Speederror1;
		  OLED_ShowNum(1,1,3,1);
		  Flag2 =0;
		  break;
		  case 3:
			Speederror = -Position_Cycle(&pid,0.04,0.01,0.05,231);//0.04 0 0.05
     	    SpeedB = 25+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
			SpeedA = 25-Speederror; 
	        Flag2=1;
		    OLED_ShowNum(1,1,4,1);
		    OLED_ShowNum(2,5,Speederror2,3);
		   break;
	  } 
	  OLED_ShowNum(3,5,Speederror2,3);
	   OLED_ShowNum(3,5,yaw4,3);
	   OLED_ShowNum(4,1,sensorValue1,4);
	    OLED_ShowNum(2,1,yaw1,3);
		Motor_SetSpeedA(-SpeedA);//原A
		Motor_SetSpeedB(-SpeedB);
			 // 任务3
                }
                else if (mode_state == 4 && up_state ==3)//任务四
                {
      if(sensorValue1<250&&Flag2==0)
	  {
		  if(Flag1==4)
		  {
		  Flag1=1;
		  }else{
		  Flag1++;
		  }
	  }
	 if((yaw1<5||yaw1>350)&&Flag1==1)
	 {
	 Flag1++;
	 }
	 if(Flag1==3&&sensorValue1==251&&yaw1<225){
		Flag1++;
		Count++;
	 }
      switch(Flag1)
	  {       
		case 0:
		  Speederror1 = -Yaw_Cycle(&pid,0.2,0.025,0.1,145); 
		  SpeedB =20-Speederror1;
	  	  SpeedA =20+Speederror1; 
		  Flag2 =0;
	      OLED_ShowNum(1,1,1,1);
		  break;
		  case 1:
		  Speederror = -Position_Cycle(&pid,0.3,0.05,0.1,231);//0.5 0.1 0.2
     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
		  SpeedA = 30-Speederror; 
		  Flag2=1;
		  OLED_ShowNum(1,1,2,1);
		  break;
	      case 2:
		  Speederror1 = Yaw_Cycle(&pid,0.3,0.05,0.2,40);  
		  SpeedB = 20+Speederror1;
		  SpeedA = 20-Speederror1;
		  OLED_ShowNum(1,1,3,1);
		  Flag2 =0;
		  break;
		  case 3:
			Speederror = -Position_Cycle(&pid,0.04,0.01,0.05,231);//0.04 0 0.05
     	    SpeedB = 25+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
			SpeedA = 25-Speederror; 
	        Flag2=1;
		    OLED_ShowNum(1,1,4,1);
		    OLED_ShowNum(2,5,Speederror2,3);
		  break;
		 case 4:
		 Speederror1 = -Yaw_Cycle(&pid,0.2,0.025,0.1,152); 
		 SpeedB =20-Speederror1;
	  	 SpeedA =20+Speederror1; 
		 Flag2 =0;
	     OLED_ShowNum(1,1,1,1);
		 break;
//		 		  case 5:
//		  Speederror = -Position_Cycle(&pid,0.3,0.05,0.1,231);//0.5 0.1 0.2
//     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//		  SpeedA = 30-Speederror; 
//		  Flag2=1;
//		  OLED_ShowNum(1,1,2,1);
//		  break;
//	      case 6:
//		  Speederror1 = Yaw_Cycle(&pid,0.3,0.05,0.2,40);  
//		  SpeedB = 20+Speederror1;
//		  SpeedA = 20-Speederror1;
//		  OLED_ShowNum(1,1,3,1);
//		  Flag2 =0;
//		  break;
//		  case 7:
//			Speederror = -Position_Cycle(&pid,0.04,0.01,0.05,231);//0.04 0 0.05
//     	    SpeedB = 25+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//			SpeedA = 25-Speederror; 
//	        Flag2=1;
//		    OLED_ShowNum(1,1,4,1);
//		    OLED_ShowNum(2,5,Speederror2,3);
//		  break;
//		  case 8:
//		 Speederror1 = -Yaw_Cycle(&pid,0.2,0.025,0.1,152); 
//		 SpeedB =20-Speederror1;
//	  	 SpeedA =20+Speederror1; 
//		 Flag2 =0;
//	     OLED_ShowNum(1,1,1,1);
//		 break;
//		 		  case 9:
//		  Speederror = -Position_Cycle(&pid,0.3,0.05,0.1,231);//0.5 0.1 0.2
//     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//		  SpeedA = 30-Speederror; 
//		  Flag2=1;
//		  OLED_ShowNum(1,1,2,1);
//		  break;
//	      case 10:
//		  Speederror1 = Yaw_Cycle(&pid,0.3,0.05,0.2,40);  
//		  SpeedB = 20+Speederror1;
//		  SpeedA = 20-Speederror1;
//		  OLED_ShowNum(1,1,3,1);
//		  Flag2 =0;
//		  break;
//		  case 11:
//			Speederror = -Position_Cycle(&pid,0.04,0.01,0.05,231);//0.04 0 0.05
//     	    SpeedB = 25+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//			SpeedA = 25-Speederror; 
//	        Flag2=1;
//		    OLED_ShowNum(1,1,4,1);
//		    OLED_ShowNum(2,5,Speederror2,3);
//		  break;
//		  case 12:
//		 Speederror1 = -Yaw_Cycle(&pid,0.2,0.025,0.1,152); 
//		 SpeedB =20-Speederror1;
//	  	 SpeedA =20+Speederror1; 
//		 Flag2 =0;
//	     OLED_ShowNum(1,1,1,1);
//		 break;
//		 		  case 13:
//		  Speederror = -Position_Cycle(&pid,0.3,0.05,0.1,231);//0.5 0.1 0.2
//     	  SpeedB = 30+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//		  SpeedA = 30-Speederror; 
//		  Flag2=1;
//		  OLED_ShowNum(1,1,2,1);
//		  break;
//	      case 14:
//		  Speederror1 = Yaw_Cycle(&pid,0.3,0.05,0.2,40);  
//		  SpeedB = 20+Speederror1;
//		  SpeedA = 20-Speederror1;
//		  OLED_ShowNum(1,1,3,1);
//		  Flag2 =0;
//		  break;
//		  case 15:
//			Speederror = -Position_Cycle(&pid,0.04,0.01,0.05,231);//0.04 0 0.05
//     	    SpeedB = 25+Speederror;//右边30> 25>20  40对应0.9  40跑的真的可以  5
//			SpeedA = 25-Speederror; 
//	        Flag2=1;
//		    OLED_ShowNum(1,1,4,1);
//		    OLED_ShowNum(2,5,Speederror2,3);
//		  break;
		  
	  }
	    OLED_ShowNum(3,5,Speederror2,3);
	    OLED_ShowNum(3,5,yaw4,3);
	    OLED_ShowNum(4,1,sensorValue1,4);
	    OLED_ShowNum(2,1,yaw1,3);
		Motor_SetSpeedA(-SpeedA);//原A
		Motor_SetSpeedB(-SpeedB);
  }
	  if (adjust_state == 1 && up_state == 4)
                {
                    //OLED_Clear();  // 清除第1行
                    // 调参1
                    OLED_ShowString(1, 3, menu[9]);
                    OLED_ShowString(2, 3, menu[10]);
                    OLED_ShowString(3, 3, menu[11]);
					MODE_adjust ();
                    
                }
                else if (adjust_state == 2 && up_state == 4)
                {
                    //OLED_Clear();  // 清除第1行
                    // 调参2
                    OLED_ShowString(1, 3, menu[9]);
                    OLED_ShowString(2, 3, menu[10]);
                    OLED_ShowString(3, 3, menu[11]);
									MODE_adjust ();
                    
                }
                else if (adjust_state == 3 && up_state == 4)
                {
                    //OLED_Clear();  // 清除第1行
                    // 调参3
                    OLED_ShowString(1, 3, menu[9]);
                    OLED_ShowString(2, 3, menu[10]);
                    OLED_ShowString(3, 3, menu[11]);				
					MODE_adjust ();  
                }
    }
}
//解析姿态角
void dataGetERROR() {
	for(uint8_t i = 0; i < 100; ++i) {
		getMPU6050Data();
		mpu6050.MPU6050ERROE[0] += msg.A[0];
		mpu6050.MPU6050ERROE[1] += msg.A[1];
		mpu6050.MPU6050ERROE[2] += msg.A[2] - 9.8;
		mpu6050.MPU6050ERROE[3] += msg.G[0];
		mpu6050.MPU6050ERROE[4] += msg.G[1];
		mpu6050.MPU6050ERROE[5] += msg.G[2];
		Delay_ms(10);
	}
	for(uint8_t i = 0; i < 6; ++i) {
		mpu6050.MPU6050ERROE[i] /= 100.0f;
	}
}


void getMPU6050Data() {
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
	msg.A[0] = (float)((float)AX/(float)32768) * 16 * 9.8;
	msg.A[1] = (float)((float)AY/(float)32768) * 16 * 9.8;
	msg.A[2] = (float)((float)AZ/(float)32768) * 16 * 9.8;//*
	msg.G[0] = (float)((float)GX/(float)32768) * 2000 * 3.51*1.81818182;//*1.09689214*1.10091743
	msg.G[1] = (float)((float)GY/(float)32768) * 2000 * 3.51*1.81818182;//.09689214*1.100917433
	msg.G[2] = (float)((float)GZ/(float)32768) * 2000 * 3.51*1.81818182;//*1.09689214*1.100917433
	
}

void dataGetAndFilter() {
	getMPU6050Data();
	msg.A[0] -= mpu6050.MPU6050ERROE[0];
	msg.A[1] -= mpu6050.MPU6050ERROE[1];
	msg.A[2] -= mpu6050.MPU6050ERROE[2];
	msg.G[0] -= mpu6050.MPU6050ERROE[3];
	msg.G[1] -= mpu6050.MPU6050ERROE[4];
	msg.G[2] -= mpu6050.MPU6050ERROE[5];
}
//编码器中断获得速度
float Speed_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value)
{
  float  Speederror;
	uint16_t AveSpeed;
	AveSpeed = (SpeedA2+SpeedB2)/2;
	PID_Init(pid,kp,ki,kd,ideal_value);
	 Speederror= PID_Updata2(pid,AveSpeed);
	return Speederror;
}
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)		//判断是否是TIM2的更新事件触发的中断
	{         //1ms
        millis++;	
        SpeedA2 = Encoder_GetA();
		SpeedB2 = Encoder_GetB();
 sensorValue1=  GraySensor_Read(sensorState1); 		//每隔固定时间段读取一次编码器计数增量值，即为速度值
		yaw1 = getYaw();
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			//清除TIM2更新事件的中断标志位
  								            //中断标志位必须清除												//否则中断将连续不断地触发，导致主程序卡死
	}
}
