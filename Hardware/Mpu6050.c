#include "stm32f10x.h"                  // Device heade
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "math.h"
#include "Delay.h"
#include "Madgwick.h"
#define MPU6050_ADDRESS    0xD0        // MPU6050 I2C

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

extern float pitch;
extern float roll;
extern float yaw;  // Yaw ???

/**
  * 函    数：MPU6050等待事件
  * 参    数：同I2C_CheckEvent
  * 返 回 值：无
  */
//void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
//{
//uint32_t Timeout;
//Timeout = 10000;									//给定超时计数时间
//while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
//{
//	Timeout --;										//等待时，计数值自减
//	if (Timeout == 0)								//自减到0后，等待超时
//	{
//		/*超时的错误处理代码，可以添加到此处*/
//		break;										//跳出等待，不等了
//	}
//}
//}



// MPU6050 ????
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

// MPU6050 ????
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    
    return Data;
}

// MPU6050 ???
void MPU6050_Init(void)
{
    MyI2C_Init(); 
    
   /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

// ?? MPU6050 ??
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;
    
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}

// ?? pitch, roll ? yaw
//#define FILTER_ALPHA 1.0
//#define deltaTime 0.01  // 10ms(按需求修改)

//static float yaw = 0.0f;  // ???

//void Compute_Attitude(float *pitchOut, float *rollOut, float *yawOut)
//{
//    int16_t accX, accY, accZ;
//    int16_t gyroX, gyroY, gyroZ;


//    MPU6050_GetData(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);


//    float accelPitch = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * 180.0f / M_PI;
//    float accelRoll = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / M_PI;
//		float accelYaw = atan2f(accY, accX) * 180.0f / M_PI;  // ???????? yaw
//    gyroX = gyroX / 131.0f;  // ??? ?s
//    gyroY = gyroY / 131.0f;
//    gyroZ = gyroZ / 131.0f;


//    float gyroYaw = yaw + gyroZ * deltaTime;

//   

//    yaw = FILTER_ALPHA * gyroYaw + (1.0f - FILTER_ALPHA) * accelYaw;
//	if (yaw > 180.0f) {
//        yaw -= 360.0f;
//    }
//    if (yaw < -180.0f) {
//        yaw += 360.0f;
//    }


//    // ????? yaw
//    *yawOut = yaw*25;
//	if (*yawOut > 180.0f) {
//        *yawOut -= 360.0f;
//    }
//    if (*yawOut < -180.0f) {
//        *yawOut += 360.0f;
//    }

//    // ?? pitch ? roll
//    *pitchOut = accelPitch;
//    *rollOut = accelRoll;
//}
//float i;
//float ax_offset = 0 ,ay_offset =0 ; //x,y的加速度
//float gx_offset =0 ,gy_offset =0 ;//x,y角速度的偏移量

////参数
//float rad2deg = 57.29578 ; //弧度到角度的换算
//float roll_v =0 ,pitch_v =0 ; //换算到x，y的角速度

////微分的时间
//float  now = 0 ,lasttime =0 ,dt =0;

////卡尔曼滤波的先看状态
//float gyro_roll = 0,gyro_pitch = 0;//陀螺仪积分算出的角度，先验状态
//float acc_roll = 0,acc_pitch =0;//加速度测出的角度，观测状态
//float k_roll =0 ,k_pitch  =0;//卡尔曼滤波后估计出最优角度，最优估计状态
////误差协方差矩阵
//float e_P[2][2] = {{1,0},{0,1}};
////卡尔曼增益
//float k_k[2][2] = {{0,0},{0,0}};

//void Coupute(void)
//{
//	MPU6050_Init();
//    for(i=1;i<=2000;i++)
//	{
//    int16_t accX, accY, accZ;
//    int16_t gyroX, gyroY, gyroZ;
//    MPU6050_GetData(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
//	ax_offset = ax_offset +accX;
//	ay_offset = ay_offset +accY;
//	gx_offset = gx_offset +gyroX;
//	gy_offset = gy_offset +gyroY;
//	}
//	ax_offset = ax_offset/2000;
//	ay_offset = ay_offset/2000;
//	gx_offset = gx_offset/2000;
//	gy_offset = gy_offset/2000;
//	Delay_ms(100);
//}