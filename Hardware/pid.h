#ifndef  __PID_H
#define  __PID_H
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
void PID_Init(PID*pid, float kp,float ki,float kd,float ideal_value);
float PID_Updata1(PID*pid,uint16_t sensor);
float PID_Updata2(PID*pid,uint16_t sensor);
void PID_SUM_Init(void);
void PID_PWM(void);
float Position_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value);
float Yaw_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value);
float Speed_Cycle(PID*pid, float kp,float ki,float kd,float ideal_value);
float Position_Cycle1(PID*pid, float kp,float ki,float kd,float ideal_value);
float Yaw_Cycle1(PID*pid, float kp,float ki,float kd,float ideal_value);

#endif  
  
