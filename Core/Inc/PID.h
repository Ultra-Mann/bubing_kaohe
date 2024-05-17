#ifndef __PID_H
#define __PID_H

#define PID_POSITION 0
#define PID_DELTA 1
#include "main.h"

typedef struct
{
	uint8_t mode;
	
	float Kp;
	float Ki;
	float Kd;
	
	float max_out;
	float max_iout;
	
	float set;
	float fdb;
	
	float out;
	float Pout;
	float Iout;
	float Dout;
	float Dbuf[3];
	float error[3];
	
	float err;
}pid_type_def;
//float loop_fp32_constrain(float Input, float minValue, float maxValue);
//float gimbal_PID_calc(PidTypeDef *pid, float get, float set);
//float limit(float input,float output);
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
float PID_calc(pid_type_def *pid,float ref,float set);
void PID_clear(pid_type_def *pid);

#endif
