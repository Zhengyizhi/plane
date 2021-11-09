#ifndef __Fuzzypid_H
#define __Fuzzypid_H

typedef struct 
{
	float setVaule;//设定值
	float deta_Kp;
	float deta_Ki;
	float deta_Kd;
	float lasterror;//角度误差
	float preerror;//前前误差,也就是速度
	
	float maximum_1;//输出限幅
	float minimum_1;
	
	float maximum_2;//输出限幅
	float minimum_2;
	
	float qKp;//增量的修正值
	float qKi;
	float qKd;
	
}FuzzyPID;


extern FuzzyPID FPID;

void Fuzzytrans(float Set_Value,float Measure_Value,float pre_Measure_Value,float limit_min,float limit_max);

#endif
