#ifndef __Fuzzypid_H
#define __Fuzzypid_H

typedef struct 
{
	float setVaule;//�趨ֵ
	float deta_Kp;
	float deta_Ki;
	float deta_Kd;
	float lasterror;//�Ƕ����
	float preerror;//ǰǰ���,Ҳ�����ٶ�
	
	float maximum_1;//����޷�
	float minimum_1;
	
	float maximum_2;//����޷�
	float minimum_2;
	
	float qKp;//����������ֵ
	float qKi;
	float qKd;
	
}FuzzyPID;


extern FuzzyPID FPID;

void Fuzzytrans(float Set_Value,float Measure_Value,float pre_Measure_Value,float limit_min,float limit_max);

#endif
