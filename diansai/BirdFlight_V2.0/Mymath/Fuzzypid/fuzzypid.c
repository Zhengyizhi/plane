#include "fuzzypid.h"

#define NB -7
#define NM -3
#define NS -1
#define ZO 0
#define PS 1
#define PM 3
#define PB 7


static const float ruleKp[7][7]=
{
	PB, PM, PS, PS, ZO, ZO, ZO,
  PS, PS, PS, ZO, ZO, ZO, NS,
  NS, NM, NM, NB, NM, NM, NM,
  NM, NM, NB, NB, NB, NM, NM,
  NM, NM, NM, NB, NM, NM, NS,
  NS, ZO, ZO, ZO, PS, PS, PS,
  ZO, ZO, ZO, PS, PS, PM, PB,	
};


//////static const float ruleKp[7][7]=
//////{
//////	PS, ZO, NM, ZO, PB, PB, PB,
//////	PS, PS, NS, NS, PM, PB, PB,
//////	PM, PS, ZO, NM, PS, PM, PB,
//////	PM, PM, PS, NB, PS, PM, PM,
//////	PB, PM, PS, NM, ZO, PS, PM,
//////	PB, PB, PM, NS, NS, PS, PS,
//////	PB, PB, PB, ZO, NM, ZO, PS,
//////};




//static const float ruleKi[7][7]=
//{
//	NB, NB, NM, PS, PB, PB, NS,
//	NB, NS, NS, PM, PS, PS, NS,
//	NB, ZO, NS, PB, PS, ZO, NM,
//	NM, ZO, ZO, PB, ZO, ZO, NM,
//	NM, ZO, PS, PB, NS, ZO, NB,
//	NS, PS, PS, PM, NS, NS, NB,
//	NS, PB, PB, PS, NM, NB, NB,
//};

static const float ruleKi[7][7]=
{
	NB, NB, NB, NM, NM, NS, NS,
	NB, NS, ZO, ZO, ZO, PS, PB,
	NM, NS, NS, ZO, PS, PS, PB,
	PS, PM, PB, PB, PB, PM, PS,
	PB, PS, PS, ZO, NS, NS, NM,
	PB, PS, ZO, ZO, ZO, NS, NB,
	NS, NS, NM, NM, NB, NB, NB,
};


FuzzyPID FPID = {0,0,0,0,0,0,0,0,0,0,1,1,1};




static void CalcMembership(float *ms, float qv, int* index)//计算隶属度函数
{
	if((qv>=NB)&&(qv<NM))
	{
		index[0] = 0;
		index[1] = 1;
		ms[0] = 0.143f*qv +1.f;//e
		ms[1] = -0.143f*qv;//ec
	}
	else if((qv>=NM)&&(qv<NS))
	{
		index[0] = 1;
		index[1] = 2;
		ms[0] = 0.143f*qv +1.f;
		ms[1] = -0.143f*qv;
	}
	else if((qv>=NS)&&(qv<ZO))
	{
		index[0] = 2;
		index[1] = 3;
		ms[0] = 0.143f*qv +1.f;
		ms[1] = -0.143f*qv;
	}
		else if((qv>=ZO)&&(qv<PS))
	{
		index[0] = 3;
		index[1] = 4;
		ms[0] = -0.143f*qv +1.f;
		ms[1] = 0.143f*qv;
	}
	else if((qv>=PS)&&(qv<PM))
	{
		index[0] = 4;
		index[1] = 5;
		ms[0] = -0.143f*qv +1.f;
		ms[1] = 0.143f*qv;
	}
	else if((qv>=PM)&&(qv<PB))
	{
		index[0] = 5;
		index[1] = 6;
		ms[0] = -0.143f*qv +1.f;
		ms[1] = 0.143f*qv;
	}
}
static void LinearQuantization(FuzzyPID *vPID,float _Real_Value,float *qValue)
{
	float thisError;
	float deltaError;

	thisError = vPID->setVaule - _Real_Value;
	deltaError = thisError - vPID->lasterror;
	
	qValue[0] =  7.0f*thisError/(vPID->maximum_1 - vPID->minimum_1);//+7
	qValue[1] =  7.0f*deltaError/(vPID->maximum_2 - vPID->minimum_2);
}



//解模糊
static void FuzzyComputation(FuzzyPID *vPID,float _Real_Value)
{
	float qValue[2] = {0,0};//量化值
	int indexE[2] = {0,0};//规则库中的索引
	float msE[2] = {0,0};//隶属度
	
	int indexEC[2] = {0,0};
	float msEC[2] = {0,0};

	float pidvalue[3];//pid增量值0-KP,1-KI
	
	LinearQuantization(vPID,_Real_Value,qValue);
	CalcMembership(msE,qValue[0],indexE);
	CalcMembership(msEC,qValue[1],indexEC);
	
	pidvalue[0] = msE[0]*(msEC[0]*ruleKp[indexE[0]] [indexEC[0]] + msEC[1]*ruleKp[indexE[0]] [indexEC[1]])//sudu
	            + msE[1]*(msEC[0]*ruleKp[indexE[1]] [indexEC[0]] + msEC[1]*ruleKp[indexE[1]] [indexEC[1]]);
	
	pidvalue[1] = msE[0]*(msEC[0]*ruleKi[indexE[0]] [indexEC[0]] + msEC[1]*ruleKi[indexE[0]] [indexEC[1]])
	            + msE[1]*(msEC[0]*ruleKi[indexE[1]] [indexEC[0]] + msEC[1]*ruleKi[indexE[1]] [indexEC[1]]);
	
	vPID ->deta_Kp = vPID->qKp * pidvalue[0];
	vPID ->deta_Ki = vPID->qKi * pidvalue[1];
}

void Fuzzytrans(float Set_Value,float Measure_Value,float pre_Measure_Value,float limit_min,float limit_max)
{
	FPID.setVaule = Set_Value;
	FPID.lasterror = Set_Value - pre_Measure_Value;//角度误差
	
	FPID.qKp = 0.28;//1.7
	FPID.qKi = 0.1;
	
	FPID.maximum_1 = limit_max;///
	FPID.minimum_1 = -limit_min;///
	FPID.maximum_2 = 2*limit_max;
	FPID.minimum_2 = -2*limit_min;

	FuzzyComputation(&FPID,Measure_Value);
}

