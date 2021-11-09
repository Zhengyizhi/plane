/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Incremental_pid.c
 * 描述   :增量式pid
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Incremental_pid.h"
/*
	增量式PID控制
*/
float Increamental_pidcontrol(float errdata,float IncreamentalKp,float IncreamentalKi,float IncreamentalKd,float *Increamentaldata)
{
	float IncreamentalOutput;
	Increamentaldata[0] = Increamentaldata[1];
	Increamentaldata[1] = Increamentaldata[2];
	Increamentaldata[2] = errdata;
	
	IncreamentalOutput = IncreamentalKp * (Increamentaldata[2] - Increamentaldata[1])
																	+ IncreamentalKi * Increamentaldata[2]
																						+IncreamentalKd * (Increamentaldata[2] - 2*Increamentaldata[1] + Increamentaldata[0]);
	return IncreamentalOutput;
}




