/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Incremental_pid.c
 * ����   :����ʽpid
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Incremental_pid.h"
/*
	����ʽPID����
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




