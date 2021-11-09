#ifndef __MEDIANFILER_H
#define __MEDIANFILER_H

#include "stm32f4xx.h"
#include "stdbool.h"
#define QUEUE_LEN 10
typedef struct
{
	int32_t nowLength;
	int32_t queueLength;//≥§∂»
	float queue[QUEUE_LEN];//÷∏’Î
	float back;
//	uint32_t zero_static_err;
	bool switches;
}QueueObj;
extern QueueObj stile_queue;
float Median_filter(int data,int measureNum,int *Filterdata);
float Median_filter_float(float data,int measureNum,float *Filterdata);

void Tension_Queue_Init(QueueObj *loadfull);
int32_t Queue_Average_Flow_Filter(QueueObj * Tque, float data);

#endif 

