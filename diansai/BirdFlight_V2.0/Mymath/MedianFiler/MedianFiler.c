/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * 作者 	:Xiluna Tech
 * 文件名 :MedianFiler.c
 * 描述   :中位值滤波函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
******************************************************************************/
#include "MedianFiler.h"
/*************中位值平均滤波***************/
float Median_filter(int data,int measureNum,int *Filterdata)
{
  unsigned int i = 0;
	unsigned int j = 0;
	int temp;
	unsigned int MAX_error_targe = 0;
	int MAX_error1;
	float Average_data;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	for(i = 0 ; i < measureNum-1 ; i++)
	{
			for(j = 0 ; j < measureNum-1-i; j++)
			{
					if(Filterdata[j] > Filterdata[j+1] )
					{
							temp = Filterdata[j];
							Filterdata[j] =  Filterdata[j+1];
							Filterdata[j+1] = temp;
					}
			}
	}
	MAX_error1 = Filterdata[1] - Filterdata[0];
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < Filterdata[i+1] - Filterdata[i] )
			{
					MAX_error1 =  Filterdata[i+1] - Filterdata[i];
					MAX_error_targe = i; 
			}
	}
	Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;
	
}

/*************中位值平均浮点滤波***************/
float Median_filter_float(float data,int measureNum,float *Filterdata)
{
  unsigned int i = 0;
	unsigned int j = 0;
	float temp;
	unsigned int MAX_error_targe = 0;
	float MAX_error1;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	for(i = 0 ; i < measureNum-1 ; i++)
	{
			for(j = 0 ; j < measureNum-1-i; j++)
			{
					if(Filterdata[j] > Filterdata[j+1] )
					{
							temp = Filterdata[j];
							Filterdata[j] =  Filterdata[j+1];
							Filterdata[j+1] = temp;
					}
			}
	}
	MAX_error1 = Filterdata[1] - Filterdata[0];
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < Filterdata[i+1] - Filterdata[i] )
			{
					MAX_error1 =  Filterdata[i+1] - Filterdata[i];
					MAX_error_targe = i; 
			}
	}
	float Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;	
}

//////////////////////////////////////////////////////////////////
void Tension_Queue_Init(QueueObj *loadfull)//写死25
{
	loadfull->nowLength = 0;
	loadfull->queueLength = QUEUE_LEN;
	loadfull->switches = false;
}
int32_t Queue_Average_Flow_Filter(QueueObj * Tque, float data)//后面5个
{
	float sumque = 0;
	float sumque_1=0,sumque_2=0;
	uint8_t j = 0;
	if(Tque->nowLength < Tque->queueLength)				//队列未满，继续填进去
		Tque->queue[Tque->nowLength++] = data;
	
	else if(Tque->nowLength == Tque->queueLength)	//队列已满，FIFO
	{
		for(uint8_t i=0; i<(Tque->queueLength-1); i++)
		{
			Tque->queue[i] = Tque->queue[i+1];
		}
		Tque->queue[Tque->queueLength-1] = data;
	}
	
	if(Tque->nowLength >= Tque->queueLength)
	{
		for(j=0; j<(Tque->nowLength ); j++)//求均值
		{
			sumque += Tque->queue[j] * 0.1;
		}
//		for(; j< Tque->queueLength;j++)
//		{
//			sumque += Tque->queue[j] * 0.0125;
//		}
//		sumque_2 = (sumque_2/10)*0.25;
//		sumque = sumque_1+sumque_2;
		Tque->back = sumque;
//		Tension_Queue_choose_Init(Tque);
		Tque->switches = true;
		return 1;
	}
	else if(Tque->nowLength < Tque->queueLength)
	{
		return 0;
	}
	return 3;
}


