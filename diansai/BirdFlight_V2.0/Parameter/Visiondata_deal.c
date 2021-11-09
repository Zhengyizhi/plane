/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * 作者 	:Xiluna Tech
 * 文件名 :Visiondata_deal.c
 * 描述   :处理视觉发来的数据
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
******************************************************************************/
#include "Visiondata_deal.h"


u8 BlackspotsFlag = 1;
u8 OpticalflowFlag = 0;
volatile float Pix_Xinfo;
volatile float Pix_Yinfo;
volatile float OpticalFlow_x;
volatile float OpticalFlow_y;
volatile float LastOpticalFlow_x;
volatile float LastOpticalFlow_y;
volatile float OpticalFlow_integralx;
volatile float OpticalFlow_integraly;
volatile float OpticalFlow_filterx;
volatile float OpticalFlow_filtery;
static int PointX_Data[20];
static int PointY_Data[20];
static int Flowvx_Data[20];
static int Flowvy_Data[20];

void Vision_datadeal(_Data_Rx rx)
{
	int Blob_num = 11;
	int Flow_num = 11;
	int tmp;
	if(rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA)
	{
		/*接收视觉模块发来的信息*/
		if(rx.buf[2]==0x10)
		{	
			if(rx.buf[3]==0x01)//点数据
			{
				/*  Target_Roll */
//				tmp = ((int16_t)rx.buf[7]<<8) + rx.buf[8];
//				Pix_Xinfo = ((Median_filter(tmp,Blob_num,PointX_Data))*0.0001f - ImageCenter_x) * US100_Altinfo;
//				/*  Target_Pitch */
//				tmp = ((int16_t)rx.buf[9]<<8) + rx.buf[10];
//				Pix_Yinfo = ((Median_filter(tmp,Blob_num,PointY_Data))*0.0001f - ImageCenter_y) * US100_Altinfo;	
//				BlackspotsFlag = 1;
//				OpticalflowFlag = 0;
			}
			else if(rx.buf[3]==0x02)//光流数据
			{
				/* 获取当前flow坐标 */
				tmp = ((int16_t)rx.buf[7]<<8) + rx.buf[8];
				if(tmp>=32768)
				{
					tmp = 32768 - tmp;
				}
				else
				{
					tmp = tmp;
				}
				OpticalFlow_x = (Median_filter(tmp,Flow_num,Flowvx_Data))*0.0005f;
				
				tmp = ((int16_t)rx.buf[9]<<8) + rx.buf[10];
				if(tmp>=32768)
				{
					tmp = 32768 - tmp;
				}
				else
				{
					tmp = tmp;
				}
				OpticalFlow_y = (Median_filter(tmp,Flow_num,Flowvy_Data))*0.0005f;
//				OpticalFlow_y = ((float)tmp)*0.0001f;//0.0001f;
				
								
//				/* 由于shitomasi点的跳变会带来速度的突变，进行简单滤波处理 */
//				if((OpticalFlow_x - LastOpticalFlow_x) > 0.05f || (OpticalFlow_x - LastOpticalFlow_x) < -0.05f)
//				{
//					OpticalFlow_filterx = 0;
//					OpticalFlow_filtery = 0;
//				}
//				else
//				{
//					OpticalFlow_filterx = OpticalFlow_x - LastOpticalFlow_x;
//					OpticalFlow_filtery = OpticalFlow_y - LastOpticalFlow_y;
//				}
//				/* 位移累加 */
//				OpticalFlow_integralx += OpticalFlow_filterx * US100_Altinfo;
//				OpticalFlow_integraly += OpticalFlow_filtery * US100_Altinfo;
//				/* 获取上一次flow坐标点 */
//				LastOpticalFlow_x = OpticalFlow_x;
//				LastOpticalFlow_y = OpticalFlow_y;
//				
				BlackspotsFlag = 0;
				OpticalflowFlag = 1;
			}
			else if(rx.buf[3]==0x03)//降落信号
			{
				FlightControl.landFlag=1;
			}						
		}		
	}
}




