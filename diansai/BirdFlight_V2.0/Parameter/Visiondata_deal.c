/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * ���� 	:Xiluna Tech
 * �ļ��� :Visiondata_deal.c
 * ����   :�����Ӿ�����������
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
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
		/*�����Ӿ�ģ�鷢������Ϣ*/
		if(rx.buf[2]==0x10)
		{	
			if(rx.buf[3]==0x01)//������
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
			else if(rx.buf[3]==0x02)//��������
			{
				/* ��ȡ��ǰflow���� */
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
				
								
//				/* ����shitomasi������������ٶȵ�ͻ�䣬���м��˲����� */
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
//				/* λ���ۼ� */
//				OpticalFlow_integralx += OpticalFlow_filterx * US100_Altinfo;
//				OpticalFlow_integraly += OpticalFlow_filtery * US100_Altinfo;
//				/* ��ȡ��һ��flow����� */
//				LastOpticalFlow_x = OpticalFlow_x;
//				LastOpticalFlow_y = OpticalFlow_y;
//				
				BlackspotsFlag = 0;
				OpticalflowFlag = 1;
			}
			else if(rx.buf[3]==0x03)//�����ź�
			{
				FlightControl.landFlag=1;
			}						
		}		
	}
}




