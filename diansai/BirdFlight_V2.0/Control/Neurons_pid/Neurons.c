/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Neurons.c
 * ����   :�⻷��Ԫ���ƺ���
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Neurons.h"
float Abs_Funcation(float pSrc);
/*
	�߶���Ԫ����
*/
static float np_height = 1,ni_height = 0,nd_height = 0.5;									//Z�߶�kp,ki,kdѧϰ����
static float Neurons_k_hight = 3.0;																				//Z�߶ȵ���ԪKֵ
static float output_height = 0,lastoutput_height = 0;
static float heightw1 = 0,heightw2 = 0,heightw3 = 0;
static float heightlastw1 = 0,heightlastw2 = 0,heightlastw3 = 0;
static float heightAverage_w1 = 0,heightAverage_w2 = 0,heightAverage_w3 = 0;
static float height_Average = 0;
static float height_err[3] = {0,0,0};
/*
	λ��X����Ԫ����
*/
static float np_posx = 1,ni_posx = 0,nd_posx = 0.5;												//Xλ��kp,ki,kdѧϰ����
static float Neurons_k_posx = 2.5;																				//Xλ�õ���ԪKֵ
static float output_posx = 0,lastoutput_posx = 0;
static float posxw1 = 0,posxw2 = 0,posxw3 = 0;
static float posxlastw1 = 0,posxlastw2 = 0,posxlastw3 = 0;
static float posxAverage_w1 = 0,posxAverage_w2 = 0,posxAverage_w3 = 0;
static float posx_Average = 0;
static float posx_err[3] = {0,0,0};
/*
	λ��Y����Ԫ����
*/
static float np_posy = 1,ni_posy = 0,nd_posy = 0.5;												//Yλ��kp,ki,kdѧϰ����
static float Neurons_k_posy = 2.5;																				//Yλ�õ���ԪKֵ
static float output_posy = 0,lastoutput_posy = 0;
static float posyw1 = 0,posyw2 = 0,posyw3 = 0;
static float posylastw1 = 0,posylastw2 = 0,posylastw3 = 0;
static float posyAverage_w1 = 0,posyAverage_w2 = 0,posyAverage_w3 = 0;
static float posy_Average = 0;
static float posy_err[3] = {0,0,0};

//���⻷
static float np_outx = 1,ni_outx = 0,nd_outx = 0.5;												//Yλ��kp,ki,kdѧϰ����
static float Neurons_k_outx = 2.5;																				//Yλ�õ���ԪKֵ
static float output_outx = 0,lastoutput_outx = 0;
static float outxw1 = 0,outxw2 = 0,outxw3 = 0;
static float outxlastw1 = 0,outxlastw2 = 0,outxlastw3 = 0;
static float outxAverage_w1 = 0,outxAverage_w2 = 0,outxAverage_w3 = 0;
static float outx_Average = 0;
static float outx_err[3] = {0,0,0};
/*
	����Ԫ�м����������ʼ��
*/
void Neurons_pidinit()
{
	//�߶Ȳ���
	output_height = 0;
	lastoutput_height = 0;
	heightw1 = 0;
	heightw2 = 0;
	heightw3 = 0;
	heightlastw1 = 1;
	heightlastw2 = 0;
	heightlastw3 = 1;
	heightAverage_w1 = 0;
	heightAverage_w2 = 0;
	heightAverage_w3 = 0;
	height_Average = 0;
	height_err[0] = 0;
	height_err[1] = 0;
	height_err[2] = 0;
	//Xλ�ò���
	output_posx = 0;
	lastoutput_posx = 0;
	posxw1 = 0;
	posxw2 = 0;
	posxw3 = 0;
	posxlastw1 = 1;
	posxlastw2 = 0;
	posxlastw3 = 1;
	posxAverage_w1 = 0;
	posxAverage_w2 = 0;
	posxAverage_w3 = 0;
	posx_Average = 0;
	posy_err[0] = 0;
	posy_err[1] = 0;
	posy_err[2] = 0;
	//Yλ�ò���
	output_posy = 0;
	lastoutput_posy = 0;
	posyw1 = 0;
	posyw2 = 0;
	posyw3 = 0;
	posylastw1 = 1;
	posylastw2 = 0;
	posylastw3 = 1;
	posyAverage_w1 = 0;
	posyAverage_w2 = 0;
	posyAverage_w3 = 0;
	posy_Average = 0;
	posx_err[0] = 0;
	posx_err[1] = 0;
	posx_err[2] = 0;
	//���⻷
	output_outx = 0;
	lastoutput_outx = 0;
	outxw1 = 0;
	outxw2 = 0;
	outxw3 = 0;
	outxlastw1 = 1;
	outxlastw2 = 0;
	outxlastw3 = 1;
	outxAverage_w1 = 0;
	outxAverage_w2 = 0;
	outxAverage_w3 = 0;
	outx_Average = 0;
	posx_err[0] = 0;
	posx_err[1] = 0;
	posx_err[2] = 0;
}
float Neurons_PID_OuterX(float Errdatay)
{
	static float x1_y,x2_y,x3_y;
	outx_err[2] = Errdatay;
	//Hebb learning algorithm
	outxw1 = outxlastw1 + np_outx * outx_err[2] * output_outx * x1_y;
	outxw2 = outxlastw2 + ni_outx * outx_err[2] * output_outx * x2_y;
	outxw3 = outxlastw3 + nd_outx * outx_err[2] * output_outx * x3_y;
	
	x1_y = outx_err[2] - outx_err[1];
	x2_y = outx_err[2];
	x3_y = outx_err[2] - 2*outx_err[1] + outx_err[0];
	
	outx_Average = (Abs_Funcation(outxw1) + Abs_Funcation(outxw2) + Abs_Funcation(outxw3));
	
	if(outx_Average == 0)
	{
		outx_Average = 0.0001;
	}
	
	outxAverage_w1 = outxw1/outx_Average;
	outxAverage_w2 = outxw2/outx_Average;
	outxAverage_w3 = outxw3/outx_Average;
	
	output_outx = lastoutput_outx + Neurons_k_outx * (outxAverage_w1*x1_y + outxAverage_w2*x2_y + outxAverage_w3*x3_y);
	
	outx_err[0] = outx_err[1];
	outx_err[1] = outx_err[2];
	
	lastoutput_outx = output_outx;
	
	outxlastw1 = outxw1;
	outxlastw2 = outxw2;
	outxlastw3 = outxw3;
	
	return output_outx;
}
/*
	����ԪPID
	���õ���Ԫ��ʽ
	u(k) = u(k-1) + K*(w11(k)*x1(k) + w22(k)*x2(k) + w33(k)*x3(k))
	Ȩ�ع�һ
	w11(k) = w1(k)/(w1(k) + w2(k) + w3(k))
	w22(k) = w2(k)/(w1(k) + w2(k) + w3(k))
	w33(k) = w3(k)/(w1(k) + w2(k) + w3(k))
	ѧϰ����
	w1(k) = w1(k-1) + ni*z(k)*u(k)*x1(k)
	w2(k) = w2(k-1) + np*z(k)*u(k)*x2(k)
	w3(k) = w3(k-1) + nd*z(k)*u(k)*x3(k)
	pid��������
	x1(k) = e(k)
	x2(k) = e(k) - e(k-1)
	x3(k) = e(k) - 2*e(k-1) + e(k-2)
	z(k) = e(k)
	
	����ni��np��nd�ֱ�Ϊ���֣�������΢�ֵ�ѧϰ���ʣ�KΪ��Ԫ�ı���������K>0��
*/
float Neurons_PID_Hight(float Errdata)
{
	static float x1,x2,x3;
	height_err[2] = Errdata;//0 e(k-2),e(k-1),e(k)
	//Hebb learning algorithm
	heightw1 = heightlastw1 + np_height * height_err[2] * output_height * x1;
	heightw2 = heightlastw2 + ni_height * height_err[2] * output_height * x2;
	heightw3 = heightlastw3 + nd_height * height_err[2] * output_height * x3;	
	
	x1 = height_err[2] - height_err[1];
	x2 = height_err[2];
	x3 = height_err[2] - 2*height_err[1] + height_err[0];	

	height_Average = (Abs_Funcation(heightw1) + Abs_Funcation(heightw2) + Abs_Funcation(heightw3));	
	
	if(height_Average == 0)
	{
		height_Average = 0.0001;
	}
	
	heightAverage_w1 = heightw1/height_Average;
	heightAverage_w2 = heightw2/height_Average;
	heightAverage_w3 = heightw3/height_Average;
	
	output_height = lastoutput_height + Neurons_k_hight * (heightAverage_w1*x1 + heightAverage_w2*x2 + heightAverage_w3*x3);
	
	height_err[0] = height_err[1];
	height_err[1] = height_err[2];
	
	lastoutput_height = output_height;

	heightlastw1 = heightw1;
	heightlastw2 = heightw2;
	heightlastw3 = heightw3;

	return output_height;
}

float Neurons_PID_Postionx(float Errdatax)
{
	static float x1_x,x2_x,x3_x;
	posx_err[2] = Errdatax;
	//Hebb learning algorithm
	posxw1 = posxlastw1 + np_posx * posx_err[2] * output_posx * x1_x;
	posxw2 = posxlastw2 + ni_posx * posx_err[2] * output_posx * x2_x;
	posxw3 = posxlastw3 + nd_posx * posx_err[2] * output_posx * x3_x;
	
	x1_x = posx_err[2] - posx_err[1];
	x2_x = posx_err[2];
	x3_x = posx_err[2] - 2*posx_err[1] + posx_err[0];
	
	posx_Average = (Abs_Funcation(posxw1) + Abs_Funcation(posxw2) + Abs_Funcation(posxw3));
	
	if(posx_Average == 0)
	{
		posx_Average = 0.0001;
	}
	
	posxAverage_w1 = posxw1/posx_Average;
	posxAverage_w2 = posxw2/posx_Average;
	posxAverage_w3 = posxw3/posx_Average;
	
	output_posx = lastoutput_posx + Neurons_k_posx * (posxAverage_w1*x1_x + posxAverage_w2*x2_x + posxAverage_w3*x3_x);
	
	posx_err[0] = posx_err[1];
	posx_err[1] = posx_err[2];
	
	lastoutput_posx = output_posx;
	
	posxlastw1 = posxw1;
	posxlastw2 = posxw2;
	posxlastw3 = posxw3;
	
	return output_posx;
}

float Neurons_PID_Postiony(float Errdatay)
{
	static float x1_y,x2_y,x3_y;
	posy_err[2] = Errdatay;
	//Hebb learning algorithm
	posyw1 = posylastw1 + np_posy * posy_err[2] * output_posy * x1_y;
	posyw2 = posylastw2 + ni_posy * posy_err[2] * output_posy * x2_y;
	posyw3 = posylastw3 + nd_posy * posy_err[2] * output_posy * x3_y;
	
	x1_y = posy_err[2] - posy_err[1];
	x2_y = posy_err[2];
	x3_y = posy_err[2] - 2*posy_err[1] + posy_err[0];
	
	posy_Average = (Abs_Funcation(posyw1) + Abs_Funcation(posyw2) + Abs_Funcation(posyw3));
	
	if(posy_Average == 0)
	{
		posy_Average = 0.0001;
	}
	
	posyAverage_w1 = posyw1/posy_Average;
	posyAverage_w2 = posyw2/posy_Average;
	posyAverage_w3 = posyw3/posy_Average;
	
	output_posy = lastoutput_posy + Neurons_k_posy * (posyAverage_w1*x1_y + posyAverage_w2*x2_y + posyAverage_w3*x3_y);
	
	posy_err[0] = posy_err[1];
	posy_err[1] = posy_err[2];
	
	lastoutput_posy = output_posy;
	
	posylastw1 = posyw1;
	posylastw2 = posyw2;
	posylastw3 = posyw3;
	
	return output_posy;
}


float Abs_Funcation(float pSrc)
{
	float pDst;
	arm_abs_f32(&pSrc,&pDst,1);
	return pDst;
}



