/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :General_Gpio.c
 * ����   :һЩͨ��GPIO�����ã������ƺͰ�����
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "General_Gpio.h"
void General_Gpioinit()
{
	GPIO_InitTypeDef GPIO_InitStructure;																					
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
	//LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;													
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;										
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;													
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_1;							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;													
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;										
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;													
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	//BEEP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	
	//KEY
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);													
	GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);	
}




