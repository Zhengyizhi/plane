/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Usart6toVision.c
 * ����   :�Ӿ��������ú���
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Usart6toVision.h"
//#define Tx_flow_len 160
//#define Rx_flow_len 65
//unsigned char Tx_Buf_Uart6[Tx_flow_len];
//unsigned char Rx_Buf_Uart6[Rx_flow_len];

//int Flag_Tx_Uart6_Busy=0;

//void Usart6toFlow_Init(u32 Bound)
//{	
//  //GPIO�˿�����
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	
//  DMA_InitTypeDef DMA_InitStructure;   
//  
//  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
//  
//	//DMA�����ж�����
//	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//	NVIC_Init(&NVIC_InitStructure);  
//	
//	//DMAͨ������
//	DMA_DeInit(DMA2_Stream6);  //������������������������������
//	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
//	//�����ַ 
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
//	//�ڴ��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart6;  
//	//dma���䷽��
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
//	//���ô��仺�����ĳ���  
//	DMA_InitStructure.DMA_BufferSize = sizeof(Tx_Buf_Uart6);  
//	//����DMA���������ģʽ һ������  
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//	//����DMA���ڴ����ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
//	//���������ֳ� 
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//	//�ڴ������ֳ�  
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
//	//����DMA����ģʽ  
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
//	//�������ȼ� 
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
//		
//	//ָ�����FIFOģʽ��ֱ��ģʽ������ָ����������ʹ��FIFOģʽ    
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
//	//ָ����FIFO����ֵˮƽ 
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
//	//ָ���������ڴ洫��
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
//	//ָ����������Χת��   
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   

//	//����DMA2��ͨ��           
//	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    
//	//�ж�ʹ��
//	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     


//  /* ������DMA���� */
//	//DMAͨ������  
//	DMA_DeInit(DMA2_Stream1);  
//	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Uart6;   
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
//	DMA_InitStructure.DMA_BufferSize = sizeof(Rx_Buf_Uart6);   
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
//  
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;           
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
//		
//	//����DMA2 ��ͨ��          
//	DMA_Init(DMA2_Stream1, &DMA_InitStructure);    
//	//ʹ��ͨ��
//	DMA_Cmd(DMA2_Stream1,ENABLE);  
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
// 
//	//����1��Ӧ���Ÿ���ӳ��
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6����ΪUSART6
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7����ΪUSART6
//	
//	//USART6�˿�����
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; //GPIOA9��GPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA9��PA10


//   //USART6 ��ʼ������
//	USART_InitStructure.USART_BaudRate = Bound;//����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//	USART_Init(USART6, &USART_InitStructure); //��ʼ������1
//	
//  // �ж�����	
//	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
//  USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
//  USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  

//	//USART6 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ�����
//	
//	//����DMA��ʽ����  
//	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  
//	//����DMA��ʽ����
//	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

//	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_TXE,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  
//	
//	//��������
//	USART_Cmd(USART6, ENABLE);
//}


//void Usart6_tx(uint8_t *data,uint16_t size)  
//{  
//    while (Flag_Tx_Uart6_Busy);  
//    Flag_Tx_Uart6_Busy = 1;   
//    memcpy(Tx_Buf_Uart6,data,size);  
//    DMA_SetCurrDataCounter(DMA2_Stream6,size);   
//    DMA_Cmd(DMA2_Stream6,ENABLE);  
//}  


//   
//uint8_t inf_Uart6_deal_irq_tx_end(void)  
//{  
//    if(USART_GetITStatus(USART6, USART_IT_TC) != RESET)  
//    {  
//        USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
//        Flag_Tx_Uart6_Busy = 0;  
//          
//        return 1;  
//    }        
//    return 0;    
//}  
//  
// 
//uint8_t inf_Uart6_deal_irq_rx_end(uint8_t *buf)  
//{  
//   uint16_t len = 0;  
//      
//    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  
//    {  
//        USART6->SR;  
//        USART6->DR;
//        DMA_Cmd(DMA2_Stream1,DISABLE);  
//        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);  
//        len = Rx_flow_len - DMA_GetCurrDataCounter(DMA2_Stream1);  
//        memcpy(buf,Rx_Buf_Uart6,len);   
//        DMA_SetCurrDataCounter(DMA2_Stream1,Rx_flow_len);  
//        DMA_Cmd(DMA2_Stream1,ENABLE);  
//        return len;  
//    }   
//    return 0;  
//}

//     
//  
//void DMA2_Stream6_IRQHandler(void)  //���������������
//{  
//		//�����жϵ���ucosϵͳ����
//		OSIntEnter();
//		if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET)   
//    {  
//        //�����־λ  
//        DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);  
//        //�ر�DMA
//        DMA_Cmd(DMA2_Stream6,DISABLE);    
//        USART_ITConfig(USART6,USART_IT_TC,ENABLE);  
//    } 
//		OSIntExit();		
//}  
//     
//_Data_Rx Flow_rx; 
//void USART6_IRQHandler(void)   //!!!!!!!!!!!
//{  
//		OS_ERR err;	 
//		OSIntEnter();	
////    inf_Uart6_deal_irq_tx_end();  	  
////    Vision.len = inf_Uart6_deal_irq_rx_end(Vision.buf);  
////    if (Vision.len != 0)  
////		{  	
////			OSSemPost(&Vision_proc,OS_OPT_POST_1,&err);
////    }
//	//Ϊ���������ص�����ϵͳ
//	  CPU_SR_ALLOC();
//		OS_CRITICAL_ENTER();
//		inf_Uart6_deal_irq_tx_end(); 
//    Flow_rx.len = inf_Uart6_deal_irq_rx_end(Flow_rx.buf);
//		if(Flow_rx.len == 14)
//		{
//			OSSemPost(&Flow_proc,OS_OPT_POST_1,&err);
//		}
//		OS_CRITICAL_EXIT();//��������ϵͳ
//		OSIntExit();
//} 

//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#define BUFflow_Len 512
unsigned char Rx_Buf_Usart6[BUFflow_Len];
unsigned char Tx_Buf_Usart6[BUFflow_Len];
int Flag_Tx_Usart6_Busy=0;

void Usart6toflow_Init(u32 Bound)
{	
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;  
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //ʹ��UART6ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //ʹ��DMA2ʱ��
	
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //��������
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	 //GPIOC����ΪUART6
	
	//DMA�����ж�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	
	//DMAͨ������
	DMA_DeInit(DMA2_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
	//�����ַ 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	//�ڴ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Usart6;  
	//dma���䷽��
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	//���ô��仺�����ĳ���  
	DMA_InitStructure.DMA_BufferSize = sizeof(Tx_Buf_Usart6);  
	//����DMA���������ģʽ һ������  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	//���������ֳ� 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	//�ڴ������ֳ�  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	//����DMA����ģʽ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	//�������ȼ� 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
	//ָ�����FIFOģʽ��ֱ��ģʽ������ָ����������ʹ��FIFOģʽ    
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	//ָ����FIFO����ֵˮƽ 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	//ָ���������ڴ洫��
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	//ָ����������Χת��   
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   

	//����DMA2��ͨ��           
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    
	//�ж�ʹ��
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     
	
	

	/* ������DMA���� */
	//DMAͨ������  
	DMA_DeInit(DMA2_Stream1);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Usart6;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	DMA_InitStructure.DMA_BufferSize = sizeof(Rx_Buf_Usart6);   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;           
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
		
	//����DMA1 ��ͨ��          
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);    
	//ʹ��ͨ��
	DMA_Cmd(DMA2_Stream1,ENABLE); 


	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				//GPIOC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 				//GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//����
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//UART5 ��ʼ������
	USART_InitStructure.USART_BaudRate = Bound;											//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;							//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); 												//��ʼ������5
	
	//Uart5 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;				//����5�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;	//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;				//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);													//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	//����DMA��ʽ����  
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	//����DMA��ʽ����
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART6,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  
	
	USART_Cmd(USART6, ENABLE);  															//ʹ�ܴ���2
	USART_ClearFlag(USART6,USART_FLAG_TC);
}

void Uart6_tx(uint8_t *data,uint16_t size)  
{ 
    while (Flag_Tx_Usart6_Busy);  
    Flag_Tx_Usart6_Busy = 1;   
    memcpy(Tx_Buf_Usart6,data,size);  
    DMA_SetCurrDataCounter(DMA2_Stream6,size);   
    DMA_Cmd(DMA2_Stream6,ENABLE);  
}  

uint8_t inf_Usart6_deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART6, USART_IT_TC) != RESET)  
    {  
        USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
        Flag_Tx_Usart6_Busy = 0;  
          
        return 1;  
    }        
    return 0;    
}


uint8_t inf_Uart6_deal_irq_rx_end(uint8_t *buf)  
{  
   uint16_t len = 0;  
      
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  
    { 	
        USART6->SR;  
        USART6->DR;
        DMA_Cmd(DMA2_Stream1,DISABLE);  
        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);  
        len = BUFflow_Len - DMA_GetCurrDataCounter(DMA2_Stream1);  
        memcpy(buf,Rx_Buf_Usart6,len);   
        DMA_SetCurrDataCounter(DMA2_Stream1,BUFflow_Len);  
        DMA_Cmd(DMA2_Stream1,ENABLE);  
        return len;  
    }   
    return 0;  
}

void DMA2_Stream6_IRQHandler(void)  //����
{  
		OSIntEnter();
		if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET)   
    {  
        DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);  
        DMA_Cmd(DMA2_Stream6,DISABLE);    
        USART_ITConfig(USART6,USART_IT_TC,ENABLE);  
    } 
		OSIntExit();		
} 




//volatile uint16_t ReceiveHeight_ff;
_Data_Rx Flosion;
void Usart6_irq_handler(void)                                
{
	OS_ERR err;
  CPU_SR_ALLOC();	
	OS_CRITICAL_ENTER();
	inf_Usart6_deal_irq_tx_end();
	Flosion.len = inf_Uart6_deal_irq_rx_end(Flosion.buf);  
	if (Flosion.len == 14)  
	{ 
//		ReceiveHeight_ff = ((((uint16_t)Flosion.buf[0])<<8) + Flosion.buf[1]);
		OSSemPost(&Flow_proc,OS_OPT_POST_1,&err);
	}	
	OS_CRITICAL_EXIT();
} 
void USART6_IRQHandler(void)   
{  
		OSIntEnter();
    Usart6_irq_handler();
		OSIntExit();	
} 

// 


//	
//void ReceiveFlowData_try()
//{
//	//����ASCII���ַ� ��1��
//	USART_SendData(USART6,0X31);//���Ǹ���ݮ�ɷ��͵ģ�����
//}
//void USART6_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
// 
// 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
//  
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //��������
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	 //GPIOD5����ΪUART2
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//����
//  GPIO_Init(GPIOC, &GPIO_InitStructure);    
//  
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
////  GPIO_InitStructure.GPIO_Mode = GPIO_OType_PP;
//	USART_DeInit(USART6);
//	USART_InitStructure.USART_BaudRate = 19200;	
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	
//	USART_InitStructure.USART_Parity = USART_Parity_No ;  
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART6, &USART_InitStructure);  
//	USART_Cmd(USART6, ENABLE);
//}


// void UART6SendByte(unsigned char SendData)
//{	   
//        USART_SendData(USART6,SendData);
//        while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);	    
//} 


//unsigned char UART6GetByte(unsigned char* GetData)
//{   	   
//        if(USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET)
//        {  return 0;
//		}
//        *GetData = USART_ReceiveData(USART6); 
//        return 1;
//}



