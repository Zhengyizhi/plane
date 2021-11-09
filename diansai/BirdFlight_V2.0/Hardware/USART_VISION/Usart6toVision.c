/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Usart6toVision.c
 * 描述   :视觉串口配置函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Usart6toVision.h"
//#define Tx_flow_len 160
//#define Rx_flow_len 65
//unsigned char Tx_Buf_Uart6[Tx_flow_len];
//unsigned char Rx_Buf_Uart6[Rx_flow_len];

//int Flag_Tx_Uart6_Busy=0;

//void Usart6toFlow_Init(u32 Bound)
//{	
//  //GPIO端口设置
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	
//  DMA_InitTypeDef DMA_InitStructure;   
//  
//  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
//  
//	//DMA发送中断设置
//	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//	NVIC_Init(&NVIC_InitStructure);  
//	
//	//DMA通道配置
//	DMA_DeInit(DMA2_Stream6);  //！！！！！！！！！！！！！！！
//	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
//	//外设地址 
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
//	//内存地址
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart6;  
//	//dma传输方向
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
//	//设置传输缓冲区的长度  
//	DMA_InitStructure.DMA_BufferSize = sizeof(Tx_Buf_Uart6);  
//	//设置DMA的外设递增模式 一个外设  
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//	//设置DMA的内存递增模式
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
//	//外设数据字长 
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//	//内存数据字长  
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
//	//设置DMA传输模式  
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
//	//设置优先级 
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
//		
//	//指定如果FIFO模式或直接模式将用于指定的流：不使能FIFO模式    
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
//	//指定了FIFO的阈值水平 
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
//	//指定的配置内存传输
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
//	//指定的配置外围转移   
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   

//	//配置DMA2的通道           
//	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    
//	//中断使能
//	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     


//  /* 串口收DMA配置 */
//	//DMA通道配置  
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
//	//配置DMA2 收通道          
//	DMA_Init(DMA2_Stream1, &DMA_InitStructure);    
//	//使能通道
//	DMA_Cmd(DMA2_Stream1,ENABLE);  
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
// 
//	//串口1对应引脚复用映射
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6复用为USART6
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART6
//	
//	//USART6端口配置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; //GPIOA9与GPIOA10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10


//   //USART6 初始化设置
//	USART_InitStructure.USART_BaudRate = Bound;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//	USART_Init(USART6, &USART_InitStructure); //初始化串口1
//	
//  // 中断配置	
//	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
//  USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
//  USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  

//	//USART6 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级0
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器、
//	
//	//采用DMA方式发送  
//	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  
//	//采用DMA方式接收
//	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

//	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_TXE,DISABLE);  
//	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  
//	
//	//启动串口
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
//void DMA2_Stream6_IRQHandler(void)  //看这个发出数据流
//{  
//		//进入中断调用ucos系统函数
//		OSIntEnter();
//		if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6) != RESET)   
//    {  
//        //清除标志位  
//        DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);  
//        //关闭DMA
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
//	//为连续工作关掉操作系统
//	  CPU_SR_ALLOC();
//		OS_CRITICAL_ENTER();
//		inf_Uart6_deal_irq_tx_end(); 
//    Flow_rx.len = inf_Uart6_deal_irq_rx_end(Flow_rx.buf);
//		if(Flow_rx.len == 14)
//		{
//			OSSemPost(&Flow_proc,OS_OPT_POST_1,&err);
//		}
//		OS_CRITICAL_EXIT();//开启操作系统
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
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;  
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); //使能UART6时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //使能DMA2时钟
	
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //接受引脚
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	 //GPIOC复用为UART6
	
	//DMA发送中断设置
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	
	//DMA通道配置
	DMA_DeInit(DMA2_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;   
	//外设地址 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Usart6;  
	//dma传输方向
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	//设置传输缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = sizeof(Tx_Buf_Usart6);  
	//设置DMA的外设递增模式 一个外设  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	//外设数据字长 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	//内存数据字长  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	//设置DMA传输模式  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	//设置优先级 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
	//指定如果FIFO模式或直接模式将用于指定的流：不使能FIFO模式    
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	//指定了FIFO的阈值水平 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	//指定的配置内存传输
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	//指定的配置外围转移   
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   

	//配置DMA2的通道           
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    
	//中断使能
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     
	
	

	/* 串口收DMA配置 */
	//DMA通道配置  
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
		
	//配置DMA1 收通道          
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);    
	//使能通道
	DMA_Cmd(DMA2_Stream1,ENABLE); 


	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				//GPIOC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 				//GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//UART5 初始化设置
	USART_InitStructure.USART_BaudRate = Bound;											//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;							//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); 												//初始化串口5
	
	//Uart5 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;				//串口5中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;	//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;				//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);													//根据指定的参数初始化VIC寄存器
	
	//采用DMA方式发送  
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	//采用DMA方式接收
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
	USART_ITConfig(USART6,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART6,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);  
	
	USART_Cmd(USART6, ENABLE);  															//使能串口2
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

void DMA2_Stream6_IRQHandler(void)  //发送
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
//	//发送ASCII码字符 “1”
//	USART_SendData(USART6,0X31);//这是给树莓派发送的！！！
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
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  //接受引脚
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	 //GPIOD5复用为UART2
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//上拉
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



