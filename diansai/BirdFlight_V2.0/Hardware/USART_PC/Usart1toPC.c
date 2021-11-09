/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Usart1toPC.c
 * 描述   :上传PC数据串口配置
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Usart1toPC.h"

unsigned char Tx_Buf_Uart1[512];
unsigned char Rx_Buf_Uart1[512];

int Flag_Tx_Uart1_Busy=0;

void Usart1toPC_Init(u32 Bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
  DMA_InitTypeDef DMA_InitStructure;   
  
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  
	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	//DMA通道配置
	DMA_DeInit(DMA2_Stream7);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	//外设地址 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart1;  
	//dma传输方向
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	//设置传输缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = TX_LEN;  
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
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);    
	//中断使能
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);     


  /* 串口收DMA配置 */
	//DMA通道配置  
	DMA_DeInit(DMA2_Stream5);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Uart1;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	DMA_InitStructure.DMA_BufferSize = RX_LEN;   
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
		
	//配置DMA2 收通道          
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);    
	//使能通道
	DMA_Cmd(DMA2_Stream5,ENABLE);  
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);		 //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);		 //使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 	//GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 	//GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ; 	//GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); 										//初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = Bound;								//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;							//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); 											//初始化串口1
	
  // 中断配置	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	//采用DMA方式发送  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  
	//采用DMA方式接收
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART1,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  
	
	//启动串口
	USART_Cmd(USART1, DISABLE);					
}

/********************************************************************* 
*                           DMA发送数据
**********************************************************************/  
  
void _Uart1_deal_irq_dma_tx(void)  
{  
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        //清除标志位  
        DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);  
        //关闭DMA
        DMA_Cmd(DMA2_Stream7,DISABLE);  

        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}  
  
  
uint8_t _Uart1_deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)  
    {  
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
        Flag_Tx_Uart1_Busy = 0;  
          
        return 1;  
    }        
    return 0;  
}  


  
void Uart1_tx(uint8_t *data,uint16_t size)  
{  
    while (Flag_Tx_Uart1_Busy);  
    Flag_Tx_Uart1_Busy = 1;  
    memcpy(Tx_Buf_Uart1,data,size);  
    DMA_SetCurrDataCounter(DMA2_Stream7,size);  
    DMA_Cmd(DMA2_Stream7,ENABLE);  
}  



uint8_t _Uart1_deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
      
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
        USART1->SR;  
        USART1->DR; 
        DMA_Cmd(DMA2_Stream5,DISABLE);   
        DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);   
        len = RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);  
        memcpy(buf,Rx_Buf_Uart1,len);  
          
        DMA_SetCurrDataCounter(DMA2_Stream5,RX_LEN);  

        DMA_Cmd(DMA2_Stream5,ENABLE);  
  
        return len;  
    }   
      
    return 0;  
}  


  
void inf_Uart1_deal_irq_dma_tx(void)  
{  
    _Uart1_deal_irq_dma_tx();  
}  

  
uint8_t inf_Uart1_deal_irq_tx_end(void)  
{  
    return _Uart1_deal_irq_tx_end();  
}  
  
  
uint8_t inf_Uart1_deal_irq_rx_end(uint8_t *buf)  
{  
    return _Uart1_deal_irq_rx_end(buf);  
}

  
void Uart1_dma_tx_irq_handler(void)  
{  
    inf_Uart1_deal_irq_dma_tx();  
}  


_Data_Rx PC_rx; 
/********************************************************************* 
*串口1DMA数据流接收函数
**********************************************************************/  
void Uart1_irq_handler(void)                                
{  
		OS_ERR err;	 
    inf_Uart1_deal_irq_tx_end();  
    PC_rx.len = inf_Uart1_deal_irq_rx_end(PC_rx.buf);  
    if (PC_rx.len != 0)  
    { 
			//抛出信号量
			OSSemPost(&DataDeal_proc,OS_OPT_POST_1,&err);
		}	
} 



  
void DMA2_Stream7_IRQHandler(void)  
{  	
		//进入中断调用ucos系统函数
		OSIntEnter();
    Uart1_dma_tx_irq_handler(); 
		OSIntExit();	
}  
      
  
void USART1_IRQHandler(void)   
{  
		//进入中断调用ucos系统函数
		OSIntEnter();
    Uart1_irq_handler(); 
		OSIntExit();
}  



