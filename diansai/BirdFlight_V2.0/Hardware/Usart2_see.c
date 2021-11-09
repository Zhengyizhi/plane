#include "usart2_see.h"
#include "string.h"
#include "stm32f4xx.h"
#include "DronePara.h"
#include <os.h>


#include "crc.h"
#define BUFFER_LEN 600
unsigned char Rx_Buf_Uart2[BUFFER_LEN];
unsigned char Tx_Buf_Uart2[BUFFER_LEN];
unsigned char CliendTxBuffer[200];
unsigned char Rx_Buf_slave[BUFFER_LEN];
int Flag_Tx_USART2_Busy=0;

void Uart2_Init(u32 Bound)
{	
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;  
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //ʹ��UART5ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //ʹ��DMA1ʱ��
	
	//����5��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);  //GPIOC12����ΪUART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 	 //GPIOD2����ΪUART5
	
	
///////////////////////////////////////////////////////////////////////	
	//DMAͨ������
	DMA_DeInit(DMA1_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	//�����ַ 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	//�ڴ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart2;  
	//dma���䷽��
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	//���ô��仺�����ĳ���  
	DMA_InitStructure.DMA_BufferSize = BUFFER_LEN;  
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

	//����DMA1��ͨ��           
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);    
	//�ж�ʹ��
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);     
	/////////////////////////////////////////////////����
	

	/* ������DMA���� */
	//DMAͨ������  
	DMA_DeInit(DMA1_Stream5);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Uart2;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	DMA_InitStructure.DMA_BufferSize = BUFFER_LEN;   
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
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);    
	//ʹ��ͨ��
	DMA_Cmd(DMA1_Stream5,ENABLE); 

///////////////////////////////////////////////////////////////////
	//UART5�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 				//GPIOC12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//����
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				//GPIOD2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//����
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//UART5 ��ʼ������
	USART_InitStructure.USART_BaudRate = Bound;											//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;							//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); 												//��ʼ������5
	
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  


	//DMA�����ж�����
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	//Uart5 NVIC ����	
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				//����5�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;	//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;				//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);													//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	//����DMA��ʽ����  
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	//����DMA��ʽ����
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
	
	USART_Cmd(USART2, ENABLE);  															//ʹ�ܴ���5 
}

//���͸��ӿ�
volatile float distance_turnstile,turn_angle_turnstile;
void Uart2_tx(uint8_t *data,uint16_t size);
mastertoslave_data_t Master_to_slave;
extern level_t flying45_action;
void send_Master_to_slave()
{
	Master_to_slave.txFrameHeader.sof = 0xAA;
	Master_to_slave.txFrameHeader.data_length = sizeof(Master_to_slave.Data);
	memcpy(CliendTxBuffer,&Master_to_slave.txFrameHeader,sizeof(Master_to_slave.txFrameHeader));
	Append_CRC8_Check_Sum(CliendTxBuffer,sizeof (Master_to_slave.txFrameHeader));
	
	Master_to_slave.CmdID = ID_sendRT_Info;
	
//	Master_to_slave.Data.step_num = ������һ���ط��޸�
	
	Master_to_slave.Data.pitch = RT_Info.Pitch - Errangle_Info.fixedErroPitch;
	Master_to_slave.Data.roll = RT_Info.Roll - Errangle_Info.fixedErroRoll;
	Master_to_slave.Data.yaw = RT_Info.Yaw;
	
	Master_to_slave.Data.pix_X = offend_left;//
	Master_to_slave.Data.pix_Y = angel_left;//
	
	Master_to_slave.Data.Target_X_point = pidYaw.target;
	Master_to_slave.Data.Flow_caculate_X = RT_Info.Longtime_imu_Array[yaw_longtime].imu_angle_sum;
	Master_to_slave.Data.Target_Y_point = Target_Info.Y_point;//
	Master_to_slave.Data.Flow_caculate_Y =RT_Info.Flow_caculate_Y;//
	
	Master_to_slave.Data.kp_out = RT_Info.US100_Alt;//
	Master_to_slave.Data.ki_out = Master_to_slave.Data.step_num;
	Master_to_slave.Data.kd_out = pid_calculateflowX.dOut;
	Master_to_slave.Data.feed_out = pidYaw.value;
	
	
	
	
	memcpy(CliendTxBuffer+(sizeof(Master_to_slave.txFrameHeader)),(uint8_t*)&Master_to_slave.CmdID,sizeof(Master_to_slave.CmdID)+sizeof(Master_to_slave.Data));
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(Master_to_slave));
	
	Uart2_tx(CliendTxBuffer,sizeof(Master_to_slave));
}
//���մӿ���Ϣ
#define ImageCenter_x    80.f//128*160
#define ImageCenter_y    60.f
int tack_isee;
Masterslave_info_struct master_slave;
static int PointX_Data[20];
static int PointY_Data[20];

volatile float tracking_turnangle;
volatile float tracking_offsetdistance;
static int tracking_turnangle_Data[20];
static int tracking_offsetdistance_Data[20];
static int turnstiling_distance_Date[20];
static int turnstiling_angle_Date[20];
int16_t temp1turn_angle,temp2offset_distance;

//bool bool_turnstile;
uint8_t tracing_flag;
char   dingyuan;
QueueObj stile_queue;
float temp_distance;

//////////////////////////////////???????????????????????///////////////////////////////////////////
/*�������*/
//////////////////////////////////////////////////
receive_alldata_from_slave  fromslave_data;
static int tracking_step3_Array[20];
volatile float track_turnangle;
int16_t angel_left,offend_left;
void from_slave(receive_alldata_from_slave* slave_data,uint8_t *rxBuf)//���վ�ä����
{
	uint8_t  res = false;
	uint8_t frame_length;
	uint8_t cmd_id;
	int Blob_num = 11;
	if(rxBuf == NULL)
	{
		slave_data->data_valid = false;
		return;
	}
		memcpy(&slave_data->frame_header,rxBuf,sizeof(slave_data->frame_header));
	if(rxBuf[SOF] == 0xAA)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, sizeof(slave_data->frame_header)) == true)
		{
			frame_length = sizeof(slave_data->frame_header) + 
				                length_of_CMD_ID +
				                    slave_data->frame_header.data_length +//���ݶγ���
			                         length_of_Frame_tail;
			if(Verify_CRC16_Check_Sum(rxBuf,frame_length) == true)
			{
				res = true;
				
				cmd_id = rxBuf[CMD_ID];
				switch(cmd_id)
				{
					//���·�������
					case ID_under_bottom_centering://��Բ
					{
						memcpy(&slave_data->centering_underbottom_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(slave_data->centering_underbottom_data));
						Pix_Xinfo = ((Median_filter(slave_data->centering_underbottom_data.x_position,Blob_num,PointX_Data)) - ImageCenter_x ) * US100_Altinfo;
						Pix_Yinfo = ((Median_filter(slave_data->centering_underbottom_data.y_position,Blob_num,PointY_Data)) - ImageCenter_y ) * US100_Altinfo;
						slave_data->step_level = slave_data->centering_underbottom_data.step_receive;
						
						if(slave_data->step_level == 1)
						{
							deal_data_step1();//������־
						}
						else if(slave_data->step_level == 2)
						{
							deal_data_step2();
						}
						else if(slave_data->step_level == 10)
						{
							deal_data_step10();
						}
						break;
					}
					case ID_under_bottom_sow://����
					{
						memcpy(&slave_data->sowing_underbottom_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(slave_data->sowing_underbottom_data));
						slave_data->step_level = slave_data->sowing_underbottom_data.step_receive;
						break;
					}	
					case ID_under_bottom_stayline://����
					{
						memcpy(&slave_data->stay_line_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(slave_data->stay_line_data));
						slave_data->step_level = slave_data->stay_line_data.step_receive;
						track_turnangle = (Median_filter(slave_data->stay_line_data.turn_angle,Blob_num,tracking_step3_Array));
						break;
					}
					//��������
					case ID_bottom_left_line_walkingtrue:
					{
						memcpy(&slave_data->line_walking_leftbottom_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(slave_data->line_walking_leftbottom_data));
						angel_left= slave_data->line_walking_leftbottom_data.turn_angle - 90;
						offend_left = slave_data->line_walking_leftbottom_data.offset_distance - 80;
						slave_data->step_level = slave_data->line_walking_leftbottom_data.step_receive;
						break;
					}
					default:break;
				}
			}
		}
	}		
}









//////////////////////////////////////////????????????????????????/////////////////////////////////////

void receive_slave_to_master(Masterslave_info_struct *master_sen,uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint8_t frame_length;
	uint8_t cmd_id;
	int Blob_num = 11;
	if(rxBuf == NULL)
	{
		master_sen->data_valid = false;
		return;
	}
		memcpy(&master_sen->frame_header,rxBuf,sizeof(master_sen->frame_header));
	if(rxBuf[SOF] == 0xAA)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, sizeof(master_sen->frame_header)) == true)
		{
			frame_length = sizeof(master_sen->frame_header) + 
				                length_of_CMD_ID +
				                    master_sen->frame_header.data_length +//���ݶγ���
			                         length_of_Frame_tail;
			if(Verify_CRC16_Check_Sum(rxBuf,frame_length) == true)
			{
				res = true;
				
				cmd_id = rxBuf[CMD_ID];
				switch(cmd_id)
				{
					case ID_receive_slavedata:
					{
						LED1_ON;
						
						master_sen->pre_vision_data.x_position = master_sen->vision_data.x_position;
						master_sen->pre_vision_data.y_position = master_sen->vision_data.y_position;
						memcpy(&master_sen->vision_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(master_sen->vision_data));
						
						if(master_sen->vision_data.x_position == master_sen->pre_vision_data.x_position
							 && master_sen->pre_vision_data.y_position == master_sen->vision_data.y_position)
						{
							tack_isee++;
						}
						else 
						{
							tack_isee = 0;
							LED2_OFF;
							all_flag.bottomvision_data = true;//���ݿ���
						}
						if(tack_isee>=900)
						{
							LED2_ON;
							tack_isee = 901;
							all_flag.bottomvision_data = false;//���ݲ�����
						}
						Pix_Xinfo = ((Median_filter(master_sen->vision_data.x_position,Blob_num,PointX_Data)) - ImageCenter_x ) * US100_Altinfo;
						Pix_Yinfo = ((Median_filter(master_sen->vision_data.y_position,Blob_num,PointY_Data)) - ImageCenter_y ) * US100_Altinfo;
						
//						if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
//						{
//							static int j;
//							float j_sumx,j_sumy;
//							j++;
//							for(int i=0;i<19;i++)//18��19��
//							{
//								X_ppoint_array[i] = X_ppoint_array[i+1];
//								Y_ppoint_array[i] = Y_ppoint_array[i+1];
//							}
//							X_ppoint_array[19] = abs(Pix_Xinfo);
//							Y_ppoint_array[19] = abs(Pix_Yinfo);
//							if(j>=20)
//							{
//								j=21;
//								for(int i = 0;i<20;i++)
//								{
//									j_sumx += X_ppoint_array[i] * 0.05f;
//									j_sumy += Y_ppoint_array[i] * 0.05f;
//								}
//								if(j_sumx<=7.0f && j_sumy <=7.0f)
//								{
//									dingyuan = 1;
//								}
//							}
//						}
						break;
					}
					case ID_receive_tracking:
					{
//						int16_t temp1,temp2;
						memcpy(&temp1turn_angle,(rxBuf+length_of_header+length_of_CMD_ID),2);//�Ƕ�
						memcpy(&temp2offset_distance,(rxBuf+length_of_header+length_of_CMD_ID+2),2);//����
						memcpy(&tracing_flag,(rxBuf+length_of_header+length_of_CMD_ID+4),1);
						if(abs(temp1turn_angle)<=100 && abs(temp2offset_distance)<=100)
						{
							if(tracing_flag == 1)
							{
							tracking_turnangle =      (Median_filter(temp1turn_angle,Blob_num,tracking_turnangle_Data));
							tracking_offsetdistance = (Median_filter(temp2offset_distance,Blob_num,tracking_offsetdistance_Data)) * US100_Altinfo;	
							}
//							if(temp1 == -1)
//								master_slave.track_vision_data.turn_angle = 0;
//							else
//								master_slave.track_vision_data.turn_angle = temp1;
//							
//							if(temp2 == -1)
//								master_slave.track_vision_data.offset_distance = 0;
//							else 
//								master_slave.track_vision_data.offset_distance = temp2;
//							master_slave.track_vision_data.bool_t = flag;
////							memcpy(&master_sen->track_vision_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(master_sen->track_vision_data));
						}
						break;
					}
					case ID_turnstile:
					{
//						float temp_distance;
							memcpy(&master_sen->turnstile_data,(rxBuf+length_of_header+length_of_CMD_ID),sizeof(master_sen->turnstile_data));
//					   	if(master_sen->turnstile_data.bool_t == 1)
//							{
//								bool_turnstile = true;
							
							//temp_distance = (Median_filter(master_sen->turnstile_data.distance,Blob_num,turnstiling_distance_Date))-35;//���־���
						distance_turnstile = (Median_filter(master_sen->turnstile_data.distance,Blob_num,turnstiling_distance_Date))-18;//���־���
						//							Queue_Average_Flow_Filter(&stile_queue,temp_distance);
//								if(stile_queue.switches == true)
//								{
//									distance_turnstile = stile_queue.back;
//								}
								turn_angle_turnstile = (Median_filter(master_sen->turnstile_data.turn_angle,Blob_num,turnstiling_angle_Date))-160;//160��pix
//							}
//							else
//								bool_turnstile = false;
						break;	
					}
					default:LED1_OFF;break;
				}
			}								
		}
	}
}


void Uart2_tx(uint8_t *data,uint16_t size)  //����
{  
    while (Flag_Tx_USART2_Busy);  
    Flag_Tx_USART2_Busy = 1;   
    memcpy(Tx_Buf_Uart2,data,size);  
    DMA_SetCurrDataCounter(DMA1_Stream6,size);   
    DMA_Cmd(DMA1_Stream6,ENABLE);  
}  

uint8_t inf_Uart2_deal_irq_tx_end(void)  //����
{  
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)  
    {  
        USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
        Flag_Tx_USART2_Busy = 0;  
          
        return 1;  
    }        
    return 0;    
}


void inf_Uart2_deal_irq_rx_end(uint8_t *buf)  //����
{  
   uint16_t len = 0;  
      
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  
    {    
			USART2->SR ;
			USART2->DR ;
			
			DMA_Cmd(DMA1_Stream5,DISABLE);
			
			len = BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			from_slave(&fromslave_data,Rx_Buf_Uart2);
//			receive_slave_to_master(&master_slave,Rx_Buf_Uart2);
			
			
			DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF5);
			DMA1_Stream5->NDTR = BUFFER_LEN;	//200
			DMA_Cmd(DMA1_Stream5,ENABLE);		
    }  		
}

void DMA1_Stream6_IRQHandler(void) //����
{  
		OSIntEnter();
		if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)   
    {  
        DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);  
        DMA_Cmd(DMA1_Stream6,DISABLE);    
        USART_ITConfig(USART2,USART_IT_TC,ENABLE);  
    } 
		OSIntExit();		
} 




volatile uint16_t ReceiveHeight_;

void Uart2_irq_handler(void) //����                               
{     
	inf_Uart2_deal_irq_tx_end();
	inf_Uart2_deal_irq_rx_end(Rx_Buf_slave);  				
} 




void USART2_IRQHandler(void)   
{  
		OSIntEnter();
    Uart2_irq_handler();
		OSIntExit();	
} 

//////////////////////////////////////////////////////
