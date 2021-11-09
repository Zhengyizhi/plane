/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Task.c
 * ����   :������
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Task.h"
#include "remote.h"
#include "usart2_see.h"
#include "flow_deal.h"
/*ȫ�ֱ���*/
DroneFlightControl FlightControl;      //������״̬����
DronePIDPara Para_Info;								 //������PID��������ȫ�ֱ���
DroneRTInfo RT_Info;                   //������ʵʱ����
DroneErrangle Errangle_Info;           //������ƽ��У׼����
DroneTargetInfo Target_Info;           //������Ŀ���ȫ�ֱ���
Controller Control_Info;               //�ֱ�����ȫ�ֱ���
Throttle Throttle_Info;                //����ȫ�ֱ���
/**
 * @Description IMU���� 500HZ
 */
OS_SEM IMU_proc;
void IMU_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	OSSemCreate ((OS_SEM*)&IMU_proc,
								(CPU_CHAR*)"IMU_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		OSSemPend (&IMU_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		/*��ȡIMU�����Ƕ�*/
		IMU_getInfo();
	}
}

/*
 * @Description ��̬�ڻ����� 400HZ
 */
OS_SEM AttitudeInner_proc;
u8 StartFly = 0;
void AttitudeInner_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	static unsigned int fly_Pretime = 0;	
	OSSemCreate ((OS_SEM*)&AttitudeInner_proc,
								(CPU_CHAR*)"AttitudeInner_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);

	while(1)
	{
		OSSemPend (&AttitudeInner_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(remote_Info.flight_switches == drone_on)
		{
			/*�ж��Ƿ�Ϊ�������ģʽ*/
			if(remote_Info.testtype == test_axisbalance)
			{
				/*Ԥ�ɳ���*/
				if(fly_Pretime<800)
				{
					fly_Pretime=fly_Pretime + 1;
					PID_OUT(300,300,300,300);
				}
				else
				{
					StartFly = 1;//�⻷�߶ȿ�ʼ���� �ڻ�λ�ÿ��Ƽ��㿪ʼ
					AttitudeInner_control();
					/*��������*/
					if(abs(RT_Info.Pitch-Errangle_Info.fixedErroPitch) >= 25 ||  
							abs(RT_Info.Roll-Errangle_Info.fixedErroRoll)>= 25)
					{
								PID_OUT(0,0,0,0);
								while(1);
					}
				}
			}
			else
			{
				AttitudeInner_control();
				StartFly = 0;
			}
		}
		else
		{
			/* �������� */
			fly_Pretime = 0;
			StartFly = 0;
			PID_OUT(0,0,0,0);
			Inner_pidinit();
			Outer_pidinit();
			Neurons_pidinit();
		}
//		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err);//3ms����
	}
}

/**
 * @Description λ���ڻ����� 250HZ
 */
void PostionInner_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		if(StartFly == 1)
		{
			/*�ڻ�λ�ÿ��Ƽ���*/
			PostionInner_control();
		}
		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description �ں����� 200HZ
 */
int flagg;
char Combine_start;
char reset_flag;
float third_kf_calculate;
volatile float US100_Altinfo;
void Combine_task(void *p_arg)
{
	OS_ERR err;
	flagg = 2;
	Combine_start = 0;
	reset_flag = 0;
//	p_arg = p_arg;
	while(1)
	{
		/*�ڵ�λ���ں�*/
//		if(BlackspotsFlag == 1 && OpticalflowFlag == 0)
//		{
		if(flagg == 0)
			Position_Estimation(US100_Altinfo,Pix_Xinfo,Pix_Yinfo,&RT_Info);
		else if(flagg == 1)
		{
			OpticalFlow_Estimation(US100_Altinfo,flosion_Info.FixFlowX,flosion_Info.FixFlowY,
		                        &RT_Info,-RT_Info.accratex,-RT_Info.accratey);
//			third_Kfilter(US100_Altinfo,flosion_Info.FixFlowX,flosion_Info.FixFlowY,
//		                        &RT_Info,-RT_Info.accratex,-RT_Info.accratey);
		}
		else if(flagg == 2)
		{
		third_Kfilter(US100_Altinfo,flosion_Info.FixFlowX,flosion_Info.FixFlowY,
		                        &RT_Info,-RT_Info.accratex,-RT_Info.accratey,reset_flag);
//		OpticalFlow_Estimation(US100_Altinfo,flosion_Info.FixFlowX,flosion_Info.FixFlowY,
//										&RT_Info,-RT_Info.accratex,-RT_Info.accratey);
			
			
			
			if(RT_Info.US100_Alt >= 0.1f)
				Combine_start = 1;
			if(Combine_start == 1)
			{
				if(abs(RT_Info.FlowX)>=0.01f)
					RT_Info.Flow_caculate_X += RT_Info.FlowX;
				if(abs(RT_Info.FlowY)>=0.01f)
					RT_Info.Flow_caculate_Y += RT_Info.FlowY;
				
				if(RT_Info.Flow_caculate_Y>=2000000000 || RT_Info.Flow_caculate_X>=2000000000)
				{
					RT_Info.Flow_caculate_Y =  RT_Info.Flow_caculate_X = 0;
					reset_flag = 1;//��������
				}
				else
					reset_flag = 0;
			}
		}
				
				
//			Height_Estimation(US100_Altinfo);
//		}
		/*����λ���ں�*/
//		if(OpticalflowFlag == 1 && BlackspotsFlag == 0)
//		{
//			OpticalFlow_Estimation(US100_Altinfo,OpticalFlow_x,OpticalFlow_y,Accel_Src,Acc_Flow_x,Acc_Flow_y);
//		}
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description ��̬�⻷���� 200HZ
 */
void AttitudeOuter_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		AttitudeOuter_control();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description λ���⻷���� 125HZ
 */
void PostionOuter_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
//		starting_task();
//		deal_all_task();
//		ttestyaw(90);
		/*�⻷�߶ȿ�ʼ����*/
		if(StartFly == 1)//����ģʽ�����
		{
			PostionOuter_control();
		}
		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description �Ӿ����ݽ���LED����
 */
OS_SEM Vision_proc;
void Vision_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
//	static _Data_Rx Vision_data;
	//�����������ļ�
	OSSemCreate ((OS_SEM*)&Vision_proc,
								(CPU_CHAR*)"Vision_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
//		OSSemPend (&Vision_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
//		memcpy(Vision_data.buf,Vision.buf,sizeof(Vision.buf));
//		Vision_data.len = Vision.len;
//		Vision_datadeal(Vision_data);
		

		send_Master_to_slave();
		OSTimeDlyHMSM(0,0,0,8,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description �������ݽ���
 */
OS_SEM Flow_proc;
void Flow_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	static _Data_Rx Flow_data;
	//�����������ļ�
	OSSemCreate ((OS_SEM*)&Flow_proc,
								(CPU_CHAR*)"Flow_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		OSSemPend (&Flow_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		memcpy(Flow_data.buf,Flosion.buf,sizeof(Flosion.buf));
		Flow_data.len = Flosion.len;
		Flow_datadeal(Flow_data);
//		try_Arr[0] = pidFlowx.pOut;
//		try_Arr[1] = pidFlowx.iOut;
//		try_Arr[2] = pidFlowx.dOut;
//		try_Arr[3] = pidFlowx.value;
//		try_Arr[4] = remote_Info.x_velocity;
//		try_Arr[5] = RT_Info.FlowX_V;
//		Vision_datadeal(Vision_data);
//		OSTimeDlyHMSM(0,0,0,8,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description PC���ݽ���
 */
bool USE_BLUETOOTH = true; 
OS_SEM DataDeal_proc;//�ź���
void DataDeal_task(void *p_arg)
{
	static _Data_Rx PC_data;
	OS_ERR err;
//	p_arg = p_arg;
	//�����ź���
	OSSemCreate ((OS_SEM*)&DataDeal_proc,
								(CPU_CHAR*)"DataDeal_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		OSSemPend (&DataDeal_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(USE_BLUETOOTH)
		{
			memcpy(PC_data.buf,Bluetooth_rx.buf,sizeof(Bluetooth_rx.buf));
			PC_data.len = Bluetooth_rx.len;
			dataStitching(PC_data);//��������ƴ��
		}
		else
		{
			memcpy(PC_data.buf,PC_rx.buf,sizeof(PC_rx.buf));
			PC_data.len = PC_rx.len;
			dataDeal(PC_data);//���ݴ���
		}
	}
}



/**
 * @Description Usart1toPC���� 40HZ
 */
void Usart1toPC_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		/*�ϴ�PID����*/
		if(FlightControl.ReportSW==Report_SET)
		{
			sendParaInfo();
			FlightControl.ReportSW=Report_RESET;
		}
		/*�ϴ�ʵʱ����*/
		sendRTInfo();
		OSTimeDlyHMSM(0,0,0,25,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description Uart5toUltra���� 20HZ
 */
void Uart5toUltra_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	static u8 Height_num = 7;
	static u8 stopflag = 0;
	static int HeightData[10];
	while(1)
	{
		/* ���ڽ��ճ��������� */
		ReceiveUltraData();
		US100_Altinfo = Median_filter(ReceiveHeight,Height_num,HeightData)/1000;
		/* 15s�Զ�������� */
		if(US100_Altinfo > 0.35f)//3.5cm
		{
			if(stopflag == 0)
			{
				stopflag = 1;
			}
		}
		else if(US100_Altinfo <= 0.35f)
		{
			stopflag = 0;
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description ��ص�ѹ���� 10HZ
 */
void Battery_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		/*��ȡ�˲���ĵ�ѹ*/
		RT_Info.batteryVoltage = Average_battery(Get_battery());
		/* ��ɵ�ѹ�������11.45V �ſ������*/
		if(RT_Info.batteryVoltage<10.8f && ( remote_Info.flight_switches != Drone_On))
		{
			RT_Info.lowPowerFlag = 1;
//			ALLLED_ON;
			OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//			ALLLED_OFF;
			OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		else 
		{
			/*�����������ѹ����10.60V���Զ�����*/
			if(RT_Info.batteryVoltage < 8.2f)//10.6
			{
				FlightControl.landFlag=1;
			}
			RT_Info.lowPowerFlag=0;
//			ALLLED_OFF;
		}
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description Usart mode �л������� 5HZ
 */
u8 Switchflag = 0;
void Usartmode_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	
	while(1)
	{
		/*����K3����*/
		if(Usart_mode)
		{
			USE_BLUETOOTH = !USE_BLUETOOTH;
		}
		/*�����������ڹرհ��ش���*/
		if((USE_BLUETOOTH) && (Switchflag == 0))
		{
			BluetoothLED_ON;
			USART_Cmd(USART3,ENABLE);
			USART_Cmd(USART1,DISABLE);
			Switchflag = 1;
		}
		else if((!USE_BLUETOOTH) && (Switchflag == 1))
		{
			BluetoothLED_OFF;
			USART_Cmd(USART1,ENABLE);
			USART_Cmd(USART3,DISABLE);
			Switchflag = 0;
		}
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description һ������л������� 2.5HZ   Ĭ�ϲ����ã���Ҫʹ�ô��߳�
*/
void TakeOff_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	static int TakeOff_flag = 0;
	static int TakeOff_cnt = 0;
	while(1)
	{
		if(Take_Off)
		{
			TakeOff_flag = 1;
		}
		if(TakeOff_flag == 1 && RT_Info.lowPowerFlag == 0 && US100_Altinfo<0.1f)
		{
			TakeOff_cnt++;
			if(TakeOff_cnt % 2 == 0)
			{
//				ALLLED_ON;
			}
			else
			{
//				ALLLED_OFF;
			}
			if(TakeOff_cnt == 10)
			{
				FlightControl.OnOff = Drone_On;
				FlightControl.droneMode = Drone_Mode_4Axis;
				throttleBasic = 550;	//��ɻ�����550
				Inner_pidinit();			//ÿ�ο�������Ҫ����PID�Լ���������
				Outer_pidinit();
				Neurons_pidinit();
				TakeOff_flag = 0;
				TakeOff_cnt = 0;
			}
		}
		OSTimeDlyHMSM(0,0,0,400,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

