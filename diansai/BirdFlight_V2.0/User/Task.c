/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Task.c
 * 描述   :任务函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Task.h"
#include "remote.h"
#include "usart2_see.h"
#include "flow_deal.h"
/*全局变量*/
DroneFlightControl FlightControl;      //飞行器状态变量
DronePIDPara Para_Info;								 //飞行器PID参数集合全局变量
DroneRTInfo RT_Info;                   //飞行器实时数据
DroneErrangle Errangle_Info;           //飞行器平地校准数据
DroneTargetInfo Target_Info;           //飞行器目标的全局变量
Controller Control_Info;               //手柄控制全局变量
Throttle Throttle_Info;                //油门全局变量
/**
 * @Description IMU任务 500HZ
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
		/*获取IMU解算后角度*/
		IMU_getInfo();
	}
}

/*
 * @Description 姿态内环控制 400HZ
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
			/*判断是否为四轴起飞模式*/
			if(remote_Info.testtype == test_axisbalance)
			{
				/*预飞程序*/
				if(fly_Pretime<800)
				{
					fly_Pretime=fly_Pretime + 1;
					PID_OUT(300,300,300,300);
				}
				else
				{
					StartFly = 1;//外环高度开始计算 内环位置控制计算开始
					AttitudeInner_control();
					/*保护程序*/
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
			/* 参数归零 */
			fly_Pretime = 0;
			StartFly = 0;
			PID_OUT(0,0,0,0);
			Inner_pidinit();
			Outer_pidinit();
			Neurons_pidinit();
		}
//		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err);//3ms任务
	}
}

/**
 * @Description 位置内环控制 250HZ
 */
void PostionInner_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		if(StartFly == 1)
		{
			/*内环位置控制计算*/
			PostionInner_control();
		}
		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description 融合任务 200HZ
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
		/*黑点位置融合*/
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
					reset_flag = 1;//开启重置
				}
				else
					reset_flag = 0;
			}
		}
				
				
//			Height_Estimation(US100_Altinfo);
//		}
		/*光流位置融合*/
//		if(OpticalflowFlag == 1 && BlackspotsFlag == 0)
//		{
//			OpticalFlow_Estimation(US100_Altinfo,OpticalFlow_x,OpticalFlow_y,Accel_Src,Acc_Flow_x,Acc_Flow_y);
//		}
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description 姿态外环任务 200HZ
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
 * @Description 位置外环任务 125HZ
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
		/*外环高度开始计算*/
		if(StartFly == 1)//四轴模式才起飞
		{
			PostionOuter_control();
		}
		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 视觉数据接收LED任务
 */
OS_SEM Vision_proc;
void Vision_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
//	static _Data_Rx Vision_data;
	//创建二进制文件
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
 * @Description 光流数据接收
 */
OS_SEM Flow_proc;
void Flow_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	static _Data_Rx Flow_data;
	//创建二进制文件
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
 * @Description PC数据接收
 */
bool USE_BLUETOOTH = true; 
OS_SEM DataDeal_proc;//信号量
void DataDeal_task(void *p_arg)
{
	static _Data_Rx PC_data;
	OS_ERR err;
//	p_arg = p_arg;
	//创建信号量
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
			dataStitching(PC_data);//蓝牙数据拼接
		}
		else
		{
			memcpy(PC_data.buf,PC_rx.buf,sizeof(PC_rx.buf));
			PC_data.len = PC_rx.len;
			dataDeal(PC_data);//数据处理
		}
	}
}



/**
 * @Description Usart1toPC任务 40HZ
 */
void Usart1toPC_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		/*上传PID数据*/
		if(FlightControl.ReportSW==Report_SET)
		{
			sendParaInfo();
			FlightControl.ReportSW=Report_RESET;
		}
		/*上传实时数据*/
		sendRTInfo();
		OSTimeDlyHMSM(0,0,0,25,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description Uart5toUltra任务 20HZ
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
		/* 串口接收超声波数据 */
		ReceiveUltraData();
		US100_Altinfo = Median_filter(ReceiveHeight,Height_num,HeightData)/1000;
		/* 15s自动降落程序 */
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
 * @Description 电池电压任务 10HZ
 */
void Battery_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	while(1)
	{
		/*获取滤波后的电压*/
		RT_Info.batteryVoltage = Average_battery(Get_battery());
		/* 起飞电压必须高于11.45V 才可以起飞*/
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
			/*飞行中如果电压低于10.60V则自动降落*/
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
 * @Description Usart mode 切换任务函数 5HZ
 */
u8 Switchflag = 0;
void Usartmode_task(void *p_arg)
{
	OS_ERR err;
//	p_arg = p_arg;
	
	while(1)
	{
		/*板子K3按键*/
		if(Usart_mode)
		{
			USE_BLUETOOTH = !USE_BLUETOOTH;
		}
		/*开启蓝牙串口关闭板载串口*/
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
 * @Description 一键起飞切换任务函数 2.5HZ   默认不适用，需要使用打开线程
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
				throttleBasic = 550;	//起飞基础量550
				Inner_pidinit();			//每次开启，都要清零PID以及控制数据
				Outer_pidinit();
				Neurons_pidinit();
				TakeOff_flag = 0;
				TakeOff_cnt = 0;
			}
		}
		OSTimeDlyHMSM(0,0,0,400,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

