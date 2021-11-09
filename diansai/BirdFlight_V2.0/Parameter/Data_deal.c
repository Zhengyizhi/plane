/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Data_deal.c
 * ����   :����PC����������
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Data_deal.h"
#include "remote.h"
/* ������������ */
volatile float throttleBasic;
int sumCheck(_Data_Rx rx)
{
	int i=0;
	unsigned char sum;
	for(i=0;i<rx.len-1;i++)
	{
		sum^=rx.buf[i];
	}
	if(sum==rx.buf[rx.len-1])
		return 1;
	else 
		return 0;
}



static u8 firstdata[512];
static u8 secdata[512];
_Data_Rx Stitchingdata;
//��������ƴ��
void dataStitching(_Data_Rx rx)
{
	static u8 i,firstlen;
	static u8 BluetoothStitch[20];
	static u8 stitchingflag = 0;
	/*�ڶ�֡����*/
	if(rx.len!=20 && stitchingflag == 1 && rx.buf[0]!=0x55 && rx.buf[1]!=0xAA)
	{
		memcpy(secdata,rx.buf,sizeof(rx.buf));
		for(i = firstlen;i<(rx.len + firstlen);i++)
		{
			BluetoothStitch[i] = secdata[i-firstlen];
		}
		Stitchingdata.len = sizeof(BluetoothStitch);
		stitchingflag = 0;
		memcpy(Stitchingdata.buf,BluetoothStitch,sizeof(BluetoothStitch));
		dataDeal(Stitchingdata);
	}
	/*��һ֡����*/
	else if(rx.len<=20 && rx.buf[0]==0x55)
	{
		memcpy(firstdata,rx.buf,sizeof(rx.buf));
		for(i = 0;i<rx.len;i++)
		{
			BluetoothStitch[i] = firstdata[i];
		}
		firstlen = 	rx.len;	
		stitchingflag = 1;
	}
	/*ʹ�ñʼǱ��Դ���������Ҫ���ݵ�ƴ��*/
	else if(rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA)
	{
		dataDeal(rx);
	}
}


remote_data_t remote_Info;
testyaw_t testyaw;
char jiajian;
void dataDeal(_Data_Rx rx)
{
	u8 HexToFloat[4];
	/*���п���ָ�� */
	if( rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA )
	{
			switch(rx.buf[2])
			{
				case control_command: remote_Info.state = control_command;
				{
					if(RT_Info.lowPowerFlag==0)//�е���ܽ�������
					{
						switch(rx.buf[3])
						{
							case drone_off: remote_Info.flight_switches = drone_off;break;//FlightControl.OnOff = Drone_Off;
							case drone_on: remote_Info.flight_switches = drone_on;//FlightControl.OnOff = Drone_On;
														 Inner_pidinit();		//ÿ�ο�������Ҫ����PID�Լ���������		
														 Outer_pidinit();	
														 Neurons_pidinit();	break;
							case drone_land: FlightControl.landFlag=1;break;//FlightControl.landFlag=1;
							default: break;
						}
					}
				}break;
				case control_angle: remote_Info.state = control_angle;
					if(remote_Info.testtype != test_axisbalance)//��������ɲ��Ե�ʱ��ı�Ŀ��Ƕ�ֵ
					{	
						memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
						remote_Info.set_pitch_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Target_Info.Pitch 
						memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
						remote_Info.set_roll_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Target_Info.Roll
						memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
						remote_Info.set_rateyaw_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Target_Info.RateYaw
						memcpy(HexToFloat,(char*)rx.buf+3+12,sizeof(HexToFloat));
						remote_Info.set_height_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					}break;
				case control_test: remote_Info.state = control_test;
				{
					switch(rx.buf[3])//ѡ�����ʵ��
					{
						case test_pitch:      remote_Info.testtype = test_pitch;throttleBasic = 250;break;//FlightControl.droneMode
						case test_roll:       remote_Info.testtype = test_roll;throttleBasic = 250;break;
						case test_axisbalance:remote_Info.testtype = test_axisbalance;throttleBasic = 500;break;
						case test_ratepitch:  remote_Info.testtype = test_ratepitch;throttleBasic = 250;break;
						case test_rateroll:   remote_Info.testtype = test_rateroll;throttleBasic = 250;break;
						default:              remote_Info.testtype = test_none; break;
					}
				}
				case read_parameter: remote_Info.state = read_parameter;
				                     FlightControl.ReportSW=Report_SET;break;
				case calibrate_magnetometer:remote_Info.state = calibrate_magnetometer;//У׼������
//						if(rx.buf[3]==1)
//						{
//							LSM303_Start_Calib();
//						}
//						else if (rx.buf[3]==2)
//						{
//							LSM303_Save_Calib();
//						}
				break;
				case control_pitch_parameter: remote_Info.state = control_pitch_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.Pitch.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Pitch.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.Pitch.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Pitch.Ki
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.Pitch.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Pitch.Kd
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				case control_roll_parameter: remote_Info.state = control_roll_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.Roll.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Roll.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.Roll.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Roll.Ki
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.Roll.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.Roll.Kd
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				//��ʱ�����⻷������ΪyawPID
				case control_yaw_parameter: remote_Info.state = control_yaw_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
//					remote_Info.set_pid.Yaw.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
          remote_Info.set_pid.x_outflowing.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.x_outflowing.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
//					remote_Info.set_pid.Yaw.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
//					remote_Info.set_pid.Yaw.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					remote_Info.set_pid.x_outflowing.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;					
	     /*����Height��PID*/	
				case control_height_parameter:remote_Info.state = control_height_parameter;//Para_Info.Height.Kp
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.x_flowing.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.x_flowing.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.x_flowing.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();		
					FlightControl.ReportSW=Report_SET;
				break;
				/*ƫ�ýǶ�*/
				case control_adjust:remote_Info.state = control_adjust;
					Errangle_Info.fixedErroPitch = RT_Info.Pitch;
					Errangle_Info.fixedErroRoll = RT_Info.Roll;
					Write_config();	
					FlightControl.ReportSW=Report_SET;break;
				/*����ratePitch��PID*/	
				case control_ratepitch_parameter: remote_Info.state = control_ratepitch_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.ratePitch.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.ratePitch.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.ratePitch.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.ratePitch.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				/*����rateRoll��PID*/
				case control_rateroll_parameter: remote_Info.state = control_rateroll_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.rateRoll.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.rateRoll.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.rateRoll.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.rateRoll.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				/*����rateYaw��PID*/
				case control_rateyaw_parameter: remote_Info.state = control_rateyaw_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.rateYaw.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.rateYaw.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.rateYaw.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.rateYaw.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				/*����accHeight��PID*/	
				case control_accheight_parameter: remote_Info.state = control_accheight_parameter;
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_pid.accHeight.Kp = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Para_Info.accHeight.Kp
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_pid.accHeight.Ki = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_pid.accHeight.Kd = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					Write_config();	
					FlightControl.ReportSW=Report_SET;
				break;
				/*����Ŀ��Rate*/
				case control_rate_target: remote_Info.state = control_rate_target;//	Target_Info.RatePitch
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
					remote_Info.set_ratepitch_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
					remote_Info.set_rateroll_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
				break;
				/*ң����Ϣ*/
				case control_angle_target: remote_Info.state = control_angle_target;//��ɴ�255���17
					memcpy(HexToFloat,(char*)rx.buf+3,sizeof(HexToFloat));
//					remote_Info.x_velocity =  - Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Control_Info.pV
					remote_Info.X_point_goal = - Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+4,sizeof(HexToFloat));
//					remote_Info.y_velocity =  - Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Control_Info.rV
					remote_Info.Y_point_goal = - Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					memcpy(HexToFloat,(char*)rx.buf+3+8,sizeof(HexToFloat));
					remote_Info.set_yaw_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));//Control_Info.yV��<->
					memcpy(HexToFloat,(char*)rx.buf+3+12,sizeof(HexToFloat));
					remote_Info.set_height_goal = Hex_To_Decimal(HexToFloat,sizeof(HexToFloat));
					if(remote_Info.set_height_goal<0.3f && remote_Info.set_height_goal>-0.3f)
					{
						float tmpH=Target_Info.Height+remote_Info.set_height_goal;			
						if(tmpH<1.3f && tmpH>0.01f)//���Ŀ��߶�1��3
							Target_Info.Height=tmpH;
//						else if(tmpH>=1.f);
//							Target_Info.Height=1.f;
//						else
//							Target_Info.Height = 0.01f;
					}
					
					
					{
						if(remote_Info.set_yaw_goal >= 200 && testyaw.turn_angle == level_0)
						{
							testyaw.turn_angle = level_1;
							jiajian = 0;
						}
						else if(remote_Info.set_yaw_goal <= -200 && testyaw.turn_angle == level_0)
						{
							testyaw.turn_angle = level_1;
							jiajian = 1;
						}
						if(abs(remote_Info.set_yaw_goal)<=50 && testyaw.turn_angle == level_1)
						{
							testyaw.turn_angle = level_2;
						}
					}
					///////////////////////////////////////////////�����⻷
//					if(abs(remote_Info.X_point_goal)>1)
//					{
////						if(abs(Target_Info.X_point - flosion_Info.FlowX)<=1)
////						{
////							float tmpX = RT_Info.Flow_caculate_X + remote_Info.X_point_goal;
////							Target_Info.X_point = tmpX;
//	//					}
//					}
//					if(abs(remote_Info.Y_point_goal)>0.005f)
//					{
//						if(abs(Target_Info.Y_point - flosion_Info.FlowY)<=1)
//						{
//							float tmpY = Target_Info.Y_point + remote_Info.Y_point_goal;
//							Target_Info.Y_point = tmpY;
//						}
//					}
						//
				break;
				default:remote_Info.state = out_of_control;break;					
			}
		}
}	


