/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Outer_control.c
 * ����   :�⻷���ƺ���
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Outer_control.h"
#include "remote.h"
#include "usart2_see.h"
PIDOut pidPitch,pidRoll,pidYaw,pidHeight,pidPointX,pidPointY,pidFlowX,pidFlowY,pid_calculateflowX,pid_calculateflowY;

static volatile float rollErroHistory=0;
static volatile float pitchErroHistory=0;
static volatile float yawErroHistory=0;
static volatile float tgtHeight=0;
extern char reset_flag,Combine_start;
level_t stile_flag;
level_t flying45_action;
void Outer_pidinit()
{
	Target_Info.Height = 1.2f;//��ʼĿ��߶�
	
	//<PostionOuter_task>����һֱ���䣬����10cm��ʾ�������
	FlightControl.landFlag = 0;
	tgtHeight = 0;//ʵ��Ŀ��߶����㻺������
	
	Target_Info.X_point = 0;
	Target_Info.Y_point = 0;//���ڹ�������������Ŀ����Ҳ����
	RT_Info.Flow_caculate_X = 0;
	RT_Info.Flow_caculate_Y = 0;
	
//	Pix_Yinfo = 0;//�Ӿ��������
//	Pix_Xinfo = 0;	
	
	stile_flag = level_0;//����ȼ�����
	flying45_action = level_0;
	//<AttitudeOuter_task>Ŀ���ȥƫ�ƽǶ����㣬��ֹd�������������
	pitchErroHistory = 0;
	rollErroHistory = 0;
	yawErroHistory = 0;
	
	//������ɱ�־����<AttitudeInner_task>
	StartFly = 0;
	
	//<Combine_task>�����ں�����
	reset_flag = 1;//�������׿�����������
	Combine_start = 0;//��ɽ׶β����������������
	
	all_time_init(&waiting_time);
	
}
uint32_t world_time_ms()
{
	static char isee_flag;
	static double time_ms;
	uint32_t time_isee;
	time_isee = CPU_TS32_to_uSec(CPU_TS_TmrRd())/1000;
	if(time_isee>=25000)
		isee_flag = 1;
	else if(time_isee<50 && isee_flag == 1)
	{
		isee_flag = 0;
		time_ms = time_ms + 25565.280;
	}
	time_isee = (uint32_t)(time_isee + time_ms);
	return (time_isee);
}
uint32_t world_time_us()
{
	static char isee_flag;
	static double time_us;
	uint32_t time_isee;
	time_isee = CPU_TS32_to_uSec(CPU_TS_TmrRd());
	if(time_isee>=25000000)
		isee_flag = 1;
	else if(time_isee<50 && isee_flag == 1)
	{
		isee_flag = 0;
		time_us = time_us + 25565280;
	}
	time_isee = (uint32_t)(time_isee + time_us);
	return (time_isee);
}


waiting_time_t waiting_time;
void delay_ms_timer( uint32_t delaytime , count_dowm_t* time)
{
		if(time->start_flag == start_true)//���flagֻ�ܿ�һ��
		{
			if(time->end_timing == count_dowm_0)
			{
				time->end_timing = count_dowm_1;
				time->F_time = world_time_ms();
			}
			else if(time->end_timing == count_dowm_1)
			{
				time->E_time = world_time_ms();
				if(time->E_time - time->F_time >= delaytime)
				{
					time->end_timing = count_dowm_2;
				}
			}
		}
		else
			time->end_timing = count_dowm_0;
}
void delay_ms_init(count_dowm_t* time)//ֻ��һ��
{
	time->start_flag = start_true;
	time->end_timing = count_dowm_0;
}
void all_time_init(waiting_time_t* time)
{
	time->centering.start_flag = start_false;
	time->centering_2.start_flag = start_false;
	time->find_A.start_flag = start_false;
	time->turn_90_1.start_flag = start_false;
	time->turn_90_2.start_flag = start_false;
	time->stay_line.start_flag = start_false;
	time->turn_90_3.start_flag = start_false;
	time->go_home.start_flag = start_false;
}








////////////////////////////////////////////
float OuterPID_dt;

void yaw_control(float turnangle)
{
	static float get_nowangle;
	float yawErro;
	if(abs(turnangle)>=2)
	{
		testyaw.vision_tracing = level_1;
	}
	else 
	{
		testyaw.vision_tracing = level_0;
	}
	if(testyaw.vision_tracing == level_1)
	{
		get_nowangle = RT_Info.Longtime_imu_Array[yaw_longtime].imu_angle_sum;
		pidYaw.target = get_nowangle + turnangle;
		yawErro = turnangle;
		pidYaw.pOut=220.f * yawErro;
		yawErroHistory=yawErro;
		/*�޷�Yaw�⻷PID*/
		pidYaw.value=pidYaw.pOut+pidYaw.dOut;
		pidYaw.value=Limits_data(pidYaw.value,20000,-20000);
	}
	else if(testyaw.vision_tracing == level_0)
	{
		pidYaw.value = 0;
	}
}


void ttestyaw(float turnangle)
{
	static float get_nowangle;
	float yawErro;
	if(testyaw.turn_angle == level_2)
	{
		get_nowangle = RT_Info.Longtime_imu_Array[yaw_longtime].imu_angle_sum;
		testyaw.turn_angle = level_3;
	}
	if(testyaw.turn_angle == level_3)
	{
		pidYaw.target = get_nowangle + turnangle;
		yawErro = (get_nowangle + turnangle) - 	RT_Info.Longtime_imu_Array[yaw_longtime].imu_angle_sum;
		pidYaw.pOut=220.f * yawErro;
		pidYaw.dOut=0.f * (yawErro- yawErroHistory)/ OuterPID_dt;
		yawErroHistory=yawErro;
		/*�޷�Yaw�⻷PID*/
		pidYaw.value=pidYaw.pOut+pidYaw.dOut;
		pidYaw.value=Limits_data(pidYaw.value,20000,-20000);
	}
	if((testyaw.turn_angle == level_3 && abs(yawErro)<=3) || testyaw.turn_angle == level_0)
	{
		testyaw.turn_angle = level_0;
		yawErro = 0;
		get_nowangle = 0;
		pidYaw.value = 0;
	}
}
extern char jiajian;
void AttitudeOuter_control()
{
	//��ȷʱ��dt����
	static unsigned int OutertPre=0;
	float pitchErro;
	float rollErro;
	unsigned int Outert;
	Outert=micros();
  OuterPID_dt = (OutertPre>0)?((Outert-OutertPre)):1;
  OutertPre=Outert;
	
	/************************ �⻷Pitch�ǶȻ�����************************/	
	pitchErro=(remote_Info.set_pitch_goal-(RT_Info.Pitch-Errangle_Info.fixedErroPitch));
	pidPitch.pOut=-650 *  pitchErro;
	pidPitch.dOut=-5 * (pitchErro- pitchErroHistory)/ OuterPID_dt;
	pitchErroHistory=pitchErro;
	/*�޷�Pitch�⻷PID*/
	pidPitch.value=pidPitch.pOut+pidPitch.dOut;
	pidPitch.value=Limits_data(pidPitch.value,20000,-20000);
	
	/************************ �⻷Roll�ǶȻ�����************************/	
	rollErro=(remote_Info.set_roll_goal-(RT_Info.Roll-Errangle_Info.fixedErroRoll));
	pidRoll.pOut=650 * rollErro;
	pidRoll.dOut=5 * (rollErro- rollErroHistory)/ OuterPID_dt;
	rollErroHistory=rollErro;
	/*�޷�Roll�⻷PID*/
	pidRoll.value=pidRoll.pOut+pidRoll.dOut;
	pidRoll.value=Limits_data(pidRoll.value,20000,-20000);
	
	/************************ �⻷Yaw�ǶȻ�����************************/	
//	if(	jiajian == 0)
//		ttestyaw(90);
//	else if(jiajian == 1)
//		ttestyaw(-90);
//	yawErro=(remote_Info.set_yaw_goal*0.003f-RT_Info.Yaw);
//	pidYaw.pOut=remote_Info.set_pid.x_outflowing.Kp * yawErro;
//	pidYaw.dOut=remote_Info.set_pid.x_outflowing.Kd * (yawErro- yawErroHistory)/ OuterPID_dt;
//	yawErroHistory=yawErro;
//	/*�޷�Yaw�⻷PID*/
//	pidYaw.value=pidYaw.pOut+pidYaw.dOut;
//	pidYaw.value=Limits_data(pidYaw.value,20000,-20000);
}
float heightErro;
float Y_pointdt_goal,X_pointdt_goal;
float feedforwardnumber;

void vision_see(void);
void tracking_see(void);
void turnstile_see(void);
void starting45_fly(void);
void PostionOuter_control()
{
	/************************ �߶��⻷����************************/
	static float Takeoff_weight = 0.005;
	static float Landing_weight = 0.005;
	float Feedforward_height;

	float FlowxErro;
	float FlowyErro;
	
	unsigned int PosOuter;
	static float PosOuterPID_dt;
	static unsigned int PosOuterPre=0;
	PosOuter = micros();
	PosOuterPID_dt =  (PosOuterPre>0)?((PosOuter-PosOuterPre)/1000000.0f):1;
	PosOuterPre = PosOuter;
	/*********************�������********************/	
	if(FlightControl.landFlag==1)
	{
		tgtHeight=tgtHeight - Landing_weight;		
		if(RT_Info.US100_Alt<0.10f)
		{
			remote_Info.flight_switches  = drone_land;
		}
	}
	else
	{
		if(tgtHeight < Target_Info.Height)
		{
			tgtHeight = tgtHeight + Takeoff_weight;
		}
		else
		{
			tgtHeight = Target_Info.Height;
		}
	}	
	Feedforward_height = 40;
	//              ��         ��
	heightErro = tgtHeight - RT_Info.US100_Alt;//�ںϺõĸ߶�
	//ǰ��У׼
	pidHeight.feedforwardOut = Feedforward_height * heightErro;
	
	pidHeight.value = Neurons_PID_Hight(heightErro);

	pidHeight.value = Limits_data(pidHeight.value,3,-3);
	
	if(RT_Info.US100_Alt>0.1f)
	{			
		float X_pointerr,Xdelta_pointerr;
		static float X_pointerr_history,lastflowX_pointDelta;
////////////////����������//////////////////////	
		starting_task();
		deal_all_task();
//		starting_task();
//		deal_all_task();
//		ttestyaw(90);
//		vision_see();
		
//		tracking_see();
		
////		if(RT_Info.US100_Alt >= 0.7f)
////				turnstile_see();
//		static char fflagg = 0;
//		if( Target_Info.Height<=0.85f && fflagg == 0 )//1.2
//		{
//				Target_Info.Height+=0.0008;
//			pidYaw.value = 0;
//		}
//		if(RT_Info.US100_Alt >= 0.8f)
//		{
//			fflagg = 1;
//		}
//		if(fflagg == 1)
//				turnstile_see();
//		
		
//		starting45_fly();
    X_pointerr = (float)(Target_Info.X_point  - RT_Info.Flow_caculate_X);

		Xdelta_pointerr = (X_pointerr - X_pointerr_history)/PosOuterPID_dt;//
		
		Xdelta_pointerr = lastflowX_pointDelta + 
							(PosOuterPID_dt / (lowpass_filter + PosOuterPID_dt)) * (Xdelta_pointerr - lastflowX_pointDelta);
		lastflowX_pointDelta = Xdelta_pointerr;
		
		X_pointerr_history = X_pointerr;
		
		pid_calculateflowX.pOut = X_pointerr * 0.00037;//0.00426height
		pid_calculateflowX.iOut += X_pointerr * 0.00005/100.f;
		pid_calculateflowX.dOut = Xdelta_pointerr * 0.00001;//0.0005
		
		pid_calculateflowX.iOut = Limits_data(pid_calculateflowX.iOut,0.5,-0.5);
		pid_calculateflowX.value = pid_calculateflowX.pOut
		                              + pid_calculateflowX.iOut
																	   + pid_calculateflowX.dOut;
		
		pid_calculateflowX.value = Limits_data(pid_calculateflowX.value,3,-3);//?3,1
		pid_calculateflowX.feedforwardOut = pid_calculateflowX.value * 35.f;//35yaw
/////////////////////////////////////		
//	if(abs(X_pointerr)<=50)
//			feedforwardnumber = 250;
//	else if(abs(X_pointerr)>50 && abs(X_pointerr)<150)
//		feedforwardnumber = 250-(abs(X_pointerr)-50)*2.3;
//	else if(abs(X_pointerr)>=150)
//		feedforwardnumber = 20;
//
//	if(abs(X_pointerr)<=50)
//			feedforwardnumber = 20;
//	else if(abs(X_pointerr)>50 && abs(X_pointerr)<150)
//		feedforwardnumber = 20+(abs(X_pointerr)-50)*0.8;
//	else if(abs(X_pointerr)>=150)
//		feedforwardnumber = 100;
//  pid_calculateflowX.feedforwardOut = pid_calculateflowX.value * feedforwardnumber;
/////////////////////////////////////
		//*************************************************************************************//
	float Y_pointerr,Ydelta_pointerr;
	static float Y_pointerr_history,lastflowY_pointDelta;
	Y_pointerr = (float)(Target_Info.Y_point  - RT_Info.Flow_caculate_Y);
	
	Ydelta_pointerr = (Y_pointerr - Y_pointerr_history)/PosOuterPID_dt;
//
	Ydelta_pointerr = lastflowY_pointDelta + 
						(PosOuterPID_dt / (lowpass_filter + PosOuterPID_dt)) * (Ydelta_pointerr - lastflowY_pointDelta);
	lastflowY_pointDelta = Ydelta_pointerr;
//
	Y_pointerr_history = Y_pointerr;
		
	pid_calculateflowY.pOut = Y_pointerr * 0.00037f;//0.00426
	pid_calculateflowY.iOut += Y_pointerr * 0.00003/100.f;
	pid_calculateflowY.dOut = Ydelta_pointerr * 0.00001;//0.0005
	
	pid_calculateflowY.iOut = Limits_data(pid_calculateflowY.iOut,0.5,-0.5);//0.5
	pid_calculateflowY.value = pid_calculateflowY.pOut
																+ pid_calculateflowY.iOut
																	 + pid_calculateflowY.dOut;
	
	pid_calculateflowY.value = Limits_data(pid_calculateflowY.value,3,-3);//?3,1
	pid_calculateflowY.feedforwardOut = pid_calculateflowY.value * 35.f;//1.5yaw	
//////////////////////////////////////////////	
		//�⻷λ�ÿ��ƻ�
		FlowxErro = pid_calculateflowX.value - RT_Info.FlowX;//�����׿������õ���Ԥ��˲ʱ����ֵ
		pidFlowX.value = 28.f * FlowxErro
		                   + pid_calculateflowX.feedforwardOut;//�⻷һ��ֻ�ó���Kp,60
		
		//X����ǰ��У׼
		pidFlowX.feedforwardOut =  0.23f * pidFlowX.value;//0.23
		//////////////////////////////////////
		FlowyErro = pid_calculateflowY.value - RT_Info.FlowY;
		pidFlowY.value =  28.f * FlowyErro//40
		                   + pid_calculateflowY.feedforwardOut;
		//Y����ǰ��У׼
		pidFlowY.feedforwardOut =  0.23f * pidFlowY.value;//0.23
	}
}

task_flag all_flag;
void vision_see(void)//Բ������ͷ��Y+��Բ������ͷ��X+
{
//	starting_task();
//	static char reset = 0;
//	if(all_flag.bottomvision_data == true)//�Ӿ����ݿ�����
//	{
//	if(RT_Info.US100_Alt>=0.5)
//	{
//			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
//			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
//	
//	}
//		reset = 1;
//	}
//	else if(all_flag.bottomvision_data == false)
//	{
//		if(reset == 1)
//		{
//			reset = 0;
//			Target_Info.X_point = RT_Info.Flow_caculate_X;
//			Target_Info.Y_point = RT_Info.Flow_caculate_Y;
//		}
//	}
}





extern uint8_t tracing_flag;
extern int16_t temp1turn_angle,temp2offset_distance;
void tracking_see()//Ѳ��
{
	static char reset = 0;
	if(tracing_flag == 1 && RT_Info.US100_Alt>=0.2f)//������
	{
		reset = 1;
		Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
		Target_Info.Y_point = RT_Info.Flow_caculate_Y -(temp2offset_distance * 5.0f);
		yaw_control(temp1turn_angle);
	}
	else if(tracing_flag == 0)//�������ϵ��յ���
	{
		if(reset == 1)
		{
			reset = 0;
			Target_Info.X_point = RT_Info.Flow_caculate_X;
			Target_Info.Y_point = RT_Info.Flow_caculate_Y;

			pidYaw.value = 0;
		}
	}
}
/////////////////////////////////////////////////////////////////////

extern volatile float distance_turnstile,turn_angle_turnstile;
//extern bool bool_turnstile;
void stileyaw_control(float stileangle)
{
	static float get_nowangle;
	float yawErro;
	if(abs(stileangle)>=2)
	{
		testyaw.stile_angle = level_1;
	}
	else 
	{
		testyaw.stile_angle = level_0;
	}
	if(testyaw.stile_angle == level_1)
	{
		get_nowangle = RT_Info.Longtime_imu_Array[yaw_longtime].imu_angle_sum;
		pidYaw.target = get_nowangle + stileangle;
		yawErro = stileangle;
		pidYaw.pOut=220.f * yawErro;
		yawErroHistory=yawErro;
		/*�޷�Yaw�⻷PID*/
		pidYaw.value=pidYaw.pOut+pidYaw.dOut;
		pidYaw.value=Limits_data(pidYaw.value,20000,-20000);
	}
	else if(testyaw.stile_angle == level_0)
	{
		pidYaw.value = 0;
	}
}

void turnstile_see()
{
	if(master_slave.turnstile_data.bool_t == level_1)//��������
	{
		stileyaw_control(turn_angle_turnstile*0.2f);//(turn_angle_turnstile/(distance_turnstile+35));////ת��
		Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
		Target_Info.Y_point = RT_Info.Flow_caculate_Y -(distance_turnstile * 20.0f);//��˵ľ���ǰ��
	}
	else if(master_slave.turnstile_data.bool_t == level_0)//��û������
	{
		Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
	}
	else if(master_slave.turnstile_data.bool_t == level_2)
	{
//		if(reset == 0)
//		{
//			reset = 1;
////			Target_Info.X_point = RT_Info.Flow_caculate_X;
////			Target_Info.Y_point = RT_Info.Flow_caculate_Y;

			pidYaw.value = 0;
//		}
	}
}

//level_t flying45_action;
void starting45_fly(void)
{
	static int reset,itime;
	pidYaw.value = 0;
	if(RT_Info.US100_Alt<=1.0f && flying45_action == level_0)
	{
		reset = 0;
		itime = 0;
//		if(abs(Target_Info.Height - RT_Info.US100_Alt)<=0.1 && Target_Info.Height<=1.0f)//1.2
//		{
////			temp = (1.2 - Target_Info.Height) * 0.5; 
//////			Target_Info.Height+=0.5;
////			Target_Info.Height += temp;
//			if(Target_Info.Height<=0.8f)
//				Target_Info.Height+=0.5;
//			else 
//				Target_Info.Height+=0.2;
//		}
		if( Target_Info.Height<=1.1f )//1.2
		{
				Target_Info.Height+=0.0005f;
				Target_Info.Y_point += 0.17f;
		}
//		Target_Info.Y_point = 370.0*RT_Info.US100_Alt;
	}
	else if(RT_Info.US100_Alt>=1.0f && flying45_action == level_0)
	{
		flying45_action = level_1;
		if(reset == 0)
		{
//			Target_Info.X_point = RT_Info.Flow_caculate_X;
//			Target_Info.Y_point = RT_Info.Flow_caculate_Y;
			Target_Info.Height = 1.0;
			itime = 0;
			reset = 1;
		}
	}
	else if(flying45_action == level_1)//���־�ֹ
	{
		itime++;
		if(itime>=1000)
			flying45_action = level_2;
	}
	else if(flying45_action == level_2)
	{
		if(Target_Info.Height >= 0.2f)
		{
//			if(abs(Target_Info.Height - RT_Info.US100_Alt)<=0.2)
				Target_Info.Height-=0.001f;
		}
		else if(Target_Info.Height<=0.2f && RT_Info.US100_Alt<=0.3f)
		{
//			if(abs(Target_Info.Height - RT_Info.US100_Alt)<=0.1)
//			{
				remote_Info.flight_switches = drone_off;
				flying45_action = level_3;
//			}
			
		}
	}

}

////////////////////////////////////////////////////////////////////

//void flowing_task(void)
//{
//	if(waiting_time.centering_2.end_timing != count_dowm_2)
//	{
//		static int left_shift_cishu = 0;
//		left_shift_cishu++;
//		if(left_shift_cishu<=2117)
//			Target_Info.Y_point += 0.2f;//0.17���1m
//		else if(left_shift_cishu>=2117 && waiting_time.centering_2.start_flag == start_false)//����Ŀ��λ��
//		{
//			left_shift_cishu = 3000;
//			delay_ms_init(&waiting_time.centering_2);
//		}
//		delay_ms_timer(10000,&waiting_time.centering_2);//��10s
//	}
//	else
//	{}
//}

//void starting_task()
//{
//	if(waiting_time.centering.end_timing != count_dowm_2)
//	{
//		Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
//		Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
//		if(dingyuan == 1)//�׶�1
//		{
//			if( waiting_time.centering.start_flag == start_false)
//			{
//				delay_ms_init(&waiting_time.centering);
//			}
//			delay_ms_timer(2000,&waiting_time.centering);
//		}
//	}
//	else//�Ѿ����2s��ͣ
//	{
//		flowing_task();
//	}
//}

