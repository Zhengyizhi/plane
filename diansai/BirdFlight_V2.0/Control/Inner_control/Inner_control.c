/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Inner_control.c
 * 描述   :内环控制函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Inner_control.h"
#include "remote.h"
#include "fuzzypid.h"
//局部变量
PIDOut  pidRatePitch,pidRateRoll,pidRateYaw,
					pidAccHeight,pidAccPitch,pidAccRoll,
						pidXSpeed,pidYSpeed,pidFlowx,pidFlowy;

static volatile float intergrateLimit = 80;
static volatile float rateErroY_History = 0;
static volatile float rateErroX_History = 0;
static volatile float rateErroZ_History = 0;
static volatile float lastratePErro = 0;
static volatile float lastrateRErro = 0;
static volatile float lastrateYErro = 0;
static volatile float heightVErr_History = 0;
static volatile float pointxVErr_History = 0;
static volatile float pointyVErr_History = 0;
static volatile float flowxVErr_History = 0;
static volatile float flowyVErr_History = 0;
static volatile float lastheightVDelta=0;
static volatile float lastpointVxDelta=0;
static volatile float lastpointVyDelta=0;
static volatile float lastflowVxDelta=0;
static volatile float	lastflowVyDelta=0;
void Calculate_output(void);
extern int flagg;
//低通参数
// low pass filter:           0.079577472903393 
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 2 Hz -> _filter = 79.5774e-3
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
volatile float lowpass_filter = 7.9577e-3;

void Inner_pidinit()
{
	
	pidRatePitch.iOut = 0;
	pidRateRoll.iOut = 0;
	pidRateYaw.iOut = 0;
	pidXSpeed.iOut = 0;
	pidYSpeed.iOut = 0;
	pidAccHeight.iOut = 0;
	rateErroX_History = 0;
	rateErroY_History = 0;
	rateErroZ_History = 0;
	lastratePErro = 0;
	lastrateRErro = 0;
	lastrateYErro = 0;
	//高度速度
	lastheightVDelta = 0;
	heightVErr_History = 0;
	//点位置速度
	lastpointVxDelta = 0;
	pointxVErr_History = 0;
	lastpointVyDelta = 0;
	pointyVErr_History = 0;
	//光流速度
	flowxVErr_History = 0;
	lastflowVxDelta = 0;
	flowyVErr_History = 0;
	lastflowVyDelta = 0;
//	remote_Info.set_pitch_goal = 0;
} 

void dianjijiaozhun()
{
	if(remote_Info.state == control_rate_target)
	{
		Throttle_Info.M1 =Throttle_Info.M2 = Throttle_Info.M3 =Throttle_Info.M4 = remote_Info.set_ratepitch_goal /5;//+-10000
		
		
		if(Throttle_Info.M1 > 900)  Throttle_Info.M1=900;
		if(Throttle_Info.M2 > 900)  Throttle_Info.M2=900;
		if(Throttle_Info.M3 > 900)  Throttle_Info.M3=900;
		if(Throttle_Info.M4 > 900)  Throttle_Info.M4=900;
		
		if(Throttle_Info.M1 < 50)  Throttle_Info.M1=0;
		if(Throttle_Info.M2 < 50)  Throttle_Info.M2=0;
		if(Throttle_Info.M3 < 50)  Throttle_Info.M3=0;
		if(Throttle_Info.M4 < 50)  Throttle_Info.M4=0;
		PID_OUT(Throttle_Info.M1,Throttle_Info.M2,Throttle_Info.M3,Throttle_Info.M4);
	}

}


//void ttestyaw(float turnangle)
//{
//	if(testyaw.turn_angle == level_2)
//	{
//		
//	}
//}



float feedback,target;
void AttitudeInner_control()
{
	static unsigned int AttitudeInnertPre=0;
	unsigned int AttitudeInnert;
	static float AttitudeInnerPID_dt;
	AttitudeInnert = micros();
  AttitudeInnerPID_dt = (AttitudeInnertPre>0)?((AttitudeInnert-AttitudeInnertPre)/1000000.0f):1;
  AttitudeInnertPre = AttitudeInnert;
	
	float deltaPitchRateErro,deltaRollRateErro,deltaYawRateErro;
	/************************ 内环Pitch角速度环调节************************/	
	//如果要测试Pitch角打舵内环实验，把期望（pidPitch.value）改为—>	Target_Info.RatePitch
	float ratePitchErro;
	if(remote_Info.testtype==test_pitch || remote_Info.testtype == test_axisbalance)
	{
		ratePitchErro = (pidPitch.value - RT_Info.ratePitch);//?
	}
	else if(remote_Info.testtype==test_ratepitch)//说明RdroneStudio选择了测试内环Pitch,将期望改为手柄发送下来的期望
	{ 
		ratePitchErro = (remote_Info.set_ratepitch_goal - RT_Info.ratePitch);//e(k)
	} 
	else 
		ratePitchErro = 0;
	//                       e(k)-e(k-1)
	deltaPitchRateErro = (ratePitchErro - rateErroY_History)/AttitudeInnerPID_dt;
	/*20Hz低通滤波器*/
	//e(k)-e(k-1)
	deltaPitchRateErro = lastratePErro + (AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaPitchRateErro - lastratePErro);	
	lastratePErro=deltaPitchRateErro;
	//此为位置式PID
	pidRatePitch.pOut = 0.03 * ratePitchErro;
	pidRatePitch.dOut = 0.0006 * deltaPitchRateErro;
	pidRatePitch.iOut += 0.0006 * ratePitchErro;
	
	pidRatePitch.iOut = Limits_data(pidRatePitch.iOut,intergrateLimit,-intergrateLimit);
	
	rateErroY_History = ratePitchErro;
	
	pidRatePitch.value = pidRatePitch.pOut
														+ pidRatePitch.dOut
															   + pidRatePitch.iOut;
	/*限幅Pitch最终输出*/
	pidRatePitch.value = Limits_data(pidRatePitch.value,300,-300);
	
	/************************ 内环Roll角速度环调节************************/	
	//如果要测试Roll角打舵内环实验，把期望（pidRoll.value）改为—>	Target_Info.RateRoll
	float rateRollErro;
  if(remote_Info.testtype==test_roll||remote_Info.testtype == test_axisbalance)
	{
		rateRollErro = (pidRoll.value- RT_Info.rateRoll); 
	}
	else if(remote_Info.testtype == test_rateroll)
	{
		rateRollErro = (remote_Info.set_rateroll_goal - RT_Info.rateRoll);
	}
	else 
		rateRollErro = 0;
	deltaRollRateErro = (rateRollErro - rateErroX_History)/AttitudeInnerPID_dt;
	
	deltaRollRateErro = lastrateRErro + 
   			(AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaRollRateErro - lastrateRErro);	
	lastrateRErro=deltaRollRateErro;

	pidRateRoll.pOut = 0.03 * rateRollErro;
	pidRateRoll.dOut = 0.0005 * deltaRollRateErro;
	pidRateRoll.iOut += 0.0005 * rateRollErro;
	/*积分限幅*/
	pidRateRoll.iOut = Limits_data(pidRateRoll.iOut,intergrateLimit,-intergrateLimit);
	
	rateErroX_History = rateRollErro;
	
	pidRateRoll.value = pidRateRoll.pOut
													+pidRateRoll.dOut
															+pidRateRoll.iOut;
	/*限幅Roll最终输出*/
	pidRateRoll.value = Limits_data(pidRateRoll.value,300,-300);
	
	/************************ 内环Yaw角速度环调节************************/
	float rateYawErro;
	if(remote_Info.testtype == test_axisbalance)
	{
		rateYawErro= ( - pidYaw.value - RT_Info.rateYaw);//只是速度内环
	}
	else
	{
		rateYawErro = 0;
	}
	feedback = RT_Info.rateYaw;
	target = 7.5f*remote_Info.set_yaw_goal;
	deltaYawRateErro = (rateYawErro - rateErroZ_History)/AttitudeInnerPID_dt;
	
	deltaYawRateErro = lastrateYErro + 
   			(AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaYawRateErro - lastrateYErro);	
	lastrateYErro=deltaYawRateErro;

	pidRateYaw.pOut =  0.08 * rateYawErro;
	pidRateYaw.dOut =  0.00012 * deltaYawRateErro;
	pidRateYaw.iOut += 0.001 * rateYawErro;
	/*积分限幅*/
	pidRateYaw.iOut = Limits_data(pidRateYaw.iOut,intergrateLimit,-intergrateLimit);	
	
	rateErroZ_History = rateYawErro;
	
	pidRateYaw.value=pidRateYaw.pOut
											+pidRateYaw.dOut
													+pidRateYaw.iOut;
	/*限幅Yaw最终输出*/
	pidRateYaw.value = Limits_data(pidRateYaw.value,300,-300);	
	
	/**********************内环最终输出到电机**********************/
	Calculate_output();
//   dianjijiaozhun();
}
float flowvx_Kp = 0.13;
float flowvx_Ki = 0.0015;
float flowvx_Kd = 0.006;
float flowvy_Kp = 0.1;
float flowvy_Ki = 0.001;
float flowvy_Kd = 0.004;
void PostionInner_control()
{
	//时间计算
	static unsigned int PosInnertPre=0;
	unsigned int PosInnert;
	static float PosInnerPID_dt;
	float verro_us100;
	float vdelta_us100;
	float pointvx_Kp;
	float pointvx_Ki;
	float pointvx_Kd;
	float verro_pointx;
	float vdelta_pointx;
	float pointvy_Kp;
	float pointvy_Ki;
	float pointvy_Kd;
	float verro_pointy;
	float vdelta_pointy;
		/*X轴位移速度调整*/
	float verro_flowx;//-0.005f*Control_Info.rV
	float vdelta_flowx;
	float pre_error_fuzzypid;
	float min_fuzzypid,max_fuzzypid,kp_fuzzypid,ki_fuzzypid,kd_fuzzypid;
	PosInnert=micros();
	PosInnerPID_dt = (PosInnertPre>0)?((PosInnert-PosInnertPre)/1000000.0f):1;
	PosInnertPre=PosInnert;

	/************************ 高度速度内环 ************************/
	//如果要测试纯油门内环实验，把期望（pidHeight.value）改为—>Target_Info.AccHeight
	
	verro_us100 = pidHeight.value - RT_Info.US100_Alt_V;//融合好的速度
	vdelta_us100 = (verro_us100 - heightVErr_History)/PosInnerPID_dt;
	/*20Hz低通滤波器*/
	vdelta_us100 = lastheightVDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_us100 - lastheightVDelta);
	
	lastheightVDelta = vdelta_us100;
	heightVErr_History = verro_us100;
	

	pidAccHeight.pOut = verro_us100  * 78;
	pidAccHeight.dOut = vdelta_us100 * 10.5;
	pidAccHeight.iOut += verro_us100 * 0.11;
	
	pidAccHeight.iOut = Limits_data(pidAccHeight.iOut,120,-120);	

	pidAccHeight.value = pidAccHeight.pOut
													+pidAccHeight.dOut
															+pidAccHeight.iOut
																	+ pidHeight.feedforwardOut;//加入前馈校准

	pidAccHeight.value = Limits_data(pidAccHeight.value,300,-100);	

	/************************ 位置速度内环 ************************/

	/************************ 光流速度控制 ************************/
	if(RT_Info.US100_Alt>0.1f)
	{
		/***************X轴PID调节***************/
//		flowvx_Kp = 0;
//		flowvx_Ki = 0.0;
//		flowvx_Kd = 0;
		/*X轴位移速度调整*/
		verro_flowx=(float)(pidFlowX.value - RT_Info.FlowX_V);//-0.005f*Control_Info.rV
//		verro_flowx=(float)(0 - RT_Info.FlowX_V);
		vdelta_flowx=(verro_flowx-flowxVErr_History)/PosInnerPID_dt;
		/*20Hz低通滤波器*/
		vdelta_flowx = lastflowVxDelta + 
					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowx - lastflowVxDelta);

		lastflowVxDelta = vdelta_flowx;
		flowxVErr_History = verro_flowx;

		pidFlowx.pOut=0.05 * verro_flowx;//0.05yaw
		pidFlowx.dOut=0.03 * vdelta_flowx;//0.03
		pidFlowx.iOut+=0.f * verro_flowx;//0.0003

		pidFlowx.iOut = Limits_data(pidFlowx.iOut,8,-8);

		pidFlowx.value = pidFlowx.pOut
												+pidFlowx.dOut
														+pidFlowx.iOut
														   + pidFlowX.feedforwardOut;

		pidFlowx.value = Limits_data(pidFlowx.value,15,-15);

		remote_Info.set_pitch_goal = pidFlowx.value;
		/***************Y轴PID调节***************/			
		/*Y轴位移速度调整*/
		float verro_flowy=(float)(pidFlowY.value - RT_Info.FlowY_V);//-0.005f*Control_Info.pV
//		float verro_flowy=(float)(0 - RT_Info.FlowY_V);
		float vdelta_flowy=(verro_flowy-flowyVErr_History)/PosInnerPID_dt;
		/*20Hz低通滤波器*/
		vdelta_flowy = lastflowVyDelta + 
					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowy - lastflowVyDelta);

		lastflowVyDelta = vdelta_flowy;
		flowyVErr_History = verro_flowy;
		
		pidFlowy.pOut=0.05 * verro_flowy;
		pidFlowy.dOut=0.03 * vdelta_flowy;
		pidFlowy.iOut+=0.f * verro_flowy;

		pidFlowy.iOut = Limits_data(pidFlowy.iOut,8,-8);

		pidFlowy.value = pidFlowy.pOut
												+pidFlowy.dOut
														+pidFlowy.iOut
                               +pidFlowY.feedforwardOut;														
														
		pidFlowy.value = Limits_data(pidFlowy.value,15,-15);

		remote_Info.set_roll_goal = pidFlowy.value;
	} 
}
void Calculate_output()
{
	if(remote_Info.testtype == test_axisbalance)
	{										
			Throttle_Info.M1 =  - pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ 600;
		
			Throttle_Info.M2 =  + pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ 600;
		
			Throttle_Info.M3 =  + pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ 600;
		
			Throttle_Info.M4 =  - pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ 600;
	}
	else if(remote_Info.testtype==test_ratepitch||remote_Info.testtype == test_pitch)
	{
			Throttle_Info.M1 = - pidRatePitch.value + throttleBasic;
			Throttle_Info.M2 = + pidRatePitch.value + throttleBasic;
			Throttle_Info.M3 = + pidRatePitch.value + throttleBasic;
			Throttle_Info.M4 = - pidRatePitch.value + throttleBasic;
	}
	else if(remote_Info.testtype==test_roll||remote_Info.testtype == test_rateroll)
	{
			Throttle_Info.M1 = - pidRateRoll.value + throttleBasic;
			Throttle_Info.M2 = - pidRateRoll.value + throttleBasic;
			Throttle_Info.M3 = + pidRateRoll.value + throttleBasic;
			Throttle_Info.M4 = + pidRateRoll.value + throttleBasic;
	}
	
	if(Throttle_Info.M1 > 900)  Throttle_Info.M1=900;
	if(Throttle_Info.M2 > 900)  Throttle_Info.M2=900;
	if(Throttle_Info.M3 > 900)  Throttle_Info.M3=900;
	if(Throttle_Info.M4 > 900)  Throttle_Info.M4=900;
	
	if(Throttle_Info.M1 < 50)  Throttle_Info.M1=50;
	if(Throttle_Info.M2 < 50)  Throttle_Info.M2=50;
	if(Throttle_Info.M3 < 50)  Throttle_Info.M3=50;
	if(Throttle_Info.M4 < 50)  Throttle_Info.M4=50;
	
	PID_OUT(Throttle_Info.M1,Throttle_Info.M2,Throttle_Info.M3,Throttle_Info.M4);
}
/*根据不同实验选择不同输出通道
* Motor1-4：电机1-4的PWM输出占空比，范围为0-1000
* FlightMode：飞行模式，也就是实验选择
*/
void PID_OUT(unsigned int Motor1,
						 unsigned int Motor2,
						 unsigned int Motor3,
						 unsigned int Motor4)
{
		Motor1+=1000;
		Motor2+=1000;
		Motor3+=1000;
		Motor4+=1000;
	
	  if(RT_Info.lowPowerFlag==1)//没电
		{
			TIM2->CCR1=1000;
			TIM2->CCR2=1000;
			TIM2->CCR3=1000;
			TIM2->CCR4=1000;
		}
		else//有电
		{
			TIM2->CCR1=Motor1;
			TIM2->CCR2=Motor2;
			TIM2->CCR3=Motor3;
			TIM2->CCR4=Motor4;
		}
}

