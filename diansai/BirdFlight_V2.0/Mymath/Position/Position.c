/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * 作者 	:Xiluna Tech
 * 文件名 :Position.c
 * 描述   :位置融合函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
******************************************************************************/
#include "Position.h"



// FlowPostionx FIR filter
//#define FlowPostionx_FIR_TAPS_BITS		4
//#define FlowPostionx_FIR_LEN				(1 << FlowPostionx_FIR_TAPS_BITS)
//#define FlowPostionx_FIR_FCUT				0.15
static volatile float FlowPostionx_fir_coeffs[FlowPostionx_FIR_LEN];
static volatile float FlowPostionx_fir_samples[FlowPostionx_FIR_LEN];
static volatile int FlowPostionx_fir_index = 0;

// FlowPostiony FIR filter
//#define FlowPostiony_FIR_TAPS_BITS		4
//#define FlowPostiony_FIR_LEN				(1 << FlowPostiony_FIR_TAPS_BITS)
//#define FlowPostiony_FIR_FCUT				0.15
static volatile float FlowPostiony_fir_coeffs[FlowPostiony_FIR_LEN];
static volatile float FlowPostiony_fir_samples[FlowPostiony_FIR_LEN];
static volatile int FlowPostiony_fir_index = 0;

void Filter_FIRinit(void)
{
	// Create FlowPostionx FIR filter
	filter_create_fir_lowpass((float*)FlowPostionx_fir_coeffs, 
										FlowPostionx_FIR_FCUT,FlowPostionx_FIR_TAPS_BITS, 1);
	// Create FlowPostiony FIR filter
	filter_create_fir_lowpass((float*)FlowPostiony_fir_coeffs, 
										FlowPostiony_FIR_FCUT,FlowPostiony_FIR_TAPS_BITS, 1);
}
void Height_Estimation(float Ultra) //高度融合
{
		float Z_est[2] = {0.0f,0.0f};//Z 轴的高度数据和速度数据
		float Accel_ned[3] = {0.0f,0.0f,0.0f};//地理坐标系下的加速度数据
		float Accel_bias[3] = {0.0f,0.0f,0.0f};//机体坐标下的加速度偏移量
		float Corr_Ultra = 0.0f;//超声波校准系数
		static float z_ultra=10.0f;
		static float acc_bias=0.8f;
		float Accel_now[3] = {0.0f,0.0f,0.0f};
		float Accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
		
		static unsigned int PostiontPre=0;
		unsigned int Postiont;
		static float Position_dt;
		Postiont=micros();
		Position_dt = (PostiontPre>0)?((Postiont-PostiontPre)/1000000.0f):1;
		PostiontPre=Postiont;	
		int i,j;
		Accel_now[0] = RT_Info.accratex;
		Accel_now[1] = RT_Info.accratey;
		Accel_now[2] = RT_Info.accratez;
		//超声波数据 计算超声波的校正系数
		Corr_Ultra = 0 - Ultra - Z_est[0];
		//加速度数据先去除偏移量
		Accel_now[0] -= Accel_bias[0];
		Accel_now[1] -= Accel_bias[1];
		Accel_now[2] -= Accel_bias[2];
		//转化为 NED 坐标系
		for(i=0; i<3; i++)
		{
		Accel_ned[i]=0.0f;
			for(j=0; j<3; j++)
			{
				Accel_ned[i] += RDrone_R[i][j]* Accel_now[j];
			}
		}
		//地理坐标系下的 z 轴加速度是有重力加速度的，因此补偿上去
		Accel_ned[2] += CONSTANTS_ONE_G;
		//正确的加速度偏移量
		Accel_bias_corr[2] -= Corr_Ultra * z_ultra * z_ultra;
		//转化为机体坐标系
		for (i = 0; i < 3; i++)
		{
			float c = 0.0f;
			for (j = 0; j < 3; j++)
			{
				c += RDrone_R[j][i] * Accel_bias_corr[j];
			}
			Accel_bias[i] += c * acc_bias * Position_dt;
		}
		//加速度推算高度
		inertial_filter_predict(Position_dt, Z_est, Accel_ned[2]);
		//超声波校正系数进行校正
		inertial_filter_correct(Corr_Ultra, Position_dt, Z_est, 0, z_ultra);//对高度矫正
		//获得融合好的高度和速度
		RT_Info.US100_Alt = -Z_est[0];
		RT_Info.US100_Alt_V = -Z_est[1];
}////////////////////
//定点位置估计
float havesee_1,havesee_2;
void Position_Estimation(float Ultrasonic,float Xvision,float Yvision,DroneRTInfo *p)
{
	unsigned int i,j;
	static unsigned int PostiontPre=0;
	unsigned int Postiont;
	static float Position_dt;
	Postiont=micros();
  Position_dt = (PostiontPre>0)?((Postiont-PostiontPre)/1000000.0f):1;
  PostiontPre=Postiont;	
	
	//预测参数
	static float x_vision=8.0f;
	static float y_vision=8.0f;
	static float z_ultra=10.0f;
	static float acc_bias=0.8f;

	static float X_est[2] = {0.0f,0.0f};//X轴的位移和速度
	static float Y_est[2] = {0.0f,0.0f};//Y轴的位移和速度
	static float Z_est[2] = {0.0f,0.0f};//Z轴的高度和速度

	static float Accel_ned[3] = {0.0f,0.0f,0.0f};//地理坐标系下的加速度数据
	static float Accel_bias[3] = {0.0f,0.0f,0.0f};//机体坐标系下的加速度偏移量

	static float Corr_Ultra = 0.0f;
	static float Corr_Xvision = 0.0f;
	static float Corr_Yvision = 0.0f;
	float Accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
	float Accel_now[3] = {0.0f,0.0f,0.0f};
									
	Accel_now[0] = p->accratex;
	Accel_now[1] = p->accratey;
	Accel_now[2] = p->accratez;
	//位置参数校准
	Corr_Xvision = 0 - Xvision - X_est[0];
	Corr_Yvision = 0 - Yvision - Y_est[0];
  Corr_Ultra = 0 - Ultrasonic - Z_est[0];
	//加速度除去偏移量
  Accel_now[0] -= Accel_bias[0];
  Accel_now[1] -= Accel_bias[1];
  Accel_now[2] -= Accel_bias[2];
	//转化为NED坐标系
  for(i=0; i<3; i++)
  {
		Accel_ned[i]=0.0f;
		for(j=0; j<3; j++)
		{
			Accel_ned[i] += RDrone_R[i][j]* Accel_now[j];
		}
  }
	Accel_ned[2] += CONSTANTS_ONE_G;//算出来是空间加速度
	//正确的加速度偏移向量
	Accel_bias_corr[0] -= Corr_Yvision * y_vision * y_vision;
	Accel_bias_corr[1] -= Corr_Xvision * x_vision * x_vision;
  Accel_bias_corr[2] -= Corr_Ultra * z_ultra * z_ultra;
	//转化为机体坐标系
  for (i = 0; i < 3; i++)
  {
		float c = 0.0f;
		for (j = 0; j < 3; j++)
		{
			c += RDrone_R[j][i] * Accel_bias_corr[j];
		}
		Accel_bias[i] += c * acc_bias * Position_dt;
  }
	//X轴预测
	inertial_filter_predict(Position_dt, X_est, Accel_ned[1]);
  inertial_filter_correct(Corr_Xvision, Position_dt, X_est, 0, x_vision);
	//Y轴预测
	inertial_filter_predict(Position_dt, Y_est, Accel_ned[0]);
  inertial_filter_correct(Corr_Yvision, Position_dt, Y_est, 0, y_vision);
	//Z轴预测
	inertial_filter_predict(Position_dt, Z_est, Accel_ned[2]);
  inertial_filter_correct(Corr_Ultra, Position_dt, Z_est, 0, z_ultra);
	
	float PointX,PointX_V,PointY,PointY_V;
	
	PointX = X_est[0];
	PointX_V = -X_est[1];

	PointY = Y_est[0];
	PointY_V = -Y_est[1];
	
	havesee_1 = -Z_est[0];
	havesee_2 = -Z_est[1];
//	US100_Alt = -Z_est[0];
//	RT_Info.US100_Alt_V = -Z_est[1];

}

/* kalman滤波融合光流数据和加速计数据  主要参照卡尔曼线性滤波的五条黄金公式*/
/* X(k|k-1)=AX(k-1|k-1)+BU(k) */
/* P(k|k-1)=AP(k-1|k-1)A'+Q */
/* X(k|k)= X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1)) */
/* Kg(k)= P(k|k-1)H'/(HP(k|k-1)H'+R) */
/* P(k|k)=(I-Kg(k)H)P(k|k-1) */
/* 卡尔曼滤波详解可参考：
		https://wenku.baidu.com/view/3c42b7733186bceb18e8bb29.html*/
//float I_see1,I_see2,I_see3;
void OpticalFlow_Estimation(float Ultrasonic,float flow_x,float flow_y,DroneRTInfo *p,float Accx,float Accy)
{
	//周期获取
	unsigned int i,j;
	static unsigned int OpticalFlowtPre=0;
	unsigned int OpticalFlowt;
	static float OpticalFlow_dt;
	OpticalFlowt=micros();
  OpticalFlow_dt = (OpticalFlowtPre>0)?((OpticalFlowt-OpticalFlowtPre)/1000000.0f):1;
  OpticalFlowtPre=OpticalFlowt;		
	/* 高度kf融合 */
	static float Z_height=10.0f;
	static float Acc_bias=0.8f;
	static float z_est[2] = {0.0f,0.0f};						//Z轴的高度和速度
	static float accel_ned[3] = {0.0f,0.0f,0.0f};		//地理坐标系下的加速度数据
	static float accel_bias[3] = {0.0f,0.0f,0.0f};	//机体坐标系下的加速度偏移量
	static float corr_Ultra = 0.0f;
	float accel_bias_corr[3] = {0.0f,0.0f,0.0f};
	float accel_now[3] = {0.0f,0.0f,0.0f};
	accel_now[0] = p->accratex;
	accel_now[1] = p->accratey;
	accel_now[2] = p->accratez;
	
	//位置参数校准
  corr_Ultra = 0 - Ultrasonic - z_est[0];
	//加速度除去偏移量
  accel_now[0] -= accel_bias[0];
  accel_now[1] -= accel_bias[1];
  accel_now[2] -= accel_bias[2];
	//转化为NED坐标系
  for(i=0; i<3; i++)
  {
		accel_ned[i]=0.0f;
		for(j=0; j<3; j++)
		{
			accel_ned[i] += RDrone_R[i][j]* accel_now[j];
		}
  }
	accel_ned[2] += CONSTANTS_ONE_G;
	//正确的加速度偏移向量
  accel_bias_corr[2] -= corr_Ultra * Z_height * Z_height;
	//转化为机体坐标系
  for (i = 0; i < 3; i++)
  {
		float c = 0.0f;
		for (j = 0; j < 3; j++)
		{
			c += RDrone_R[j][i] * accel_bias_corr[j];
		}
		accel_bias[i] += c * Acc_bias * OpticalFlow_dt;
  }
	//Z轴预测
	inertial_filter_predict(OpticalFlow_dt, z_est, accel_ned[2]);
  inertial_filter_correct(corr_Ultra, OpticalFlow_dt, z_est, 0, Z_height);
	
	RT_Info.US100_Alt = -z_est[0];
	RT_Info.US100_Alt_V  = -z_est[1];
//	I_see1 = -z_est[0];
	/* x轴kf融合 */
	static float OpticalFlowx_Velocity;
	static float Q_OpticalFlowxVelocity = 0.001,Q_xbias = 0.001;
	static float R_OpticalFlowx = 0.1;
	static float OpticalFlowx_Velocityerr,OpticalFlowxbias;
	static float OpticalFlowxPCt_0 = 0,OpticalFlowxPCt_1 = 0,OpticalFlowxE = 0;
	static float OpticalFlowxK_0 = 0,OpticalFlowxK_1 = 0,OpticalFlowxt_0 = 0,OpticalFlowxt_1 = 0;
	static char OpticalFlowxC_0 = 1;
	static float OpticalFlowxPdot[4],OpticalFlowxP[2][2];
	
	OpticalFlowx_Velocity += (Accy - OpticalFlowxbias) * OpticalFlow_dt;
	
	OpticalFlowxPdot[0] = Q_OpticalFlowxVelocity - OpticalFlowxP[0][1] - OpticalFlowxP[1][0];
	OpticalFlowxPdot[1] =	-OpticalFlowxP[1][1];
	OpticalFlowxPdot[2] =	-OpticalFlowxP[1][1];
	OpticalFlowxPdot[3] = Q_xbias;
	
	OpticalFlowxP[0][0] += OpticalFlowxPdot[0] * OpticalFlow_dt;
	OpticalFlowxP[0][1] += OpticalFlowxPdot[1] * OpticalFlow_dt;
	OpticalFlowxP[1][0] += OpticalFlowxPdot[2] * OpticalFlow_dt;
	OpticalFlowxP[1][1] += OpticalFlowxPdot[3] * OpticalFlow_dt;
	
	OpticalFlowxPCt_0 = OpticalFlowxC_0 * OpticalFlowxP[0][0];
	OpticalFlowxPCt_1 = OpticalFlowxC_0 * OpticalFlowxP[1][0];
	OpticalFlowxE = R_OpticalFlowx + OpticalFlowxC_0 * OpticalFlowxPCt_0;
	OpticalFlowxK_0 = OpticalFlowxPCt_0 / OpticalFlowxE;
  OpticalFlowxK_1 = OpticalFlowxPCt_1 / OpticalFlowxE;
	
	OpticalFlowx_Velocityerr = flow_x - OpticalFlowx_Velocity;
	OpticalFlowx_Velocity += OpticalFlowxK_0 * OpticalFlowx_Velocityerr; 
  OpticalFlowxbias += OpticalFlowxK_1 * OpticalFlowx_Velocityerr;
	
	OpticalFlowxt_0 = OpticalFlowxPCt_0;
  OpticalFlowxt_1 = OpticalFlowxC_0 * OpticalFlowxP[0][1];
	
	OpticalFlowxP[0][0] -= OpticalFlowxK_0 * OpticalFlowxt_0;
  OpticalFlowxP[0][1] -= OpticalFlowxK_0 * OpticalFlowxt_1;
  OpticalFlowxP[1][0] -= OpticalFlowxK_1 * OpticalFlowxt_0;
  OpticalFlowxP[1][1] -= OpticalFlowxK_1 * OpticalFlowxt_1;
	
	RT_Info.FlowX_V  = OpticalFlowx_Velocity ;
//	I_see2 = OpticalFlowx_Velocity;
	/* y轴kf融合 */
	static float OpticalFlowy_Velocity;
	static float Q_OpticalFlowyVelocity = 0.001,Q_ybias = 0.001;
	static float R_OpticalFlowy = 0.1;
	static float OpticalFlowy_Velocityerr,OpticalFlowybias;
	static float OpticalFlowyPCt_0 = 0,OpticalFlowyPCt_1 = 0,OpticalFlowyE = 0;
	static float OpticalFlowyK_0 = 0,OpticalFlowyK_1 = 0,OpticalFlowyt_0 = 0,OpticalFlowyt_1 = 0;
	static char OpticalFlowyC_0 = 1;
	static float OpticalFlowyPdot[4],OpticalFlowyP[2][2];
	
	OpticalFlowy_Velocity += (Accx - OpticalFlowybias) * OpticalFlow_dt;
	
	OpticalFlowyPdot[0] = Q_OpticalFlowyVelocity - OpticalFlowyP[0][1] - OpticalFlowyP[1][0];
	OpticalFlowyPdot[1] =	-OpticalFlowyP[1][1];
	OpticalFlowyPdot[2] =	-OpticalFlowyP[1][1];
	OpticalFlowyPdot[3] = Q_ybias;
	
	OpticalFlowyP[0][0] += OpticalFlowyPdot[0] * OpticalFlow_dt;
	OpticalFlowyP[0][1] += OpticalFlowyPdot[1] * OpticalFlow_dt;
	OpticalFlowyP[1][0] += OpticalFlowyPdot[2] * OpticalFlow_dt;
	OpticalFlowyP[1][1] += OpticalFlowyPdot[3] * OpticalFlow_dt;
	
	OpticalFlowyPCt_0 = OpticalFlowyC_0 * OpticalFlowyP[0][0];
	OpticalFlowyPCt_1 = OpticalFlowyC_0 * OpticalFlowyP[1][0];
	OpticalFlowyE = R_OpticalFlowy + OpticalFlowyC_0 * OpticalFlowyPCt_0;
	OpticalFlowyK_0 = OpticalFlowyPCt_0 / OpticalFlowyE;
  OpticalFlowyK_1 = OpticalFlowyPCt_1 / OpticalFlowyE;
	
	OpticalFlowy_Velocityerr = flow_y - OpticalFlowy_Velocity;
	OpticalFlowy_Velocity += OpticalFlowyK_0 * OpticalFlowy_Velocityerr; 
  OpticalFlowybias += OpticalFlowyK_1 * OpticalFlowy_Velocityerr;
	
	OpticalFlowyt_0 = OpticalFlowyPCt_0;
  OpticalFlowyt_1 = OpticalFlowyC_0 * OpticalFlowyP[0][1];
	
	OpticalFlowyP[0][0] -= OpticalFlowyK_0 * OpticalFlowyt_0;
  OpticalFlowyP[0][1] -= OpticalFlowyK_0 * OpticalFlowyt_1;
  OpticalFlowyP[1][0] -= OpticalFlowyK_1 * OpticalFlowyt_0;
  OpticalFlowyP[1][1] -= OpticalFlowyK_1 * OpticalFlowyt_1;
	
	RT_Info.FlowY_V  = OpticalFlowy_Velocity ;
//	I_see3 = OpticalFlowy_Velocity;
}

void third_Kfilter(float Ultrasonic,float flow_x,float flow_y,DroneRTInfo *p,float Accx,float Accy,char reset_flag)
{
	//周期获取
	unsigned int i,j;
	static unsigned int OpticalFlowtPre=0;//
	unsigned int OpticalFlowt;
	static float OpticalFlow_dt;//
	OpticalFlowt=micros();
  OpticalFlow_dt = (OpticalFlowtPre>0)?((OpticalFlowt-OpticalFlowtPre)/1000000.0f):1;
  OpticalFlowtPre=OpticalFlowt;
	/* 高度kf融合 */
	static float Z_height=10.0f;
	static float Acc_bias=0.8f;
	static float z_est[2] = {0.0f,0.0f};						//Z轴的高度和速度
	static float accel_ned[3] = {0.0f,0.0f,0.0f};		//地理坐标系下的加速度数据
	static float accel_bias[3] = {0.0f,0.0f,0.0f};	//机体坐标系下的加速度偏移量
	static float corr_Ultra = 0.0f;//
	float accel_bias_corr[3] = {0.0f,0.0f,0.0f};
	float accel_now[3] = {0.0f,0.0f,0.0f};
	accel_now[0] = p->accratex;
	accel_now[1] = p->accratey;
	accel_now[2] = p->accratez;
	//位置参数校准
  corr_Ultra = 0 - Ultrasonic - z_est[0];
	//加速度除去偏移量
  accel_now[0] -= accel_bias[0];
  accel_now[1] -= accel_bias[1];
  accel_now[2] -= accel_bias[2];
	//转化为NED坐标系
  for(i=0; i<3; i++)
  {
		accel_ned[i]=0.0f;
		for(j=0; j<3; j++)
		{
			accel_ned[i] += RDrone_R[i][j]* accel_now[j];
		}
  }
	accel_ned[2] += CONSTANTS_ONE_G;
	//正确的加速度偏移向量
  accel_bias_corr[2] -= corr_Ultra * Z_height * Z_height;
	//转化为机体坐标系
  for (i = 0; i < 3; i++)
  {
		float c = 0.0f;
		for (j = 0; j < 3; j++)
		{
			c += RDrone_R[j][i] * accel_bias_corr[j];
		}
		accel_bias[i] += c * Acc_bias * OpticalFlow_dt;
  }
	//Z轴预测
	inertial_filter_predict(OpticalFlow_dt, z_est, accel_ned[2]);
  inertial_filter_correct(corr_Ultra, OpticalFlow_dt, z_est, 0, Z_height);
	
	RT_Info.US100_Alt = -z_est[0];
	RT_Info.US100_Alt_V  = -z_est[1];//初始0.09
//////////////////////////////////////////////////////////////////////////
	//x轴融合
	static float OpticalFlowx = 0,OpticalFlowx_Velocity = 0;
	static float Q_OpticalFlowx = 0.001,Q_OpticalFlowxVelocity = 0.001,Q_OpticalFlowxbias = 0.001;
	static float OpticalFlowxbias = 0,OpticalFlowx_err = 0;
	static float R_OpticalFlowx = 0.5;
	static char OpticalFlowxC_0 = 1;
	static float OpticalFlowxPCt_0 = 0,OpticalFlowxPCt_1 = 0,OpticalFlowxPCt_2 = 0,OpticalFlowxE = 0;	
	static float OpticalFlowxK_0 = 0,OpticalFlowxK_1 = 0,OpticalFlowxK_2 = 0;
	static float OpticalFlowxt_0 = 0,OpticalFlowxt_1 = 0,OpticalFlowxt_2 = 0;
	static float OpticalFlowxPdot[9] = {0,0,0,0,0,0,0,0,0};
	static float OpticalFlowxPP[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	//
	
	//v(k+1) = v(k)+[a(k)-bias(k)]*dt
	OpticalFlowx_Velocity += (Accy - OpticalFlowxbias)*OpticalFlow_dt;
	//Height(k+1) = Height(k) + 1/2[a(k)-bias(k)]dt^2 + v(k)dt
	OpticalFlowx += (Accy - OpticalFlowxbias)*OpticalFlow_dt*OpticalFlow_dt/2 + OpticalFlowx_Velocity*OpticalFlow_dt;
	
	OpticalFlowxPdot[0] = Q_OpticalFlowx + OpticalFlowxPP[0][1] + OpticalFlowxPP[1][0]+(OpticalFlowxPP[1][1]-OpticalFlowxPP[2][0]/2-OpticalFlowxPP[0][2]/2)*OpticalFlow_dt;
	OpticalFlowxPdot[1] = OpticalFlowxPP[1][1]-OpticalFlowxPP[0][2]-(OpticalFlowxPP[2][1]/2+OpticalFlowxPP[1][2])*OpticalFlow_dt;
	OpticalFlowxPdot[2] = OpticalFlowxPP[1][2]-OpticalFlowxPP[2][2]*OpticalFlow_dt/2;
	OpticalFlowxPdot[3] = OpticalFlowxPP[1][1]-OpticalFlowxPP[2][0]-(OpticalFlowxPP[2][1]+OpticalFlowxPP[1][2]/2)*OpticalFlow_dt;
	OpticalFlowxPdot[4] = Q_OpticalFlowxVelocity-OpticalFlowxPP[2][1]-OpticalFlowxPP[1][2]+OpticalFlowxPP[2][2]*OpticalFlow_dt;
	OpticalFlowxPdot[5] = -OpticalFlowxPP[2][2];
	OpticalFlowxPdot[6] = OpticalFlowxPP[2][1]-OpticalFlowxPP[2][2]*OpticalFlow_dt/2;
	OpticalFlowxPdot[7] = -OpticalFlowxPP[2][2];
	OpticalFlowxPdot[8] = Q_OpticalFlowxbias;
	
	OpticalFlowxPP[0][0] += OpticalFlowxPdot[0]*OpticalFlow_dt;
	OpticalFlowxPP[0][1] += OpticalFlowxPdot[1]*OpticalFlow_dt;
	OpticalFlowxPP[0][2] += OpticalFlowxPdot[2]*OpticalFlow_dt;
	OpticalFlowxPP[1][0] += OpticalFlowxPdot[3]*OpticalFlow_dt;
	OpticalFlowxPP[1][1] += OpticalFlowxPdot[4]*OpticalFlow_dt;
	OpticalFlowxPP[1][2] += OpticalFlowxPdot[5]*OpticalFlow_dt;
	OpticalFlowxPP[2][0] += OpticalFlowxPdot[6]*OpticalFlow_dt;
	OpticalFlowxPP[2][1] += OpticalFlowxPdot[7]*OpticalFlow_dt;
	OpticalFlowxPP[2][2] += OpticalFlowxPdot[8]*OpticalFlow_dt;
	
	OpticalFlowxPCt_0 = OpticalFlowxC_0 * OpticalFlowxPP[0][0];				
	OpticalFlowxPCt_1 = OpticalFlowxC_0 * OpticalFlowxPP[1][0];				
	OpticalFlowxPCt_2 = OpticalFlowxC_0 * OpticalFlowxPP[2][0];
	
	OpticalFlowxE = R_OpticalFlowx + OpticalFlowxC_0 * OpticalFlowxPCt_0;	
	
	OpticalFlowxK_0 = OpticalFlowxPCt_0/OpticalFlowxE;						
	OpticalFlowxK_1 = OpticalFlowxPCt_1/OpticalFlowxE;
	OpticalFlowxK_2 = OpticalFlowxPCt_2/OpticalFlowxE;
	
	OpticalFlowx_err = flow_x - OpticalFlowx;
	OpticalFlowx += OpticalFlowxK_0 * OpticalFlowx_err;		
	OpticalFlowx_Velocity += OpticalFlowxK_1 * OpticalFlowx_err;	
	OpticalFlowxbias += OpticalFlowxK_2 * OpticalFlowx_err;	
	
	OpticalFlowxt_0 = OpticalFlowxPCt_0;						
	OpticalFlowxt_1 = OpticalFlowxC_0 * OpticalFlowxPP[0][1];				
	OpticalFlowxt_2 = OpticalFlowxC_0 * OpticalFlowxPP[0][2];				
	
	OpticalFlowxPP[0][0] -= OpticalFlowxK_0*OpticalFlowxt_0;
	OpticalFlowxPP[0][1] -= OpticalFlowxK_0*OpticalFlowxt_1;
	OpticalFlowxPP[0][2] -= OpticalFlowxK_0*OpticalFlowxt_2;
	OpticalFlowxPP[1][0] -= OpticalFlowxK_1*OpticalFlowxt_0;
	OpticalFlowxPP[1][1] -= OpticalFlowxK_1*OpticalFlowxt_1;
	OpticalFlowxPP[1][2] -= OpticalFlowxK_1*OpticalFlowxt_2;
	OpticalFlowxPP[2][0] -= OpticalFlowxK_2*OpticalFlowxt_0;
	OpticalFlowxPP[2][1] -= OpticalFlowxK_2*OpticalFlowxt_1;
	OpticalFlowxPP[2][2] -= OpticalFlowxK_2*OpticalFlowxt_2;
	
	RT_Info.FlowX = OpticalFlowx;
	RT_Info.FlowX_V  = OpticalFlowx_Velocity;
	
	//	/* y轴kf融合 */
	static float OpticalFlowy = 0,OpticalFlowy_Velocity = 0;
	static float Q_OpticalFlowy = 0.001,Q_OpticalFlowyVelocity = 0.001,Q_OpticalFlowybias = 0.001;
	static float OpticalFlowybias = 0,OpticalFlowy_err = 0;
	static float R_OpticalFlowy = 0.5;
	static char OpticalFlowyC_0 = 1;
	static float OpticalFlowyPCt_0 = 0,OpticalFlowyPCt_1 = 0,OpticalFlowyPCt_2 = 0,OpticalFlowyE = 0;	
	static float OpticalFlowyK_0 = 0,OpticalFlowyK_1 = 0,OpticalFlowyK_2 = 0;
	static float OpticalFlowyt_0 = 0,OpticalFlowyt_1 = 0,OpticalFlowyt_2 = 0;
	static float OpticalFlowyPdot[9] = {0,0,0,0,0,0,0,0,0};
	static float OpticalFlowyPP[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	
	
	//v(k+1) = v(k)+[a(k)-bias(k)]*dt
	OpticalFlowy_Velocity += (Accx - OpticalFlowybias)*OpticalFlow_dt;
	//Height(k+1) = Height(k) + 1/2[a(k)-bias(k)]dt^2 + v(k)dt
	OpticalFlowy += (Accx - OpticalFlowybias)*OpticalFlow_dt*OpticalFlow_dt/2 + OpticalFlowy_Velocity*OpticalFlow_dt;
	
	OpticalFlowyPdot[0] = Q_OpticalFlowy + OpticalFlowyPP[0][1] + OpticalFlowyPP[1][0]+(OpticalFlowyPP[1][1]-OpticalFlowyPP[2][0]/2-OpticalFlowyPP[0][2]/2)*OpticalFlow_dt;
	OpticalFlowyPdot[1] = OpticalFlowyPP[1][1]-OpticalFlowyPP[0][2]-(OpticalFlowyPP[2][1]/2+OpticalFlowyPP[1][2])*OpticalFlow_dt;
	OpticalFlowyPdot[2] = OpticalFlowyPP[1][2]-OpticalFlowyPP[2][2]*OpticalFlow_dt/2;
	OpticalFlowyPdot[3] = OpticalFlowyPP[1][1]-OpticalFlowyPP[2][0]-(OpticalFlowyPP[2][1]+OpticalFlowyPP[1][2]/2)*OpticalFlow_dt;
	OpticalFlowyPdot[4] = Q_OpticalFlowyVelocity-OpticalFlowyPP[2][1]-OpticalFlowyPP[1][2]+OpticalFlowyPP[2][2]*OpticalFlow_dt;
	OpticalFlowyPdot[5] = -OpticalFlowyPP[2][2];
	OpticalFlowyPdot[6] = OpticalFlowyPP[2][1]-OpticalFlowyPP[2][2]*OpticalFlow_dt/2;
	OpticalFlowyPdot[7] = -OpticalFlowyPP[2][2];
	OpticalFlowyPdot[8] = Q_OpticalFlowybias;
	
	OpticalFlowyPP[0][0] += OpticalFlowyPdot[0]*OpticalFlow_dt;
	OpticalFlowyPP[0][1] += OpticalFlowyPdot[1]*OpticalFlow_dt;
	OpticalFlowyPP[0][2] += OpticalFlowyPdot[2]*OpticalFlow_dt;
	OpticalFlowyPP[1][0] += OpticalFlowyPdot[3]*OpticalFlow_dt;
	OpticalFlowyPP[1][1] += OpticalFlowyPdot[4]*OpticalFlow_dt;
	OpticalFlowyPP[1][2] += OpticalFlowyPdot[5]*OpticalFlow_dt;
	OpticalFlowyPP[2][0] += OpticalFlowyPdot[6]*OpticalFlow_dt;
	OpticalFlowyPP[2][1] += OpticalFlowyPdot[7]*OpticalFlow_dt;
	OpticalFlowyPP[2][2] += OpticalFlowyPdot[8]*OpticalFlow_dt;
	
	OpticalFlowyPCt_0 = OpticalFlowyC_0 *OpticalFlowyPP[0][0];				
	OpticalFlowyPCt_1 = OpticalFlowyC_0 *OpticalFlowyPP[1][0];				
	OpticalFlowyPCt_2 = OpticalFlowyC_0 *OpticalFlowyPP[2][0];
	
	OpticalFlowyE = R_OpticalFlowy + OpticalFlowyC_0 * OpticalFlowyPCt_0;		
	
	OpticalFlowyK_0 = OpticalFlowyPCt_0/OpticalFlowyE;						
	OpticalFlowyK_1 = OpticalFlowyPCt_1/OpticalFlowyE;
	OpticalFlowyK_2 = OpticalFlowyPCt_2/OpticalFlowyE;
	
	OpticalFlowy_err = flow_y - OpticalFlowy;
	OpticalFlowy += OpticalFlowyK_0 * OpticalFlowy_err;		
	OpticalFlowy_Velocity += OpticalFlowyK_1 * OpticalFlowy_err;	
	OpticalFlowybias += OpticalFlowyK_2 * OpticalFlowy_err;	
	
	OpticalFlowyt_0 = OpticalFlowyPCt_0;						
	OpticalFlowyt_1 = OpticalFlowyC_0 * OpticalFlowyPP[0][1];				
	OpticalFlowyt_2 = OpticalFlowyC_0 * OpticalFlowyPP[0][2];				
	
	
	OpticalFlowyPP[0][0] -= OpticalFlowyK_0*OpticalFlowyt_0;
	OpticalFlowyPP[0][1] -= OpticalFlowyK_0*OpticalFlowyt_1;
	OpticalFlowyPP[0][2] -= OpticalFlowyK_0*OpticalFlowyt_2;
	OpticalFlowyPP[1][0] -= OpticalFlowyK_1*OpticalFlowyt_0;
	OpticalFlowyPP[1][1] -= OpticalFlowyK_1*OpticalFlowyt_1;
	OpticalFlowyPP[1][2] -= OpticalFlowyK_1*OpticalFlowyt_2;
	OpticalFlowyPP[2][0] -= OpticalFlowyK_2*OpticalFlowyt_0;
	OpticalFlowyPP[2][1] -= OpticalFlowyK_2*OpticalFlowyt_1;
	OpticalFlowyPP[2][2] -= OpticalFlowyK_2*OpticalFlowyt_2;
	
	RT_Info.FlowY = OpticalFlowy;
	RT_Info.FlowY_V  = OpticalFlowy_Velocity;
	
	
////////////////////////////////////重置代码
	if(reset_flag == 1)
	{
//		OpticalFlowtPre=0;
//		OpticalFlow_dt = 0;
//		Z_height=10.0f;
//		Acc_bias=0.8f;
//		for(int i =0; i<2; i++)
//			z_est[i] = 0.0f;						//Z轴的高度和速度
//		for (int i=0;i<3;i++)
//			accel_ned[i] = 0.0f;		//地理坐标系下的加速度数据
//		for (int i=0;i<3;i++)
//			accel_bias[i] = 0.0f;	//机体坐标系下的加速度偏移量
//		corr_Ultra = 0.0f;
		//////////////////////////
		OpticalFlowx = 0;
		OpticalFlowx_Velocity = 0;
		Q_OpticalFlowx = 0.001;
		Q_OpticalFlowxVelocity = 0.001;
		Q_OpticalFlowxbias = 0.001;
		OpticalFlowxbias = 0;
		OpticalFlowx_err = 0;
		R_OpticalFlowx = 0.5;
		OpticalFlowxC_0 = 1;
		OpticalFlowxPCt_0 = 0;
		OpticalFlowxPCt_1 = 0;
		OpticalFlowxPCt_2 = 0;
		OpticalFlowxE = 0;	
		OpticalFlowxK_0 = 0;
		OpticalFlowxK_1 = 0;
		OpticalFlowxK_2 = 0;
		OpticalFlowxt_0 = 0;
		OpticalFlowxt_1 = 0;
		OpticalFlowxt_2 = 0;
		for(int i=0;i<9;i++)
			OpticalFlowxPdot[i] = 0;
		for(int j=0;j<3;j++)
		{
			for(int i=0;i<3;i++)
			{
				if(i==j)
					OpticalFlowxPP[j][i] = 1;
				else 
					OpticalFlowxPP[j][i] = 0;
			}
		}
		////////////////////////
		OpticalFlowy = 0;
		OpticalFlowy_Velocity = 0;
		Q_OpticalFlowy = 0.001;
		Q_OpticalFlowyVelocity = 0.001;
		Q_OpticalFlowybias = 0.001;
		OpticalFlowybias = 0;
		OpticalFlowy_err = 0;
		R_OpticalFlowy = 0.5;
		OpticalFlowyC_0 = 1;
		OpticalFlowyPCt_0 = 0;
		OpticalFlowyPCt_1 = 0;
		OpticalFlowyPCt_2 = 0;
		OpticalFlowyE = 0;	
		OpticalFlowyK_0 = 0;
		OpticalFlowyK_1 = 0;
		OpticalFlowyK_2 = 0;
		OpticalFlowyt_0 = 0;
		OpticalFlowyt_1 = 0;
		OpticalFlowyt_2 = 0;
		for (int i=0;i<9;i++)
			OpticalFlowyPdot[i] = 0;
		for(int j=0;j<3;j++)
		{
			for(int i=0;i<3;i++)
			{
				if(i==j)
					OpticalFlowyPP[j][i] = 1;
				else 
					OpticalFlowyPP[j][i] = 0;
			}
		}	
	}
}

/* y轴kf融合 */
//	static float OpticalFlowx = 0,OpticalFlowx_Velocity = 0;
//	static float Q_OpticalFlowx = 0.001,Q_OpticalFlowxVelocity = 0.001,Q_OpticalFlowxbias = 0.001;
//	static float OpticalFlowxbias = 0,OpticalFlowx_err = 0;
//	static float R_OpticalFlowx = 0.5;
//	static char OpticalFlowxC_0 = 1;
//	static float OpticalFlowxPCt_0 = 0,OpticalFlowxPCt_1 = 0,OpticalFlowxPCt_2 = 0,OpticalFlowxE = 0;	
//	static float OpticalFlowxK_0 = 0,OpticalFlowxK_1 = 0,OpticalFlowxK_2 = 0;
//	static float OpticalFlowxt_0 = 0,OpticalFlowxt_1 = 0,OpticalFlowxt_2 = 0;
//	static float OpticalFlowxPdot[9] = {0,0,0,0,0,0,0,0,0};
//	static float OpticalFlowxPP[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	
//	
//	//v(k+1) = v(k)+[a(k)-bias(k)]*dt
//	OpticalFlowx_Velocity += (Accy - OpticalFlowxbias)*OpticalFlow_dt;
//	//Height(k+1) = Height(k) + 1/2[a(k)-bias(k)]dt^2 + v(k)dt
//	OpticalFlowx += (Accy - OpticalFlowxbias)*OpticalFlow_dt*OpticalFlow_dt/2 + OpticalFlowx_Velocity*OpticalFlow_dt;
//	
//	OpticalFlowxPdot[0] = Q_OpticalFlowx + OpticalFlowxPP[0][1] + OpticalFlowxPP[1][0]+(OpticalFlowxPP[1][1]-OpticalFlowxPP[2][0]/2-OpticalFlowxPP[0][2]/2)*OpticalFlow_dt;
//	OpticalFlowxPdot[1] = OpticalFlowxPP[1][1]-OpticalFlowxPP[0][2]-(OpticalFlowxPP[2][1]/2+OpticalFlowxPP[1][2])*OpticalFlow_dt;
//	OpticalFlowxPdot[2] = OpticalFlowxPP[1][2]-OpticalFlowxPP[2][2]*OpticalFlow_dt/2;
//	OpticalFlowxPdot[3] = OpticalFlowxPP[1][1]-OpticalFlowxPP[2][0]-(OpticalFlowxPP[2][1]+OpticalFlowxPP[1][2]/2)*OpticalFlow_dt;
//	OpticalFlowxPdot[4] = Q_OpticalFlowxVelocity-OpticalFlowxPP[2][1]-OpticalFlowxPP[1][2]+OpticalFlowxPP[2][2]*OpticalFlow_dt;
//	OpticalFlowxPdot[5] = -OpticalFlowxPP[2][2];
//	OpticalFlowxPdot[6] = OpticalFlowxPP[2][1]-OpticalFlowxPP[2][2]*OpticalFlow_dt/2;
//	OpticalFlowxPdot[7] = -OpticalFlowxPP[2][2];
//	OpticalFlowxPdot[8] = Q_OpticalFlowxbias;
//	
//	OpticalFlowxPP[0][0] += OpticalFlowxPdot[0]*OpticalFlow_dt;
//	OpticalFlowxPP[0][1] += OpticalFlowxPdot[1]*OpticalFlow_dt;
//	OpticalFlowxPP[0][2] += OpticalFlowxPdot[2]*OpticalFlow_dt;
//	OpticalFlowxPP[1][0] += OpticalFlowxPdot[3]*OpticalFlow_dt;
//	OpticalFlowxPP[1][1] += OpticalFlowxPdot[4]*OpticalFlow_dt;
//	OpticalFlowxPP[1][2] += OpticalFlowxPdot[5]*OpticalFlow_dt;
//	OpticalFlowxPP[2][0] += OpticalFlowxPdot[6]*OpticalFlow_dt;
//	OpticalFlowxPP[2][1] += OpticalFlowxPdot[7]*OpticalFlow_dt;
//	OpticalFlowxPP[2][2] += OpticalFlowxPdot[8]*OpticalFlow_dt;
//	
//	OpticalFlowxPCt_0 = OpticalFlowxC_0 * OpticalFlowxPP[0][0];				
//	OpticalFlowxPCt_1 = OpticalFlowxC_0 * OpticalFlowxPP[1][0];				
//	OpticalFlowxPCt_2 = OpticalFlowxC_0 * OpticalFlowxPP[2][0];
//	
//	OpticalFlowxE = R_OpticalFlowx + OpticalFlowxC_0 * OpticalFlowxPCt_0;	
//	
//	OpticalFlowxK_0 = OpticalFlowxPCt_0/OpticalFlowxE;						
//	OpticalFlowxK_1 = OpticalFlowxPCt_1/OpticalFlowxE;
//	OpticalFlowxK_2 = OpticalFlowxPCt_2/OpticalFlowxE;
//	
//	OpticalFlowx_err = flow_x - OpticalFlowx;
//	OpticalFlowx += OpticalFlowxK_0 * OpticalFlowx_err;		
//	OpticalFlowx_Velocity += OpticalFlowxK_1 * OpticalFlowx_err;	
//	OpticalFlowxbias += OpticalFlowxK_2 * OpticalFlowx_err;	
//	
//	OpticalFlowxt_0 = OpticalFlowxPCt_0;						
//	OpticalFlowxt_1 = OpticalFlowxC_0 * OpticalFlowxPP[0][1];				
//	OpticalFlowxt_2 = OpticalFlowxC_0 * OpticalFlowxPP[0][2];				
//	
//	OpticalFlowxPP[0][0] -= OpticalFlowxK_0*OpticalFlowxt_0;
//	OpticalFlowxPP[0][1] -= OpticalFlowxK_0*OpticalFlowxt_1;
//	OpticalFlowxPP[0][2] -= OpticalFlowxK_0*OpticalFlowxt_2;
//	OpticalFlowxPP[1][0] -= OpticalFlowxK_1*OpticalFlowxt_0;
//	OpticalFlowxPP[1][1] -= OpticalFlowxK_1*OpticalFlowxt_1;
//	OpticalFlowxPP[1][2] -= OpticalFlowxK_1*OpticalFlowxt_2;
//	OpticalFlowxPP[2][0] -= OpticalFlowxK_2*OpticalFlowxt_0;
//	OpticalFlowxPP[2][1] -= OpticalFlowxK_2*OpticalFlowxt_1;
//	OpticalFlowxPP[2][2] -= OpticalFlowxK_2*OpticalFlowxt_2;
//	
//	RT_Info.FlowX = OpticalFlowx;
//	RT_Info.FlowX_V  = OpticalFlowx_Velocity;
//	/* y轴kf融合 */
//	static float OpticalFlowy = 0,OpticalFlowy_Velocity = 0;
//	static float Q_OpticalFlowy = 0.001,Q_OpticalFlowyVelocity = 0.001,Q_OpticalFlowybias = 0.001;
//	static float OpticalFlowybias = 0,OpticalFlowy_err = 0;
//	static float R_OpticalFlowy = 0.5;
//	static char OpticalFlowyC_0 = 1;
//	static float OpticalFlowyPCt_0 = 0,OpticalFlowyPCt_1 = 0,OpticalFlowyPCt_2 = 0,OpticalFlowyE = 0;	
//	static float OpticalFlowyK_0 = 0,OpticalFlowyK_1 = 0,OpticalFlowyK_2 = 0;
//	static float OpticalFlowyt_0 = 0,OpticalFlowyt_1 = 0,OpticalFlowyt_2 = 0;
//	static float OpticalFlowyPdot[9] = {0,0,0,0,0,0,0,0,0};
//	static float OpticalFlowyPP[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	
//	
//	//v(k+1) = v(k)+[a(k)-bias(k)]*dt
//	OpticalFlowy_Velocity += (Accx - OpticalFlowybias)*OpticalFlow_dt;
//	//Height(k+1) = Height(k) + 1/2[a(k)-bias(k)]dt^2 + v(k)dt
//	OpticalFlowy += (Accx - OpticalFlowybias)*OpticalFlow_dt*OpticalFlow_dt/2 + OpticalFlowy_Velocity*OpticalFlow_dt;
//	
//	OpticalFlowyPdot[0] = Q_OpticalFlowy + OpticalFlowyPP[0][1] + OpticalFlowyPP[1][0]+(OpticalFlowyPP[1][1]-OpticalFlowyPP[2][0]/2-OpticalFlowyPP[0][2]/2)*OpticalFlow_dt;
//	OpticalFlowyPdot[1] = OpticalFlowyPP[1][1]-OpticalFlowyPP[0][2]-(OpticalFlowyPP[2][1]/2+OpticalFlowyPP[1][2])*OpticalFlow_dt;
//	OpticalFlowyPdot[2] = OpticalFlowyPP[1][2]-OpticalFlowyPP[2][2]*OpticalFlow_dt/2;
//	OpticalFlowyPdot[3] = OpticalFlowyPP[1][1]-OpticalFlowyPP[2][0]-(OpticalFlowyPP[2][1]+OpticalFlowyPP[1][2]/2)*OpticalFlow_dt;
//	OpticalFlowyPdot[4] = Q_OpticalFlowyVelocity-OpticalFlowyPP[2][1]-OpticalFlowyPP[1][2]+OpticalFlowyPP[2][2]*OpticalFlow_dt;
//	OpticalFlowyPdot[5] = -OpticalFlowyPP[2][2];
//	OpticalFlowyPdot[6] = OpticalFlowyPP[2][1]-OpticalFlowyPP[2][2]*OpticalFlow_dt/2;
//	OpticalFlowyPdot[7] = -OpticalFlowyPP[2][2];
//	OpticalFlowyPdot[8] = Q_OpticalFlowybias;
//	
//	OpticalFlowyPP[0][0] += OpticalFlowyPdot[0]*OpticalFlow_dt;
//	OpticalFlowyPP[0][1] += OpticalFlowyPdot[1]*OpticalFlow_dt;
//	OpticalFlowyPP[0][2] += OpticalFlowyPdot[2]*OpticalFlow_dt;
//	OpticalFlowyPP[1][0] += OpticalFlowyPdot[3]*OpticalFlow_dt;
//	OpticalFlowyPP[1][1] += OpticalFlowyPdot[4]*OpticalFlow_dt;
//	OpticalFlowyPP[1][2] += OpticalFlowyPdot[5]*OpticalFlow_dt;
//	OpticalFlowyPP[2][0] += OpticalFlowyPdot[6]*OpticalFlow_dt;
//	OpticalFlowyPP[2][1] += OpticalFlowyPdot[7]*OpticalFlow_dt;
//	OpticalFlowyPP[2][2] += OpticalFlowyPdot[8]*OpticalFlow_dt;
//	
//	OpticalFlowyPCt_0 = OpticalFlowyC_0 *OpticalFlowyPP[0][0];				
//	OpticalFlowyPCt_1 = OpticalFlowyC_0 *OpticalFlowyPP[1][0];				
//	OpticalFlowyPCt_2 = OpticalFlowyC_0 *OpticalFlowyPP[2][0];
//	
//	OpticalFlowyE = R_OpticalFlowy + OpticalFlowyC_0 * OpticalFlowyPCt_0;		
//	
//	OpticalFlowyK_0 = OpticalFlowyPCt_0/OpticalFlowyE;						
//	OpticalFlowyK_1 = OpticalFlowyPCt_1/OpticalFlowyE;
//	OpticalFlowyK_2 = OpticalFlowyPCt_2/OpticalFlowyE;
//	
//	OpticalFlowy_err = flow_y - OpticalFlowy;
//	OpticalFlowy += OpticalFlowyK_0 * OpticalFlowy_err;		
//	OpticalFlowy_Velocity += OpticalFlowyK_1 * OpticalFlowy_err;	
//	OpticalFlowybias += OpticalFlowyK_2 * OpticalFlowy_err;	
//	
//	OpticalFlowyt_0 = OpticalFlowyPCt_0;						
//	OpticalFlowyt_1 = OpticalFlowyC_0 * OpticalFlowyPP[0][1];				
//	OpticalFlowyt_2 = OpticalFlowyC_0 * OpticalFlowyPP[0][2];				
//	
//	
//	OpticalFlowyPP[0][0] -= OpticalFlowyK_0*OpticalFlowyt_0;
//	OpticalFlowyPP[0][1] -= OpticalFlowyK_0*OpticalFlowyt_1;
//	OpticalFlowyPP[0][2] -= OpticalFlowyK_0*OpticalFlowyt_2;
//	OpticalFlowyPP[1][0] -= OpticalFlowyK_1*OpticalFlowyt_0;
//	OpticalFlowyPP[1][1] -= OpticalFlowyK_1*OpticalFlowyt_1;
//	OpticalFlowyPP[1][2] -= OpticalFlowyK_1*OpticalFlowyt_2;
//	OpticalFlowyPP[2][0] -= OpticalFlowyK_2*OpticalFlowyt_0;
//	OpticalFlowyPP[2][1] -= OpticalFlowyK_2*OpticalFlowyt_1;
//	OpticalFlowyPP[2][2] -= OpticalFlowyK_2*OpticalFlowyt_2;
//	
//	RT_Info.FlowY = OpticalFlowy;
//	RT_Info.FlowY_V  = OpticalFlowy_Velocity;








