#include "flow_deal.h"
#include "Task.h"
#include "digital_filter.h"
flosion_t flosion_Info;
//int16_t tmpX,tmpY =0;
float FlowTime;

Butter_Parameter FlowX_Parameter;
Butter_BufferData  flowX_filter_buf[3];
Butter_Parameter FlowY_Parameter;
Butter_BufferData  flowY_filter_buf[3];
Butter_Parameter RotateFlowX_Parameter;
Butter_BufferData  RotateFlowX_filter_buf[3];
Butter_Parameter RotateFlowY_Parameter;
Butter_BufferData  RotateFlowY_filter_buf[3];
void flowing_init(void)
{
//	Set_Cutoff_Frequency(200.0f, 25.0f,&RotateX_Parameter);       //光流旋转X巴特沃斯参数初始化
//	Set_Cutoff_Frequency(200.0f, 25.0f,&RotateY_Parameter);       //光流旋转Y巴特沃斯参数初始化
//	Set_Cutoff_Frequency(200.0f, 25.0f,&RotateZ_Parameter);       //光流旋转Z巴特沃斯参数初始化
	Set_Cutoff_Frequency(25.0f, 2.0f,&FlowX_Parameter);          // 光流速度巴特沃斯参数初始化
	Set_Cutoff_Frequency(25.0f, 2.0f,&FlowY_Parameter);          // 光流速度巴特沃斯参数初始化
	Set_Cutoff_Frequency(25.0f, 1.2f,&RotateFlowX_Parameter);       //光流旋转X巴特沃斯参数初始化
	Set_Cutoff_Frequency(25.0f, 1.2f,&RotateFlowY_Parameter);       //光流旋转Y巴特沃斯参数初始化
}

float whypitch_isee,whyroll_isee;
void Flow_datadeal(_Data_Rx rx)
{	
//	static int16_t tmpX,tmpY =0;
	float Zheight = 1.0f;
	float RotateScale = 230.0f;
	float RotateX,RotateY =0;
//	float FlowTime =0;
	if(rx.buf[0]==0xfe  && rx.buf[1]==0x0A && rx.buf[13]==0x55)
	{
		if(rx.buf[10]==0xf5)
		{
			flosion_Info.tmpX = ( (rx.buf[3]<<8) + rx.buf[2] ) ;		
			flosion_Info.tmpY = ( (rx.buf[5]<<8) + rx.buf[4] ) ;
			FlowTime = (float)( (int16_t)(rx.buf[7]<<8) + rx.buf[6])/1000000;
		
			Zheight = RT_Info.US100_Alt;//Z轴高度
			
			flosion_Info.FlowVelX =  -LPButterworth((float)flosion_Info.tmpY,&flowX_filter_buf[0],&FlowX_Parameter);
			flosion_Info.FlowVelY =  -LPButterworth((float)flosion_Info.tmpX,&flowY_filter_buf[0],&FlowY_Parameter) ; //-
			
//			RotateX = RotateScale * Limits_data( ((LPButterworth(RT_Info.ratePitch,&RotateFlowX_filter_buf[0],&RotateFlowX_Parameter) )* PI/180),3.0f,-3.0f);//!!!
//			RotateY = RotateScale *  Limits_data(  ((LPButterworth(RT_Info.rateRoll,&RotateFlowY_filter_buf[0],&RotateFlowY_Parameter)) * PI/180) ,3.0f,-3.0f);
//			
			flosion_Info.pitchrate = KalmanFilter(&KAL_dealinfo[KAL_flowpitchrate].info->ratepitch_flowkal,RT_Info.ratePitch);
			flosion_Info.rollrate = KalmanFilter(&KAL_dealinfo[KAL_flowrollrate].info->rateroll_flowkal,RT_Info.rateRoll);
			RotateX = RotateScale * Limits_data( ((LPButterworth(flosion_Info.pitchrate,&RotateFlowX_filter_buf[0],&RotateFlowX_Parameter) )* PI/180),3.0f,-3.0f);//!!!
			RotateY = RotateScale *  Limits_data(  ((LPButterworth(flosion_Info.rollrate,&RotateFlowY_filter_buf[0],&RotateFlowY_Parameter)) * PI/180) ,3.0f,-3.0f);
				
			flosion_Info.LPFTest1 = RotateX;
			flosion_Info.LPFTest2 = RotateY;
			
			flosion_Info.FixFlowX = ((flosion_Info.FlowVelX - flosion_Info.LPFTest1 ) /(10000 *FlowTime) )*Zheight ;
			flosion_Info.FixFlowY = (( flosion_Info.FlowVelY + flosion_Info.LPFTest2 ) /(10000 *FlowTime) )*Zheight ;
		
			if(flosion_Info.FixFlowX<10000 && flosion_Info.FixFlowY<10000)
			{				
				flosion_Info.FlowX += flosion_Info.FixFlowX * FlowTime;
				flosion_Info.FlowY += flosion_Info.FixFlowY * FlowTime;
			}
		}
		
		else
		{
			  flosion_Info.FixFlowX =0;
				flosion_Info.FixFlowY =0;
		}
			
	}

}




