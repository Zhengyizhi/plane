/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Data_PC.c
 * 描述   :上传PC端数据格式
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Data_PC.h"
#include "remote.h"
void sendParaInfo(void)
{
	u8 paraToPC[255];
	u8 floatToHex[4];	
	u8 intToHex[4];
	int i=0;

	paraToPC[0]=0X55;
	paraToPC[1]=0XAA;
	paraToPC[2]=0X02;

	/* Pitch PidPara */
	FloatToByte(remote_Info.set_pid.Pitch.Kp,floatToHex);
	arrycat(paraToPC,3,floatToHex,4);
	FloatToByte(remote_Info.set_pid.Pitch.Ki,floatToHex);
	arrycat(paraToPC,7,floatToHex,4);
	FloatToByte(remote_Info.set_pid.Pitch.Kd,floatToHex);
	arrycat(paraToPC,11,floatToHex,4);

	/* Roll PidPara */
	FloatToByte(remote_Info.set_pid.Roll.Kp,floatToHex);
	arrycat(paraToPC,15,floatToHex,4);
	FloatToByte(remote_Info.set_pid.Roll.Ki,floatToHex);
	arrycat(paraToPC,19,floatToHex,4);
	FloatToByte(remote_Info.set_pid.Roll.Kd,floatToHex);
	arrycat(paraToPC,23,floatToHex,4);

	/* Yaw PidPara */
	FloatToByte(remote_Info.set_pid.x_outflowing.Kp,floatToHex);
	arrycat(paraToPC,27,floatToHex,4);
	FloatToByte(remote_Info.set_pid.x_outflowing.Ki,floatToHex);
	arrycat(paraToPC,31,floatToHex,4);
	FloatToByte(remote_Info.set_pid.x_outflowing.Kd,floatToHex);
	arrycat(paraToPC,35,floatToHex,4);
	
	
	/* Height PidPara */
	FloatToByte(remote_Info.set_pid.x_flowing.Kp,floatToHex);
	arrycat(paraToPC,39,floatToHex,4);
	FloatToByte(remote_Info.set_pid.x_flowing.Ki,floatToHex);
	arrycat(paraToPC,43,floatToHex,4);
	FloatToByte(remote_Info.set_pid.x_flowing.Kd,floatToHex);
	arrycat(paraToPC,47,floatToHex,4);
			
	
	/* ratePitch PidPara */
	FloatToByte(remote_Info.set_pid.ratePitch.Kp,floatToHex);
	arrycat(paraToPC,51,floatToHex,4);
	FloatToByte(remote_Info.set_pid.ratePitch.Ki,floatToHex);
	arrycat(paraToPC,55,floatToHex,4);
	FloatToByte(remote_Info.set_pid.ratePitch.Kd,floatToHex);
	arrycat(paraToPC,59,floatToHex,4);

	/* rateRoll PidPara */
	FloatToByte(remote_Info.set_pid.rateRoll.Kp,floatToHex);
	arrycat(paraToPC,63,floatToHex,4);
	FloatToByte(remote_Info.set_pid.rateRoll.Ki,floatToHex);
	arrycat(paraToPC,67,floatToHex,4);
	FloatToByte(remote_Info.set_pid.rateRoll.Kd,floatToHex);
	arrycat(paraToPC,71,floatToHex,4);

	/* rateYaw PidPara */
	FloatToByte(remote_Info.set_pid.rateYaw.Kp,floatToHex);
	arrycat(paraToPC,75,floatToHex,4);
	FloatToByte(remote_Info.set_pid.rateYaw.Ki,floatToHex);
	arrycat(paraToPC,79,floatToHex,4);
	FloatToByte(remote_Info.set_pid.rateYaw.Kd,floatToHex);
	arrycat(paraToPC,83,floatToHex,4);

	
	/* accHeight PidPara */
	FloatToByte(remote_Info.set_pid.accHeight.Kp,floatToHex);
	arrycat(paraToPC,87,floatToHex,4);
	FloatToByte(remote_Info.set_pid.accHeight.Ki,floatToHex);
	arrycat(paraToPC,91,floatToHex,4);
	FloatToByte(remote_Info.set_pid.accHeight.Kd,floatToHex);
	arrycat(paraToPC,95,floatToHex,4);
	

	for(i=99;i<157;i++)
	{
		paraToPC[i]=0;
	}	
	
	IntToByte(0.0,intToHex);
	arrycat(paraToPC,157,intToHex,2);
	
	for(i=0;i<159;i++)
	{
		paraToPC[159]+=paraToPC[i];
	}
	
	if(USE_BLUETOOTH)
		Uart3_tx(paraToPC,160);
	else
		Uart1_tx(paraToPC,160);
		
}


/*上传实时信息*/
void sendRTInfo(void)
{
  float temp;
	u8 floatToHex[4];		
	u8 dataToPC[64];	
	u8 i=0;

	dataToPC[0]=0X55;
	dataToPC[1]=0XAA;
	dataToPC[2]=0X01;
		
	temp = RT_Info.Pitch - Errangle_Info.fixedErroPitch;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,3,floatToHex,4);
	
	temp = RT_Info.Roll - Errangle_Info.fixedErroRoll;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,7,floatToHex,4);
	
	temp = RT_Info.Yaw;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,11,floatToHex,4);
	
	temp = RT_Info.US100_Alt;
	//temp = RT_Info.FlowX_V;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,15,floatToHex,4);

	temp = RT_Info.batteryVoltage;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,19,floatToHex,4);
	
	temp = RT_Info.FlowY_V * 100;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,23,floatToHex,4);
	
	for(i=27;i<47;i++)
	{
		dataToPC[i]=0;
	}
	for(i=0;i<47;i++)
	{
		dataToPC[47]+=dataToPC[i];
	}
	
	if(USE_BLUETOOTH)
		Uart3_tx(dataToPC,48);
	else
		Uart1_tx(dataToPC,48);
}
