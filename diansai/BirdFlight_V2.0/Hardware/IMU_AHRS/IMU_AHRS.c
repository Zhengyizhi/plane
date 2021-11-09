/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :IMU_AHRS.c
 * ����   :IMUԭʼ���ݶ�ȡ�������ں�
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "IMU_AHRS.h"
#include "MPU6500.h"
#include "LSM303D.h"
#include "explore_systick.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include <math.h>
#include "Task.h"

// asin����
float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0f;
	}
	if (v >= 1.0f) {
		return PI/2;
	}
	if (v <= -1.0f) {
		return -PI/2;
	}
	return asin(v);
}

void IMU_HardwareInit()
{
//	LSM303_Initial();           //lsm303�Ĵ�������
//	delay_ms(100);
	MPU6500_initialize();					//mpu6500�Ĵ�������
	delay_ms(100);
	MPU6500_Init_Offset();    		//6500��ʼ��У׼��ƽ���˲���
	delay_ms(100);
}
//int16_t accgyroval[9];
//int isee_1,isee_2,isee_3,isee_4,isee_5,isee_6,isee_7,isee_8,isee_9;
void IMU_getValues(float * values) 
{
//  isee_1 = accgyroval[0];
//	isee_2 = accgyroval[1];
//	isee_3 = accgyroval[2];
//	isee_4 = accgyroval[3];
//	isee_5 = accgyroval[4];
//	isee_6 = accgyroval[5];
//	isee_7 = accgyroval[6];
//	isee_8 = accgyroval[7];
//	isee_9 = accgyroval[8];
	int16_t accgyroval[9];
	int i;
	//��ȡ�����Ƶ�ԭʼֵ
	LSM303_readMag(&accgyroval[6]);	
	//��ȡ���ٶȼƺ������ǵ�ԭʼֵ
	MPU6500_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
	for(i = 0; i<6; i++) 
	{
    if(i < 3) 
		{
  			values[i] = (float) accgyroval[i];
    }
		//�����ǵ�λת��
    else 
		{
        values[i] = ((float) accgyroval[i]) / 16.4f;
    }
  }	
	values[6] = (float)accgyroval[6];
	values[7] = (float)accgyroval[7];
	values[8] = (float)accgyroval[8];
}

volatile float RDrone_R[3][3];
volatile float RDrone_R2[3][3];
volatile float Acc_Flow_x;
volatile float Acc_Flow_y;
volatile float Acc_Flow_z;
//float Accel_Src[3];
//float isee1,isee2,isee3;
void check_longtimeangle(void);
void IMU_getInfo()
{
	static float q[4];
	static float getValue[9];
	static float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	IMU_getValues(getValue);										//��ȡԭʼ����
	MahonyAHRSupdateIMU(getValue[3] * PI/180, getValue[4] * PI/180, getValue[5] * PI/180,
  getValue[0], getValue[1], getValue[2]);			//Mahony�㷨�ں�����
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
	
	//ʹ�þ����ʱ����Կ���ʹ��
	q0q0 = q[0]*q[0];
	q0q1 = q[0]*q[1];
	q0q2 = q[0]*q[2];
	q0q3 = q[0]*q[3];
	q1q1 = q[1]*q[1];
	q1q2 = q[1]*q[2];
	q1q3 = q[1]*q[3];
	q2q2 = q[2]*q[2];
	q2q3 = q[2]*q[3];
	q3q3 = q[3]*q[3]; 
	//�����ں�ʹ�õľ���
	RDrone_R2[0][0] = 2*(q0q0+q1q1)-1;
	RDrone_R2[0][1] = 2*(q1q2-q0q3);
	RDrone_R2[0][2] = 2*(q1q3+q0q2);
	RDrone_R2[1][0] = 2*(q1q2+q0q3);		
	RDrone_R2[1][1] = 2*(q0q0+q2q2)-1;
	RDrone_R2[1][2] = 2*(q2q3-q0q1);
	RDrone_R2[2][0] = 2*(q1q3-q0q2);
	RDrone_R2[2][1] = 2*(q2q3+q0q1);
	RDrone_R2[2][2] = 2*(q0q0+q3q3)-1;
	
	RDrone_R[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
	RDrone_R[0][1] = 2.f * (q1q2 + q0q3);
	RDrone_R[0][2] = 2.f * (q1q3 - q0q2);
	RDrone_R[1][0] = 2.f * (q1q2 - q0q3);		
	RDrone_R[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
	RDrone_R[1][2] = 2.f * (q2q3 + q0q1);
	RDrone_R[2][0] = 2.f * (q1q3 + q0q2);
	RDrone_R[2][1] = 2.f * (q2q3 - q0q1);
	RDrone_R[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
	//���ٶ�����
	RT_Info.accratex= getValue[0] / 16384;//Accel_Src[1]
	RT_Info.accratey = getValue[1] / 16384;
	RT_Info.accratez = (getValue[2] / 16384);

	//���ٶ�����
	RT_Info.rateRoll=getValue[3]*100; 
	RT_Info.ratePitch=getValue[4]*100;
	RT_Info.rateYaw=getValue[5]*100;
	//�Ƕ�����
	RT_Info.Roll = (atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),
                       1 - 2.0f*(q[1]*q[1] + q[2]*q[2])))* 180/PI;										 
	RT_Info.Pitch = -safe_asin(2.0f*(q[0]*q[2] - q[1]*q[3]))* 180/PI;
	//�ںϴ�����
	RT_Info.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/PI; // yaw
	check_longtimeangle();
}


void check_longtimeangle()
{
	static int i;
	float err;
	longtime_t *yaw_info_longtime = &(RT_Info.Longtime_imu_Array[yaw_longtime]);
	i++;
	if(i>=500)
	{
		if(!RT_Info.IMU_initflag)
		{
			RT_Info.IMU_initflag = true;
			yaw_info_longtime->imu_angle_pre = RT_Info.Yaw;
			yaw_info_longtime->imu_angle_sum = 0.f;
		}
		err =  RT_Info.Yaw - yaw_info_longtime->imu_angle_pre;
		if(abs(err)>200)
		{
			/*-170->��180->+160,160+170,sumΪ����*/
			if(err>=0)
				yaw_info_longtime->imu_angle_sum += -360+err;
			else
			/*sumΪ����*/
				yaw_info_longtime->imu_angle_sum += 360+err;
		}
		else
			yaw_info_longtime->imu_angle_sum+=err;
		yaw_info_longtime->imu_angle_pre = RT_Info.Yaw;
		i=501;
	}
}



