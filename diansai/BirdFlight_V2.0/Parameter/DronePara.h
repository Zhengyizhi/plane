#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"
#include "stdbool.h"
#define TX_LEN  512
#define RX_LEN  512

typedef enum
{ 
	Drone_Mode_None=0,
  Drone_Mode_Pitch= 1,//�⻷ʵ��	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, //�ڻ�ʵ��
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum
{  
	Drone_Off  = 0x00,//��ɻ��ߵ��Դ򿪵��
  Drone_On   = 0x01,//�رյ��
  Drone_Land = 0x02,//����	
}DroneFlightOnOff_TypeDef;

typedef enum
{  
	Report_SET      = 0x01,
  Report_RESET    = 0x00, 		 	
}DroneReportSW_TypeDef;

typedef struct{
	bool bottomvision_data;
}task_flag;
extern task_flag all_flag;

typedef struct
{
	DroneFlightOnOff_TypeDef OnOff;
	DroneFlightMode_TypeDef droneMode;
	DroneReportSW_TypeDef ReportSW;
	int landFlag;
}DroneFlightControl;


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float Height;
	float AccHeight;
	
	float X_point;
	float Y_point;
	
}DroneTargetInfo;


typedef struct
{
	float feedforwardOut;//ǰ���������
	float pOut;
  float iOut;
	float dOut;
	float value;//PID�������
	float target;
}PIDOut;

typedef struct
{
	float Kp;
	float Ki;
	float	Kd;
}PID;

typedef struct
{
	PID Pitch;
	PID Roll;
//	PID Yaw;
	PID x_outflowing;
 	PID x_flowing;
	
	PID ratePitch;
	PID rateRoll;
	PID rateYaw;
	PID accHeight;
	
//	PID x_flowing;
//	PID x_outflowing;
//	PID y_flowing;
}DronePIDPara;

/* ƽ��������� */
typedef struct
{
	float fixedErroPitch;
	float fixedErroRoll;
}DroneErrangle;

/*ң����Ϣ*/
typedef struct
{
	float pV;//pitch�ٶ�
  float rV;//roll�ٶ�
	float hV;//�����ٶ�
	float yV;//�����ٶ�
}Controller;

typedef enum
{
	level_0 = 0,
	level_1 = 1,
	level_2 = 2,
	level_3 = 3,
}level_t;


typedef struct
{
	level_t turn_angle;
	level_t vision_tracing;
	level_t stile_angle;//�Ƹ�
}testyaw_t;
extern testyaw_t testyaw;
/*���˻�ʵʱ��Ϣ*/
typedef struct{
	float imu_center;
	float imu_angle_sum;
	float imu_angle_pre;
}longtime_t;

typedef enum{
	yaw_longtime,
	IMU_CNT,
}imu_cnt_t;

typedef struct
{
	float Pitch; 
	float Roll;
  float Yaw;
	float batteryVoltage;
  float	ratePitch;			//Pitch��Ľ��ٶ�
	float rateRoll;				//Roll��Ľ��ٶ�
	float rateYaw;				//Yaw��Ľ��ٶ�
	float accratex;//��ͷ����
	float accratey;
	float accratez;
	float US100_Alt;			//US100�������߶�
	float US100_Alt_V;		//US100�������ٶ�
	float FlowX;					//����λ��X
	float FlowY;          //����λ��Y
	float FlowX_V;				//�����ٶ�X
	float FlowY_V;				//�����ٶ�Y
	float PointX;					//������Xλ��
	float PointY;					//������Yλ��
	float PointX_V;				//������X���ٶ�
	float PointY_V;				//������Y���ٶ�
	int lowPowerFlag;			//�͵�ѹ��־λ
	float Flow_caculate_X;
	float Flow_caculate_Y;
	
	bool IMU_initflag;
	longtime_t Longtime_imu_Array[IMU_CNT];
}DroneRTInfo;

typedef struct{
	int16_t tmpX;
	int16_t tmpY;
	float FlowX;
	float FlowY;
	float FlowVelX;
	float FlowVelY;
	
	float pitchrate;
	float rollrate;
	
	float FixFlowX;
	float FixFlowY;
	
	float LPFTest1;//�˲�ǰ������
	float LPFTest2;
}flosion_t;//����
//extern flosion_t flosion_Info;


typedef struct
{
	unsigned int M1;
	unsigned int M2;
	unsigned int M3;
	unsigned int M4;
}Throttle;

typedef struct
{
	unsigned int len;
	unsigned char buf[256];
}_Data_Rx;

#endif

