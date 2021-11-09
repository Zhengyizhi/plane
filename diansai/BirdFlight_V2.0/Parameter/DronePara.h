#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"
#include "stdbool.h"
#define TX_LEN  512
#define RX_LEN  512

typedef enum
{ 
	Drone_Mode_None=0,
  Drone_Mode_Pitch= 1,//外环实验	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, //内环实验
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum
{  
	Drone_Off  = 0x00,//起飞或者调试打开电机
  Drone_On   = 0x01,//关闭电机
  Drone_Land = 0x02,//降落	
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
	float feedforwardOut;//前馈控制输出
	float pOut;
  float iOut;
	float dOut;
	float value;//PID控制输出
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

/* 平地误差数据 */
typedef struct
{
	float fixedErroPitch;
	float fixedErroRoll;
}DroneErrangle;

/*遥控信息*/
typedef struct
{
	float pV;//pitch速度
  float rV;//roll速度
	float hV;//上升速度
	float yV;//航向速度
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
	level_t stile_angle;//绕杆
}testyaw_t;
extern testyaw_t testyaw;
/*无人机实时信息*/
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
  float	ratePitch;			//Pitch轴的角速度
	float rateRoll;				//Roll轴的角速度
	float rateYaw;				//Yaw轴的角速度
	float accratex;//插头方向
	float accratey;
	float accratez;
	float US100_Alt;			//US100超声波高度
	float US100_Alt_V;		//US100超声波速度
	float FlowX;					//光流位置X
	float FlowY;          //光流位置Y
	float FlowX_V;				//光流速度X
	float FlowY_V;				//光流速度Y
	float PointX;					//点数据X位移
	float PointY;					//点数据Y位移
	float PointX_V;				//点数据X的速度
	float PointY_V;				//点数据Y的速度
	int lowPowerFlag;			//低电压标志位
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
	
	float LPFTest1;//滤波前的数据
	float LPFTest2;
}flosion_t;//优象
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

