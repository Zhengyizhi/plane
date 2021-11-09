#ifndef __usart2_see_H
#define  __usart2_see_H

#include "stm32f4xx.h"
#include "All_Task.h"
void Usart2toUltra_Init(u32 Bound);
//void ReceiveUltraData_try(void);
void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll);
//*********************************************************************************
#include "stdbool.h"
#include "stm32f4xx.h"
#define Judge_HaedFrame 0xAA
//void Usart2toUltra_Init(u32 Bound);
//void ReceiveUltraData_try(void);
//void SendStopflag_try(void);
//void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll);

//extern float try_Arr[6];
////////////////////////////
enum IDcontrol_info_t{
	SOF			= 0,
	DATA_LENGTH	= 1,
	CRC8		= 2,
	length_of_header = 3,
	////////////////////////////////
	CMD_ID = 3,
	length_of_CMD_ID = 1,
	
	ID_sendRT_Info = 1,
	ID_receive_slavedata = 10,
	ID_receive_tracking = 11,//循地面的迹
	ID_turnstile = 13,//绕杆
//不是这个
	ID_bottomvision_Info = 2,
	ID_send_tomaster = 3,
	
	/////////////////////////////////
	length_of_Frame_tail = 2,
	
};



typedef __packed struct
{
	uint8_t  sof;
	uint8_t  data_length;
	uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD
extern int16_t angel_left,offend_left;
typedef __packed struct
{
	float pitch;
	float roll;
	float yaw;
	
	float pix_X;
	float pix_Y;
	
	float Target_X_point;
	float Flow_caculate_X;
	float Target_Y_point;
	float Flow_caculate_Y;
	
	float kp_out;
	float ki_out;
	float kd_out;
	float feed_out;
	
	char step_num;
}mastertoslave_t;


typedef __packed struct
{
	std_frame_header_t txFrameHeader;			//帧头
	uint8_t  CmdID;										//命令码
	mastertoslave_t Data;//数据段
	uint16_t	FrameTail;								//帧尾
}mastertoslave_data_t;
extern mastertoslave_data_t Master_to_slave;

//void Send_UltraData(void);
void send_Master_to_slave(void);

/////////////////////////////////////////////以下是接收信息

typedef __packed struct
{
	uint16_t x_position;
	uint16_t y_position;
}slavetomaster_t;

typedef __packed struct
{
	int16_t turn_angle;
	int16_t offset_distance;
	uint8_t bool_t;
}tracking_t;

typedef __packed struct
{
	int16_t distance;
	int16_t turn_angle;
	uint8_t  bool_t;
}turnstile_t;//绕杆

typedef struct Masterslave_info_struct{
	std_frame_header_t frame_header;
	slavetomaster_t    vision_data;
	slavetomaster_t    pre_vision_data;
	tracking_t         track_vision_data;
	turnstile_t        turnstile_data;
	bool data_valid;
	bool err_cnt;
}Masterslave_info_struct;
extern Masterslave_info_struct master_slave;
void Uart2_Init(u32 Bound);
extern char   dingyuan;

//////////////接收所有数据///////////////????????????????????????????????????///////////////////////////
typedef enum operation_code{//命令码
	ID_under_bottom_centering = 1,
	ID_under_bottom_sow = 2,
	ID_under_bottom_stayline = 5,
	
	ID_bottom_left_line_walkingtrue = 3,
	ID_bottom_left_line_walkingwrong = 4,
}operation_code;
//左下巡线
typedef __packed struct
{
	uint16_t turn_angle;//-90
	uint16_t offset_distance;//-160
	uint8_t step_receive;
}line_walking_t;
//正下
typedef __packed struct
{
	uint16_t x_position;
	uint16_t y_position;
	uint8_t step_receive;
}centering_t;
typedef __packed struct
{
	uint8_t sow_where;
	uint8_t step_receive;
}sowing_t;
typedef __packed struct
{
	uint16_t turn_angle;//-90
	uint16_t x_offset_distance;//-160
	uint16_t y_offset_distance;//-120
	uint8_t step_receive;
} stay_line_t;//定在直线上

typedef struct receive_alldata_from_slave{
	std_frame_header_t frame_header;
	line_walking_t  line_walking_leftbottom_data;
	
	centering_t     centering_underbottom_data;
	sowing_t        sowing_underbottom_data;
	stay_line_t     stay_line_data;
	
	bool data_valid;
	uint8_t         step_level;
}receive_alldata_from_slave;

extern receive_alldata_from_slave  fromslave_data;


extern volatile float track_turnangle;


//extern float try_Arr[6];
#endif
