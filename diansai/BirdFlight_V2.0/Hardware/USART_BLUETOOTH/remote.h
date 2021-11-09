#ifndef __Remote_H
#define __Remote_H
#include "DronePara.h"
typedef unsigned char _bool;
typedef enum{
	out_of_control = 0,
	control_command = 0xff,
	control_angle = 0x01,
	control_test = 0x02,
	read_parameter = 0x03,
	calibrate_magnetometer = 0x04,
	control_adjust = 0x0A,
	
	control_pitch_parameter = 0x05,
	control_roll_parameter = 0x06,
	control_yaw_parameter = 0x07,
	control_height_parameter = 0x08,
	control_ratepitch_parameter = 0x0B,
	control_rateroll_parameter = 0x0c,
	control_rateyaw_parameter = 0x0d,
	control_accheight_parameter = 0x0e,
	
	control_rate_target = 0x0f,
	control_angle_target = 0x11//17

}remote_mode_bit2;


typedef enum{
	test_none = 0,
	test_pitch = 1,
	test_roll = 2,
	test_axisbalance = 3,
	test_ratepitch = 4,
	test_rateroll = 5,
}drone_mode_control_test;

typedef enum{
	drone_off = 0,
	drone_on = 1,
	drone_land = 2,
	have_drone_land = 3,//已经降落
	err_drone_data = 4
}flight_switches_state;

typedef struct{
	remote_mode_bit2 state;
	drone_mode_control_test testtype;
	flight_switches_state flight_switches;
	float set_pitch_goal;
	float set_roll_goal;
	float set_yaw_goal;
	float set_rateyaw_goal;
	float set_ratepitch_goal;
	float set_rateroll_goal;
	float set_height_goal;
	DronePIDPara set_pid;

	//调光流外环
	float X_point_goal;
	float Y_point_goal;
	
	float x_velocity;
	float y_velocity;
}remote_data_t;

extern remote_data_t remote_Info;

#endif
