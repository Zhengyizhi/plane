#include "all_task.h"

////////////////////////////////////////
char dingA,gohome;



////////////////////////////////////
void flowing_task(void);
void find_A_task(void);
void walking_on_line_3task(void);
void walking_on_line_4task(void);
void stay_at_line_5task(void);
void walking_on_line_6task(void);
void walking_on_line_7task(void);
void starting_task()
{
	if(waiting_time.centering.end_timing != count_dowm_2)
	{
		Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
		Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
		if(dingyuan == 1)//阶段1
		{
			if( waiting_time.centering.start_flag == start_false)
			{
				delay_ms_init(&waiting_time.centering);
			}
			delay_ms_timer(2000,&waiting_time.centering);
		}
	}
	else//已经完成2s悬停
	{
		flowing_task();
	}
}
void flowing_task(void)
{
	if(waiting_time.centering_2.end_timing != count_dowm_2)
	{
		static int left_shift_cishu = 0;
		left_shift_cishu++;
		if(left_shift_cishu<=2117)
			Target_Info.Y_point += 0.2f;//0.17差不多1m
		else if(left_shift_cishu>=2117 && waiting_time.centering_2.start_flag == start_false)//到达目标位置
		{
			left_shift_cishu = 3000;
			delay_ms_init(&waiting_time.centering_2);
		}
		delay_ms_timer(10000,&waiting_time.centering_2);//定10s
	}
	else
	{
		find_A_task();
	}
}
void find_A_task(void)
{
	if(waiting_time.find_A.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 2)//确定那边收到我的2了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
			if(dingA == 1)
			{
				if( waiting_time.find_A.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.find_A);
				}
				delay_ms_timer(2000,&waiting_time.find_A);//定住2s
			}
		}
	}
	else
	{
		if(RT_Info.US100_Alt >= 1.0f)//后面删掉
		walking_on_line_3task();
	}
}

void walking_on_line_3task(void)
{
	if(waiting_time.turn_90_1.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 3)//确定那边收到我的3了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//前进
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(offend_left);
			yaw_control(angel_left);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 20 )//告诉我要旋转
		{
			if(testyaw.turn_angle == level_0)//转完 了
			{
				if( waiting_time.turn_90_1.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_1);
				}
				delay_ms_timer(5000,&waiting_time.turn_90_1);//停止5s
			}
		}
	}
	else
	{
		walking_on_line_4task();
	}
}

void walking_on_line_4task(void)
{
	if(waiting_time.turn_90_2.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 4)//确定那边收到我的4了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//前进
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 19 )//告诉我要旋转
		{
			if(testyaw.turn_angle == level_0)//转完 了
			{
				if( waiting_time.turn_90_2.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_2);
				}
				delay_ms_timer(500,&waiting_time.turn_90_2);//停止0.5s
			}
		}
	}
	else
	{
		stay_at_line_5task();
	}
}

void stay_at_line_5task(void)//定线
{
	if(waiting_time.stay_line.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 5)//确定那边收到我的5了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
			yaw_control(track_turnangle);
			if( waiting_time.stay_line.start_flag == start_false)
			{
				delay_ms_init(&waiting_time.stay_line);
			}
			delay_ms_timer(5000,&waiting_time.stay_line);//停止5s
		}
	}
	else
	{
		walking_on_line_6task();
	}
}
void walking_on_line_6task(void)
{
	if(waiting_time.turn_90_3.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 6)//确定那边收到我的6了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//前进
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 18 )//告诉我要旋转
		{
	//		ttestyaw(90);//只转了一次
			if(testyaw.turn_angle == level_0)//转完 了
			{
				if( waiting_time.turn_90_3.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_3);
				}
				delay_ms_timer(5000,&waiting_time.turn_90_3);//停止0.5s
			}
		}
	}
	else
	{
		walking_on_line_7task();
	}
}

void walking_on_line_7task(void)//
{
	if(waiting_time.go_home.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 7)//确定那边收到我的7了才使用他们的数据
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//前进
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
		}
		else if(fromslave_data.step_level == 8 )//走到终点了给我发后退
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X - 200;//后退
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
		}
		else if(fromslave_data.step_level == 9)
		{
			Target_Info.Y_point = RT_Info.Flow_caculate_Y + 200;//纯光流左移
		}
		else if(fromslave_data.step_level == 10)
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
			if(gohome == 1)
			{
				if( waiting_time.go_home.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.go_home);
				}
				delay_ms_timer(3000,&waiting_time.go_home);//停止3
			}
		}
	}
	else//降落
	{
		if(Target_Info.Height >= 0.2f)
		{
				Target_Info.Height-=0.001f;
		}
		else if(Target_Info.Height<=0.2f && RT_Info.US100_Alt<=0.3f)
		{
				remote_Info.flight_switches = drone_off;
			
		}
	
	}

}




void deal_all_task(void)
{
	if(fromslave_data.step_level == 1 && waiting_time.centering_2.end_timing == count_dowm_2)//第一阶段接收数据优先级最高
	{
		Master_to_slave.Data.step_num = 2;//到了阶段2
	}
	else if(fromslave_data.step_level == 1 && waiting_time.centering_2.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段1
	}
	////////////////
	else if(fromslave_data.step_level == 2 && waiting_time.find_A.end_timing == count_dowm_2) 
	{
		Master_to_slave.Data.step_num = 3;//定完A到了3阶段
	}
	else if(fromslave_data.step_level == 2 && waiting_time.find_A.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段
	}
	////////////////
	else if((fromslave_data.step_level == 3 || fromslave_data.step_level == 20 )&& waiting_time.turn_90_1.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 4;//已经转完第一个转角
	}
	else if(fromslave_data.step_level == 3 && waiting_time.turn_90_1.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段
	}
	////////////////////
	else if((fromslave_data.step_level == 4 || fromslave_data.step_level == 19 )&& waiting_time.turn_90_2.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 5;//已经转完第二个转角
	}
	else if(fromslave_data.step_level == 4 && waiting_time.turn_90_2.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段
	}
	////////////////////
	else if(fromslave_data.step_level == 5 && waiting_time.stay_line.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 6;//已经定完线了
	}
	else if(fromslave_data.step_level == 5 && waiting_time.stay_line.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段
	}
	/////////////////////////
	else if((fromslave_data.step_level == 6 || fromslave_data.step_level == 18 )&& waiting_time.turn_90_3.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 7;//已经转完第三个转角了
	}
	else if(fromslave_data.step_level == 6 && waiting_time.turn_90_3.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//发送随阶段
	}
	else if((fromslave_data.step_level == 7 || fromslave_data.step_level == 8 || fromslave_data.step_level == 9 || fromslave_data.step_level == 10)
		&& waiting_time.go_home.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;
	}
	
}
///////////////////////////////
float X_ppoint_array[20],Y_ppoint_array[20];
void deal_data_step1(void)//调试看这里
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18放19的
		{
			X_ppoint_array[i] = X_ppoint_array[i+1];
			Y_ppoint_array[i] = Y_ppoint_array[i+1];
		}
		X_ppoint_array[19] = abs(Pix_Xinfo);
		Y_ppoint_array[19] = abs(Pix_Yinfo);
		if(j>=20)
		{
			j=21;
			for(int i = 0;i<20;i++)
			{
				j_sumx += X_ppoint_array[i] * 0.05f;
				j_sumy += Y_ppoint_array[i] * 0.05f;
			}
			if(j_sumx<=7.0f && j_sumy <=7.0f)
			{
				dingyuan = 1;
			}
		}
	}
}

void deal_data_step2(void)//调试看这里
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18放19的
		{
			X_ppoint_array[i] = X_ppoint_array[i+1];
			Y_ppoint_array[i] = Y_ppoint_array[i+1];
		}
		X_ppoint_array[19] = abs(Pix_Xinfo);
		Y_ppoint_array[19] = abs(Pix_Yinfo);
		if(j>=20)
		{
			j=21;
			for(int i = 0;i<20;i++)
			{
				j_sumx += X_ppoint_array[i] * 0.05f;
				j_sumy += Y_ppoint_array[i] * 0.05f;
			}
			if(j_sumx<=7.0f && j_sumy <=7.0f)
			{
				dingA = 1;
			}
		}
	}
}

void deal_data_step10(void)//调试看这里
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18放19的
		{
			X_ppoint_array[i] = X_ppoint_array[i+1];
			Y_ppoint_array[i] = Y_ppoint_array[i+1];
		}
		X_ppoint_array[19] = abs(Pix_Xinfo);
		Y_ppoint_array[19] = abs(Pix_Yinfo);
		if(j>=20)
		{
			j=21;
			for(int i = 0;i<20;i++)
			{
				j_sumx += X_ppoint_array[i] * 0.05f;
				j_sumy += Y_ppoint_array[i] * 0.05f;
			}
			if(j_sumx<=7.0f && j_sumy <=7.0f)
			{
				gohome = 1;
			}
		}
	}
}



