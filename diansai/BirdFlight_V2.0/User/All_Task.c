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
		if(dingyuan == 1)//�׶�1
		{
			if( waiting_time.centering.start_flag == start_false)
			{
				delay_ms_init(&waiting_time.centering);
			}
			delay_ms_timer(2000,&waiting_time.centering);
		}
	}
	else//�Ѿ����2s��ͣ
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
			Target_Info.Y_point += 0.2f;//0.17���1m
		else if(left_shift_cishu>=2117 && waiting_time.centering_2.start_flag == start_false)//����Ŀ��λ��
		{
			left_shift_cishu = 3000;
			delay_ms_init(&waiting_time.centering_2);
		}
		delay_ms_timer(10000,&waiting_time.centering_2);//��10s
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
		if(fromslave_data.step_level == 2)//ȷ���Ǳ��յ��ҵ�2�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
			if(dingA == 1)
			{
				if( waiting_time.find_A.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.find_A);
				}
				delay_ms_timer(2000,&waiting_time.find_A);//��ס2s
			}
		}
	}
	else
	{
		if(RT_Info.US100_Alt >= 1.0f)//����ɾ��
		walking_on_line_3task();
	}
}

void walking_on_line_3task(void)
{
	if(waiting_time.turn_90_1.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 3)//ȷ���Ǳ��յ��ҵ�3�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(offend_left);
			yaw_control(angel_left);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 20 )//������Ҫ��ת
		{
			if(testyaw.turn_angle == level_0)//ת�� ��
			{
				if( waiting_time.turn_90_1.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_1);
				}
				delay_ms_timer(5000,&waiting_time.turn_90_1);//ֹͣ5s
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
		if(fromslave_data.step_level == 4)//ȷ���Ǳ��յ��ҵ�4�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 19 )//������Ҫ��ת
		{
			if(testyaw.turn_angle == level_0)//ת�� ��
			{
				if( waiting_time.turn_90_2.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_2);
				}
				delay_ms_timer(500,&waiting_time.turn_90_2);//ֹͣ0.5s
			}
		}
	}
	else
	{
		stay_at_line_5task();
	}
}

void stay_at_line_5task(void)//����
{
	if(waiting_time.stay_line.end_timing != count_dowm_2)
	{
		if(fromslave_data.step_level == 5)//ȷ���Ǳ��յ��ҵ�5�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X -( Pix_Yinfo * 3.0f);
			Target_Info.Y_point = RT_Info.Flow_caculate_Y - Pix_Xinfo * 3.0f;
			yaw_control(track_turnangle);
			if( waiting_time.stay_line.start_flag == start_false)
			{
				delay_ms_init(&waiting_time.stay_line);
			}
			delay_ms_timer(5000,&waiting_time.stay_line);//ֹͣ5s
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
		if(fromslave_data.step_level == 6)//ȷ���Ǳ��յ��ҵ�6�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
			testyaw.turn_angle = level_2;
		}
		else if(fromslave_data.step_level == 18 )//������Ҫ��ת
		{
	//		ttestyaw(90);//ֻת��һ��
			if(testyaw.turn_angle == level_0)//ת�� ��
			{
				if( waiting_time.turn_90_3.start_flag == start_false)
				{
					delay_ms_init(&waiting_time.turn_90_3);
				}
				delay_ms_timer(5000,&waiting_time.turn_90_3);//ֹͣ0.5s
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
		if(fromslave_data.step_level == 7)//ȷ���Ǳ��յ��ҵ�7�˲�ʹ�����ǵ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X + 200;//ǰ��
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
		}
		else if(fromslave_data.step_level == 8 )//�ߵ��յ��˸��ҷ�����
		{
			Target_Info.X_point = RT_Info.Flow_caculate_X - 200;//����
			Target_Info.Y_point = RT_Info.Flow_caculate_Y -(fromslave_data.line_walking_leftbottom_data.offset_distance * 5.0f);
			yaw_control(track_turnangle);
		}
		else if(fromslave_data.step_level == 9)
		{
			Target_Info.Y_point = RT_Info.Flow_caculate_Y + 200;//����������
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
				delay_ms_timer(3000,&waiting_time.go_home);//ֹͣ3
			}
		}
	}
	else//����
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
	if(fromslave_data.step_level == 1 && waiting_time.centering_2.end_timing == count_dowm_2)//��һ�׶ν����������ȼ����
	{
		Master_to_slave.Data.step_num = 2;//���˽׶�2
	}
	else if(fromslave_data.step_level == 1 && waiting_time.centering_2.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�1
	}
	////////////////
	else if(fromslave_data.step_level == 2 && waiting_time.find_A.end_timing == count_dowm_2) 
	{
		Master_to_slave.Data.step_num = 3;//����A����3�׶�
	}
	else if(fromslave_data.step_level == 2 && waiting_time.find_A.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�
	}
	////////////////
	else if((fromslave_data.step_level == 3 || fromslave_data.step_level == 20 )&& waiting_time.turn_90_1.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 4;//�Ѿ�ת���һ��ת��
	}
	else if(fromslave_data.step_level == 3 && waiting_time.turn_90_1.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�
	}
	////////////////////
	else if((fromslave_data.step_level == 4 || fromslave_data.step_level == 19 )&& waiting_time.turn_90_2.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 5;//�Ѿ�ת��ڶ���ת��
	}
	else if(fromslave_data.step_level == 4 && waiting_time.turn_90_2.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�
	}
	////////////////////
	else if(fromslave_data.step_level == 5 && waiting_time.stay_line.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 6;//�Ѿ���������
	}
	else if(fromslave_data.step_level == 5 && waiting_time.stay_line.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�
	}
	/////////////////////////
	else if((fromslave_data.step_level == 6 || fromslave_data.step_level == 18 )&& waiting_time.turn_90_3.end_timing == count_dowm_2)
	{
		Master_to_slave.Data.step_num = 7;//�Ѿ�ת�������ת����
	}
	else if(fromslave_data.step_level == 6 && waiting_time.turn_90_3.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;//������׶�
	}
	else if((fromslave_data.step_level == 7 || fromslave_data.step_level == 8 || fromslave_data.step_level == 9 || fromslave_data.step_level == 10)
		&& waiting_time.go_home.end_timing != count_dowm_2)
	{
		Master_to_slave.Data.step_num = fromslave_data.step_level;
	}
	
}
///////////////////////////////
float X_ppoint_array[20],Y_ppoint_array[20];
void deal_data_step1(void)//���Կ�����
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18��19��
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

void deal_data_step2(void)//���Կ�����
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18��19��
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

void deal_data_step10(void)//���Կ�����
{
	if(RT_Info.US100_Alt>=0.5f && StartFly == 1 )//StartFly == 1
	{
		static int j;
		float j_sumx,j_sumy;
		j++;
		for(int i=0;i<19;i++)//18��19��
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



