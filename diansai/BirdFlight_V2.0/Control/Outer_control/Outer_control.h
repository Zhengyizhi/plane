#ifndef __OUTER_CONTROL_H
#define __OUTER_CONTROL_H
#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "Neurons.h"

extern PIDOut pidPitch,pidRoll,pidYaw,pidHeight,pidPointX,pidPointY,pidFlowX,pidFlowY,pid_calculateflowX,pid_calculateflowY;
void Outer_pidinit(void);
void AttitudeOuter_control(void);
void PostionOuter_control(void);
uint32_t world_time_ms(void);
uint32_t world_time_us(void);
typedef enum
{
	start_true = 1,
	start_false = 0,
	count_dowm_0 = 0,
	count_dowm_1 = 1,
	count_dowm_2 = 2,
}time_delay_t;

typedef struct count_dowm
{
	time_delay_t start_flag;
	uint32_t F_time;
	uint32_t E_time;
	time_delay_t end_timing;
}count_dowm_t;

typedef struct all_time_dowm
{
	count_dowm_t centering;
	count_dowm_t centering_2;
	
	count_dowm_t find_A;
	
	count_dowm_t turn_90_1;
	count_dowm_t turn_90_2;
	
	count_dowm_t stay_line;
	
	count_dowm_t turn_90_3;
	count_dowm_t go_home;
}waiting_time_t;

extern waiting_time_t waiting_time;

//void starting_task(void);
void all_time_init(waiting_time_t* time);
void delay_ms_init(count_dowm_t* time);//÷ª≈‹“ª¥Œ
void delay_ms_timer( uint32_t delaytime , count_dowm_t* time);
void ttestyaw(float turnangle);
void yaw_control(float turnangle);
//extern float X_pointdt_goal,Y_pointdt_goal;
#endif


