/******************* (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者    ：Xiluna Tech
 
 * 控制提供者：郑怡芝
 
 * 文件名  ：main.c
 * 描述    ：主循环
 * 官网    ：http://xiluna.com/
 * 公众号  ：XilunaTech
**********************************************************************************/
#include "Head_File.h"
#define START_TASK_PRIO 3						                     // 任务优先级
#define START_STK_SIZE 1024						                   // 任务堆栈大小
OS_TCB StartTaskTCB;							                       // 任务控制块
CPU_STK START_TASK_STK[START_STK_SIZE];					         // 任务堆栈
void start_task(void *p_arg);						                 // 任务函数

//IMU任务
#define IMU_TASK_PRIO 4						
#define IMU_STK_SIZE 1024						
OS_TCB IMUTaskTCB;							
CPU_STK IMU_TASK_STK[IMU_STK_SIZE];					
void IMU_task(void *p_arg);

//姿态内环任务
#define AttitudeInner_TASK_PRIO 2					
#define AttitudeInner_STK_SIZE 1024						
OS_TCB AttitudeInnerTaskTCB;							
CPU_STK AttitudeInner_TASK_STK[AttitudeInner_STK_SIZE];					
void AttitudeInner_task(void *p_arg);

//位置内环任务
#define PostionInner_TASK_PRIO 6						
#define PostionInner_STK_SIZE 1024						
OS_TCB PostionInnerTaskTCB;							
CPU_STK PostionInner_TASK_STK[PostionInner_STK_SIZE];					
void PostionInner_task(void *p_arg);

//融合任务
#define Combine_TASK_PRIO 7						
#define Combine_STK_SIZE 1024					
OS_TCB CombineTaskTCB;							
CPU_STK Combine_TASK_STK[Combine_STK_SIZE];					
void Combine_task(void *p_arg);

//姿态外环任务
#define AttitudeOuter_TASK_PRIO 8						
#define AttitudeOuter_STK_SIZE 1024						
OS_TCB AttitudeOuterTaskTCB;							
CPU_STK AttitudeOuter_TASK_STK[AttitudeOuter_STK_SIZE];					
void AttitudeOuter_task(void *p_arg);

//位置外环任务
#define PostionOuter_TASK_PRIO 9						
#define PostionOuter_STK_SIZE 1024						
OS_TCB PostionOuterTaskTCB;							
CPU_STK PostionOuter_TASK_STK[PostionOuter_STK_SIZE];					
void PostionOuter_task(void *p_arg);

//视觉数据接收任务
#define Vision_TASK_PRIO 10						
#define Vision_STK_SIZE	1024						
OS_TCB VisionTaskTCB;							
CPU_STK Vision_TASK_STK[Vision_STK_SIZE];					
void Vision_task(void *p_arg);

//DataDeal任务
#define DataDeal_TASK_PRIO 11						
#define DataDeal_STK_SIZE 1024						
OS_TCB DataDealTaskTCB;							
CPU_STK DataDeal_TASK_STK[DataDeal_STK_SIZE];					
void DataDeal_task(void *p_arg);

//Usart1toPC任务
#define Usart1toPC_TASK_PRIO 12						
#define Usart1toPC_STK_SIZE 1024						
OS_TCB Usart1toPCTaskTCB;							
CPU_STK Usart1toPC_TASK_STK[Usart1toPC_STK_SIZE];					
void Usart1toPC_task(void *p_arg);

//Uart5toUltra任务
#define Uart5toUltra_TASK_PRIO 13						
#define Uart5toUltra_STK_SIZE 1024						
OS_TCB Uart5toUltraTaskTCB;							
CPU_STK Uart5toUltra_TASK_STK[Uart5toUltra_STK_SIZE];					
void Uart5toUltra_task(void *p_arg);

//电池电压任务
#define Battery_TASK_PRIO 14						
#define Battery_STK_SIZE 1024						
OS_TCB BatteryTaskTCB;							
CPU_STK Battery_TASK_STK[Battery_STK_SIZE];					
void Battery_task(void *p_arg);

//串口切换任务
#define Usartmode_TASK_PRIO 15				
#define Usartmode_STK_SIZE 1024					
OS_TCB UsartmodeTaskTCB;							
CPU_STK Usartmode_TASK_STK[Usartmode_STK_SIZE];					
void Usartmode_task(void *p_arg);

//一键起飞
#define TakeOff_TASK_PRIO 16				
#define TakeOff_STK_SIZE 1024						
OS_TCB TakeOffTaskTCB;							
CPU_STK TakeOff_TASK_STK[TakeOff_STK_SIZE];					
void TakeOff_task(void *p_arg);

//光流
#define Flow_TASK_PRIO 17				
#define Flow_STK_SIZE 1024						
OS_TCB FlowTaskTCB;							
CPU_STK Flow_TASK_STK[Flow_STK_SIZE];					
void Flow_task(void *p_arg);

/**
 * @Description 主函数启动操作系统
 */	
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	Systick_Init(168);																			// 时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					// 中断分组配置
	Adc_Batteryinit();                                      // 电池电压ADC初始化
	SPI3_Configuration();  																	// SPI3初始化
	Timer3_Ahrsinit();																			// IMU解算定时器
	Timer4_ImuInnerinit();																	// 姿态内环控制
	Timer5_Timinginit();                                    // 记录时间定时器
	IMU_HardwareInit();                                     // IMU各个传感器寄存器配置
	PWM_Init();                                             // pwm定时器初始化
	Bluetooth_init();                                       // 蓝牙串口 默认使用
	Usart1toPC_Init(115200);                                // Usart1发送数据给PC 默认不使用
	Usart6toflow_Init(19200);                            // Usart6接收视觉模块数据
//	USART6_Config();
	Uart5toUltra_Init(9600);                                // Uart5接收超声波模块数据
	Uart2_Init(115200);
	
	Tension_Queue_Init(&stile_queue);
	
//	Usart2toUltra_Init(115200);//
	General_Gpioinit();                                     // 通用GPIO初始化
	load_config();                                          // 配置用户信息
	Filter_FIRinit();                                       // FIR滤波器配置             
	flowing_init();
	OSInit(&err);																						// 初始化UCOSIII
	OS_CRITICAL_ENTER();																		// 进入临界区
	OSTaskCreate(																						// 创建开始任务
		(OS_TCB*)&StartTaskTCB,																// 任务控制块
		(CPU_CHAR*)"start task", 															// 任务名字
		(OS_TASK_PTR)start_task, 															// 任务函数
		(void*)0,																							// 传递给任务函数的参数
		(OS_PRIO)START_TASK_PRIO,															// 任务优先级
		(CPU_STK*)&START_TASK_STK[0],													// 任务堆栈基地址
		(CPU_STK_SIZE)START_STK_SIZE/10,											// 任务堆栈深度限位
		(CPU_STK_SIZE)START_STK_SIZE,													// 任务堆栈大小
		(OS_MSG_QTY)0,																				// 任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
		(OS_TICK)0,																						// 当使能时间片轮转时的时间片长度，为0时为默认长度，
		(void*)0,																							// 用户补充的存储区
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,			// 任务选项
		(OS_ERR*)&err																					// 存放该函数错误时的返回值
		);
	OS_CRITICAL_EXIT();																			// 退出临界区
	OSStart(&err);																					// 开启UCOSIII
	while(1);
}

/**
 * @Description 开始任务函数
 */
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);											// 统计任务
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN											// 如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();	
#endif
	//默认打开
#if OS_CFG_SCHED_ROUND_ROBIN_EN											// 当使用时间片轮转的时候
	//使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

	OS_CRITICAL_ENTER();															// 进入临界区
	OSTaskCreate(																			// 创建姿态IMU任务
		(OS_TCB*)&IMUTaskTCB,
		(CPU_CHAR*)"IMU task",
		(OS_TASK_PTR )IMU_task,//任务名
		(void*)0,
		(OS_PRIO)IMU_TASK_PRIO,
		(CPU_STK*)&IMU_TASK_STK[0],
		(CPU_STK_SIZE)IMU_STK_SIZE/10,
		(CPU_STK_SIZE)IMU_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建姿态Inner任务
		(OS_TCB*)&AttitudeInnerTaskTCB,
		(CPU_CHAR*)"AttitudeInner task",
		(OS_TASK_PTR )AttitudeInner_task,
		(void*)0,
		(OS_PRIO)AttitudeInner_TASK_PRIO,
		(CPU_STK*)&AttitudeInner_TASK_STK[0],
		(CPU_STK_SIZE)AttitudeInner_STK_SIZE/10,
		(CPU_STK_SIZE)AttitudeInner_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建位置Inner任务
		(OS_TCB*)&PostionInnerTaskTCB,
		(CPU_CHAR*)"PostionInner task",
		(OS_TASK_PTR )PostionInner_task,
		(void*)0,
		(OS_PRIO)PostionInner_TASK_PRIO,
		(CPU_STK*)&PostionInner_TASK_STK[0],
		(CPU_STK_SIZE)PostionInner_STK_SIZE/10,
		(CPU_STK_SIZE)PostionInner_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建融合Combine任务
		(OS_TCB*)&CombineTaskTCB,
		(CPU_CHAR*)"Combine task",
		(OS_TASK_PTR )Combine_task,
		(void*)0,
		(OS_PRIO)Combine_TASK_PRIO,
		(CPU_STK*)&Combine_TASK_STK[0],
		(CPU_STK_SIZE)Combine_STK_SIZE/10,
		(CPU_STK_SIZE)Combine_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建姿态Outer任务
		(OS_TCB*)&AttitudeOuterTaskTCB,
		(CPU_CHAR*)"AttitudeOuter task",
		(OS_TASK_PTR )AttitudeOuter_task,
		(void*)0,
		(OS_PRIO)AttitudeOuter_TASK_PRIO,
		(CPU_STK*)&AttitudeOuter_TASK_STK[0],
		(CPU_STK_SIZE)AttitudeOuter_STK_SIZE/10,
		(CPU_STK_SIZE)AttitudeOuter_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建位置Outer任务
		(OS_TCB*)&PostionOuterTaskTCB,
		(CPU_CHAR*)"PostionOuter task",
		(OS_TASK_PTR )PostionOuter_task,
		(void*)0,
		(OS_PRIO)PostionOuter_TASK_PRIO,
		(CPU_STK*)&PostionOuter_TASK_STK[0],
		(CPU_STK_SIZE)PostionOuter_STK_SIZE/10,
		(CPU_STK_SIZE)PostionOuter_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建视觉Vision任务
		(OS_TCB*)&VisionTaskTCB,
		(CPU_CHAR*)"Vision task",
		(OS_TASK_PTR )Vision_task,
		(void*)0,
		(OS_PRIO)Vision_TASK_PRIO,
		(CPU_STK*)&Vision_TASK_STK[0],
		(CPU_STK_SIZE)Vision_STK_SIZE/10,
		(CPU_STK_SIZE)Vision_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建数据处理DataDeal任务
		(OS_TCB*)&DataDealTaskTCB,
		(CPU_CHAR*)"DataDeal task",
		(OS_TASK_PTR )DataDeal_task,
		(void*)0,
		(OS_PRIO)DataDeal_TASK_PRIO,
		(CPU_STK*)&DataDeal_TASK_STK[0],
		(CPU_STK_SIZE)DataDeal_STK_SIZE/10,
		(CPU_STK_SIZE)DataDeal_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建数据发送Usart1toPC任务
		(OS_TCB*)&Usart1toPCTaskTCB,
		(CPU_CHAR*)"Usart1toPC task",
		(OS_TASK_PTR )Usart1toPC_task,
		(void*)0,
		(OS_PRIO)Usart1toPC_TASK_PRIO,
		(CPU_STK*)&Usart1toPC_TASK_STK[0],
		(CPU_STK_SIZE)Usart1toPC_STK_SIZE/10,
		(CPU_STK_SIZE)Usart1toPC_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建高度Uart5toUltra任务
		(OS_TCB*)&Uart5toUltraTaskTCB,
		(CPU_CHAR*)"Uart5toUltra task",
		(OS_TASK_PTR )Uart5toUltra_task,
		(void*)0,
		(OS_PRIO)Uart5toUltra_TASK_PRIO,
		(CPU_STK*)&Uart5toUltra_TASK_STK[0],
		(CPU_STK_SIZE)Uart5toUltra_STK_SIZE/10,
		(CPU_STK_SIZE)Uart5toUltra_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建电池电压Battery任务
		(OS_TCB*)&BatteryTaskTCB,
		(CPU_CHAR*)"Battery task",
		(OS_TASK_PTR )Battery_task,
		(void*)0,
		(OS_PRIO)Battery_TASK_PRIO,
		(CPU_STK*)&Battery_TASK_STK[0],
		(CPU_STK_SIZE)Battery_STK_SIZE/10,
		(CPU_STK_SIZE)Battery_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																			// 创建usart mode切换任务
		(OS_TCB*)&UsartmodeTaskTCB,
		(CPU_CHAR*)"Usartmode task",
		(OS_TASK_PTR )Usartmode_task,
		(void*)0,
		(OS_PRIO)Usartmode_TASK_PRIO,
		(CPU_STK*)&Usartmode_TASK_STK[0],
		(CPU_STK_SIZE)Usartmode_STK_SIZE/10,
		(CPU_STK_SIZE)Usartmode_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
//	OSTaskCreate(																			// 创建一键起飞任务
//		(OS_TCB*)&TakeOffTaskTCB,
//		(CPU_CHAR*)"TakeOff task",
//		(OS_TASK_PTR )TakeOff_task,
//		(void*)0,
//		(OS_PRIO)TakeOff_TASK_PRIO,
//		(CPU_STK*)&TakeOff_TASK_STK[0],
//		(CPU_STK_SIZE)TakeOff_STK_SIZE/10,
//		(CPU_STK_SIZE)TakeOff_STK_SIZE,
//		(OS_MSG_QTY)0,
//		(OS_TICK)0,
//		(void*)0,
//		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//		(OS_ERR*)&err
//		);	
		OSTaskCreate(																			// 创建光流Flow任务
		(OS_TCB*)&FlowTaskTCB,
		(CPU_CHAR*)"Flow task",
		(OS_TASK_PTR )Flow_task,
		(void*)0,
		(OS_PRIO)Flow_TASK_PRIO,
		(CPU_STK*)&Flow_TASK_STK[0],
		(CPU_STK_SIZE)Flow_STK_SIZE/10,
		(CPU_STK_SIZE)Flow_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
			
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);			// 挂起开始任务
	OS_CRITICAL_EXIT();																// 离开临界区 
}



