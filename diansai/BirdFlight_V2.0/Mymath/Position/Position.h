#ifndef __POSITION_H
#define __POSITION_H


#include "stm32f4xx.h"
#include "InertialFilter.h"
#include "IMU_AHRS.h"
#include "DronePara.h"
#include "Task.h"
#include "Timer5_Timing.h"
#include "digital_filter.h"
#include <os.h>

#define CONSTANTS_ONE_G					8.80665f		/* m/s^2*/
#define FlowPostionx_FIR_TAPS_BITS		4
#define FlowPostionx_FIR_LEN				(1 << FlowPostionx_FIR_TAPS_BITS)
#define FlowPostionx_FIR_FCUT				0.15
#define FlowPostiony_FIR_TAPS_BITS		4
#define FlowPostiony_FIR_LEN				(1 << FlowPostiony_FIR_TAPS_BITS)
#define FlowPostiony_FIR_FCUT				0.15
void Filter_FIRinit(void);
void Position_Estimation(float Ultrasonic,float Xvision,float Yvision,DroneRTInfo *p);
void Height_Estimation(float Ultra);
void OpticalFlow_Estimation(float Ultrasonic,float flow_x,float flow_y,DroneRTInfo *p,float Accx,float Accy);
void third_Kfilter(float Ultrasonic,float flow_x,float flow_y,DroneRTInfo *p,float Accx,float Accy,char reset_flag);
#endif 

