#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"
#include "MedianFiler.h"
#include "arm_math.h"
#include "Usart6toVision.h"



extern u8 BlackspotsFlag;
extern u8 OpticalflowFlag;
//点位数据
extern volatile float Pix_Xinfo;
extern volatile float Pix_Yinfo;
//光流数据
extern volatile float OpticalFlow_x;
extern volatile float OpticalFlow_y;
extern volatile float OpticalFlow_integralx;
extern volatile float OpticalFlow_integraly;;
void Vision_datadeal(_Data_Rx rx);
#endif


