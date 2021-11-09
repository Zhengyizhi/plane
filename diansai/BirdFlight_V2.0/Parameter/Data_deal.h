#ifndef __DATA_DEAL_H
#define __DATA_DEAL_H

#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "arm_math.h"
#include "Flash.h"
#include "Inner_control.h"

void dataDeal(_Data_Rx rx);
void dataStitching(_Data_Rx rx);
 
 

extern volatile float throttleBasic;
 
#endif 

