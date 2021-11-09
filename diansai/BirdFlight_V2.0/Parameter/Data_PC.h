#ifndef __DATA_PC_H
#define __DATA_PC_H

#include "DronePara.h"
#include "Usart1toPC.h"
#include "stm32f4xx.h"
#include "Task.h"
#include "Type_conversion.h"
#include "Usart3toBluetooth.h"
#include "Outer_control.h"
#include "Neurons.h"
void sendParaInfo(void);
void sendRTInfo(void);


#endif 

