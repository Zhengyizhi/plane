#ifndef __USART6TOVISION_H
#define __USART6TOVISION_H

#include "DronePara.h"
#include "Task.h"
#include <os.h>
#include <string.h>
//extern _Data_Rx Flow_rx; 

//void Usart6toFlow_Init(u32 Bound);
//void Usart6_tx(uint8_t *data,uint16_t size);

//////////////////////////////////////
extern _Data_Rx Flosion;

void Usart6toflow_Init(u32 Bound);//main.c

//void UART6Test(void);
//void USART6_Config(void);
#endif

