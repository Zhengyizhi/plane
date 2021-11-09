#ifndef _USARTTOPC_H
#define _USARTTOPC_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include <os.h>
#include "Task.h"
void Uart1_tx(uint8_t *data,uint16_t size);
void Usart1toPC_Init(u32 Bound);

extern _Data_Rx PC_rx;

#endif 
