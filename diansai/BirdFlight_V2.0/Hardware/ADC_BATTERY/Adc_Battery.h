#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H

#include "stm32f4xx.h"

void Adc_Batteryinit(void);
float Get_battery(void);
float Average_battery(float batteryValue);

#endif 
