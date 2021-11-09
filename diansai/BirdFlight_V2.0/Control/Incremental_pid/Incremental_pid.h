#ifndef __INCREMENTAL_PID_H
#define __INCREMENTAL_PID_H

#include "stm32f4xx.h"
#include "arm_math.h"
#include "Limits.h"
#include "math.h"

float Increamental_pidcontrol(float errdata,float IncreamentalKp,float IncreamentalKi,float IncreamentalKd,float *Increamentaldata);

#endif

