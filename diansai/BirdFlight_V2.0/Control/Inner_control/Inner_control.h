#ifndef __INNER_CONTROL_H
#define __INNER_CONTROL_H
#include "stm32f4xx.h"
#include "DronePara.h"
#include "arm_math.h"
#include "Task.h"
#include "Timer5_Timing.h"
#include "Limits.h"
#include "Outer_control.h"
#include "Neurons.h"
#include "Incremental_pid.h"

extern volatile float lowpass_filter;
extern  PIDOut  pidRatePitch,pidRateRoll,pidRateYaw,
					pidAccHeight,pidAccPitch,pidAccRoll,
						pidXSpeed,pidYSpeed,pidFlowx,pidFlowy;

void Inner_pidinit(void);
void AttitudeInner_control(void);
void PostionInner_control(void);
void Calculate_output(void);
void PID_OUT(	unsigned int Motor1,
							unsigned int Motor2,
							unsigned int Motor3,
							unsigned int Motor4);

#endif

