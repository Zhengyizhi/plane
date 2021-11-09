#ifndef __IMU_AHRS_H
#define __IMU_AHRS_H

extern volatile float RDrone_R[3][3];
extern volatile float Acc_Flow_x;
extern volatile float Acc_Flow_y;
extern volatile float Acc_Flow_z;
//extern float Accel_Src[3];
void IMU_HardwareInit(void);
void IMU_getInfo(void);

#endif


