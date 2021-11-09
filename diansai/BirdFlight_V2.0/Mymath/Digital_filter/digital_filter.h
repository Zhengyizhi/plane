/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * digital_filter.h
 *
 *  Created on: 24 nov 2012
 *      Author: benjamin
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_

#include <stdint.h>
#define abs(x) ((x)>0? (x):(-(x)))
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;
typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


//卡尔曼
typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
		float B;
    float Q;
    float R;
    float H;
}extKalman_t;

typedef struct {
	extKalman_t ratepitch_flowkal;
	extKalman_t rateroll_flowkal;
}kalman_pid_t;

typedef enum{
	KAL_flowpitchrate,//光流数据处理
	KAL_flowrollrate,
	KAL_CNT
}kalman_mode_cnt_t;

typedef struct kalman_pid_struct{
	kalman_pid_t *info;
	kalman_mode_cnt_t id;
 
	void        (*update)(struct kalman_pid_struct *self,extKalman_t *data);
	void        (*init)(struct kalman_pid_struct * self);
}kalman_pid_struct_t;//最终的
float KalmanFilter(extKalman_t* p,float dat);
extern kalman_pid_struct_t KAL_dealinfo[KAL_CNT];

void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


// Functions
void filter_fft(int dir, int m, float *real, float *imag);
//void filter_dft(int dir, int len, float *real, float *imag);
void filter_fftshift(float *data, int len);
void filter_hamming(float *data, int len);
void filter_zeroPad(float *data, float *result, int dataLen, int resultLen);
void filter_create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming);
float filter_run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset);
void filter_add_sample(float *buffer, float sample, int bits, uint32_t *offset);


#endif /* DIGITAL_FILTER_H_ */
