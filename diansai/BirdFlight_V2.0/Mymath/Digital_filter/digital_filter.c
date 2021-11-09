/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * 作者 	:Xiluna Tech
 * 文件名 :digital_filter.c
 * 描述   :FIR数字滤波器
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
******************************************************************************/
#include  "digital_filter.h"
#include  <math.h>
#include  "arm_math.h"
#include  <stdint.h>

// Found at http://paulbourke.net/miscellaneous//dft/
void filter_fft(int dir, int m, float *real, float *imag) {
	long n,i,i1,j,k,i2,l,l1,l2;
	float c1,c2,tx,ty,t1,t2,u1,u2,z;

	// Calculate the number of points
	n = 1 << m;

	// Do the bit reversal
	i2 = n >> 1;
	j = 0;
	for (i=0;i<n-1;i++) {
		if (i < j) {
			tx = real[i];
			ty = imag[i];
			real[i] = real[j];
			imag[i] = imag[j];
			real[j] = tx;
			imag[j] = ty;
		}
		k = i2;
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}

	// Compute the FFT
	c1 = -1.0;
	c2 = 0.0;
	l2 = 1;
	for (l=0;l<m;l++) {
		l1 = l2;
		l2 <<= 1;
		u1 = 1.0;
		u2 = 0.0;
		for (j=0;j < l1;j++) {
			for (i=j;i < n;i += l2) {
				i1 = i + l1;
				t1 = u1 * real[i1] - u2 * imag[i1];
				t2 = u1 * imag[i1] + u2 * real[i1];
				real[i1] = real[i] - t1;
				imag[i1] = imag[i] - t2;
				real[i] += t1;
				imag[i] += t2;
			}
			z =  u1 * c1 - u2 * c2;
			u2 = u1 * c2 + u2 * c1;
			u1 = z;
		}
		c2 = sqrt((1.0f - c1) / 2.0f);
		if (dir) {
			c2 = -c2;
		}
		c1 = sqrt((1.0f + c1) / 2.0f);
	}

	// Scaling for reverse transform
	if (dir) {
		for (i=0;i < n;i++) {
			real[i] /= n;
			imag[i] /= n;
		}
	}
}

// Found at http://paulbourke.net/miscellaneous//dft/
//void filter_dft(int dir, int len, float *real, float *imag) {
//	long i,k;
//	float arg;
//	float cosarg, sinarg;
//	float x2[len];
//	float y2[len];
//	if(dir) {
//		dir = 1;
//	} else {
//		dir = -1;
//	}



//	for (i=0;i < len;i++) {
//		x2[i] = 0;
//		y2[i] = 0;
//		arg = -(float)dir * 2.0f * PI * (float)i / (float)len;
//		for (k=0;k<len;k++) {
//			cosarg = cosf(k * arg);
//			sinarg = sinf(k * arg);
//			x2[i] += (real[k] * cosarg - imag[k] * sinarg);
//			y2[i] += (real[k] * sinarg + imag[k] * cosarg);
//		}
//	}

//	// Copy the data back
//	if (dir == 1) {
//		for (i=0;i<len;i++) {
//			real[i] = x2[i] / (float)len;
//			imag[i] = y2[i] / (float)len;
//		}
//	} else {
//		for (i=0;i<len;i++) {
//			real[i] = x2[i];
//			imag[i] = y2[i];
//		}
//	}
//}

void filter_fftshift(float *data, int len) {
	int i;
	for (i = 0;i < (len / 2);i++) {
		float r1 = data[i];
		float r2 = data[len/2 + i];

		data[i] = r2;
		data[len / 2 + i] = r1;
	}
}

void filter_hamming(float *data, int len) {
	int i;
	if (len % 2 == 0) {
		for (i = 0;i < (len / 2);i++) {
			float val = 0.54f - 0.46f * cosf((2.0f * PI * (float)i)/(float)(len - 1));
			data[i] *= val;
			data[len - i - 1] *= val;
		}
	} else {
		for (i = 0;i < len;i++) {
			data[i] *= 0.54f - 0.46f * cosf((2.0f * PI * (float)i)/(float)(len - 1));
		}
	}
}

void filter_zeroPad(float *data, float *result, int dataLen, int resultLen) {
	int i;
	for (i = 0;i < resultLen;i++) {
		if (i < dataLen) {
			result[i] = data[i];
		} else {
			result[i] = 0;
		}
	}
}

void filter_create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming) {
	int taps = 1 << bits;
	float imag[taps];

	for(int i = 0;i < taps;i++) {
		if (i < (int)((float)taps * f_break)) {
			filter_vector[i] = 1;
		} else {
			filter_vector[i] = 0;
		}
		imag[i] = 0;
	}

	// Make filter symmetric
	for (int i = 0;i < taps / 2;i++) {
		filter_vector[taps - i - 1] = filter_vector[i];
	}

	filter_fft(1, bits, filter_vector, imag);
	filter_fftshift(filter_vector, taps);

	if (use_hamming) {
		filter_hamming(filter_vector, taps);
	}
}

/*
 * Run FIR filter iteration.
 *
 * bits: A power of two representing the length of the filter
 * filter: The FIR filter coefficients
 * offset: an offset into the vector buffer. Will wrap around when going past
 * length while filtering. Useful for keeping a circular buffer with samples
 * and avoiding to shift the whole buffer.
 *
 * returns: The filtered result sample.
 */
float filter_run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset) {
	float result = 0;
	int size = 1 << bits;
	uint32_t cnt_mask = 0xFFFFFFFF >> (32 - bits);

	for (int i = 0;i < size;i++) {
		result += filter[i] * vector[offset];
		offset++;
		offset &= cnt_mask;
	}

	return result;
}

/**
 * Add sample to buffer
 * @param buffer
 * The buffer to add the sample to
 * @param sample
 * The sample to add
 * @param bits
 * The length of the buffer in bits
 * @param offset
 * Pointer to the current offset in the buffer. Will be updated in this call
 * and wrapped at the length of this buffer.
 */
void filter_add_sample(float *buffer, float sample, int bits, uint32_t *offset) {
	uint32_t cnt_mask = 0xFFFFFFFF >> (32 - bits);
	buffer[*offset] = sample;
	*offset += 1;
	*offset &= cnt_mask;
}
/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(3.141592653589793f / fr);
  float c = 1.0f + 2.0f * cosf(3.141592653589793f / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(3.141592653589793f / 4.0f) * ohm + ohm * ohm) / c;
}
/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
		p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}
/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}
kalman_pid_t kal_Info[KAL_CNT]=
{
	[KAL_flowpitchrate] = {
		.ratepitch_flowkal.Q = 1,
		.ratepitch_flowkal.R = 100,
	},
	[KAL_flowrollrate] = {
		.rateroll_flowkal.Q = 1,
		.rateroll_flowkal.R = 100,
	},

};
 void kalman_pid_init(kalman_pid_struct_t * self)
{
	kalman_pid_t *KAL = self->info;
	if(self->id == KAL_flowpitchrate)
		KalmanCreate(&KAL->ratepitch_flowkal,KAL->ratepitch_flowkal.Q,KAL->ratepitch_flowkal.R);
	else if(self->id == KAL_flowrollrate)
		KalmanCreate(&KAL->rateroll_flowkal,KAL->rateroll_flowkal.Q,KAL->rateroll_flowkal.R);
}


kalman_pid_struct_t KAL_dealinfo[KAL_CNT] =
{
	[KAL_flowpitchrate] ={
		.info = &kal_Info[KAL_flowpitchrate],
		.id = KAL_flowpitchrate,
		.init=kalman_pid_init,
	},
	[KAL_flowrollrate] ={
		.info = &kal_Info[KAL_flowrollrate],
		.id = KAL_flowrollrate,
		.init = kalman_pid_init,
	}
};


