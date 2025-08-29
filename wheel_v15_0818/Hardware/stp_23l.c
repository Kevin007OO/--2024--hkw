#include "usart.h"
#include "stp_23l.h"
/* 不知道为什么定义，但是后面没用到
char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t point1 ;
*/
LidarPointTypedef Pack_Data1[12];
LidarPointTypedef Pack_Data2[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef Pack_sum1,Pack_sum2;     /* 输出结果储存 */
uint16_t receive_cnt1,receive_cnt2;
uint8_t confidence1,confidence2;
uint16_t distance1,noise1,reftof1,distance2,noise2,reftof2;
uint32_t peak1,intg1,peak2,intg2;


void data_process1(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
		/* 计算距离 */
		static u8 cnt = 0;
		u8 i;
		static u16 count = 0;
		static u32 sum = 0;
		static u32 distance_sum = 0;
		static u32 noise_sum = 0;
		static u32 peak_sum = 0;
		static u32 confidence_sum = 0;
		static u32 intg_sum = 0;
		static u32 reftof_sum = 0;
		for(i=0;i<12;i++)									/* 12个点取平均 */
		{
				if(Pack_Data1[i].distance != 0)  /* 去除0的点 */
				{
						count++;
						distance_sum += Pack_Data1[i].distance;
						noise_sum += Pack_Data1[i].noise;
						peak_sum += Pack_Data1[i].peak;
						confidence_sum += Pack_Data1[i].confidence;
						intg_sum += Pack_Data1[i].intg;
						reftof_sum += Pack_Data1[i].reftof;
				}
		}
		if(count !=0)
		{
			distance1 = distance_sum/count;
			noise1 = noise_sum/count;
			peak1 = peak_sum/count;
			confidence1 = confidence_sum/count;
			intg1 = intg_sum/count;
			reftof1 = reftof_sum/count;
			count = 0;
			
			sum = 0;
			distance_sum = 0;
			noise_sum = 0;
			peak_sum = 0;
			confidence_sum = 0;
			intg_sum = 0;
			reftof_sum = 0;
		}
}

void data_process2(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
		/* 计算距离 */
		static u8 cnt = 0;
		u8 i;
		static u16 count = 0;
		static u32 sum = 0;
		static u32 distance_sum = 0;
		static u32 noise_sum = 0;
		static u32 peak_sum = 0;
		static u32 confidence_sum = 0;
		static u32 intg_sum = 0;
		static u32 reftof_sum = 0;
		for(i=0;i<12;i++)									/* 12个点取平均 */
		{
				if(Pack_Data2[i].distance != 0)  /* 去除0的点 */
				{
						count++;
						distance_sum += Pack_Data2[i].distance;
						noise_sum += Pack_Data2[i].noise;
						peak_sum += Pack_Data2[i].peak;
						confidence_sum += Pack_Data2[i].confidence;
						intg_sum += Pack_Data2[i].intg;
						reftof_sum += Pack_Data2[i].reftof;
				}
		}
		if(count !=0)
		{
			distance2 = distance_sum/count;
			noise2 = noise_sum/count;
			peak2 = peak_sum/count;
			confidence2 = confidence_sum/count;
			intg2 = intg_sum/count;
			reftof2 = reftof_sum/count;
			count = 0;
			
			sum = 0;
			distance_sum = 0;
			noise_sum = 0;
			peak_sum = 0;
			confidence_sum = 0;
			intg_sum = 0;
			reftof_sum = 0;
		}
}
