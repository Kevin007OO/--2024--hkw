#ifndef _STP_23L_H__
#define	_STP_23L_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdlib.h"
#include "string.h"

/*不知道为什么定义，但是根本没用到的东西
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern char  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	
*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define false 0
#define true 1

#define HEADER 0xAA							/* 起始符 */
#define device_address 0x00     /* 设备地址 */
#define chunk_offset 0x00       /* 偏移地址命令 */
#define PACK_GET_DISTANCE 0x02 	/* 获取测量数据命令 */
#define PACK_RESET_SYSTEM 0x0D 	/* 复位命令 */
#define PACK_STOP 0x0F 				  /* 停止测量数据传输命令 */
#define PACK_ACK 0x10           /* 应答码命令 */
#define PACK_VERSION 0x14       /* 获取传感器信息命令 */

typedef struct {
	int16_t distance;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef;

struct AckResultData{
	uint8_t ack_cmd_id;						/* 答复的命令 id */
	uint8_t result; 							/* 1表示成功,0表示失败 */
};

struct LiManuConfig
{
	uint32_t version; 						/* 软件版本号 */
	uint32_t hardware_version; 		/* 硬件版本号 */
	uint32_t manufacture_date; 		/* 生产日期 */
	uint32_t manufacture_time; 		/* 生产时间 */
	uint32_t id1; 								/* 设备 id1 */
	uint32_t id2; 								/* 设备 id2 */
	uint32_t id3; 								/* 设备 id3 */
	uint8_t sn[8]; 								/* sn */
	uint16_t pitch_angle[4]; 			/* 角度信息 */
	uint16_t blind_area[2]; 			/* 盲区信息 */
	uint32_t frequence; 					/* 数据点频 */
};

// =================== 在这里声明所有共享的全局变量 ===================

// --- 声明雷达原始数据缓冲区 ---
// 任何需要填充这些缓冲区的文件（比如 lidar_processing.c）都可以通过 extern 访问
extern LidarPointTypedef Pack_Data1[12];
extern LidarPointTypedef Pack_Data2[12];

// --- 声明雷达最终结果变量 ---
// 任何需要使用这些结果的文件（比如 motor.c）都可以通过 extern 访问
extern uint16_t receive_cnt1,receive_cnt2;
extern uint8_t confidence1,confidence2;
extern uint16_t distance1,noise1,reftof1,distance2,noise2,reftof2;
extern uint32_t peak1,intg1,peak2,intg2;
// ===================================================================

void data_process1(void);
void data_process2(void);
#endif
