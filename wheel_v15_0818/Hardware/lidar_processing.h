#ifndef LIDAR_PROCESSING_H
#define LIDAR_PROCESSING_H

#include "main.h"

// 1. 定义环形缓冲区结构体
typedef struct {
    uint8_t buffer[256];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

// 2. 声明（extern）环形缓冲区，让 main.c 中的中断函数可以访问它们
extern RingBuffer_t uart1_rx_buffer;
extern RingBuffer_t uart2_rx_buffer;

// 3. 声明本模块提供的核心功能函数
void Lidar_Process(void);

#endif // LIDAR_PROCESSING_H
