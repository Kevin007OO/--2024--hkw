#include "lidar_processing.h"
#include "stp_23l.h" // 包含这个头文件，来获取所有 extern 变量的声明

// --- 环形缓冲区的实体定义 ---
RingBuffer_t uart1_rx_buffer = { {0}, 0, 0 };
RingBuffer_t uart2_rx_buffer = { {0}, 0, 0 };

// --- 雷达数据解析所需的状态变量 ---
// 这些变量只在本文件内部使用，所以定义为 static
static uint8_t state1 = 0, state2 = 0;
static uint8_t crc1 = 0, crc2 = 0;
static uint8_t cnt1 = 0, cnt2 = 0;
static uint8_t PACK_FLAG1 = 0, PACK_FLAG2 = 0;
static uint16_t data_len1 = 0, data_len2 = 0;
static uint32_t timestamp1 = 0, timestamp2 = 0;
static uint8_t state_flag1 = 1, state_flag2 = 1;

void Lidar_Process(void)
{
    // --- 处理传感器1的数据 ---
    while (uart1_rx_buffer.head != uart1_rx_buffer.tail)
    {
        uint8_t temp_data1 = uart1_rx_buffer.buffer[uart1_rx_buffer.tail];
        uart1_rx_buffer.tail = (uart1_rx_buffer.tail + 1) % 256;
        
        if(state1 < 4)
        {
            if(temp_data1 == HEADER) state1++;
            else state1 = 0;
        }
        else if(state1 < 10 && state1 > 3)
        {
            switch(state1) {
                case 4:
                    if(temp_data1 == device_address) { state1++; crc1 += temp_data1; break; }
                    else { state1 = 0; crc1 = 0; }
                case 5:
                    if(temp_data1 == PACK_GET_DISTANCE) { PACK_FLAG1 = PACK_GET_DISTANCE; state1++; crc1 += temp_data1; break; }
                    // ... (其他PACK类型判断，如果需要的话) ...
                    else { state1 = 0; crc1 = 0; }
                case 6: if(temp_data1 == chunk_offset) { state1++; crc1 += temp_data1; break; } else { state1 = 0; crc1 = 0; }
                case 7: if(temp_data1 == chunk_offset) { state1++; crc1 += temp_data1; break; } else { state1 = 0; crc1 = 0; }
                case 8: data_len1 = (u16)temp_data1; state1++; crc1 += temp_data1; break;
                case 9: data_len1 += ((u16)temp_data1 << 8); state1++; crc1 += temp_data1; break;
                default: break;
            }
        }
        else if(state1 == 10) state_flag1 = 0;

        if(PACK_FLAG1 == PACK_GET_DISTANCE && state_flag1 == 0)
        {
            if(state1 > 9)
            {
                if(state1 < 190)
                {
                    uint8_t state_num1 = (state1 - 10) % 15;
                    switch(state_num1) {
                        case 0:  Pack_Data1[cnt1].distance = (uint16_t)temp_data1; crc1 += temp_data1; state1++; break;
                        case 1:  Pack_Data1[cnt1].distance = ((u16)temp_data1<<8) + Pack_Data1[cnt1].distance; crc1 += temp_data1; state1++; break;
                        case 2:  Pack_Data1[cnt1].noise = (u16)temp_data1; crc1 += temp_data1; state1++; break;
                        case 3:  Pack_Data1[cnt1].noise = ((u16)temp_data1<<8) + Pack_Data1[cnt1].noise; crc1 += temp_data1; state1++; break;
                        case 4:  Pack_Data1[cnt1].peak = (u32)temp_data1; crc1 += temp_data1; state1++; break;
                        case 5:  Pack_Data1[cnt1].peak = ((u32)temp_data1<<8) + Pack_Data1[cnt1].peak; crc1 += temp_data1; state1++; break;
                        case 6:  Pack_Data1[cnt1].peak = ((u32)temp_data1<<16) + Pack_Data1[cnt1].peak; crc1 += temp_data1; state1++; break;
                        case 7:  Pack_Data1[cnt1].peak = ((u32)temp_data1<<24) + Pack_Data1[cnt1].peak; crc1 += temp_data1; state1++; break;
                        case 8:  Pack_Data1[cnt1].confidence = temp_data1; crc1 += temp_data1; state1++; break;
                        case 9:  Pack_Data1[cnt1].intg = (u32)temp_data1; crc1 += temp_data1; state1++; break;
                        case 10: Pack_Data1[cnt1].intg = ((u32)temp_data1<<8) + Pack_Data1[cnt1].intg; crc1 += temp_data1; state1++; break;
                        case 11: Pack_Data1[cnt1].intg = ((u32)temp_data1<<16) + Pack_Data1[cnt1].intg; crc1 += temp_data1; state1++; break;
                        case 12: Pack_Data1[cnt1].intg = ((u32)temp_data1<<24) + Pack_Data1[cnt1].intg; crc1 += temp_data1; state1++; break;
                        case 13: Pack_Data1[cnt1].reftof = (int16_t)temp_data1; crc1 += temp_data1; state1++; break;
                        case 14: Pack_Data1[cnt1].reftof = ((int16_t)temp_data1<<8) + Pack_Data1[cnt1].reftof; crc1 += temp_data1; state1++; cnt1++; break;
                        default: break;
                    }
                }

                if(state1 == 191) { timestamp1 = temp_data1; state1++; crc1 += temp_data1; }
                else if(state1 == 192) { timestamp1 = ((u32)temp_data1<<8) + timestamp1; state1++; crc1 += temp_data1; }
                else if(state1 == 193) { timestamp1 = ((u32)temp_data1<<16) + timestamp1; state1++; crc1 += temp_data1; }
                else if(state1 == 194) { timestamp1 = ((u32)temp_data1<<24) + timestamp1; state1++; crc1 += temp_data1; }
                else if(state1 == 195)
                {
                    if(temp_data1 == crc1) { data_process1(); receive_cnt1++; if(receive_cnt1 > 1) receive_cnt1 = 1; }
                    crc1 = 0; state1 = 0; state_flag1 = 1; cnt1 = 0;
                }
                if(state1 == 190) state1++;
            }
        }
    }

    // --- 处理传感器2的数据 ---
    while (uart2_rx_buffer.head != uart2_rx_buffer.tail)
    {
        uint8_t temp_data2 = uart2_rx_buffer.buffer[uart2_rx_buffer.tail];
        uart2_rx_buffer.tail = (uart2_rx_buffer.tail + 1) % 256;

        if(state2 < 4)
        {
            if(temp_data2 == HEADER) state2++;
            else state2 = 0;
        }
        else if(state2 < 10 && state2 > 3)
        {
            switch(state2) {
                case 4:
                    if(temp_data2 == device_address) { state2++; crc2 += temp_data2; break; }
                    else { state2 = 0; crc2 = 0; }
                case 5:
                    if(temp_data2 == PACK_GET_DISTANCE) { PACK_FLAG2 = PACK_GET_DISTANCE; state2++; crc2 += temp_data2; break; }
                    // ... (其他PACK类型判断) ...
                    else { state2 = 0; crc2 = 0; }
                case 6: if(temp_data2 == chunk_offset) { state2++; crc2 += temp_data2; break; } else { state2 = 0; crc2 = 0; }
                case 7: if(temp_data2 == chunk_offset) { state2++; crc2 += temp_data2; break; } else { state2 = 0; crc2 = 0; }
                case 8: data_len2 = (u16)temp_data2; state2++; crc2 += temp_data2; break;
                case 9: data_len2 = data_len2 + ((u16)temp_data2<<8); state2++; crc2 += temp_data2; break;
                default: break;
            }
        }
        else if(state2 == 10) state_flag2 = 0;

        if(PACK_FLAG2 == PACK_GET_DISTANCE && state_flag2 == 0)
        {
            if(state2 > 9)
            {
                if(state2 < 190)
                {
                    uint8_t state_num2 = (state2 - 10) % 15;
                    switch(state_num2) {
                        case 0:  Pack_Data2[cnt2].distance = (uint16_t)temp_data2; crc2 += temp_data2; state2++; break;
                        case 1:  Pack_Data2[cnt2].distance = ((u16)temp_data2<<8) + Pack_Data2[cnt2].distance; crc2 += temp_data2; state2++; break;
                        case 2:  Pack_Data2[cnt2].noise = (u16)temp_data2; crc2 += temp_data2; state2++; break;
                        case 3:  Pack_Data2[cnt2].noise = ((u16)temp_data2<<8) + Pack_Data2[cnt2].noise; crc2 += temp_data2; state2++; break;
                        case 4:  Pack_Data2[cnt2].peak = (u32)temp_data2; crc2 += temp_data2; state2++; break;
                        case 5:  Pack_Data2[cnt2].peak = ((u32)temp_data2<<8) + Pack_Data2[cnt2].peak; crc2 += temp_data2; state2++; break;
                        case 6:  Pack_Data2[cnt2].peak = ((u32)temp_data2<<16) + Pack_Data2[cnt2].peak; crc2 += temp_data2; state2++; break;
                        case 7:  Pack_Data2[cnt2].peak = ((u32)temp_data2<<24) + Pack_Data2[cnt2].peak; crc2 += temp_data2; state2++; break;
                        case 8:  Pack_Data2[cnt2].confidence = temp_data2; crc2 += temp_data2; state2++; break;
                        case 9:  Pack_Data2[cnt2].intg = (u32)temp_data2; crc2 += temp_data2; state2++; break;
                        case 10: Pack_Data2[cnt2].intg = ((u32)temp_data2<<8) + Pack_Data2[cnt2].intg; crc2 += temp_data2; state2++; break;
                        case 11: Pack_Data2[cnt2].intg = ((u32)temp_data2<<16) + Pack_Data2[cnt2].intg; crc2 += temp_data2; state2++; break;
                        case 12: Pack_Data2[cnt2].intg = ((u32)temp_data2<<24) + Pack_Data2[cnt2].intg; crc2 += temp_data2; state2++; break;
                        case 13: Pack_Data2[cnt2].reftof = (int16_t)temp_data2; crc2 += temp_data2; state2++; break;
                        case 14: Pack_Data2[cnt2].reftof = ((int16_t)temp_data2<<8) + Pack_Data2[cnt2].reftof; crc2 += temp_data2; state2++; cnt2++; break;
                        default: break;
                    }
                }
                
                if(state2 == 191) { timestamp2 = temp_data2; state2++; crc2 += temp_data2; }
                else if(state2 == 192) { timestamp2 = ((u32)temp_data2<<8) + timestamp2; state2++; crc2 += temp_data2; }
                else if(state2 == 193) { timestamp2 = ((u32)temp_data2<<16) + timestamp2; state2++; crc2 += temp_data2; }
                else if(state2 == 194) { timestamp2 = ((u32)temp_data2<<24) + timestamp2; state2++; crc2 += temp_data2; }
                else if(state2 == 195)
                {
                    if(temp_data2 == crc2) { data_process2(); receive_cnt2++; if(receive_cnt2 > 1) receive_cnt2 = 1; }
                    crc2 = 0; state2 = 0; state_flag2 = 1; cnt2 = 0;
                }
                if(state2 == 190) state2++;
            }
        }
    }
}
