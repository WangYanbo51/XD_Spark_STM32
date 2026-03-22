/*
 * task_main.h
 *
 *  Created on: Mar 12, 2026
 *      Author: lenovo
 */

#ifndef INC_TASK_MAIN_H_
#define INC_TASK_MAIN_H_

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>

#define CMD_LEN             4       // 4字节指令协议
#define PWM_MAX             999     // PWM最大值（与手套端匹配）
#define DEAD_ZONE_STEER     5       // 转向死区j
#define LOST_CONNECT_TIMEOUT 1000   // 断联保护1秒
#define UART2_DMA_BUF_LEN   128     // DMA接收缓冲区
#define SPIN_MAX_SPEED  500

typedef struct {
    uint8_t  cmd;                // 0=停止 1=前进 2=后退 3=紧急刹车
    uint16_t speed;              // 速度0~999
    int8_t   steer;              // 转向-100~100
    uint32_t last_recv_time;     // 最后接收指令时间
    uint8_t  dma_recv_buf[UART2_DMA_BUF_LEN]; // USART2 DMA缓冲区
    uint16_t dma_recv_len;       // DMA已接收字节数
} CarControl_t;

extern CarControl_t car;

void Debug_Print(const char *fmt, ...); // USART1调试打印

void MainTask_Init();
void MainTask_Loop();

#endif /* INC_TASK_MAIN_H_ */
