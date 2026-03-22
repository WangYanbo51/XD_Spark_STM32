/*
 * task_main.c
 *
 *  Created on: Mar 12, 2026
 *      Author: lenovo
 */
#include "task_main.h"
#include "motor.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

CarControl_t car = { 0 };

/**
 * @brief  USART1调试打印（输出到电脑，波特率115200）
 */
void Debug_Print(const char *fmt, ...)
{
	char debug_buf[128];
	va_list args;
	va_start(args, fmt);
	vsnprintf(debug_buf, sizeof(debug_buf), fmt, args);
	HAL_UART_Transmit(&huart1, (uint8_t*) debug_buf, strlen(debug_buf), 100);
	va_end(args);
}

/**
 * @brief  限幅函数（防止PWM溢出）
 */
static int16_t constrain(int16_t value, int16_t min, int16_t max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

/**
 * @brief  解析手套4字节指令
 */
static void Car_ParseCmd(uint8_t *buf)
{
	car.cmd = buf[0];
	car.speed = (buf[1] << 8) | buf[2];
	car.steer = buf[3];

	Debug_Print("[Cmd] Cmd:%d, Speed:%d, Steer:%d\r\n",
				car.cmd,
				car.speed, car.steer);

	switch (car.cmd)
	{
		case 0:
			if (car.steer != 0)		// 自转逻辑
			{
				double spin_ratio = fabs(car.steer) / 100.0f;
				spin_ratio = spin_ratio > 1.0f ? 1.0f : spin_ratio;

				// 线性计算自转速度（0~SPIN_MAX_SPEED）
				int16_t spin_speed = (int16_t) (spin_ratio * SPIN_MAX_SPEED);

				// 自转关键→左右轮反向（steer>0=右自转，<0=左自转）
				int16_t left_speed = (car.steer > 0) ? -spin_speed : spin_speed;
				int16_t right_speed = -left_speed; // 右轮速度与左轮相反

				// 执行自转（同步控制前后电机）
				Motor_Set(left_speed, right_speed);
			}
			else
			{
				Motor_Set(0, 0); // 无转向停止
			}
			break;

		case 1: // 前进
		case 2: // 后退
		{
			int16_t base_speed = (car.cmd == 1) ? car.speed : -car.speed;
			int16_t steer_offset = 0;

			// 转向差速（加入死区，避免轻微抖动）
			if (abs(car.steer) > DEAD_ZONE_STEER)
			{
				steer_offset = (base_speed * car.steer) / 100;
			}

			// 四驱差速计算（左右轮同步）
			int16_t left_speed = base_speed - steer_offset;
			int16_t right_speed = base_speed + steer_offset;
			left_speed = constrain(left_speed, -PWM_MAX, PWM_MAX);
			right_speed = constrain(right_speed, -PWM_MAX, PWM_MAX);

			Motor_Set(left_speed, right_speed);
			break;
		}

		case 3: // 紧急刹车
			Motor_Set(0, 0);
			Debug_Print("[Brake] Emergency Brake Triggered\r\n");
			break;

		default: // 未知指令
			Motor_Set(0, 0);
			Debug_Print("[Error] Unknown Cmd:%d\r\n", car.cmd);
			break;
	}
}

/**
 * @brief  处理USART2 DMA接收的数据（凑4字节解析）
 */
static void Car_ProcessDMAData(void)
{
	static uint8_t cmd_buf[CMD_LEN];
	static uint8_t cmd_cnt = 0;

	for (uint16_t i = 0; i < car.dma_recv_len; i++)
	{
		cmd_buf[cmd_cnt++] = car.dma_recv_buf[i];
		if (cmd_cnt >= CMD_LEN)
		{
			Car_ParseCmd(cmd_buf);
			cmd_cnt = 0;
			car.last_recv_time = HAL_GetTick(); // 更新接收时间
		}
	}
	car.dma_recv_len = 0;
}

/**
 * @brief  小车初始化（按你要求命名：MainTask_Init）
 */
void MainTask_Init(void)
{
	// 初始化电机 PWM_Start
	Motor_Init();

	car.last_recv_time = HAL_GetTick();
	memset(car.dma_recv_buf, 0, UART2_DMA_BUF_LEN);
	car.dma_recv_len = 0;

	HAL_UART_Receive_DMA(&huart2, car.dma_recv_buf, UART2_DMA_BUF_LEN);

	Debug_Print("[Init] Car System Ready (USART2 DMA, Baud:115200)\r\n");
}

/**
 * @brief  主循环处理（按你要求命名：Loop）
 */
void MainTask_Loop(void)
{
	car.dma_recv_len = UART2_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

	// 解析指令
	if (car.dma_recv_len > 0)
	{
		Car_ProcessDMAData();
		HAL_UART_AbortReceive(&huart2);
		HAL_UART_Receive_DMA(&huart2, car.dma_recv_buf, UART2_DMA_BUF_LEN);
	}

	// 断联保护
	if (HAL_GetTick() - car.last_recv_time > LOST_CONNECT_TIMEOUT)
	{
		Motor_Set(0, 0);
		Debug_Print("[Protect] Lost Connection, Stop Motor\r\n");
		car.last_recv_time = HAL_GetTick();
	}
}
