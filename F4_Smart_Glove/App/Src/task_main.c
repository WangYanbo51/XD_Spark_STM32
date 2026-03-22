/*
 * task_main.c
 *
 *  Created on: Mar 12, 2026
 *      Author: lenovo
 */

#include "task_main.h"
#include "mpu6050.h"
#include "gesture.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
//char nrf_failed[] = "NRF24L01 Check Failed!\r\n";
//char nrf_success[] = "NRF24L01 Check Success!\r\n";

MPU6050_t mpu_data;    // MPU数据结构体
Car_CmdDef car_cmd;    // 小车控制指令
uint32_t last_send_time = 0; // 上次发送指令时间
uint32_t sample_timer = 0;   // MPU采样计时

//#define _DEBUG_

#define PRINT_BUF_SIZE 256
char print_buf[PRINT_BUF_SIZE];

/**
 * @brief  USART1调试打印
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
 * @brief  串口打印函数
 */
static void Print_Info(void)
{
	const char *brake_state_str = (brake_state == BRAKE_STATE_LOCKED) ? "Brake Locked" : "No Brake";
	const char *cmd_str = "";
	switch (car_cmd.cmd)
	{
		case 0:
			cmd_str = "Stop";
			break;
		case 1:
			cmd_str = "Forward";
			break;
		case 2:
			cmd_str = "Backward";
			break;
		case 3:
			cmd_str = "Emergency Brake";
			break;
		default:
			cmd_str = "Unknown";
			break;
	}

	sprintf(print_buf,
			"[Gesture State] %s | [Car Cmd] %s | Speed: %d | Steering: %d\r\n"
			"[Kalman Angle] X: %.1f | Y: %.1f | [Angular Velocity] Gz: %.1f/s\r\n"
			"------------------------------------------------\r\n",
			brake_state_str,        // Brake state
			cmd_str,                // Car command
			car_cmd.speed,          // Speed value
			car_cmd.steer,          // Steering value
			mpu_data.KalmanAngleX,  // Kalman pitch angle (X)
			mpu_data.KalmanAngleY,  // Kalman roll angle (Y)
			mpu_data.Gz             // Z-axis angular velocity
			);

	HAL_UART_Transmit(&huart1, (uint8_t*) print_buf, strlen(print_buf), 20);
}
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint16_t addr;

	HAL_UART_Transmit(&huart1, (uint8_t*) "I2C Scan start...\r\n", 19, 100);

	for (addr = 1; addr < 127; addr++)
	{
		status = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10);

		if (status == HAL_OK)
		{
			char buf[30];
			sprintf(buf, "Found addr: 0x%02X\r\n", addr);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 100);
		}
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "Scan done.\r\n\r\n", 13, 100);
}

void MainTask_Init()
{
	I2C_Scan(&hi2c1);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "Init...\r\n", 10);
	HAL_Delay(1000);
	while (MPU6050_Init(&hi2c1) == 1);
	Gesture_Init();
	last_send_time = HAL_GetTick();
	sample_timer = HAL_GetTick();
}

void MainTask_Loop()
{
//	MPU6050_Read_All(&hi2c1, &MPU6050);
////	sprintf(tmp, "%.3lf, %.3lf, %.3lf\r\n", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY, MPU6050.KalmanAngleZ);
//	sprintf(tmp, "%.3lf, %.3lf, %.3lf\r\n", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
////	sprintf(tmp, "%d, %d, %d\r\n", MPU6050.Gyro_X_RAW, MPU6050.Gyro_Y_RAW,
////			MPU6050.Gyro_Z_RAW);
//
//	HAL_UART_Transmit(&huart1, (uint8_t*) tmp, sizeof(tmp), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t*) tmp, sizeof(tmp), HAL_MAX_DELAY);
//	HAL_Delay(5);
	if (HAL_GetTick() - sample_timer >= SAMPLE_PERIOD_MS)
	{
		MPU6050_Read_All(&hi2c1, &mpu_data);		// 采集传感器数据
		Gesture_GetCmd(&mpu_data, &car_cmd);		// 转换成控制指令
		sample_timer = HAL_GetTick();				// 延时
	}
	if (HAL_GetTick() - last_send_time >= SEND_PERIOD_MS)
	{
		// cmd(1) + speed高8位(1) + speed低8位(1) + steer(1)
		uint8_t tx_buf[4] = { 0 };
		tx_buf[0] = car_cmd.cmd;
		tx_buf[1] = (car_cmd.speed >> 8) & 0xFF;
		tx_buf[2] = car_cmd.speed & 0xFF;
		tx_buf[3] = car_cmd.steer;

		// 非阻塞
		HAL_UART_Transmit_DMA(&huart2, tx_buf, 4);

#ifdef _DEBUG_
		Print_Info();
#else
		Debug_Print("%.2f,%.2f,%.2f\r\n",
		            mpu_data.KalmanAngleX,
		            mpu_data.KalmanAngleY,
					mpu_data.KalmanAngleZ);
#endif
		// 更新发送时间
		last_send_time = HAL_GetTick();
	}

	HAL_Delay(1);
}
