/*
 * delay.c
 *
 *  Created on: 2026年3月13日
 *      Author: lenovo
 */

#include "delay.h"

void Delay_us(uint32_t nus)
{
	uint16_t differ = 0xffff - nus - 5;
	__HAL_TIM_SET_COUNTER(&htim9, differ);
	HAL_TIM_Base_Start(&htim9);

	while (differ < 0xffff - 5)
	{
		differ = __HAL_TIM_GET_COUNTER(&htim9);
	}
	HAL_TIM_Base_Stop(&htim9);
}
