/*
 * motor.c
 *
 *  Created on: 2026年3月20日
 *      Author: lenovo
 */
#include "motor.h"
#include "task_main.h"

/**
 * @brief  电机初始化（复用你的引脚，启动TIM2/TIM3 PWM）
 */
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, LEFT_PWM_CH1);
	HAL_TIM_PWM_Start(&htim2, LEFT_PWM_CH2);
	HAL_TIM_PWM_Start(&htim3, RIGHT_PWM_CH1);
	HAL_TIM_PWM_Start(&htim3, RIGHT_PWM_CH2);

	HAL_TIM_PWM_Start(&htim2, LEFT_BACK_PWM_CH1);
	HAL_TIM_PWM_Start(&htim2, LEFT_BACK_PWM_CH2);
	HAL_TIM_PWM_Start(&htim3, RIGHT_BACK_PWM_CH1);
	HAL_TIM_PWM_Start(&htim3, RIGHT_BACK_PWM_CH2);

	// 初始状态停止
	Motor_Set(0, 0);
}

/**
 * @brief  HW-856差分PWM控制（无方向引脚，适配四驱）
 * @param  speed: 速度(-999~999，正=正转，负=反转)
 * @param  tim: TIM句柄（&htim2=左轮，&htim3=右轮）
 * @param  ch1: 正转通道（IN1），ch2: 反转通道（IN2）
 */
static void HW856_SetSpeed(int16_t speed, TIM_HandleTypeDef *tim,
							uint32_t ch1, uint32_t ch2,
							uint32_t back_ch1, uint32_t back_ch2)
{
	uint16_t pwm = abs(speed) > PWM_MAX ? PWM_MAX : abs(speed);

	if (speed > 0)
	{ // 正转：CH1=PWM，CH2=0（前后电机同步）
		__HAL_TIM_SET_COMPARE(tim, ch1, pwm);       // 前电机CH1
		__HAL_TIM_SET_COMPARE(tim, ch2, 0);         // 前电机CH2
		__HAL_TIM_SET_COMPARE(tim, back_ch1, pwm);  // 后电机CH1（同步）
		__HAL_TIM_SET_COMPARE(tim, back_ch2, 0);    // 后电机CH2（同步）
	}
	else if (speed < 0)
	{ // 反转：CH1=0，CH2=PWM（前后电机同步）
		__HAL_TIM_SET_COMPARE(tim, ch1, 0);
		__HAL_TIM_SET_COMPARE(tim, ch2, pwm);
		__HAL_TIM_SET_COMPARE(tim, back_ch1, 0);
		__HAL_TIM_SET_COMPARE(tim, back_ch2, pwm);
	}
	else
	{ // 停止：所有通道PWM=0
		__HAL_TIM_SET_COMPARE(tim, ch1, 0);
		__HAL_TIM_SET_COMPARE(tim, ch2, 0);
		__HAL_TIM_SET_COMPARE(tim, back_ch1, 0);
		__HAL_TIM_SET_COMPARE(tim, back_ch2, 0);
	}
}

/**
 * @brief  四驱电机控制（复用你的L_IN1~L_IN4/R_IN1~R_IN4，前后轮并联）
 * @param  left_speed: 左轮速度(-999~999)，right_speed: 右轮速度(-999~999)
 */
void Motor_Set(int16_t left_speed, int16_t right_speed)
{
	// 左右轮子是互相反着装的
	HW856_SetSpeed(-left_speed, &htim2, LEFT_PWM_CH1, LEFT_PWM_CH2,
	LEFT_BACK_PWM_CH1,
					LEFT_BACK_PWM_CH2);
	HW856_SetSpeed(right_speed, &htim3, RIGHT_PWM_CH1, RIGHT_PWM_CH2,
	RIGHT_BACK_PWM_CH1,
					RIGHT_BACK_PWM_CH2);

	Debug_Print("[Motor] Left:%d, Right:%d\r\n", left_speed, right_speed);
}
