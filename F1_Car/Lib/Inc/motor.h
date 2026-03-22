/*
 * motor.h
 *
 *  Created on: 2026年3月20日
 *      Author: lenovo
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "tim.h"

#define LEFT_PWM_CH1    TIM_CHANNEL_1  // 左前 → TIM2_CH1（L_IN1）
#define LEFT_PWM_CH2    TIM_CHANNEL_2  // 左前 → TIM2_CH2（L_IN2）
#define RIGHT_PWM_CH1   TIM_CHANNEL_1  // 右前 → TIM3_CH1（R_IN1）
#define RIGHT_PWM_CH2   TIM_CHANNEL_2  // 右前 → TIM3_CH2（R_IN2）

#define LEFT_BACK_PWM_CH1  TIM_CHANNEL_3  // 左后 → TIM2_CH3（L_IN3）
#define LEFT_BACK_PWM_CH2  TIM_CHANNEL_4  // 左后 → TIM2_CH4（L_IN4）
#define RIGHT_BACK_PWM_CH1 TIM_CHANNEL_3  // 右后 → TIM3_CH3（R_IN3）
#define RIGHT_BACK_PWM_CH2 TIM_CHANNEL_4  // 右后 → TIM3_CH4（R_IN4）

// 函数声明
void Motor_Init(void);
void Motor_Set(int16_t left_speed, int16_t right_speed);

#endif /* INC_MOTOR_H_ */
