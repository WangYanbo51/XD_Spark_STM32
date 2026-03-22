#ifndef __GESTURE_H
#define __GESTURE_H

#include "mpu6050.h"

// =====================================================================+
#define DEAD_ZONE_PITCH      10.0f   // 俯仰死区
#define DEAD_ZONE_ROLL       10.0f   // 横滚死区
#define MAX_PITCH_ANGLE      50.0f   // 最大俯仰角（°）- 10°~50°线性调速
#define MAX_ROLL_ANGLE       40.0f   // 最大横滚角（°）- 10°~40°线性转向
#define BRAKE_GYRO_THRESHOLD 80.0f   // 抬手触发刹车的角速度阈值（°/s）<80°/s上限	（弃用）
#define BRAKE_HOLD_PITCH     10.0f   // 刹车保持：掌面朝前（俯仰角<10°）			（弃用）
#define BRAKE_BACK_PITCH     20.0f   // 刹车保持：后倾角度>20°				（弃用）
#define SAMPLE_PERIOD_MS     10      // 10ms高频采样（保证手势响应速度）
#define SEND_PERIOD_MS       50      // 50ms发送指令（避免串口过载）
#define BRAKE_BACK_ANGLE     60.0f   // 后抬角度触发阈值（KalmanAngleX < -30°）
#define BRAKE_UNLOCK_PITCH   10.0f   // 解锁条件（手放平阈值|KalmanAngleX| < 10°）	（弃用）
// ======================================================================

typedef enum
{
	BRAKE_STATE_RELEASE = 0,
	BRAKE_STATE_LOCKED = 1   // 刹车锁定
} Brake_StateDef;

typedef struct
{
	uint8_t cmd;    // 0-停止 1-前进 2-后退 3-紧急刹车
	uint16_t speed;  // 0~999（PWM占空比，线性调速）
	int8_t steer;  // -100~100（左转负/右转正，线性转向）
} Car_CmdDef;

extern Brake_StateDef brake_state;

void Gesture_Init(void);
void Gesture_GetCmd(MPU6050_t *mpu_data, Car_CmdDef *cmd);

#endif
