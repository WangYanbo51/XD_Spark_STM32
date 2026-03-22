#include "gesture.h"
#include "math.h"
#include "mpu6050.h"

Brake_StateDef brake_state = BRAKE_STATE_RELEASE; // 刹车状态
static uint8_t brake_trigger_cnt = 0;            // 刹车触发计数
#define BRAKE_TRIGGER_CNT 1						//连续满足条件触发brake

/**
 * @brief  手势识别初始化
 */
void Gesture_Init(void)
{
	brake_state = BRAKE_STATE_RELEASE;
	brake_trigger_cnt = 0;
}

/**
 * @brief  刹车判断核心逻辑
 * @param  mpu_data: MPU数据
 * @retval 1-刹车锁定，0-刹车释放
 */
static uint8_t Check_Brake(MPU6050_t *mpu_data)
{
	if (mpu_data->KalmanAngleX < -BRAKE_BACK_ANGLE)
	{
		brake_trigger_cnt++;
		if (brake_trigger_cnt >= BRAKE_TRIGGER_CNT)
		{
			brake_state = BRAKE_STATE_LOCKED;
			brake_trigger_cnt = 0; // 复位计数
			return 1;
		}
	}
	else
	{
		brake_trigger_cnt = 0; // 不满足阈值则复位计数
	}

	if (brake_state == BRAKE_STATE_LOCKED)
	{
		if (fabs(mpu_data->KalmanAngleX) < BRAKE_UNLOCK_PITCH)
		{
			brake_state = BRAKE_STATE_RELEASE; // 解锁刹车
			return 0;
		}
		else
		{
			return 1; // 未放平，持续锁定
		}
//        if(fabs(mpu_data->KalmanAngleX) < BRAKE_HOLD_PITCH || mpu_data->KalmanAngleX < -BRAKE_BACK_PITCH)
//        {
//            return 1;
//        }
//        else
//        {
//            brake_state = BRAKE_STATE_RELEASE;
//            return 0;
//        }
	}

	return 0;
}

/**
 * @brief  核心：手势解析→小车控制指令
 * @param  mpu_data: MPU数据
 * @param  cmd:      控制指令
 */
void Gesture_GetCmd(MPU6050_t *mpu_data, Car_CmdDef *cmd)
{
	if (Check_Brake(mpu_data))		// 紧急刹车优先级最高
	{
		cmd->cmd = 3;    // 紧急刹车指令
		cmd->speed = 0;  // 速度清零
		cmd->steer = 0;  // 转向清零
		return;
	}

	double pitch_abs = fabs(mpu_data->KalmanAngleX);
	if (pitch_abs < DEAD_ZONE_PITCH)
	{
		cmd->cmd = 0;
		cmd->speed = 0;
	}
	else
	{
		// 线性调速公式：(真实角度-死区)/(最大角度-死区) * 999
		double speed_ratio = (pitch_abs - DEAD_ZONE_PITCH) / (MAX_PITCH_ANGLE - DEAD_ZONE_PITCH);
		cmd->speed = (uint16_t) (speed_ratio * 999);
		cmd->speed = cmd->speed > 999 ? 999 : cmd->speed; // 限幅

		// KalmanAngleX>0=前进；KalmanAngleX<0=后退
		cmd->cmd = (mpu_data->KalmanAngleX > 0) ? 1 : 2;
	}

	double roll_abs = fabs(mpu_data->KalmanAngleY);
	if (roll_abs < DEAD_ZONE_ROLL)
	{
		cmd->steer = 0;  // 死区内→无转向
	}
	else
	{
		// 线性转向公式：(真实角度-死区)/(最大角度-死区) * 100
		double steer_ratio = (roll_abs - DEAD_ZONE_ROLL) / (MAX_ROLL_ANGLE - DEAD_ZONE_ROLL) * (mpu_data->KalmanAngleY > 0 ? 1 : -1);
		steer_ratio = steer_ratio < -1 ? -1 : (steer_ratio > 1 ? 1 : steer_ratio);
		cmd->steer = (int8_t) (steer_ratio * 100);
	}
}
