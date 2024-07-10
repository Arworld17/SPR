/**
 * @file gimbal_behaviour.c
 * @author 何清华
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "config.h"
#include "filter.h"
#include "chassis_behaviour.h"
#include "shoot_task.h"

//当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()
chassis_mode_e chassis_mode; 

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

		
extern shoot_control_t shoot_control;
		
/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台进入自瞄模式，通过上位机发送的数据控制
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//云台行为状态机
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */

void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint8_t last_behaviour = GIMBAL_ZERO_FORCE;
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO)
    {
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
    }

    if (last_behaviour != gimbal_behaviour)
    {
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    }

    last_behaviour = gimbal_behaviour;
}

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //开关控制 云台状态(非必要不建议改)
    if (switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //上
    {
#ifdef AUTO_DEBUG
        gimbal_behaviour = GIMBAL_AUTO;
#else
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
#endif
    }
    else if (switch_is_mid(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //中
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_down(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //下
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

	
	
	//当处于跟随模式时，长按右键开启自瞄模式
//#ifdef AUTO_DEBUG
//        

//        
//#else
//   if (chassis_mode == CHASSIS_FOLLOW_GIMBAL)
//   {
//	    if(rc_ctrl.mouse.press_r == 1)
//	    {
//	        gimbal_behaviour = GIMBAL_AUTO;
//			chassis_mode = CHASSIS_NO_MOVE;
//	    }
//	    else
//	    {
//	        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//			chassis_mode = CHASSIS_FOLLOW_GIMBAL;
//	    }	
//	}

//#endif
	
	
   
}

/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	if(shoot_control.press_r_time < 100)
	{
	*yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN;
	*pitch = - pitch_channel * PITCH_RC_SEN - rc_ctrl.mouse.y * PITCH_MOUSE_SEN;
	}
	else
	{
	*yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN * 0.1f;
	*pitch = - pitch_channel * PITCH_RC_SEN - rc_ctrl.mouse.y * PITCH_MOUSE_SEN * 0.1f;
	}
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

		if(shoot_control.press_r_time < 0x0064)
		{
			*yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN;
			*pitch = pitch_channel * PITCH_RC_SEN + rc_ctrl.mouse.y * PITCH_MOUSE_SEN;
		}
		else
		{
			*yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN * 0.2f;
			*pitch = pitch_channel * PITCH_RC_SEN + rc_ctrl.mouse.y * PITCH_MOUSE_SEN * 0.2f;
		}
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

Kf kf_yaw = {0.01f, 30.0f, 0.0f, 0.0f};
Kf kf_pitch = {0.01f, 60.0f, 0.0f, 0.0f};
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static float auto_yaw;
    static float auto_pitch;

    //向上位机发送姿态数据
    // gimbal_data_send();

    if (auto_shoot.new_usart_data_flag == 1)
    {
        auto_shoot.new_usart_data_flag = 0;
        auto_yaw = gimbal_control_set->gimbal_yaw_motor.absolute_angle + auto_shoot.yaw_add +0.005f;
        auto_pitch = gimbal_control_set->gimbal_pitch_motor.absolute_angle + auto_shoot.pitch_add;

    }

    if (toe_is_error(USER_USART_DATA_TOE))
    {
        *yaw = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        *pitch = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
    }
    else
    {
        *yaw = KalmanFilter(auto_yaw, &kf_yaw);
        *pitch = KalmanFilter(auto_pitch, &kf_pitch);
    }
}


