/**
 * @file gimbal_behaviour.h
 * @author 何清华
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"
#include "gimbal_task.h"

typedef enum
{
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_INIT,
  GIMBAL_ABSOLUTE_ANGLE,
  GIMBAL_RELATIVE_ANGLE,
  GIMBAL_MOTIONLESS,
  GIMBAL_AUTO
} gimbal_behaviour_e;

/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */
extern void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_chassis_stop(void);

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
extern bool_t gimbal_cmd_to_shoot_stop(void);

#endif
