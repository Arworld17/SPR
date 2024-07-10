/**
 * @file shoot_task.h
 * @author 何清华
 * @brief 包含对摩擦轮以及拨盘的控制
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __shoot_TASK_H__
#define __shoot_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//射速优先置为1
#define PRE_SHOOT 0

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME        300
//射击任务控制间隔 5ms
#define SHOOT_CONTROL_TIME          5

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       RIGHT_SWITCH

//鼠标长按判断 100 * 5 = 500ms
#define PRESS_LONG_TIME             100
//遥控器射击开关开启一段时间后 连续发射子弹 用于清弹 400 * 5 = 2000ms
#define RC_S_LONG_TIME              400

//发射机构pid参数
//摩擦轮
#define FRICTION_KP 13.5f// 13.5f
#define FRICTION_KD 0.0f//0.0f
#define FRICTION_KI 0.0f//0.0f
#define FRICTION_MAX_OUT 8000.0f//8000.0f
#define FRICTION_MAX_IOUT 4000.0f



//#define FRICTION_KP 28.5f// 13.5f
//#define FRICTION_KD 0.85f//0.0f
//#define FRICTION_KI 0.23f//0.0f
//#define FRICTION_MAX_OUT 18500.0f//8000.0f
//#define FRICTION_MAX_IOUT 4000.0f
// Kp 9.0 Kd 0.05 Ki 0.0
//拨盘
#define TRIGGER_KP 12.0f 
#define TRIGGER_KI 0.01f//0.05
#define TRIGGER_KD 0.0f
#define TRIGGER_MAX_OUT 10000.0f
#define TRIGGER_MAX_IOUT 2000.0f//9500

//拨弹速度
#define TRIGGER_SPEED     -7000.0f//-1250单发转速
//#define CONTINUE_TRIGGER_SPEED      -7000.0f//连发转速
#define READY_TRIGGER_SPEED         0.0f//勿改
//-350
//-500
//摩擦轮速度
#define FRICTION_SPEED_SET         9000.0f
//一级步兵4005.0f  上限15
//二级步兵4580.0f  上限18 
//大幅 -11000.0f

#define FRICTION_ACCEL_MAX_OUT      9000.0f//摩擦轮启动加速电流限幅

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  25 //5 * 400 = 2000ms 
#define REVERSE_TIME                200 //5 * 150 = 750ms
#define REVERSE_SPEED_LIMIT         13.0f

//热量预留值，防止超热量
#define SHOOT_HEAT_REMAIN_VALUE     5
//80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,//开摩擦轮
    SHOOT_BULLET,//子弹
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
    int8_t ecd_count;
} trigger_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //加速度
    float speed;          //当前速度
    float speed_set;      //设定速度
    int16_t given_current; //发送电流
} friction_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;

    trigger_motor_t trigger_motor;
    friction_motor_t right_fricition_motor;
    friction_motor_t left_fricition_motor;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;

    uint16_t heat_limit;
    uint16_t heat;
    uint16_t bullet_speed_limit;
    uint16_t bullet_speed;
    uint16_t bullet_last_speed;

} shoot_control_t;

void fire_task(void const *pvParameters);

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
