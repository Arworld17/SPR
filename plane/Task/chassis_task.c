/**
 * @file chassis_task.c/h
 * @author 何清华，周鸣阳
 * @brief 底盘控制任务线程
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "chassis_task.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "referee.h"
#include "custom_ui_draw.h"
#include "CAN_receive.h"
/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @retval         none
 */
static void chassis_init(void);

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @retval         none
 */
static void chassis_set_mode(void);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(void);

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @retval         none
 */
static void chassis_set_contorl(void);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(void);

//底盘运动数据
chassis_move_t chassis_move;
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         nonex
 */
void chassis_data_send(void);
void super_cap_send_power(uint16_t Power);

float chassis_debug[10] = {0};

extern ext_game_robot_state_t robot_state;  //步兵等级
extern super_capacity_t super_capacity;
extern aim_dynamic_t aim_dynamic;

extern tank_ui_t tank;
void chassis_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //底盘初始化
  chassis_init();
  //5.判断底盘电机是否都在线
  // while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
  // {
  //   vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  // }

  while (1)
  {
    //设置底盘控制模式
    chassis_set_mode();
    //底盘数据更新
    chassis_feedback_update();
    //底盘控制量设置
    chassis_set_contorl();
    //ui界面
    cal_draw_gimbal_relative_angle_tangle(&tank);
    cal_capacity(&super_capacity);
    cal_draw_pitch_aim(&aim_dynamic);

    //底盘控制PID计算
    chassis_control_loop();
    chassis_data_send();

    //当遥控器掉线的时候，发送给底盘电机零电流.
    if (toe_is_error(DBUS_TOE))
    {
      CAN_cmd_chassis(0, 0, 0, 0);
    }
    else
    {
      //发送控制电流
      CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                    chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			//CAN_cmd_chassis(0,0,0,0);
    }
    
    //底盘js调试数据
		chassis_debug[0] = chassis_move.motor_chassis[0].speed;
		chassis_debug[1] = chassis_move.motor_chassis[1].speed;
		chassis_debug[2] = chassis_move.motor_chassis[2].speed;
		chassis_debug[3] = chassis_move.motor_chassis[3].speed;
		chassis_debug[4] = chassis_move.motor_chassis[0].speed+chassis_move.motor_chassis[1].speed+chassis_move.motor_chassis[2].speed+chassis_move.motor_chassis[3].speed;
        chassis_debug[5] = chassis_move.motor_chassis[0].speed_set;

    //系统延时
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @retval         none
 */
static void chassis_init(void)
{
  const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  //获取云台电机数据指针
  chassis_move.chassis_yaw_motor = get_yaw_motor_point();
  chassis_move.chassis_pitch_motor = get_pitch_motor_point();
  //获取底盘电机数据指针，初始化PID
  chassis_move.motor_chassis[0].chassis_motor_measure = get_chassis_motor_measure_point(0);
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT, CHASSIS_MOTOR_SPEED_MAX_IOUT);
  }

  //初始化角度PID(旋转跟随云台)
  PID_init(&chassis_move.chassis_angle_pid, CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);

  //用一阶滤波代替斜波函数生成
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  //最大 最小速度
  chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  chassis_move.top_max_speed = TOP_MAX_CHASSIS_SPEED;
  chassis_move.top_min_speed = -TOP_MAX_CHASSIS_SPEED;

  //更新一下数据
  chassis_feedback_update();
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @retval         none
 */
static void chassis_set_mode()
{
  chassis_behaviour_mode_set(&chassis_move);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @retval         none
 */
static void chassis_feedback_update(void)
{
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //更新电机速度，加速度是速度的PID微分
    chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  //更新底盘纵向速度 x，平移速度y，旋转速度wz，坐标系为右手系
  chassis_move.vx = (chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move.vy = (chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move.wz = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @retval         none
 */
static void chassis_set_contorl(void)
{
  chassis_behaviour_control_set(&chassis_move);
}
////////////////电调ID/////////////////
/////////1********前*******2///////////
/////////*                 *///////////
/////////*        x        *///////////
/////////左       ↑→y      右//////////
/////////*                 *///////////
/////////*                 *///////////
/////////4********后********3//////////
///////////////////////////////////////
/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
  wheel_speed[0] = vx_set - vy_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = -vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(void)
{
  float max_vector = 0.0f, vector_rate = 0.0f;
  float temp = 0.0f;
  float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

  //麦轮运动分解
  chassis_vector_to_mecanum_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);

  if (chassis_move.chassis_mode == CHASSIS_OPEN || chassis_move.chassis_mode == CHASSIS_ZERO_FORCE)
  {

    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
    }
    // raw控制直接返回
    return;
  }

  //计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move.motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].speed_set *= vector_rate;
    }
  }

  //计算pid
  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);
  }

  //功率控制
  //chassis_power_control(&chassis_move);

  //赋值电流值
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(chassis_move.motor_speed_pid[i].out);
  }
}

void chassis_data_send(void)
{
  static uint8_t send_time;
  //每5毫秒发送一次  500Hz
  if (send_time == 50)
  {
    super_cap_send_power(robot_state.chassis_power_limit * 100);
    send_time = 0;
  }
  send_time++;
}

