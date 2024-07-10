/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ******************************************************************
                        AGV_CHASIS
****************************************************************** */
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//云台任务控制间隔 1ms
#define GIMBAL_CONTROL_TIME 1

// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH

//掉头云台速度
#define TURN_SPEED 0.06f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

//灵敏度
#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000004f 
#define YAW_MOUSE_SEN 0.00004f
#define PITCH_MOUSE_SEN 0.0001f
#define YAW_ENCODE_SEN 0.0f  
#define PITCH_ENCODE_SEN 0.01f

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

///////////////////////////电机，陀螺仪安装方向////////////////////////////////
#define PITCH_TURN 0
#define PITCH_OUT_TURN 0
#define YAW_TURN 1
#define YAW_OUT_TURN 1
#define YAW_TURN_GYRO 0

// pitch轴角度环（绝对角度）
#define PITCH_ABSOLUTE_ANGLE_KP 25.0f//90.0f
#define PITCH_ABSOLUTE_ANGLE_KI 0.0f//0.02f//0.5f//0.3f
#define PITCH_ABSOLUTE_ANGLE_KD 0.0f//-0.5f//-1.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 20.0f//5.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT 1.0f//2.0f
// pitch轴速度环
#define PITCH_GYRO_KP 5000.0f
#define PITCH_GYRO_KI 0.0f//0.8f
#define PITCH_GYRO_KD 0.0f//-0.5f
#define PITCH_GYRO_MAX_OUT 17000.0f//23000.0f
#define PITCH_GYRO_MAX_IOUT 0.0f
// yaw轴角度环（绝对角度）
#define YAW_ABSOLUTE_ANGLE_KP 45.0f//30
#define YAW_ABSOLUTE_ANGLE_KI 0.0f
#define YAW_ABSOLUTE_ANGLE_KD -1.0f
#define YAW_ABSOLUTE_ANGLE_MAX_OUT 40.0f//10.0f//15
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 0.0f//3.0f
// yaw轴速度环
#define YAW_GYRO_KP 5000.0f//2500.0f//1000
#define YAW_GYRO_KI 0.0f
#define YAW_GYRO_KD 0.0f//-0.5f
#define YAW_GYRO_MAX_OUT 20000.0f//2000.0f//5000.0f
#define YAW_GYRO_MAX_IOUT 0.0f//1000.0f

//云台限位及中值参数设定
// extern float GIMBAL_YAW_OFFSET_ECD;
#define GIMBAL_YAW_OFFSET_ECD 3342//2768//0.64f
#define GIMBAL_YAW_MAX_ECD 1.57f//5.0f
#define GIMBAL_YAW_MIN_ECD -1.57f//-5.0f
#define GIMBAL_PITCH_OFFSET_ECD 7458//1.72f
#define GIMBAL_PITCH_MAX_ECD 0.12f//2.06f
#define GIMBAL_PITCH_MIN_ECD -0.6f//1.33f

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif


typedef enum
{
  GIMBAL_MOTOR_RAW = 0, //电机原始值控制
  GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
  GIMBAL_MOTOR_ENCONDE  //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float set;
  float get;
  float err;

  float max_out;
  float max_iout;

  float Pout;
  float Iout;
  float Dout;

  float out;
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  gimbal_PID_t gimbal_motor_acc_pid;//zbl
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  uint16_t offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief 云台姿态数据发送到上位机
 *
 */
extern void gimbal_data_send(void);

extern void gimbal_task(void const *pvParameters);

#endif
