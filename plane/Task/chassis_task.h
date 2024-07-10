/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//����PID��������
//����ٶ�PID
#define CHASSIS_MOTOR_SPEED_KP 12000.0f 
#define CHASSIS_MOTOR_SPEED_KI 0.0f
#define CHASSIS_MOTOR_SPEED_KD 0.0f
#define CHASSIS_MOTOR_SPEED_MAX_OUT 26000.0f//26000
#define CHASSIS_MOTOR_SPEED_MAX_IOUT 1000.0f

//���̸�����̨��תPID
#define CHASSIS_ANGLE_KP 28.0f//30
#define CHASSIS_ANGLE_KI 0.0f
#define CHASSIS_ANGLE_KD 0.0f
#define CHASSIS_ANGLE_MAX_OUT 20.0f
#define CHASSIS_ANGLE_MAX_IOUT 2.5f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ���δ���������
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// m3508ת���ɶ����ٶ�(m/s)�ı���
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�����������ٶ�
#define MAX_WHEEL_SPEED 5.0f
//4.0
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f//2.5f
//0.85
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 4.0f//2.0f
//0.75
//����С�����ٶ�
#define CHASSIS_TOP_SPEED 8.5f//7
//����С���������ƽ���ٶ�
#define TOP_MAX_CHASSIS_SPEED 5.5f

#define CHASSIS_WZ_SET_SCALE 0.20f//0.125f

typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   //��������
  CHASSIS_NO_MOVE,          //���̲���
  CHASSIS_FOLLOW_GIMBAL,    //������̨
  CHASSIS_TOP,              //С����
  CHASSIS_NO_FOLLOW_GIMBAL, //������
  CHASSIS_OPEN              //ң�ػ���ֵ���Ե���ֱ�ӷ��͵��
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;          //���ٶ�
  float speed;          //��ǰ�ٶ�
  float speed_set;      //�趨�ٶ�
  int16_t give_current; //���͵���
} chassis_motor_t;

typedef struct
{
  const gimbal_motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const float *chassis_INS_angle;            //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_motor_t motor_chassis[4];          //���̵������
  pid_type_def motor_speed_pid[4];           //���̵���ٶ�pid
  pid_type_def chassis_angle_pid;            //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx; //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy; //ʹ��һ�׵�ͨ�˲������趨ֵ

  float vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad
  float chassis_relative_angle_set; //���������̨���ƽǶ�
  float chassis_yaw_set;

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //���˷�������ٶ� ��λm/s
  float vy_max_speed;  //��������ٶ� ��λm/s
  float vy_min_speed;  //�ҷ�������ٶ� ��λm/s
  float top_max_speed; //С����ƽ������������ٶ�
  float top_min_speed; //С����ƽ�Ʒ���������ٶ�
  float chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  float chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  float chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_move_t;

/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
