/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"


#define POWER_LIMIT_1 60.0f;        //��������
#define POWER_LIMIT_2 80.0f; 
#define POWER_LIMIT_3 100.0f; 

float POWER_LIMIT;   //��������
float WARNING_POWER; //���湦��

#define WARNING_CAP_VOLTAGE 16.0f //���ݾ����ѹ
#define MIN_CAP_VOLTAGE 13.0f     //���ݵ�ѹ����
float WARNING_CAP_BUFF = WARNING_CAP_VOLTAGE - MIN_CAP_VOLTAGE;

extern ext_game_robot_state_t robot_state;
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 10000.0f

float power_total_current_limit = 0.0f;
float buff_total_current_limit = 0.0f;
float total_current_limit = 0.0f;
float total_current = 0.0f;

float cap_voltage = 0.0f;
float cap_buff = 0.0f;

extern ext_game_robot_state_t robot_state;
float limit_k = 0.0f;

float powerbuffer_const = 0.0f;

/**
 * @brief          limit the power, mainly limit motor current
 * @param[in]      chassis_power_control: chassis data
 * @retval         none
 */
/**
 * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
 * @param[in]      chassis_power_control: ��������
 * @retval         none
 */
void chassis_power_control(chassis_move_t *chassis_power_control)
{

    // /**************************** shift���� **********************************************************/
    //  if (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)
    //  {
    //      chassis_power_control->vx_max_speed = 3.0;
    //      chassis_power_control->vx_min_speed = -3.0;
    //  }
    //  else
    //  {
    //      chassis_power_control->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    //      chassis_power_control->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    //  }

    power_total_current_limit = (robot_state.chassis_power_limit / 24.0f) * 1000.0f;
    cap_voltage = PowerData[1];//��ȡ��ǰ��ѹ
    cap_buff = cap_voltage - MIN_CAP_VOLTAGE;

    if (cap_buff < WARNING_CAP_BUFF)
    {
        float power_scale;
        if (cap_buff > 0.5f)
        {
            power_scale = cap_buff / WARNING_CAP_BUFF;
        }
        else
        {
            power_scale = 0.5f / WARNING_CAP_BUFF;
        }
        //��С
        buff_total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
    }
    else
    {
        buff_total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT;
    }

			  total_current_limit = power_total_current_limit + buff_total_current_limit;

    total_current = 0.0f;
   // calculate the original motor current set
   // //����ԭ����������趨
    for (uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }

    if (total_current > total_current_limit)
    {
        float current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out *= current_scale;
        chassis_power_control->motor_speed_pid[1].out *= current_scale;
        chassis_power_control->motor_speed_pid[2].out *= current_scale;
        chassis_power_control->motor_speed_pid[3].out *= current_scale;
    }
	
	
	
	float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
   //     /**************************************�ϰ湦������****************************************************************************/

       //�жϻ��������Ƿ�λ��60J��250J֮��
        if (chassis_power_buffer > 60.0f && chassis_power_buffer <= 250.0f)
            powerbuffer_const = chassis_power_buffer;
        else
            powerbuffer_const = 60.0f; //���򻺳�����Ϊ60J
        limit_k = 0.25f * ((float)chassis_power_buffer / powerbuffer_const) * ((float)chassis_power_buffer / powerbuffer_const) *
                      ((float)chassis_power_buffer / powerbuffer_const) +
                  0.5f * ((float)chassis_power_buffer / powerbuffer_const) *
                      ((float)chassis_power_buffer / powerbuffer_const) +
                 0.25f * ((float)chassis_power_buffer / powerbuffer_const);
        chassis_power_control->motor_speed_pid[0].out *= limit_k;
        chassis_power_control->motor_speed_pid[1].out *= limit_k;
        chassis_power_control->motor_speed_pid[2].out *= limit_k;
        chassis_power_control->motor_speed_pid[3].out *= limit_k;
   
}
