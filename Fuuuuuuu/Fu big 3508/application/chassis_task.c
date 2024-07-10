/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"

#include "gpio.h"

#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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

chassis_motor_t motor_debug;
	
	int16_t speed_set_debug = 600;//小fu的转速是pi/3 rad/s   换算为rpm是10

	
float speed=0; 
float a;//  0.780~1.045
float b;//  b=2.090- a
float w;//  1.884~2.000
float t=0;
float tt=0;
uint8_t skip_flag=0;
uint8_t get_flag=0;
	
float getRandomFloat(float min, float max)//获得在min和max之间的一个值
{
    float random = ((float)rand()) / (float)RAND_MAX;//随机0到1之间的值
    return min + (random * (max - min));
}

float Calculation(float speed)//  rad/s换算为rpm
{
	float rpm=speed*60/(2*3.14159);
	return rpm;
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;

	int sin_n;
	int sin_factor = 4096/2 ;
	float _sin; 
	short  is;
	double pi=3.14159;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{

	
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    //判断底盘电机是否都在线
    /*while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
	*/
    while (1)
    {
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

{

		a=getRandomFloat(0.780,1.045);
		b=2.090- a;
		w=getRandomFloat(1.884,2.000);
		//tt=HAL_GetTick();
		speed = a*sin(w*t)+b;
		speed=Calculation(speed);
		speed_set_debug=speed*36;
		tt++;
		if(tt>=1000)
		{
		t++;
		tt=0;
		//tt=HAL_GetTick();
		}
//		if(speed_set_debug<200)
//		{
//		speed_set_debug=200;
//		}
//		if(get_flag==2)
//		{
//		get_flag=0;
//		speed_set_debug	=speed_set_debug+300;
//		}
//		if(key_pressed())
//		{
//			speed_set_debug += 500;
//		}
//		
//		if( speed_set_debug>5000 ) speed_set_debug = 5000;
	
	
		motor_debug.speed = chassis_move.motor_chassis[0].chassis_motor_measure->speed_rpm;
		motor_debug.speed_set = chassis_move.motor_chassis[0].speed_set;
		
		
		
		CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		
//		vTaskDelay(2);
	
//		CAN_cmd_gimbal(chassis_move.motor_chassis[4].give_current, chassis_move.motor_chassis[5].give_current,
//                                chassis_move.motor_chassis[6].give_current, chassis_move.motor_chassis[7].give_current);
	
}
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static uint8_t key_pressed(void)
{
	static uint8_t key_first_pressed_flag = 1;
	
	if(key_first_pressed_flag)
	{
		if(!HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0))
		{
			HAL_Delay(1);
			if(!HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0)) 
			{
				key_first_pressed_flag = 0;
				return 1;
			}
		}
	}else
	{
		if( HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0) )key_first_pressed_flag = 1;
		
		return 0;
	}
	return 0;
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //获取云台电机数据指针
//    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 8; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 8; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    chassis_move_control->motor_chassis[0].speed_set = -speed_set_debug;

}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    uint8_t i = 0;

    //calculate pid
    //计算pid
    for (i = 0; i < 8; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->speed_rpm, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //赋值电流值
    for (i = 0; i < 8; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
