/**
 * @file USART_receive.h
 * @author ���廪
 * @brief �����жϽ��պ���������������Ƭ���������豸�Ĵ���ͨ������
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 18u
#define USER_FRAME_LENGTH 6u

#define USART_PI 3.1416f

typedef struct
{
    float pitch_add;
    float yaw_add;
    bool_t new_usart_data_flag;
} auto_shoot_t;

typedef struct
{
    float *gimbal_pitch_angle;
    float *gimbal_yaw_gyro;
    uint8_t enemy_color;
    uint8_t level;
} user_send_data_t;

extern auto_shoot_t auto_shoot;
extern user_send_data_t user_send_data;

extern void user_data_pack_handle(void);

#endif
