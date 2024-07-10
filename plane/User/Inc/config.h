/**
 * @file config.h
 * @author 何清华；周鸣阳
 * @brief 预编译设置
 * @version 1.0
 * @date 2022-03-05
 * 
 * @copyright Copyright (c) 2022 SPR
 * 
 */

#ifndef __CONFIG_H
#define __CONFIG_H

//线程开关
//#define CHASSIS_TASK//底盘线程
#define GIMBAL_TASK//云台线程
#define REFEREE_TASK//裁判系统收发线程
#define SHOOT_TASK//舵机线程
#define DETECT_TASK//错误检测线程
#define INS_TASK//姿态传感器线程

//自瞄调试开关(打开后左开关上将变为自瞄，关闭则为小陀螺)
//#define AUTO_DEBUG

#define INS_PITCH_TURN 1 //tuoluoyifanxiang

#endif
