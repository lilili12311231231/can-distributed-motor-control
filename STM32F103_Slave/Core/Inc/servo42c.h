// 42C 电机控制头文件 头文件保护
#ifndef __SERVO42C_H
#define __SERVO42C_H

#include "main.h"
//宏定义
#define SERVO_ADDR 0xE0 // 42C 电机地址
#define SERVO_FUNC_ENABLE 0xF3 // 42C 电机使能功能
#define SERVO_FUNC_SPEED 0xF6 // 42C 电机速度控制功能
#define SERVO_FUNC_STOP 0xF7 // 42C 电机停止功能
#define SERVO_FUNC_READ_ENC 0x30 // 42C 电机读取编码器功能
#define SERVO_FUNC_POS 0xFD // 42C 电机位置控制功能
#define SERVO_FUNC_SET_ZERO 0x3F // 42C 置零功能（把当前位置设为零）
//函数声明
void Servo_Enable(void); // 42C 电机使能函数
void Servo_Stop(void); // 42C 电机停止函数
void Servo_SetSpeed(uint8_t dir, uint8_t speed); // 42C 电机速度控制函数
int32_t Servo_ReadEncoder(void); // 42C 电机读取编码器函数
void Servo_SetPosition(uint8_t dir, uint8_t speed, int32_t step); // 42C 电机位置控制函数

#endif