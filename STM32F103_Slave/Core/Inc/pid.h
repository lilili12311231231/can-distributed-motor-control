// PID 控制器头文件
#ifndef __PID_H
#define __PID_H

#include "main.h"

// PID 系数（定义在 pid.c，此处为 extern 声明）
extern float Kp;
extern float Ki;
extern float Kd;

// PID 函数
float PID_Inc(float err);

#endif
