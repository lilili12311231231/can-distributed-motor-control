// PID 控制器源文件
#include "pid.h"

// PID 系数定义（真正分配内存 + 赋初值）
float Kp = 0.005f; // 降 10 倍：速度全程正比于误差不再饱和（0.005*5000=25），先验证能停不振荡
float Ki = 0.0f; // 临时设 0 先验证方向（纯 P），确认收敛后再加回
float Kd = 0.0f;

// 增量式位置环 PID：输入误差，输出速度增量
float PID_Inc(float err)
{
    static float err_last = 0;
    static float err_prev = 0;

    float delta_u = Kp * (err - err_last) + Ki * err + Kd * (err - 2.0f * err_last + err_prev);

    err_prev = err_last;
    err_last = err;

    return delta_u;
}
