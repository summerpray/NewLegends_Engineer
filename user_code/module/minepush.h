#ifndef MINEPUSH_H
#define MINEPUSH_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"

#define MINEPUSH_TASK_INIT_TIME 357

//拨矿电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 6000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MOTOR_SPEED_TO_MINE_SPEED 0.25f


//拨矿过程最大速度
#define NORMAL_MAX_MINE_SPEED 4.0f //2.0


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

struct speed_t
{
    fp32 speed;
    fp32 speed_set;

    fp32 max_speed;
    fp32 min_speed;
};

class MinePush {
public:
    const RC_ctrl_t *mine_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_mine_RC; //底盘使用的遥控器指针

    uint16_t mine_last_key_v;  //遥控器上次按键

    M2006_motor mine_motive_motor[2]; //底盘动力电机数据

    First_order_filter chassis_cmd_slow_set_vx;        //使用一阶低通滤波减缓设定值
    First_order_filter chassis_cmd_slow_set_vy;        //使用一阶低通滤波减缓设定值

    speed_t mine_upload;        //拨矿电机速度环设置

    void init();
    
    void feedback_update();

    void solve();

    void output();

    void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set);
};





#endif