#ifndef MINEPUSH_H
#define MINEPUSH_H

#include "system_config.h"

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"

//拨矿电机方向
#define MINE_UPLOAD_MOTOR_TURN 1
//伸爪电机方向
#define MINE_STRETCH_MOTOR_TURN 1

//任务控制间隔 2ms
#define MINE_CONTROL_TIME_MS 2

//前后的遥控器通道号码
#define MINE_X_CHANNEL 3
//左右的遥控器通道号码
#define MINE_Y_CHANNEL 2

#define MINE_OPEN_RC_SCALE 5 // 遥控器乘以该比例发送到can上

//选择取矿机构状态 开关通道号
#define MINE_MODE_CHANNEL 1
//选择取矿机构状态 开关通道号
#define STRETCH_MODE_CHANNEL 0

//拨矿电机速度环PID
#define MOTIVE_MOTOR_SPEED_PID_KP 6000.0f
#define MOTIVE_MOTOR_SPEED_PID_KI 0.0f
#define MOTIVE_MOTOR_SPEED_PID_KD 2.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define MOTIVE_MOTOR_SPEED_PID_MAX_OUT 6000.0f

//拨矿电机角度环PID
#define MOTIVE_MOTOR_ANGLE_PID_KP 30.0f 
#define MOTIVE_MOTOR_ANGLE_PID_KI 0.1f
#define MOTIVE_MOTOR_ANGLE_PID_KD 2.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_OUT 40.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define MINE_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MOTOR_SPEED_TO_MINE_SPEED 0.25f

//拨矿过程最大速度
#define NORMAL_MAX_MINE_SPEED 4.0f //2.0

//伸爪最大速度
#define NORMAL_MAX_STRETCH_SPEED 4.0f //2.0
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

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

typedef enum
{
    MINE_PUSH_LEFT_ID = 0,
    MINE_PUSH_RIGHT_ID,
    MINE_STRETCH_LEFT_ID,
    MINE_STRETCH_RIGHT_ID,
};                   

typedef enum
{
    MINE_ZERO_FORCE,                  //无力,电机电流控制值为0,应用于遥控器掉线或者需要底盘上电时方便推动的场合

    MINE_OPEN,                        //遥控器的通道值直接转化成电机电流值发送到can总线上

    MINE_CLOSE,                       //全自动，操作手没有权限控制

} mine_behaviour_e;                   //拨矿机构部分行为模式

typedef enum
{
    MINE_AUTO,      //无敌的自动模式

    MINE_HAND,      //用了自动模式的都说好

} mine_mode_e;      //控制模式


class MinePush {
public:
    const RC_ctrl_t *mine_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_mine_RC; //底盘使用的遥控器指针

    uint16_t mine_last_key_v;  //遥控器上次按键

    mine_behaviour_e mine_behaviour_mode; //底盘行为状态机
    mine_behaviour_e last_mine_behaviour_mode; //底盘上次行为状态机

    mine_mode_e mine_mode; //底盘控制状态机
    mine_mode_e last_mine_mode; //底盘上次控制状态机

    Mine_motor mine_motive_motor[4];

    void init();

    void set_mode();

    void behaviour_mode_set();
    
    void feedback_update();

    void set_control();

    //行为模式
    void behaviour_control_set(fp32 *vx_set, fp32 *vy_set);

    void mine_open_set_control(fp32 *vx_set, fp32 *vy_set);

    void motor_set_control(Mine_motor *motor);

    void mine_angle_control(int32_t *add);

    void solve();

    void output();

};


extern MinePush minepush;


#endif