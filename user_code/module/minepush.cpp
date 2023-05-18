#include "minepush.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

MinePush minepush;

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
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

void MinePush::init()
{
    mine_RC = remote_control.get_remote_control_point();
    last_mine_RC = remote_control.get_last_remote_control_point();

    for (uint8_t i = 0; i < 4; ++i)
    {

        //动力电机数据
        mine_motive_motor[i].init(can_receive.get_mine_motive_motor_measure_point(i));
        //初始化pid
        fp32 mine_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
        mine_motive_motor[i].speed_pid.init(PID_SPEED, mine_speed_pid_parm, &mine_motive_motor[i].speed, &mine_motive_motor[i].speed_set, NULL);
        mine_motive_motor[i].speed_pid.pid_clear();

        fp32 mine_angle_pid_parm[5] = {MOTIVE_MOTOR_ANGLE_PID_KP, MOTIVE_MOTOR_ANGLE_PID_KI, MOTIVE_MOTOR_ANGLE_PID_KD, MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT, MOTIVE_MOTOR_ANGLE_PID_MAX_OUT};
        mine_motive_motor[i].angle_pid.init(PID_ANGLE, mine_angle_pid_parm, &mine_motive_motor[i].angle, &mine_motive_motor[i].angle_set, 0);
        mine_motive_motor[i].angle_pid.pid_clear();
    }

    //更新一下数据
    feedback_update();
    mine_motive_motor[MINE_STRETCH_LEFT_ID].angle_set = mine_motive_motor[MINE_STRETCH_LEFT_ID].angle;
    mine_motive_motor[MINE_STRETCH_RIGHT_ID].angle_set = mine_motive_motor[MINE_STRETCH_RIGHT_ID].angle;
}


void MinePush::feedback_update(){
    //记录上一次遥控器值
    mine_last_key_v = mine_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度，加速度是速度的PID微分
        mine_motive_motor[i].speed = MINE_MOTOR_RPM_TO_VECTOR_SEN * mine_motive_motor[i].motor_measure->speed_rpm  * MOTOR_SPEED_TO_MINE_SPEED;
    }


}


void MinePush::set_mode(){
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"mine_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void MinePush::behaviour_mode_set()
{
    last_mine_behaviour_mode = mine_behaviour_mode;
    last_mine_mode = mine_mode;

    //遥控器设置模式
    if (switch_is_up(mine_RC->rc.s[MINE_MODE_CHANNEL])) //右拨杆上
    {
        mine_behaviour_mode = MINE_ZERO_FORCE;
    }
    else if (switch_is_mid(mine_RC->rc.s[MINE_MODE_CHANNEL])) //右拨杆中
    {
        mine_behaviour_mode = MINE_OPEN;
    }
    else if (switch_is_down(mine_RC->rc.s[MINE_MODE_CHANNEL])) //右拨杆下
    {
        mine_behaviour_mode = MINE_CLOSE;
    }


    //根据行为模式选择一个控制模式
    if (mine_behaviour_mode == MINE_ZERO_FORCE || mine_behaviour_mode == MINE_OPEN)
    {
        mine_mode = MINE_HAND;
    }
    else if(mine_behaviour_mode == MINE_CLOSE)
    {
        mine_mode = MINE_AUTO;
    }
}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void MinePush::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制拨矿电机和伸爪电机
    //vmine_set控制拨矿电机速度，vstretch_set控制伸爪电机速度
    fp32 vmine_set = 0.0f, vstretch_set = 0.0f;
    int32_t angle_set = 0;

    //获取控制设置值
    behaviour_control_set(&vmine_set, &vstretch_set);

    if (mine_mode == MINE_HAND)
    {
        mine_motive_motor[MINE_PUSH_LEFT_ID].speed_set = vmine_set;
        mine_motive_motor[MINE_PUSH_RIGHT_ID].speed_set = vmine_set;
        mine_motive_motor[MINE_STRETCH_LEFT_ID].speed_set = vstretch_set;
        mine_motive_motor[MINE_STRETCH_RIGHT_ID].speed_set = vstretch_set;
    }
    //TODO:手动写完就写个自动
    else if (mine_mode == MINE_AUTO)
    {
        mine_angle_control(&angle_set);
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vmine_set, 通常控制纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void MinePush::behaviour_control_set(fp32 *vmine_set, fp32 *vstretch_set)
{

    if (vmine_set == NULL || vstretch_set == NULL)
    {
        return;
    }
    //无力
    if (mine_behaviour_mode == MINE_ZERO_FORCE)
    {
        *vmine_set = 0.0f;
        *vstretch_set = 0.0f;
    }
    else if (mine_behaviour_mode == MINE_OPEN)
    {
        mine_open_set_control(vmine_set, vstretch_set);
    }

    last_mine_RC->key.v = mine_RC->key.v;
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void MinePush::mine_open_set_control(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    static int16_t mine_channel = 0, stretch_channel = 0;

    rc_deadband_limit(mine_RC->rc.ch[MINE_X_CHANNEL], mine_channel, RC_DEADBAND);
    rc_deadband_limit(mine_RC->rc.ch[MINE_Y_CHANNEL], stretch_channel, RC_DEADBAND);

    *vx_set = mine_RC->rc.ch[MINE_X_CHANNEL] * MINE_OPEN_RC_SCALE;
    *vy_set = -mine_RC->rc.ch[MINE_Y_CHANNEL] * MINE_OPEN_RC_SCALE;

}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void MinePush::solve()
{
    fp32 motive_temp_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //动力电机目标速度

    if (mine_behaviour_mode == MINE_OPEN)
    {
        motive_temp_speed[MINE_PUSH_LEFT_ID] = mine_motive_motor[MINE_PUSH_LEFT_ID].speed_set * MINE_UPLOAD_MOTOR_TURN; 
        motive_temp_speed[MINE_PUSH_RIGHT_ID] = -mine_motive_motor[MINE_PUSH_RIGHT_ID].speed_set * MINE_UPLOAD_MOTOR_TURN; 
        motive_temp_speed[MINE_STRETCH_LEFT_ID] = mine_motive_motor[MINE_STRETCH_LEFT_ID].speed_set * MINE_STRETCH_MOTOR_TURN; 
        motive_temp_speed[MINE_STRETCH_RIGHT_ID] = -mine_motive_motor[MINE_STRETCH_RIGHT_ID].speed_set * MINE_STRETCH_MOTOR_TURN; 
        for (int i = 0; i < 4; i++)
        {
            mine_motive_motor[i].current_give = (int16_t)(motive_temp_speed[i]);
        }
        // raw控制直接返回
        return;
    }
    else if (mine_behaviour_mode == MINE_CLOSE)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_set_control(&mine_motive_motor[i]);
        }
        //TODO:其实这里需要把两个2006只用速度环，伸出电机继续使用双环
    }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void MinePush::output()
{
    if (mine_behaviour_mode == MINE_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            mine_motive_motor[i].current_give = 0.0f;
        }
    }
    can_receive.can_cmd_mine_motive_motor(mine_motive_motor[MINE_PUSH_LEFT_ID].current_give, mine_motive_motor[MINE_PUSH_RIGHT_ID].current_give,
                                          mine_motive_motor[MINE_STRETCH_LEFT_ID].current_give, mine_motive_motor[MINE_STRETCH_RIGHT_ID].current_give);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */

void MinePush::motor_set_control(Mine_motor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed_set = motor->angle_pid.pid_calc();
    motor->current_set = motor->speed_pid.pid_calc();
    
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     add: 角度控制，为角度的增量 单位 rad
 * @retval         none
 */
void MinePush::mine_angle_control(int32_t *add)
{
    add = 0;
    if (add == NULL)
    {
        return;
    }

    if (switch_is_up(mine_RC->rc.s[MINE_MODE_CHANNEL])) //伸出模式
    {
        *add = 100000;
    }
    else
    {
        *add = -100000;
    }
}