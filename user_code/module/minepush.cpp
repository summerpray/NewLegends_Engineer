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

    }

    mine_upload.min_speed = -NORMAL_MAX_MINE_SPEED;
    mine_upload.max_speed = -NORMAL_MAX_MINE_SPEED;

    //更新一下数据
    feedback_update();
}


void MinePush::feedback_update(){
    //记录上一次遥控器值
    mine_last_key_v = mine_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度，加速度是速度的PID微分
        mine_motive_motor[i].speed = MINE_MOTOR_RPM_TO_VECTOR_SEN * mine_motive_motor[i].motor_measure->speed_rpm;
        mine_motive_motor[i].accel = mine_motive_motor[i].speed_pid.data.error_delta;
    }
    //计算拨矿电机的速度设置值
    mine_upload.speed = mine_motive_motor[0].speed * MOTOR_SPEED_TO_MINE_SPEED;
}


void MinePush::set_mode(){
    behaviour_control_set();
}

/**
 * @brief          通过逻辑判断，赋值"mine_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Chassis::behaviour_mode_set()
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


    //根据行为模式选择一个底盘控制模式
    if (mine_behaviour_mode == MINE_ZERO_FORCE || mine_behaviour_mode == MINE_OPEN) //右拨杆下 底盘控制 开环 直接将遥控器杆量转化为电流值 当前逻辑表现为无力
    {
        mine_mode = MINE_HAND;
    }
    else if(mine_behaviour_mode == MINE_CLOSE) //右拨杆下 底盘控制 开环 直接将遥控器杆量转化为电流值 当前逻辑表现为无力
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
    fp32 vmine_set = 0.0f, vstretch_set = 0.0f, angle_set = 0.0f;

    //获取三个控制设置值
    behaviour_control_set(&vmine_set, &vstretch_set, &angle_set);

    if (mine_mode == MINE_HAND)
    {
        mine_upload.speed_set = vmine_set;
        stretch_motor.speed_set = vstretch_set;
    }
    //TODO:手动写完就写个自动
    else if (mine_mode == MINE_AUTO)
    {
        ;
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
void MinePush::behaviour_control_set(fp32 *vmine_set, fp32 *vstretch_set, fp32 *angle_set)
{

    if (vmine_set == NULL || vstretch_set == NULL || angle_set == NULL)
    {
        return;
    }
    //无力
    if (mine_behaviour_mode == MINE_ZERO_FORCE)
    {
        *vmine_set = 0.0f;
        *vstretch_set = 0.0f;
        *angle_set = 0.0f;
    }
    else if (mine_behaviour_mode == MINE_OPEN)
    {
        mine_open_set_control(vmine_set, vstretch_set, angle_set);
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
void MinePush::mine_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    *vx_set = mine_RC->rc.ch[MINE_X_CHANNEL] * MINE_OPEN_RC_SCALE;
    *vy_set = -mine_RC->rc.ch[MINE_Y_CHANNEL] * MINE_OPEN_RC_SCALE;

    //*wz_set = -mine_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void MinePush::solve()
{
    fp32 mine_speed[2] = {0.0f, 0.0f}; //动力电机目标速度
    fp32 stretch_speed[2] = {0.0f, 0.0f}; //动力电机目标速度
    
    mine_speed[0] = mine_upload.speed_set * MINE_UPLOAD_MOTOR_TURN; 
    mine_speed[1] = -mine_upload.speed_set * MINE_UPLOAD_MOTOR_TURN; 

    stretch_speed[0] = stretch_motor.speed_set * MINE_STRETCH_MOTOR_TURN; 
    stretch_speed[1] = -stretch_motor.speed_set * MINE_STRETCH_MOTOR_TURN; 

    if (mine_mode == MINE_HAND)
    {
        for (i = 0; i < 2; i++)
        {
            mine_motive_motor[i].current_give = (int16_t)(mine_speed[i]);
        }

        for (i = 0; i < 2; i++)
        {
            mine_stretch_motor[i].current_give = (int16_t)(stretch_speed[i]);
        }
        // raw控制直接返回
        return;
    }
}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void MinePush::output()
{
    can_receive.can_cmd_mine_motive_motor(mine_motive_motor[0].current_give, mine_motive_motor[1].current_give,
                                          mine_stretch_motor[0].current_give, mine_stretch_motor[1].current_give);
}