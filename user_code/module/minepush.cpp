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
    mine_upload = mine_motive_motor[0].speed * MOTOR_SPEED_TO_MINE_SPEED;
}

