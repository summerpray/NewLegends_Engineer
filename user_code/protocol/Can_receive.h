#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan2
#define BOARD_COM_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MINE_FR_MOTOR_ID = 0x201,
  CAN_MINE_FL_MOTOR_ID = 0x202,
  CAN_STRETCH_FR_MOTOR_ID = 0x203,
  CAN_STRETCH_FL_MOTOR_ID = 0x204,

  CAN_GIMBAL_MOTIVE_ALL_ID = 0x200,


} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;

// TODO 超电还未对接
//  //rm motor data
//  typedef struct
//  {
//    fp32 input_vot;
//    fp32 supercap_vot;
//    fp32 input_current;
//    fp32 target_power;
//  } super_cap_measure_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_0;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32 gimbal_yaw_angle;

  // UI状态
  fp32 gimbal_pitch_angle;
  bool_t auto_state;
  bool_t aim_state;
  bool_t fric_state;
} chassis_receive_t;


class Can_receive
{
public:
  //动力电机反馈数据结构体
  motor_measure_t mine_motive_motor[2];

  //发送数据结构体
  CAN_TxHeaderTypeDef mine_tx_message;
  uint8_t mine_can_send_data[8];

  void init();

  //电机数据接收
  void get_mine_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_mine_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //动力电机数据

  const motor_measure_t *get_mine_motive_motor_measure_point(uint8_t i);

};

#endif
