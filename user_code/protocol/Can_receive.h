#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan1;

#define MINE_CAN hcan1
#define CATCH_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MINE_FR_MOTOR_ID = 0x201,
  CAN_MINE_FL_MOTOR_ID = 0x202,
  CAN_STRETCH_FR_MOTOR_ID = 0x203,
  CAN_STRETCH_FL_MOTOR_ID = 0x204,

  CAN_MINE_MOTIVE_ALL_ID = 0x200,

  CAN_SPIN_R_MOTOR_ID = 0x205,
  CAN_SPIN_L_MOTOR_ID = 0x206,
  CAN_CATCH_YAW_MOTOR_ID = 0x207,
  CAN_CATCH_SUCTION_MOTOR_ID = 0x208,

  CAN_CATCH_MOTIVE_ALL_ID = 0x1FF,

} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;

  uint16_t offset_angle;     //补偿角度
	fp32  round_cnt;        //转子转动圈数
	fp32  total_angle;      //转子转动总角度
	fp32  last_total_angle;
	fp32  angle_err;
} motor_measure_t;

class Can_receive
{
public:
  //动力电机反馈数据结构体
  motor_measure_t mine_motive_motor[4];
  motor_measure_t catch_motive_motor[4];
  //发送数据结构体
  CAN_TxHeaderTypeDef mine_tx_message;
  CAN_TxHeaderTypeDef catch_tx_message;
  uint8_t mine_can_send_data[8];
  uint8_t catch_can_send_data[8];

  void init();

  //电机数据接收
  void get_mine_motor_measure(uint8_t num, uint8_t data[8]);

  void get_catch_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_mine_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //矿仓部分动力电机数据

  void can_cmd_catch_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //抓取动力电机数据

  const motor_measure_t *get_mine_motive_motor_measure_point(uint8_t i);

  const motor_measure_t *get_catch_motive_motor_measure_point(uint8_t i);
};

#endif
