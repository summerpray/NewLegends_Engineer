#ifndef CONFIG_H
#define CONFIG_H

/*----------------------底盘---------------------------*/
//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0


/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯  底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 0

/*---------------------按键---------------------------*/
//伸出机构 单击F
#define KEY_PRESSED_STRETCH_STATE     'F'

//初始模式 单击G
#define KEY_PRESSED_INIT_STATE     'G'

//初始化UI界面
#define KEY_PRESSED_UI_UPDATE      'B'

#endif