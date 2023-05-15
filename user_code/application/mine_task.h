#ifndef MINE_TASK_H
#define MINE_TASK_H

#include "cmsis_os.h"
#include "main.h"


//任务开始空闲一段时间
#define MINEPUSH_TASK_INIT_TIME 30



/**
  * @brief          chassis_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void chassis_task(void *pvParameters);



#endif