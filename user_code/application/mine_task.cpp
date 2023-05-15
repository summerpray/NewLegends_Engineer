#include "mine_task.h"

#include "system_config.h" 

#include "minepush.h"


/**
  * @brief          mine_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void mine_task(void *pvParameters) {

    //空闲一段时间
    vTaskDelay(MINEPUSH_TASK_INIT_TIME);
    minepush.init();
	
    while(true) {
      //设置模式
      minepush.set_mode();
      //反馈数据
      minepush.feedback_update();
      //设置控制量
      minepush.set_control();
      //解算
      minepush.solve();
      //电流输出
      minepush.output();
      //系统延时
      vTaskDelay(MINE_CONTROL_TIME_MS);
    }
}
