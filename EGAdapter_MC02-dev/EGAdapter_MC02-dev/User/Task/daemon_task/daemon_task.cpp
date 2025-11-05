//
// Created by An on 2025/10/3.
//
#include "daemon_task.h"
#include "daemon.h"
#include "cmsis_os.h"
void DaemonTask(void *pv){
	while (true) {
		DaemonInstance::tick();//所有已注册的守护进程tick一次
		vTaskDelay(pdMS_TO_TICKS(10)); // 例如在RTOS中每10ms调用一次
	}
}