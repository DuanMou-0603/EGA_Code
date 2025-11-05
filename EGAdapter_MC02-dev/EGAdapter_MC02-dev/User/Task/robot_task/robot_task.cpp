//
// Created by An on 2025/10/24.
//
#include "robot_task.h"
#include "cmsis_os.h"
#include "logger.h"
#include "mecanum_chassis.h"

void RobotTask(void *pv) {
	//初始化机器人状态，需要指定遥控器类型
	RobotState::init({.remote_type=RobotState::RemoteType::DT7});
	//初始化小电脑

	//初始化云台、发射机构、底盘等Application


	for (;;) {
		//执行各实例的控制循环
		//更新遥控器状态
		RobotState::update();

		//logger_printf("ProtectMode:%d\r\n", RobotState::getInstance().isProtectMode());

		vTaskDelay(pdMS_TO_TICKS(5));//控制周期200hz
	}
}