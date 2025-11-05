//
// Created by An on 2025/10/10.
//
#include "motor_task.h"
#include "cmsis_os.h"

#include "motor.h"
#include "DJImotor.h"
#include "DMmotor.h"

#include "robot_state.h"

void MotorTask(void *pv){
	RobotState &robot_state = RobotState::getInstance();

	///< 如果想要脱离遥控器测试，取消注释下面这些行
	//等待电机上电完成和实例化完成
	//vTaskDelay(pdMS_TO_TICKS(1500));
	//Motor::enableAll();//使能所有电机

	for(;;){
		if(robot_state.isProtectMode()){
			//保护模式下，禁用所有电机。反复执行，防止达妙电机垂死病中惊坐起
			Motor::disableAll();
		}else{
			//不在保护模式时，启用所有电机，并执行控制循环
			Motor::enableAll();
			Motor::controlLoop();
		}
		///< 如果想要脱离遥控器测试，取消注释下面一行，注释掉上面那些行
		//Motor::controlLoop();

		vTaskDelay(pdMS_TO_TICKS(2));//电机控制周期500hz
	}
}