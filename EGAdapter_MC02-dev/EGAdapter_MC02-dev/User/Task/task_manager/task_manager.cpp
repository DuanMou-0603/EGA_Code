//
// Author: Breezeee
// Date: 25-5-28
//

// =============================== 引入头文件 ===============================
#include <cstdint>
#include "memory.h"
#include "usart.h"
#include "cmsis_os.h"

#include "driver_dwt.h"
#include "driver_usart.h"
#include "driver_can.h"
#include "driver_usb.h"

#include "logger.h"
#include "remote_DT7.h"
#include "remote_VT13.h"
#include "remote_FS_I6X.h"
#include "DMmotor.h"
#include "daemon.h"

#include "task_manager.h"
#include "daemon_task.h"
#include "motor_task.h"
#include "imu_task.h"
#include "robot_task.h"

#include "DJImotor.h"
#include "led_on_board.h"


// 默认任务句柄
TaskHandle_t default_task_handle;

/**
 * @brief  初始化用户编写的外设
 * @note 	 主要写框架内的设备。HAL库的不要写在这里
 */
void BSP_Init() {
	/* 用户初始化内容，在进入默认任务之后被调用 */
	DWTInstance::init(480); //用于计时
	Logger::init(&huart1);  //用于输出日志，默认使用串口1
	LEDOnBoard::init();
	LEDOnBoard::setColorRGB(0, 0, 0);
}

/**
 * @brief  创建系统任务
 * @note   使用 FreeRTOS 的 xTaskCreate 注册默认任务入口。
 */
void Task_Init() {
	// 创建默认任务
	BaseType_t xReturn;
	xReturn = xTaskCreate(DefaultTask,
												"默认任务",
												2048,
												NULL,
												7,
												&default_task_handle);
	if (pdFALSE != xReturn)
	{
		vTaskStartScheduler();
	}
}

/**
 * @brief  默认任务，在此任务中注册所有实际运行的任务，注册完成后自毁
 * @param  pv: 任务参数（当前未使用）
 * @note
 */

void DefaultTask(void *pv) {
	/* HAL库初始化函数，部分设备初始化必须发生在调度器启动后 */
	MX_USB_DEVICE_Init();

	/* 用户初始化函数 */
	BSP_Init();

	/* 创建一系列任务 */
	BaseType_t xReturn = pdPASS;

	taskENTER_CRITICAL();//创建任务时关闭中断
	//判断任务是否创建成功，一旦存在创建不成功的任务，xReturn置0
	xReturn &= xTaskCreate(
			DaemonTask,
			"守护进程",
			256,
			NULL,
			6,
			NULL)==pdPASS;

	xReturn &= xTaskCreate(
			IMUTask,
			"陀螺仪",
			1024,    //必须够大，小了会卡死
			NULL,
			5,
			NULL)==pdPASS;

	xReturn &= xTaskCreate(
			MotorTask,
			"电机控制",
			1024,
			NULL,
			4,
			NULL)==pdPASS;

	xReturn &= xTaskCreate(
			RobotTask,
			"机器人控制任务",
			1024,
			NULL,
			4,
			NULL)==pdPASS;

	xReturn &= xTaskCreate(
			TestTask,
			"测试任务",
			1024 ,
			NULL,
			4,
			NULL)==pdPASS;
	taskEXIT_CRITICAL();

	//如果存在任务没有被分配成功，直接进入死循环
	if (xReturn != pdPASS) {
		Error_Handler();
	}
	vTaskDelete(default_task_handle);
}

/**
 * @brief  测试任务
 * @param  pv: 任务参数（当前未使用）
 * @note   当前作为测试任务
 */
void TestTask(void *pv) {
	auto dm_motor=Motor::create({
		.motor_type=Motor::MotorType::DM_MOTOR,
		.motor_setting={
				.work_mode=Motor::WorkMode::VELOCITY,
		},
		.can_config={
				.can_handle=&hfdcan2,
				.tx_id=0x01,
				.rx_id=0x11,
		},
		.dm_motor_config{
			.kp=5,
			.kd=0.5,
			.p_max_abs=12.5,
			.v_max_abs=30,
			.t_max_abs=10,
		}
	});

	auto dji_motor = Motor::create({
		.motor_type=Motor::MotorType::GM6020,
		.motor_setting={
				.work_mode=Motor::WorkMode::VELOCITY,
				.loop_type=Motor::LoopType::POSITION_AND_VELOCITY
		},
		.position_controller_config{
			.type=Controller::Type::PID,
			.pid_config={
					.kp=5,
					.ki=0,
					.kd=0.5
			}
		},
		.velocity_controller_config{
			.type=Controller::Type::PID,
			.pid_config={
					.kp=5,
					.ki=0.5,
					.kd=0,
					.limit_output=10000,
			}
		},
		.can_config{
			.can_handle=&hfdcan1
		},
		.dji_motor_config{
			.motor_id=1
		}
	});
	for (;;) {
		dji_motor->setRef(120);
		dm_motor->setRef(120);
		auto dm_measure = dm_motor->getMeasure();
		auto dji_measure = dji_motor->getMeasure();
		logger_printf("%f,%f,%f,%f\r\n",dji_measure.total_position,dji_measure.velocity,dm_measure.total_position,dm_measure.velocity);
		LEDOnBoard::loop();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}