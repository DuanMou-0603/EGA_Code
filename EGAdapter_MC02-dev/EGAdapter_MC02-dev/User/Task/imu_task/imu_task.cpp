//
// Created by An on 2025/10/14.
//
#include "imu_task.h"
#include "cmsis_os.h"

#include "imu.h"
#include "bmi088.h"

#include "logger.h"

void IMUTask(void *pv) {
	IMU::Config imu_config{
			.type=IMU::Type::BMI088,
			.filter_type=IMU::FilterType::EKF,
			.sample_time=0.001,
	};
	BMI088 bmi088(imu_config);
	bmi088.init();
	// bmi088.calibrate();

	for (;;) {
		bmi088.update();
		bmi088.compute();
		auto result = bmi088.getQuaternion();
		//logger_printf("%f,%f,%f,%f\r\n", result.w, result.x, result.y, result.z);
		vTaskDelay(pdMS_TO_TICKS(1)); //imu更新周期1khz
	}
}
