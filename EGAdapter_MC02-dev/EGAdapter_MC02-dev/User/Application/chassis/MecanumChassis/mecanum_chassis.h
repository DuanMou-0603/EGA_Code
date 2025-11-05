#ifndef MECANUMCHASSIS_H_
#define MECANUMCHASSIS_H_

#include "motor.h"
#include "chassis.h"

class MecanumChassis : public Chassis {
 public:
	static constexpr uint8_t LEFT_FRONT_INDEX = 0;
	static constexpr uint8_t LEFT_BACK_INDEX = 1;
	static constexpr uint8_t RIGHT_BACK_INDEX = 2;
	static constexpr uint8_t RIGHT_FRONT_INDEX = 3;

 public:
	struct Config {
		Motor::Config left_front_wheel_config;
		Motor::Config left_back_wheel_config;
		Motor::Config right_back_wheel_config;
		Motor::Config right_front_wheel_config;
		float wheel_radius = 0.05f;                	// 轮半径，单位m
		float chassis_length = 0.3f;                // 前后轮距，单位m
		float chassis_width = 0.3f;                 // 左右轮距，单位m
		float max_speed_const = 5;								// 车体速度系数
		float max_rotate_speed_const=5;						// 车体角速度系数
	};

	/* 底盘控制主循环 */
	void controlLoop();
 public:
	explicit MecanumChassis(const Config &config);

 private:
	float motor_speed_[4]{};//todo 确认下是否必要？

	float wheel_radius_;                  // 轮半径
	float chassis_length_;                // 前后轮距
	float chassis_width_;                 // 左右轮距
	float max_speed_const_;								// 车体速度系数
	float max_rotate_speed_const_;				// 车体角速度系数

	std::unique_ptr<Motor> motors_[4];
};

#endif