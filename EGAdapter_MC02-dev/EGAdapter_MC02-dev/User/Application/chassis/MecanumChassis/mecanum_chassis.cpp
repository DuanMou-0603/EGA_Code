//
// Created by lenovo on 2025/10/28.
//

#include "mecanum_chassis.h"
#include "DJImotor.h"
#include "logger.h"
#include "robot_task.h"

MecanumChassis::MecanumChassis(const Config &config) :
		wheel_radius_(config.wheel_radius),
		chassis_length_(config.chassis_length),
		chassis_width_(config.chassis_width),
		max_speed_const_(config.max_speed_const),
		max_rotate_speed_const_(config.max_rotate_speed_const) {

	//todo 对用户传入config进行安全检查

	//生成四个电机，分别为左前 左后，右后，右前
	motors_[LEFT_FRONT_INDEX] = Motor::create(config.left_front_wheel_config);
	motors_[LEFT_BACK_INDEX] = Motor::create(config.left_back_wheel_config);
	motors_[RIGHT_BACK_INDEX] = Motor::create(config.right_back_wheel_config);
	motors_[RIGHT_FRONT_INDEX] = Motor::create(config.right_front_wheel_config);
}

void MecanumChassis::controlLoop() {
	const auto chassis_state = RobotState::getInstance().getChassisState();

	const auto &work_mode = chassis_state.work_mode;
	const auto &power_mode = chassis_state.power_mode;
	//根据杆量调整车体坐标系的目标速度
	const auto speed_x = chassis_state.speed_x * max_speed_const_;
	const auto speed_y = chassis_state.speed_y * max_speed_const_;
	const auto speed_z = chassis_state.speed_z * max_rotate_speed_const_;

	const float temp_distance = (chassis_length_ + chassis_width_) / 2; //减少运算开销

	if (work_mode == Chassis::WorkMode::FOLLOW) {
		//跟随模式下四轮电机的速度解算
		motor_speed_[LEFT_FRONT_INDEX] = (speed_x - speed_y - temp_distance * speed_z) / wheel_radius_;
		motor_speed_[LEFT_BACK_INDEX] = (speed_x + speed_y - temp_distance * speed_z) / wheel_radius_;
		motor_speed_[RIGHT_BACK_INDEX] = (-speed_x + speed_y - temp_distance * speed_z) / wheel_radius_;
		motor_speed_[RIGHT_FRONT_INDEX] = (-speed_x - speed_y - temp_distance * speed_z) / wheel_radius_;
	} else if (work_mode == Chassis::WorkMode::SPIN) {

	} else if (work_mode == Chassis::WorkMode::STOP) {
		motor_speed_[LEFT_FRONT_INDEX] = 0;
		motor_speed_[LEFT_BACK_INDEX] = 0;
		motor_speed_[RIGHT_BACK_INDEX] = 0;
		motor_speed_[RIGHT_FRONT_INDEX] = 0;
	}

	logger_printf("%f,%f,%f,%f\r\n", motor_speed_[0], motor_speed_[1], motor_speed_[2], motor_speed_[3]);

	//转换为角度值后设置为电机参考值
	for (int i = 0; i < 4; i++) {
		motors_[i]->setRef(motor_speed_[i] * Motor::RAD_2_DEGREE);
	}
}
