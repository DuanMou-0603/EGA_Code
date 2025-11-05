//
// Created by lenovo on 2025/10/28.
//

#include "SwerveChassis.h"

#include "omni_chassis.h"
#include "mecanum_chassis.h"
#include "DJImotor.h"
#include "logger.h"
#include "robot_task.h"

//
//SwerveChassis::SwerveChassis(const Config &config) {
//    motors_[0] = Motor::create(config.leftfront_wheel_config);
//    motors_[1] = Motor::create(config.leftback_wheel_config);
//    motors_[2] = Motor::create(config.rightback_wheel_config);
//    motors_[3] = Motor::create(config.rightfront_wheel_config);
//}
//
//void SwerveChassis::controlLoop() {
//    auto chassis_state = robot_state.getChassisState();
//    Chassis::Mode chassis_mode = chassis_state.chassis_mode;
//    float chassis_powermode = chassis_state.chassis_powermode;
//    float chassis_speed_x = chassis_state.chassis_speed_x / 5;
//    float chassis_speed_y = -chassis_state.chassis_speed_y / 5;
//    float chassis_speed_z = -chassis_state.chassis_speed_z / 5;
//
//    if (chassis_mode == (uint8_t)chassis_mode::FOLLOW) {
//
//
//
//    } else if (chassis_mode == (uint8_t) chassis_mode::SPIN) {
//
//
//    } else if (chassis_mode == (uint8_t) chassis_mode::STOP) {
//
//
//    }
//
//    for (int i = 0; i < 4; i++) {
//        motors_[i]->setRef(motor_speed[i]);
//    }
//}
