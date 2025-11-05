//
// Created by lenovo on 2025/10/29.
//

#ifndef SIXFRICHERO_OMNICHASSIS_H
#define SIXFRICHERO_OMNICHASSIS_H


#include "motor.h"

//class OmniChassis
//{
//public:
//
//    enum class chassis_mode:uint8_t {
//        FOLLOW = 1,
//        SPIN,
//        STOP
//    };
//
//    struct Config {
//        Motor::Config leftfront_wheel_config;
//        Motor::Config leftback_wheel_config;
//        Motor::Config rightback_wheel_config;
//        Motor::Config rightfront_wheel_config;
//        float wheel_radius = 0.05f;               // 轮半径
//        float wheel_base = 0.3f;                  // 前后轮距
//        float track_width = 0.3f;                 // 左右轮距
//    };
//
//    void controlLoop();
//public:
//    OmniChassis(const Config &config);
//
//private:
//    float motor_speed[4];
//    float wheel_radius  = 0.05f;               // 轮半径
//    float wheel_base = 0.3f;                  // 前后轮距
//    float track_width = 0.3f;                 // 左右轮距
//    std::unique_ptr<Motor> motors_[4];
//};

#endif //SIXFRICHERO_OMNICHASSIS_H