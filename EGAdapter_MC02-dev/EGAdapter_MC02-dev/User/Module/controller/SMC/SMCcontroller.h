//
// Created by An on 2025/11/1.
//

#ifndef SMC_CONTROLLER_H_
#define SMC_CONTROLLER_H_

#include "controller.h"

class SMCInstance : public Controller {
    /* ***************************** 静态函数 ***************************** */
private:
    static float _abs(const float x) { return x > 0 ? x : -x; }
    static float _limit(const float x, const float min, const float max) { return x < min ? min : (x > max ? max : x); }
    static float _sign(const float x) { return (x > 0) - (x < 0); }

    /* ***************************** 构造函数 ***************************** */
public:
    explicit SMCInstance(const Config::SMCConfig &config);

    float calculate(float target, float measure, float forward) override;
    void clear() override;

    /* ***************************** 成员变量 ***************************** */
private:
    // 控制参数
    float lambda_ = 1.0f;    // 滑模面系数
    float k_ = 2.0f;         // 大误差区开关增益
    float phi_ = 0.02f;      // 大误差区边界层厚度

    // 小误差区参数（减小抖动）
    float k_soft_ = 0.5f;    // 小误差时较弱的开关增益
    float phi_soft_ = 0.1f;  // 小误差时更大的边界层厚度

    // 误差切换阈值
    float err_switch_ = 0.1f;

    float limit_output_ = 1e6f;
    float dead_band_ = 0.0f;

    bool use_ramp_ = false;
    float ramp_step_ = 0.0f;

    float dt_ = 2.0f;

    float c_ = 0.0f;        // 滑模面系数
    float measure_;

    float target_;
    // 状态变量
    float error_ = 0.0f;
    float last_error_ = 0.0f;
    float s_ = 0.0f;
    float output_ = 0.0f;
    float last_output_ = 0.0f;
};

#endif //SMC_CONTROLLER_H_
