//
// Modified SMCcontroller.cpp
// 改进版滑模控制算法：大误差快速收敛，小误差平滑减抖
//

#include "SMCcontroller.h"

#include "logger.h"

SMCInstance::SMCInstance(const Config::SMCConfig &config)
    : lambda_(config.lambda), k_(config.k), phi_(config.phi),
      limit_output_(config.limit_output), dead_band_(config.dead_band),
      use_ramp_(config.use_ramp), ramp_step_(config.ramp_step),
      dt_(config.dt),
      err_switch_(config.err_switch),        // 新增误差切换阈值
      k_soft_(config.k_soft),                // 小误差时的弱开关增益
      phi_soft_(config.phi_soft)             // 小误差时更宽的边界层
{
}

static float _sat(const float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;   // 在 [-1,1] 区间内保持线性
}


float SMCInstance::calculate(float target, float measure, float forward) {
    measure_ = measure;
    target_ = target;
    error_ = target_ - measure_;

    // 死区处理
    if (_abs(error_) <= dead_band_) {
        output_ = 0.0f;
    } else {
        float edot = (error_ - last_error_) / dt_;
        float s = error_ + lambda_ * edot;

        // --------------------
        // 自适应滑模区切换
        // --------------------
        float k_eff, phi_eff;
        if (_abs(error_) > err_switch_) {
            // 大误差 -> 强滑模
            k_eff = k_;
            phi_eff = phi_;
        } else {
            // 小误差 -> 柔滑模（降低增益 + 增大边界层）
            k_eff = k_soft_;
            phi_eff = phi_soft_;
        }

        // switching control with adaptive boundary layer
        float sat_arg = (phi_eff > 0.0f) ? (s / phi_eff) : (s > 0 ? 1.0f : -1.0f);

        float u_sw = -k_eff * _sat(sat_arg);

        output_ = -u_sw + forward;
    }

    // ramp 约束
    if (use_ramp_) {
        float delta = output_ - last_output_;
        if (_abs(delta) > ramp_step_ * dt_) {
            output_ = last_output_ + (delta > 0 ? ramp_step_ * dt_ : -ramp_step_ * dt_);
        }
    }

    output_ = _limit(output_, -limit_output_, limit_output_);

    last_error_ = error_;
    last_output_ = output_;

    logger_printf("%f\r\n", output_);
    return output_;
}

void SMCInstance::clear() {
    output_ = 0.0f;
    last_output_ = 0.0f;
    last_error_ = 0.0f;
}
