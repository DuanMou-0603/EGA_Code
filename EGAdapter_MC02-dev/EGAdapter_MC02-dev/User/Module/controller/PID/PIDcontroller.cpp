//
// Created by 13513 on 25-10-14.
//
#include "PIDcontroller.h"
#include "logger.h"

PIDInstance::PIDInstance(const Config::PIDConfig &config)
		: kp_(config.kp), ki_(config.ki), kd_(config.kd),
			limit_integral_(config.limit_integral),
			limit_diff_(config.limit_diff), limit_error_(config.limit_error),
			dead_band_(config.dead_band), ramp_step_(config.ramp_step), limit_output_(config.limit_output),
			use_ramp_(config.use_ramp), dt_(config.dt) {
}

float PIDInstance::calculate(float target, float measure, float forward) {
	measure_ = measure;
	target_ = target;
	error_ = target_ - measure_;

	if (_abs(error_) > dead_band_) {
		pout_ = kp_ * error_;
		iout_ += ki_ * error_ * dt_;
		dout_ = kd_ * (last_measure_ - measure_ ) / dt_; // 微分先行

		iout_ = _limit(iout_, -limit_integral_, limit_integral_);
		dout_ = _limit(dout_, -limit_diff_, limit_diff_);

		output_ = pout_ + dout_ + iout_ + forward;
	} else {
		output_ = 0.0f;
	}

	if (use_ramp_) {
		float delta = output_ - last_output_;
		if (_abs(delta) > ramp_step_ * dt_) {
			output_ = last_output_ + (delta > 0 ? ramp_step_ * dt_ : -ramp_step_ * dt_);
		}
	}
	// ✅ 输出限幅
	output_ = _limit(output_, -limit_output_, limit_output_);

	last_error_ = error_;
	last_measure_ = measure_;
	last_output_ = output_;

	return output_;
}

void PIDInstance::clear() {
	output_ = 0.0f;
	iout_ = 0.0f;
	last_output_ = 0.0f;
}