//
// Created by An on 2025/9/30.
//

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_
#include "controller.h"

class PIDInstance : public Controller {
/* ***************************** 常量 ***************************** */


/* ***************************** 静态 ***************************** */
 private:
	static float _abs(const float x) { return x > 0 ? x : -x; }
	static float _limit(const float x, const float min, const float max) { return x < min ? min : (x > max ? max : x); }

/* ***************************** 结构体 ***************************** */
 public:

/* ***************************** 函数 ***************************** */
 public:
	explicit PIDInstance(const Config::PIDConfig &config);

	float calculate(float target, float measure, float forward) override;

	void clear() override;
/* ***************************** 变量 ***************************** */
 private:
	float kp_ = 0.0f;
	float ki_ = 0.0f;
	float kd_ = 0.0f;

	float limit_output_ = 0.0f;
	float limit_integral_ = 0.0f;
	float limit_error_ = 0.0f;
	float limit_diff_ = 0.0f;

	float dead_band_ = 0.0f;

	float ramp_step_ = 0.0f;

	bool use_ramp_ = false;

	float measure_ = 0.0f;
	float last_measure_ = 0.0f;

	float error_ = 0.0f;
	float last_error_ = 0.0f;

	float pout_ = 0.0f;
	float iout_ = 0.0f;
	float dout_ = 0.0f;

	float output_ = 0.0f;
	float last_output_ = 0.0f;

	float target_ = 0.0f;

	float dt_ = 2.0f;
};

#endif //PID_H_
