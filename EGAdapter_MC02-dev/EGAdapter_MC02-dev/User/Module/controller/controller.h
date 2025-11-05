//
// Created by An on 2025/9/30.
//

#ifndef SIXFRICHERO_USER_MODULE_CONTROLLER_CONTROLLER_H_
#define SIXFRICHERO_USER_MODULE_CONTROLLER_CONTROLLER_H_
#include <memory>

class Controller {
 public:
	virtual ~Controller() = default;

	/* 控制器类型枚举 */
	//跃鹿框架假定所有控制器均为PID，此框架不能做此假定，因此添加了这个枚举类型（不知是否有用）
	enum class Type {
		NONE = 0,
		PID,
		SMC
		//...待添加
	};

	struct Config {
		Type type = Type::NONE;

		struct PIDConfig {
			float kp = 0;
			float ki = 0;
			float kd = 0;

			float limit_output = 1e6f;
			float limit_integral = 1e6f;
			float limit_error = 1e6f;
			float limit_diff = 1e6f;

			float dead_band = 0.0f;

//			enum class Mode {
//				NORMAL = 0,
//				RAMP,
//			} mode = Mode::NORMAL;
			bool use_ramp = false;
			float ramp_step = 0;

			float dt = 2;//ms
		} pid_config;

		struct SMCConfig {
			float lambda = 1.0f;    // 滑模面系数
			float k = 2.0f;         // 大误差区开关增益
			float phi = 0.02f;      // 大误差区边界层厚度

			// 小误差区参数（减小抖动）
			float k_soft = 0.5f;    // 小误差时较弱的开关增益
			float phi_soft = 0.1f;  // 小误差时更大的边界层厚度

			// 误差切换阈值
			float err_switch = 0.1f;

			float limit_output = 1e6f;
			float dead_band = 0.0f;

			bool use_ramp = false;
			float ramp_step = 0.0f;

			float dt = 2.0f;
		} smc_config;
	};

	//纯虚函数
	virtual float calculate(float target, float measure, float forward) = 0;
	virtual void clear() = 0;

	static std::unique_ptr<Controller> create(Config config);
};

#endif //SIXFRICHERO_USER_MODULE_CONTROLLER_CONTROLLER_H_
