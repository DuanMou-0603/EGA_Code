//
// Created by An on 2025/9/29.
//

#ifndef DMMOTOR_H_
#define DMMOTOR_H_

#include "driver_can.h"
#include "motor.h"
#include "daemon.h"
#include <array>
#include <algorithm>  // for std::find
#include <optional>

class DMMotor : public Motor {
 public:
	static constexpr size_t MAX_MOTORS = 12;//理论上同一个车的达妙电机数量没有上限，这里是从跃鹿抄的（主要目的可能是静态数组分配）
	static constexpr float KP_MIN = 0.0f;
	static constexpr float KP_MAX = 500.0f;
	static constexpr float KD_MIN = 0.0f;
	static constexpr float KD_MAX = 5.0f;

//	enum class MotorType {
//		OTHER = 0,
//		//关节电机
//		J3507, J4310, J4310P, J4340, J4340P,
//		J6006, J6248P, J8006, J8009, J8009P,
//		J10010, J10010L,
//		//中空电机
//		G6220,
//		//轮毂电机
//		H3510, H6215,
//		//分立系列
//		S2325, S3519
//	};

	enum ModeCommand {
		MOTOR_MODE = 0xfc,   // 使能,会响应指令
		RESET_MODE = 0xfd,   // 停止
		ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
		CLEAR_ERROR = 0xfb // 清除电机过热错误
	};

	struct Measure {
		uint8_t id{};//电机ID低8位
		uint8_t state{};//错误状态码

		//达妙原装数据
		float position_rad = 0.0f;        //rad,大概四圈
		float velocity_rads = 0.0f;        //rad/s
		//float torque = 0.0f;//基类已有
		float temperature_mos = 0.0f;      //摄氏度
		float temperature_rotor = 0.0f;    //摄氏度

		float last_position = 0.0f; //degree
	};

	struct SendMsg {
		uint16_t position_des;
		uint16_t velocity_des;
		uint16_t torque_des;
		uint16_t kp;
		uint16_t kd;
	};

	//静态
 public:
	static void controlLoop();//控制循环
	static void enableAll();//使能所有电机，建议上电后等待几秒再启用
	static void disableAll();//禁用所有电机

 private:
	static inline std::array<DMMotor *, MAX_MOTORS> motors_{};
	static inline size_t idx_ = 0;

	//成员函数
 public:
	explicit DMMotor(const Config &config);
	~DMMotor() override;

	//核心控制流程
	void setRef(float ref) override { ref_ = ref; };
	void setAllRef(float position, float velocity, float torque); //直接由外部给定三个参数，在WorkMode为Other时启用
	void decodeData(const uint8_t *data, uint8_t len) override;
	void updateControl() override;
	void sendCommand() override;

	void enable() override;
	void disable() override;

	//getter
	//Measure &getMeasure() { return measure_; }

 private:
	static float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
		float span = x_max - x_min;
		float offset = x_min;
		return static_cast<float>(x_int) * span /
				static_cast<float>((1 << bits) - 1) + offset;
	}
	static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
		float span = x_max - x_min;
		float offset = x_min;
		return static_cast<uint16_t>(((x - offset) * static_cast<float>((1 << bits) - 1)) / span);
	}
	static float limit(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); };

	// 电机掉线回调
	void offlineCallback() override;
	// 发送使能失能或清除命令
	void sendModeCommand(ModeCommand cmd);
 private:
	Measure measure_;
	SendMsg mailbox_{};

	bool use_mit_=true;


	//float output_ = 0.0f;//单输入模式（POSITION,VELOCITY,TORQUE）下的output

	float kp_;
	float kd_;


	float p_max_abs_=12.5f;
	float v_max_abs_=45.0f;
	float t_max_abs_=18.0f;
	//减速比
	//指外部添加的减速比,与大疆含义不同。达妙电机可以直接输出输出轴速度和位置
	float dm_reduction_radio_=1.0f;

};

#endif //DMMOTOR_H_
