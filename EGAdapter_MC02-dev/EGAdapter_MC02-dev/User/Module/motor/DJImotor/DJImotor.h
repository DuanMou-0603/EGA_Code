//
// Created by An on 2025/9/29.
//

#ifndef DJIMOTOR_H_
#define DJIMOTOR_H_

#include "driver_can.h"
#include "motor.h"

//todo rx_id的安全检查

class DJIMotor : public Motor {
/* ***************************** 常量 ***************************** */
 public:
	static constexpr size_t MAX_MOTORS = 8;//每条can线上挂载的最大电机数量，用于静态资源分配
	static constexpr size_t CAN_DEV_NUM = 3;

	static constexpr float RPM2DPS = 6.0f;
	static constexpr float ECD2DEGREE = 0.043945f;

	static constexpr float SPEED_SMOOTH_COEF = 0.85f;      // 最好大于0.85
	static constexpr float CURRENT_SMOOTH_COEF = 0.9f;     // 必须大于0.9

	//todo 利用转矩常数在decode中计算出等效扭矩?如果要做的话，确认一下这个常数是只有转子的还是带输出轴的
	static constexpr float TORQUE_CONSTANT_GM6020 = 0.741f;//N·m/A
	static constexpr float TORQUE_CONSTANT_M3508 = 0.3f;
	static constexpr float TORQUE_CONSTANT_M2006 = 0.18f;

	static constexpr float DEFAULT_REDUCTION_RADIO_GM6020 = 1.0f;//减速比
	static constexpr float DEFAULT_REDUCTION_RADIO_M3508 = (3591.0f / 187.0f);
	static constexpr float DEFAULT_REDUCTION_RADIO_M2006 = 36.0f;

/* ***************************** 静态 ***************************** */
 public:
	static DJIMotor &getMotor4Test();
	static void controlLoop();//所有大疆电机的控制循环
	static void enableAll();//使能所有电机，建议上电后等待几秒再启用
	static void disableAll();//禁用所有电机

 private:
	static inline std::array<DJIMotor *, MAX_MOTORS> motors_[CAN_DEV_NUM]{};
	static inline size_t idx_[CAN_DEV_NUM] = {0};
// 在 DJIMotor 内部：
	static CANInstance &getTransmitCANInstance(size_t index) {
		static CANInstance instances[9] = {
				{CANInstance::Config{.can_handle = &hfdcan1, .tx_id = 0x200}},
				{CANInstance::Config{.can_handle = &hfdcan1, .tx_id = 0x1FF}},
				{CANInstance::Config{.can_handle = &hfdcan1, .tx_id = 0x2FF}},
				{CANInstance::Config{.can_handle = &hfdcan2, .tx_id = 0x200}},
				{CANInstance::Config{.can_handle = &hfdcan2, .tx_id = 0x1FF}},
				{CANInstance::Config{.can_handle = &hfdcan2, .tx_id = 0x2FF}},
				{CANInstance::Config{.can_handle = &hfdcan3, .tx_id = 0x200}},
				{CANInstance::Config{.can_handle = &hfdcan3, .tx_id = 0x1FF}},
				{CANInstance::Config{.can_handle = &hfdcan3, .tx_id = 0x2FF}},
		};
		return instances[index];
	}


/* ***************************** 结构体 ***************************** */
 public:

	struct Measure {
		uint16_t encoder = 0;             // 本次读取的编码器值 【0~8191】 3508上电时编码器数值很随机，不知为啥
		uint16_t last_encoder = 0;        // 上次读取的编码器值
		int16_t current = 0;          		// 实际转矩电流
		uint8_t temperature = 0;          // 电机温度         # Celsius
	};

/* ***************************** 函数 ***************************** */
 public:
	explicit DJIMotor(const Motor::Config &config);
	~DJIMotor() override;

	void updateControl() override;

	void decodeData(const uint8_t *data, uint8_t len) override;

	static void sendDJICommand();

	//Measure &getMeasure() { return measure_; }
	[[nodiscard]] float getReductionRadio() const { return dji_reduction_radio_; };

 private:
	void offlineCallback() override;

/* ***************************** 变量 ***************************** */
 private:
	const uint8_t motor_id_;

	Measure measure_{};

	float output_ = 0.0f;
	float dji_reduction_radio_; //减速比
	float torque_constant_; //转矩常数
};

#endif //DJIMOTOR_H_
