#include <algorithm>
#include "DJImotor.h"

#include "controller.h"
#include "PIDcontroller.h"

#include "logger.h"
/**** 构造函数、析构函数、接口函数等 ****/
DJIMotor::DJIMotor(const Config &config) : motor_id_(config.dji_motor_config.motor_id) {

	//初始化基类参数
	initParams(config);

	/* ===== 计算减速比 ===== */
	switch (motor_type_) {
		case MotorType::GM6020:
			dji_reduction_radio_ = 1.0f; // 云台电机 1:1
			break;

		case MotorType::M3508:
			dji_reduction_radio_ = (config.dji_motor_config.reduction_radio >= 1e-6f)
														 ? config.dji_motor_config.reduction_radio
														 : DEFAULT_REDUCTION_RADIO_M3508;
			break;

		case MotorType::M2006:
			dji_reduction_radio_ = (config.dji_motor_config.reduction_radio >= 1e-6f)
														 ? config.dji_motor_config.reduction_radio
														 : DEFAULT_REDUCTION_RADIO_M2006;
			break;

		default:
			dji_reduction_radio_ = (config.dji_motor_config.reduction_radio >= 1e-6f)
														 ? config.dji_motor_config.reduction_radio
														 : 1.0f;
			break;
	}

	/* ===== 计算转矩常数 ===== */
	switch (motor_type_) {
		case MotorType::M3508:
			torque_constant_ = TORQUE_CONSTANT_M3508;
			break;
		case MotorType::M2006:
			torque_constant_ = TORQUE_CONSTANT_M2006;
			break;
		case MotorType::GM6020:
			torque_constant_ = TORQUE_CONSTANT_GM6020;
			break;
		default:
			torque_constant_ = 0.0f;
			break;
	}

	/* 生成控制器 */
	if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::POSITION)) {
		position_controller_ = Controller::create(config.position_controller_config);
	}
	if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::VELOCITY)) {
		velocity_controller_ = Controller::create(config.velocity_controller_config);
	}
	if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::CURRENT)) {
		current_controller_ = Controller::create(config.current_controller_config);
	}

	/* ===== 注册 CAN 回调 ===== */
	CANInstance::Config cfg = {
			.can_handle = config.can_config.can_handle,
			.rx_id = static_cast<uint16_t>(
					(config.motor_type == MotorType::GM6020 ? 0x204 : 0x200) + config.dji_motor_config.motor_id
			),
			.rx_callback = [this](const uint8_t *data, uint8_t len) {
				this->decodeData(data, len);
			}
	};
	can_instance_.emplace(cfg);
	//实例化守护进程
	DaemonInstance::Config daemon_cfg = {
			.reload_count=20,//超时200ms认为电机掉线
			.init_count=10,
			.callback=[this]() { this->offlineCallback(); }
	};
	daemon_.emplace(daemon_cfg);

	/* ===== 注册静态数组 ===== */
	uint8_t can_dev =
			cfg.can_handle == &hfdcan1 ? 0 :
			cfg.can_handle == &hfdcan2 ? 1 : 2;

	if (idx_[can_dev] < MAX_MOTORS)
		motors_[can_dev][idx_[can_dev]++] = this;
}

DJIMotor::~DJIMotor() {
	uint8_t can_dev =
			can_instance_->getCanHandle() == &hfdcan1 ? 0 :
			can_instance_->getCanHandle() == &hfdcan2 ? 1 : 2;
	auto it = std::find(motors_[can_dev].begin(), motors_[can_dev].begin() + idx_[can_dev], this);
	if (it != motors_[can_dev].begin() + idx_[can_dev]) {
		size_t pos = std::distance(motors_[can_dev].begin(), it);
		motors_[can_dev][pos] = motors_[can_dev][idx_[can_dev] - 1];  // 用最后一个覆盖
		motors_[can_dev][idx_[can_dev] - 1] = nullptr;
		idx_[can_dev]--;
	}
}

void DJIMotor::updateControl() {
	//如果电机掉线，什么都不做
	if (!isOnline()) {
		return;
	}
	// 参考值
	float temp_ = ref_;//避免ref_在中途被改变带来影响
	if (motor_direction_ == Motor::Direction::REVERSE) {
		temp_ *= -1;
	}

	//位置反馈源
	float position_feedback = measure_base_.total_position;
	//如果设置了外部反馈源，根据标志位进行修改
	if (position_feedback_type_ == Motor::FeedbackType::OTHER_FEED) {
		if(position_feedback_source_func_!= nullptr){
			position_feedback = position_feedback_source_func_();
		}
		if (position_feedback_direction_ == Motor::Direction::REVERSE) {
			position_feedback *= -1.0f;
		}
	}

	//速度反馈源
	float velocity_feedback = measure_base_.velocity;
	//如果设置了外部反馈源，根据标志位进行修改
	if (velocity_feedback_type_ == Motor::FeedbackType::OTHER_FEED) {
		if(velocity_feedback_source_func_!= nullptr){
			velocity_feedback = velocity_feedback_source_func_();
		}
		if (velocity_feedback_direction_ == Motor::Direction::REVERSE) {
			velocity_feedback *= -1.0f;
		}
	}

	//电流反馈源，一律默认使用电机反馈源
	float current_feedback = measure_.current;

	switch (work_mode_) {
		case WorkMode::NONE:
			output_ = 0.0f;
			return;//什么都不做
		case WorkMode::OPEN:
			break;//直接将参考值作为输出值
		case WorkMode::POSITION:
			if (static_cast<uint8_t>(loop_type_) & static_cast<uint8_t>(LoopType::POSITION)) { //如果启用了位置环
				temp_ = position_controller_->calculate(temp_, position_feedback, 0);//todo 前馈也没加
			}
			//故意不加break，串行计算
		case WorkMode::VELOCITY:
			if (static_cast<uint8_t>(loop_type_) & static_cast<uint8_t>(LoopType::VELOCITY)) {
				temp_ = velocity_controller_->calculate(temp_, velocity_feedback, 0);
			}
			//故意不加break，串行计算
		case WorkMode::TORQUE://对于DJI电机来说应该没办法控这个环，但是地位与电流类似，所以把电流环写在这里
			if (static_cast<uint8_t>(loop_type_) & static_cast<uint8_t>(LoopType::CURRENT)) {
				temp_ = current_controller_->calculate(temp_, current_feedback, 0);
			}

	}

	output_ = temp_;
}

void DJIMotor::sendDJICommand() {
	// 每个 CAN 控制器维护 3 组帧 (0x200 / 0x1FF / 0x2FF)
	constexpr int FRAME_COUNT = 3;

	// 三帧 CAN 报文缓冲区
	can_msg_t msg_buf[FRAME_COUNT] = {
			{.data = {0}, .length = 8},
			{.data = {0}, .length = 8},
			{.data = {0}, .length = 8},
	};

	// 遍历三个 CAN 设备
	for (int can_idx = 0; can_idx < CAN_DEV_NUM; can_idx++) {
		// 清空缓冲
		for (auto &msg : msg_buf)
			std::fill(std::begin(msg.data), std::end(msg.data), 0);

		// 标记每个帧中是否有启用电机
		bool frame_has_motor[FRAME_COUNT] = {false, false, false};

		// 遍历当前 CAN 下的电机
		for (int motor_idx = 0; motor_idx < idx_[can_idx]; motor_idx++) {
			DJIMotor *motor = motors_[can_idx][motor_idx];
			//跳过没有被使能的电机，发送电流值为0
			if (!motor || !(motor->isEnabled()))
				continue;

			const auto id = motor->motor_id_;
			const auto type = motor->motor_type_;

			// 确定电机对应帧索引
			uint8_t frame_index = 0;
			switch (type) {
				case MotorType::M2006:
				case MotorType::M3508:
					frame_index = (id <= 4) ? 0 : 1; // 0x200 / 0x1FF
					break;
				case MotorType::GM6020:
					frame_index = (id <= 4) ? 1 : 2; // 0x1FF / 0x2FF
					break;
				default:
					frame_index = 0;
					break;
			}

			frame_has_motor[frame_index] = true;

			// 每帧最多容纳4个电机：ID1-4->偏移0~6
			uint8_t offset = ((id - 1) % 4) * 2;
			auto output = static_cast<int16_t>(motor->output_);

			msg_buf[frame_index].data[offset] = static_cast<uint8_t>(output >> 8);
			msg_buf[frame_index].data[offset + 1] = static_cast<uint8_t>(output & 0xFF);
		}

		// 发送三帧，只要该帧中存在启用电机，就发送（即使数据全 0）
		for (int frame_idx = 0; frame_idx < FRAME_COUNT; frame_idx++) {
			if (!frame_has_motor[frame_idx])
				continue;

			// 映射至全局 CAN 实例索引
			int global_index = can_idx * FRAME_COUNT + frame_idx;
			CANInstance &can_tx = getTransmitCANInstance(global_index);
			can_tx.transmit(msg_buf[frame_idx]);
		}
	}
}

void DJIMotor::decodeData(const uint8_t *data, uint8_t len) {
	measure_.last_encoder = measure_.encoder;

	measure_.encoder = (uint16_t)(data[0] << 8) | data[1];
	measure_base_.position_rotor = ECD2DEGREE * (float)measure_.encoder;

	// ---- 修正版平滑滤波（防止FPU下溢） ----
	constexpr float EPS = 1e-6f;

	auto raw_speed = (int16_t)((data[2] << 8) | data[3]);
	auto raw_current = (int16_t)((data[4] << 8) | data[5]);

	float filtered_velocity =
			(1.0f - SPEED_SMOOTH_COEF) * measure_base_.velocity_rotor +
					SPEED_SMOOTH_COEF * RPM2DPS * (float)raw_speed;

	float filtered_current =
			(1.0f - CURRENT_SMOOTH_COEF) * float(measure_.current) +
					CURRENT_SMOOTH_COEF * (float)raw_current;

	// 下溢保护：若结果极小则直接归零
	if (fabsf(filtered_velocity) < EPS) filtered_velocity = 0.0f;
	if (fabsf(filtered_current) < EPS) filtered_current = 0.0f;

	//计算转子速度和输出轴速度
	measure_base_.velocity_rotor = filtered_velocity;
	measure_base_.velocity = measure_base_.velocity_rotor / dji_reduction_radio_;

	measure_.current = int16_t(filtered_current);

	//计算等效扭矩（如果想省性能也可以删除）
	//先不做，感觉意义不大

	// ---- 其他原逻辑保持不变 ----
	measure_.temperature = data[6];

	if (measure_.last_encoder - measure_.encoder > 4096) {
		measure_base_.round++;
	} else if (measure_.last_encoder - measure_.encoder < -4096) {
		measure_base_.round--;
	}

	//计算转子和输出轴的总位置
	measure_base_.total_position_rotor = 360.0f * float(measure_base_.round) + measure_base_.position_rotor;
	measure_base_.total_position = measure_base_.total_position_rotor / dji_reduction_radio_;

	//还未计算等效扭矩

	is_online_ = true;//接收到回调，标记为在线
	daemon_->reload();
}

DJIMotor &DJIMotor::getMotor4Test() {
	static Config cfg;

	cfg.motor_type = MotorType::GM6020;
	cfg.can_config.can_handle = &hfdcan1;
	cfg.dji_motor_config.motor_id = 1;
	cfg.motor_setting.work_mode = WorkMode::POSITION;

	static DJIMotor motor6020(cfg);

	return motor6020;
}

void DJIMotor::controlLoop() {
	// 第一阶段：计算所有电机控制输出
	for (int can = 0; can < 3; can++) {
		for (int i = 0; i < idx_[can]; i++) {
			DJIMotor *motor = motors_[can][i];
			if (motor && motor->isEnabled()) {
				motor->updateControl();
			}
		}
	}

	// 第二阶段：如果电机为使能状态，统一发送控制指令
	sendDJICommand();
}
void DJIMotor::enableAll() {
	for (int can = 0; can < 3; can++) {
		for (int i = 0; i < idx_[can]; i++) {
			DJIMotor *motor = motors_[can][i];
			if (motor) {
				motor->enable();
			}
		}
	}
}
void DJIMotor::disableAll() {
	for (int can = 0; can < 3; can++) {
		for (int i = 0; i < idx_[can]; i++) {
			DJIMotor *motor = motors_[can][i];
			if (motor) {
				motor->disable();
			}
		}
	}
}


void DJIMotor::offlineCallback() {
	//电机掉线回调
	is_online_ = false;
	// 清除积分项等操作 todo
}
