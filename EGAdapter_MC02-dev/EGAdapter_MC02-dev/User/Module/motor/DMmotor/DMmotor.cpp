#include "DMmotor.h"

#include "driver_can.h"

#include "logger.h"

/****构造函数、析构函数、接口函数等****/
DMMotor::DMMotor(const Config &config)
		: use_mit_(config.dm_motor_config.use_mit),
			kp_(config.dm_motor_config.kp),
			kd_(config.dm_motor_config.kd),
			p_max_abs_(config.dm_motor_config.p_max_abs),
			v_max_abs_(config.dm_motor_config.v_max_abs),
			t_max_abs_(config.dm_motor_config.t_max_abs),
			dm_reduction_radio_(config.dm_motor_config.reduction_radio){

	//初始化基类参数
	initParams(config);

	//实例化can
	CANInstance::Config can_cfg = {
			.can_handle = config.can_config.can_handle,
			.tx_id=config.can_config.tx_id,
			.tx_callback=nullptr,
			.rx_id=config.can_config.rx_id,
			.rx_callback=[this](const uint8_t *data, uint8_t len) {
				this->decodeData(data, len);
			}
	};
	can_instance_.emplace(can_cfg);
	//实例化守护进程
	DaemonInstance::Config daemon_cfg = {
			.reload_count=20,//超时200ms认为电机掉线
			.init_count=10,
			.callback=[this]() { this->offlineCallback(); }
	};
	daemon_.emplace(daemon_cfg);

	//非MIT模式下，注册控制器
	/* 生成控制器 */
	if (!use_mit_) {
		if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::POSITION)) {
			position_controller_ = Controller::create(config.position_controller_config);
		}
		if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::VELOCITY)) {
			velocity_controller_ = Controller::create(config.velocity_controller_config);
		}
//		if (static_cast<uint8_t>(config.motor_setting.loop_type) & static_cast<uint8_t>(LoopType::CURRENT)) {
//			current_controller_ = Controller::create(config.current_controller_config);
//		}//达妙电机不注册电流环
	}


	//注册到列表中
	if (idx_ < MAX_MOTORS) {
		motors_[idx_++] = this;
	}

}

DMMotor::~DMMotor() {
	auto it = std::find(motors_.begin(), motors_.begin() + idx_, this);
	if (it != motors_.begin() + idx_) {
		size_t pos = std::distance(motors_.begin(), it);
		motors_[pos] = motors_[idx_ - 1];  // 用最后一个覆盖
		motors_[idx_ - 1] = nullptr;
		--idx_;
	}
}

/****静态函数***/
void DMMotor::controlLoop() {
	// 第一次遍历：全部电机先完成输出量计算
	for (size_t i = 0; i < idx_; ++i) {
		DMMotor *m = motors_[i];
		if (!m) continue;
		if (!m->isEnabled()) continue; // 禁用时跳过
		m->updateControl();
	}

	// 第二次遍历：统一发送指令，保证“先算后发”的一致性
	for (size_t i = 0; i < idx_; ++i) {
		DMMotor *m = motors_[i];
		if (!m) continue;
		if (!m->isEnabled()) continue;
//		if (!m->isOnline()) {//如果电机存在，且使能，但却掉线，尝试发送一帧命令唤醒它。更新：这一步在enableAll中应当已经完成，故注释掉
//			m->sendModeCommand(ModeCommand::MOTOR_MODE);
//			continue;
//		}
		m->sendCommand();
	}
}
void DMMotor::enableAll() {
	// 遍历所有已注册电机，将其使能
	// 建议上电后等待几秒再调用
	for (size_t i = 0; i < idx_; ++i) {
		DMMotor *m = motors_[i];
		if (!m) continue;//跳过未注册的电机
		if(m->isEnabled() and m->isOnline()) continue;//跳过已经使能且确实被激活使能的电机，注意，这里的在线指的是电机有反馈帧，且state返回值为1
		m->enable();
	}
}
void DMMotor::disableAll() {
	// 遍历所有已注册电机，将其失能
	for (size_t i = 0; i < idx_; ++i) {
		DMMotor *m = motors_[i];
		if (!m) continue;
		m->disable();
	}
}


void DMMotor::enable() {
	//与基类不同的是，主动发出一帧命令使能电机
	//使用can驱动中的msg类型
	sendModeCommand(ModeCommand::MOTOR_MODE);

	//赋值基类成员
	is_enabled_ = true;

}
void DMMotor::disable() {
	//与基类不同的是，主动发出一帧命令失能电机
	sendModeCommand(ModeCommand::RESET_MODE);

	//赋值基类成员
	is_enabled_ = false;
}

void DMMotor::offlineCallback() {
	//电机掉线回调
	is_online_ = false;
	// 清除积分项等操作 todo

}

void DMMotor::decodeData(const uint8_t *data, uint8_t len) {
	if (len < 8 || data == nullptr) return;

	uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量

	// 保存上一次位置
	measure_.last_position = measure_base_.position_rotor;

	// 解析基本数据
	measure_.id = data[0] & 0x0f;
	measure_.state = (data[0] >> 4) & 0x0f;
	//注意！在本框架的语境下，有回调但是state不使能同样视为掉线。
	//这要求达妙电机tx_id推荐设置范围为0x01-0x0F
	//即txid 不能覆盖data[0]的高4位
	if((measure_.state & 0x01) == 0x01){
		is_online_ = true;
	}else{
		is_online_=false;
	}

	tmp = (uint16_t)((data[1] << 8) | data[2]);
	measure_.position_rad = uint_to_float(tmp, -p_max_abs_, p_max_abs_, 16);

	tmp = (uint16_t)((data[3] << 4) | data[4] >> 4);
	measure_.velocity_rads = uint_to_float(tmp, -v_max_abs_, v_max_abs_, 12);

	tmp = (uint16_t)(((data[4] & 0x0f) << 8) | data[5]);
	measure_base_.torque = uint_to_float(tmp, -t_max_abs_, t_max_abs_, 12);

	measure_.temperature_mos = (float)data[6];
	measure_.temperature_rotor = (float)data[7];

	//计算转子位置
	measure_base_.position_rotor = RAD_2_DEGREE * measure_.position_rad;
	//计算转子速度和输出轴速度
	measure_base_.velocity_rotor = RAD_2_DEGREE * measure_.velocity_rads;
	measure_base_.velocity = measure_base_.velocity_rotor/dm_reduction_radio_;


	// 计算总圈数（检测跨越 ±π 或 ±12.5 过零点）
	float diff = measure_base_.position_rotor - measure_.last_position;
	if (diff > RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) / 2) {
		measure_base_.round--;
	} else if (diff < -RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) / 2) {
		measure_base_.round++;
	}

	// 计算累计位置
	measure_base_.total_position_rotor = float(measure_base_.round) * RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) + measure_base_.position_rotor;
	// 计算输出轴总位置
	measure_base_.total_position = measure_base_.total_position_rotor/dm_reduction_radio_;

	daemon_->reload();
}

//直接由外部给定三个参数，在MIT模式时可用,用户需要自己保证自己的参数对不对
void DMMotor::setAllRef(const float position, const float velocity, const float torque) {
	// 目标量（以协议单位记，位置/速度需要从度换算到弧度）
	float p_des = position;   // rad
	float v_des = velocity;   // rad/s
	float t_des = torque;   // N·m
	float kp = kp_;      // N·m/rad
	float kd = kd_;      // N·m*s/rad

	//将计算好的参考值放进邮箱
	p_des = limit(p_des, -p_max_abs_, p_max_abs_);
	v_des = limit(v_des, -v_max_abs_, v_max_abs_);
	t_des = limit(t_des, -t_max_abs_, t_max_abs_);
	kp = limit(kp, KP_MIN, KP_MAX);
	kd = limit(kd, KD_MIN, KD_MAX);

	mailbox_.position_des = float_to_uint(p_des, -p_max_abs_, p_max_abs_, 16);
	mailbox_.velocity_des = float_to_uint(v_des, -v_max_abs_, v_max_abs_, 12);
	mailbox_.torque_des = float_to_uint(t_des, -v_max_abs_, v_max_abs_, 12);
	mailbox_.kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	mailbox_.kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
}

void DMMotor::updateControl() {
	//如果电机掉线，什么都不做
	if (!isOnline()) {
		return;
	}

	float temp_ = ref_;//避免ref_在中途被改变带来影响
	if (motor_direction_ == Motor::Direction::REVERSE) {
		temp_ = -temp_;
	}

	// 目标量（以协议单位记，位置/速度需要从度换算到弧度）
	float p_des = 0.0f;   // rad
	float v_des = 0.0f;   // rad/s
	float t_des = 0.0f;   // N·m
	float kp = 0.0f;      // N·m/rad
	float kd = 0.0f;      // N·m*s/rad

	if (use_mit_) {
		switch (work_mode_) {
			case Motor::WorkMode::POSITION:
				p_des = temp_ * DEGREE_2_RAD;   // 度 -> 弧度
				v_des = 0.0f;
				t_des = 0.0f;
				kp = kp_;                // 位置环刚度
				kd = kd_;                // 速度阻尼
				break;

			case Motor::WorkMode::VELOCITY:
				p_des = 0.0f;
				v_des = temp_ * DEGREE_2_RAD;   // 度每秒 -> 弧度每秒
				t_des = 0.0f;
				kp = 0.0f;               // 只用速度控制 => 位置环关闭
				kd = kd_;                // 速度环增益
				break;

			case Motor::WorkMode::TORQUE:
				p_des = 0.0f;
				v_des = 0.0f;
				t_des = temp_;             // 牛米直接下发
				kp = 0.0f;
				kd = 0.0f;
				break;

			case Motor::WorkMode::OPEN:
			default:
				// 不工作：发一帧“零命令”
				p_des = 0.0f;
				v_des = 0.0f;
				t_des = 0.0f;
				kp = 0.0f;
				kd = 0.0f;
				break;
		}
	}
		//如果控制模式不是MIT，则使用传统两环控制，没有电流环
	else {
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

		switch (work_mode_) {
			case WorkMode::NONE:
				//闭环控制时，牛米直接下发
				p_des = 0.0f;
				v_des = 0.0f;
				t_des = 0.0f;
				kp = 0.0f;
				kd = 0.0f;
				return;
			case WorkMode::OPEN:
				break;//如果工作模式为None，输入值直接当作输出值
			case WorkMode::POSITION:
				if (static_cast<uint8_t>(loop_type_) & static_cast<uint8_t>(LoopType::POSITION)) { //如果启用了位置环
					temp_ = position_controller_->calculate(temp_, position_feedback, 0);//todo 前馈没加
				}
				//故意不加break，串行计算
			case WorkMode::VELOCITY:
				if (static_cast<uint8_t>(loop_type_) & static_cast<uint8_t>(LoopType::VELOCITY)) {
					temp_ = velocity_controller_->calculate(temp_, velocity_feedback, 0);
				}
				//故意不加break，串行计算
			case WorkMode::TORQUE:
				break;

		}
		//闭环控制时，牛米直接下发
		p_des = 0.0f;
		v_des = 0.0f;
		t_des = temp_;
		kp = 0.0f;
		kd = 0.0f;
	}
	//将计算好的参考值放进邮箱
	p_des = limit(p_des, -p_max_abs_, p_max_abs_);
	v_des = limit(v_des, -v_max_abs_, v_max_abs_);
	t_des = limit(t_des, -t_max_abs_, t_max_abs_);
	kp = limit(kp, KP_MIN, KP_MAX);
	kd = limit(kd, KD_MIN, KD_MAX);

	mailbox_.position_des = float_to_uint(p_des, -p_max_abs_, p_max_abs_, 16);
	mailbox_.velocity_des = float_to_uint(v_des, -v_max_abs_, v_max_abs_, 12);
	mailbox_.torque_des = float_to_uint(t_des, -v_max_abs_, v_max_abs_, 12);
	mailbox_.kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	mailbox_.kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);

}

void DMMotor::sendCommand() {
	if (!isEnabled()) return;

	can_msg_t msg{};
	msg.length = 8;

	msg.data[0] = (uint8_t)(mailbox_.position_des >> 8);
	msg.data[1] = (uint8_t)(mailbox_.position_des);
	msg.data[2] = (uint8_t)(mailbox_.velocity_des >> 4);
	msg.data[3] = (uint8_t)(((mailbox_.velocity_des & 0xF) << 4) | (mailbox_.kp >> 8));
	msg.data[4] = (uint8_t)(mailbox_.kp);
	msg.data[5] = (uint8_t)(mailbox_.kd >> 4);
	msg.data[6] = (uint8_t)(((mailbox_.kd & 0xF) << 4) | (mailbox_.torque_des >> 8));
	msg.data[7] = (uint8_t)(mailbox_.torque_des);

	// 如果电机使能，通过 CAN 实例发送
	if (isEnabled()) {
		can_instance_->transmit(msg);
	}
}
void DMMotor::sendModeCommand(DMMotor::ModeCommand cmd) {
	//使用can驱动中的msg类型
	can_msg_t msg;
	msg.length = 8;
	for (size_t i = 0; i < msg.length - 1; i++) {
		msg.data[i] = 0xFF;
	}
	msg.data[7] = static_cast<uint8_t>(cmd);

	can_instance_->transmit(msg);
}


