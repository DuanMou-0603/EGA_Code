//
// Created by zhangzhiwen on 25-10-29.
//

#include "remote_FS_I6X.h"

#include "logger.h"

FS_I6X &FS_I6X::getInstance() {
	static FS_I6X instance;
	return instance;
}

bool FS_I6X::init() {
	UARTInstance::Config config;
	config.handle = &huart5; // 默认串口
	config.tx_type = UARTInstance::DMA;
	config.rx_type = UARTInstance::DMA_IDLE;
	config.rx_size = DBUS_FRAME_SIZE;
	config.rx_cbk = SBUSCallback;

	return getInstance().doInit(config);
}

bool FS_I6X::init(const UARTInstance::Config &config) {
	return getInstance().doInit(config);
}

bool FS_I6X::init(UART_HandleTypeDef *huart) {
	if (huart == nullptr) {
		return false;
	}

	UARTInstance::Config config;
	config.handle = huart;
	config.tx_type = UARTInstance::DMA;
	config.rx_type = UARTInstance::IT_IDLE;
	config.rx_size = DBUS_FRAME_SIZE;
	config.rx_cbk = SBUSCallback;

	return getInstance().doInit(config);
}

bool FS_I6X::doInit(const UARTInstance::Config &config) {
	if (inited_) {
		return false; // 防止重复初始化
	}

	uart_instance_.emplace(config); // 注册到实例列表
	uart_instance_->startRecv();    // 启动接收
	// 注册daemon
	daemon_.emplace(DaemonInstance::Config{
			.reload_count = 20, // 超时200ms认为遥控器丢失
			.init_count = 10,   // 可选：上电初始延时，按需调整
			.callback = [this]() { this->offlineCallback(); }
	});

	inited_ = true;
	return true;
}

void FS_I6X::stop() {
	if (!inited_) {
		return; // 防止初始化前调用
	}
	is_online_ = false;
	frame_valid_ = false;
	uart_instance_->stopRecv();
}

void FS_I6X::start() {
	if (!inited_) {
		return; // 防止初始化前调用
	}
	uart_instance_->startRecv();
}

void FS_I6X::SBUSCallback(uint8_t *data, uint16_t size) {
	getInstance().SBUSParse(data, size);
}

FS_I6X::SwitchStatus FS_I6X::getSwitchStatus(int16_t val) {
	switch (val) {
		case -784:
			return SwitchStatus::HIGH; // 1(这是一个枚举，对应的值就是1，下同)
		case 0:
			return SwitchStatus::MIDDLE; // 3
		case 783:
			return SwitchStatus::LOW; // 2
		default:
			return SwitchStatus::HIGH; // 1
	}
}

void FS_I6X::SBUSParse(uint8_t *data, uint16_t size) {
	if (data[0] != 0x0F || data[24] != 0x00)
		return;

	rc_current_.rocker_r_h = static_cast<int16_t>(((data[2] << 8 | data[1]) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.rocker_r_v = static_cast<int16_t>(((data[3] << 5 | data[2] >> 3) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.rocker_l_v =
			static_cast<int16_t>(((data[5] << 10 | data[4] << 2 | data[3] >> 6) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.rocker_l_h = static_cast<int16_t>(((data[6] << 7 | data[5] >> 1) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.knob_l = static_cast<int16_t>(((data[7] << 4 | data[6] >> 4) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.knob_r =
			static_cast<int16_t>(((data[9] << 9 | data[8] << 1 | data[7] >> 7) & 0x7FF) - RC_CH_VALUE_OFFSET);
	auto switch_data_A = static_cast<int16_t>(((data[10] << 6 | data[9] >> 2) & 0x7FF) - RC_CH_VALUE_OFFSET);
	auto switch_data_B = static_cast<int16_t>(((data[11] << 3 | data[10] >> 5) & 0x7FF) - RC_CH_VALUE_OFFSET);
	auto switch_data_C = static_cast<int16_t>(((data[13] << 8 | data[12]) & 0x7FF) - RC_CH_VALUE_OFFSET);
	auto switch_data_D = static_cast<int16_t>(((data[14] << 5 | data[13] >> 3) & 0x7FF) - RC_CH_VALUE_OFFSET);
	rc_current_.switch_A = getSwitchStatus(switch_data_A);
	rc_current_.switch_B = getSwitchStatus(switch_data_B);
	rc_current_.switch_C = getSwitchStatus(switch_data_C);
	rc_current_.switch_D = getSwitchStatus(switch_data_D);

	if (!checkRockerValid()) {
		frame_valid_ = false;
		return;
	}
	frame_valid_ = true;

	// 左上角的开关拨下时，认为遥控器开启，喂狗
	if (rc_current_.switch_A != SwitchStatus::HIGH) {
		daemon_->reload();
		is_online_ = true;
		rc_last_ = rc_current_;
	} else {
		rc_last_ = rc_current_ = RCData{};
	}

}

void FS_I6X::offlineCallback() {
	is_online_ = false;
	rc_current_ = RCData{};                          //清空目前的所有遥控器数据
	if (uart_instance_) uart_instance_->startRecv(); //遥控器尝试重新接收数据
	//如果遥控器掉线后还需要做些别的就在这里加
}

bool FS_I6X::checkRockerValid() const {
	auto in_range = [](float val) {
		return val >= -FS_I6X_FORMAT && val <= FS_I6X_FORMAT;
	};

	return in_range(rc_current_.rocker_r_h) &&
			in_range(rc_current_.rocker_l_h) &&
			in_range(rc_current_.rocker_r_v) &&
			in_range(rc_current_.rocker_l_v);
}
