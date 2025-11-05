#include "remote_DT7.h"
#include "logger.h"

// 单例获取
DT7 &DT7::getInstance() {
	static DT7 instance;
	return instance;
}

// 使用默认配置初始化（默认串口 5）
bool DT7::init() {
	UARTInstance::Config config;
	config.handle = &huart5;       // 默认串口
	config.tx_type = UARTInstance::DMA;
	config.rx_type = UARTInstance::IT_IDLE;
	config.rx_size = DBUS_FRAME_SIZE;
	config.rx_cbk = DBUSCallback;

	return getInstance().doInit(config);
}

// 使用外部配置初始化
bool DT7::init(const UARTInstance::Config &config) {
	return getInstance().doInit(config);
}

// 使用 huart 指针初始化
bool DT7::init(UART_HandleTypeDef *huart) {
	if (huart == nullptr) {
		return false;
	}

	UARTInstance::Config config;
	config.handle = huart;
	config.tx_type = UARTInstance::DMA;
	config.rx_type = UARTInstance::IT_IDLE;
	config.rx_size = DBUS_FRAME_SIZE;
	config.rx_cbk = DBUSCallback;

	return getInstance().doInit(config);
}

// 实际初始化逻辑
bool DT7::doInit(const UARTInstance::Config &config) {
	if (inited_) {
		return false; // 防止重复初始化
	}
	uart_instance_.emplace(config);  // 注册到实例列表
	uart_instance_->startRecv();           // 启动接收
	// 注册daemon
	daemon_.emplace(DaemonInstance::Config{
			.reload_count = 10,                         // 超时100ms认为遥控器丢失
			.init_count   = 10,                         // 可选：上电初始延时，按需调整
			.callback     = [this]() { this->offlineCallback(); }
	});

	inited_ = true;
	return true;
}

void DT7::stop() {
	if (!inited_) {
		return; // 防止初始化前调用
	}
	is_online_ = false;
	frame_valid_ = false;
	uart_instance_->stopRecv();
}

void DT7::start() {
	if (!inited_) {
		return; // 防止初始化前调用
	}
	uart_instance_->startRecv();
}

void DT7::DBUSCallback(uint8_t *data, uint16_t size) {
	getInstance().DBUSParse(data, size);
}

void DT7::DBUSParse(uint8_t *data, uint16_t size) {
	if (size != DBUS_FRAME_SIZE) {
		frame_valid_ = false;
		return;
	}
	daemon_->reload();//如果接收到大小正确的数据就喂狗
	is_online_ = true;

	// 摇杆
	rc_current_.rc.rocker_r_h = ((data[0] | (data[1] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
	rc_current_.rc.rocker_r_v = (((data[1] >> 3) | (data[2] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET;
	rc_current_.rc.rocker_l_h = (((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET;
	rc_current_.rc.rocker_l_v = (((data[4] >> 1) | (data[5] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;
	rc_current_.rc.dial = ((data[16] | (data[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET; //返回的数据很奇怪？
	if (!checkRockerValid()) {
		frame_valid_ = false;
		return;
	}
	frame_valid_ = true;

	// 开关
	rc_current_.rc.sw_right = (data[5] >> 4) & 0x03;
	rc_current_.rc.sw_left = ((data[5] >> 4) & 0x0C) >> 2;

	// 鼠标
	rc_current_.mouse.x = (data[6] | (data[7] << 8));
	rc_current_.mouse.y = (data[8] | (data[9] << 8));
	rc_current_.mouse.z = (data[10] | (data[11] << 8));
	rc_current_.mouse.press_l[MOUSE_PRESS] = data[12];
	rc_current_.mouse.press_r[MOUSE_PRESS] = data[13];
	// 检测点击（上升沿检测）
	rc_current_.mouse.press_l[MOUSE_CLICK] =
			(rc_current_.mouse.press_l[MOUSE_PRESS] == 1 && rc_last_.mouse.press_l[MOUSE_PRESS] == 0) ? 1 : 0;
	rc_current_.mouse.press_r[MOUSE_CLICK] =
			(rc_current_.mouse.press_r[MOUSE_PRESS] == 1 && rc_last_.mouse.press_r[MOUSE_PRESS] == 0) ? 1 : 0;

	// 键盘
	uint16_t raw_keys = (uint16_t)(data[14] | (data[15] << 8));
	parseKeys(raw_keys);

	// 保存历史
	rc_last_ = rc_current_;

}

void DT7::parseKeys(uint16_t raw_keys) {
	// 1. 保存当前按键状态
	*(uint16_t *)&rc_current_.key[KEY_PRESS] = raw_keys;
	// 2. 利用掩码处理上升沿
	rc_current_.key[KEY_CLICK].keys = rc_current_.key[KEY_PRESS].keys & (~rc_last_.key[KEY_PRESS].keys);
	// 3. 处理 Ctrl / Shift 组合键
	if (rc_current_.key[KEY_PRESS].ctrl) {
		rc_current_.key[KEY_PRESS_WITH_CTRL] = rc_current_.key[KEY_PRESS];
	} else {
		rc_current_.key[KEY_PRESS_WITH_CTRL] = Key_t{};
	}

	if (rc_current_.key[KEY_PRESS].shift) {
		rc_current_.key[KEY_PRESS_WITH_SHIFT] = rc_current_.key[KEY_PRESS];
	} else {
		rc_current_.key[KEY_PRESS_WITH_SHIFT] = Key_t{};
	}

	// 4. 统计按下次数，不过似乎没用
//	uint16_t key_now = rc_current_.key[KEY_PRESS].keys;
//	uint16_t key_last = rc_last_.key[KEY_PRESS].keys;
//	uint16_t key_with_ctrl   = rc_current_.key[KEY_PRESS_WITH_CTRL].keys;
//	uint16_t key_with_shift  = rc_current_.key[KEY_PRESS_WITH_SHIFT].keys;
//
//	uint16_t key_last_ctrl   = rc_last_.key[KEY_PRESS_WITH_CTRL].keys;
//	uint16_t key_last_shift  = rc_last_.key[KEY_PRESS_WITH_SHIFT].keys;

//	// 4. 遍历 16 个按键，检测边沿触发
//	for (uint16_t i = 0, mask = 0x1; i < 16; ++i, mask <<= 1) {
//		// 跳过 Ctrl/Shift 本身（bit4 / bit5）
//		if (i == 4 || i == 5) continue;
//
//		// (a) 普通按键：当前按下 && 上一次没按下 && 非组合键
//		if ((key_now & mask) && !(key_last & mask) &&
//				!(key_with_ctrl & mask) && !(key_with_shift & mask)) {
//			rc_current_.key_count[KEY_PRESS][i]++;
//		}
//
//		// (b) Ctrl 组合键：检测 Ctrl+某键 的上升沿
//		if ((key_with_ctrl & mask) && !(key_last_ctrl & mask)) {
//			rc_current_.key_count[KEY_PRESS_WITH_CTRL][i]++;
//		}
//
//		// (c) Shift 组合键：检测 Shift+某键 的上升沿
//		if ((key_with_shift & mask) && !(key_last_shift & mask)) {
//			rc_current_.key_count[KEY_PRESS_WITH_SHIFT][i]++;
//		}
//	}
}
// 打印所有按键状态
void DT7::debugPrintKeys() {
	const auto &key = rc_current_.key[KEY_PRESS];

	logger_printf("[Keys] "
								"W=%d S=%d A=%d D=%d "
								"Shift=%d Ctrl=%d "
								"Q=%d E=%d R=%d F=%d "
								"G=%d Z=%d X=%d C=%d "
								"V=%d B=%d\r\n",
								key.w, key.s, key.a, key.d,
								key.shift, key.ctrl,
								key.q, key.e, key.r, key.f,
								key.g, key.z, key.x, key.c,
								key.v, key.b);
}

void DT7::debugPrintComboKeys() {
	const auto &key_ctrl = rc_current_.key[KEY_PRESS_WITH_CTRL];
	const auto &key_shift = rc_current_.key[KEY_PRESS_WITH_SHIFT];

	// Ctrl 组合键
	logger_printf("[Keys+Ctrl] "
								"W=%d S=%d A=%d D=%d Q=%d E=%d R=%d F=%d "
								"G=%d Z=%d X=%d C=%d V=%d B=%d\n",
								key_ctrl.w, key_ctrl.s, key_ctrl.a, key_ctrl.d,
								key_ctrl.q, key_ctrl.e, key_ctrl.r, key_ctrl.f,
								key_ctrl.g, key_ctrl.z, key_ctrl.x, key_ctrl.c,
								key_ctrl.v, key_ctrl.b);

	// Shift 组合键
	logger_printf("[Keys+Shift] "
								"W=%d S=%d A=%d D=%d Q=%d E=%d R=%d F=%d "
								"G=%d Z=%d X=%d C=%d V=%d B=%d\n",
								key_shift.w, key_shift.s, key_shift.a, key_shift.d,
								key_shift.q, key_shift.e, key_shift.r, key_shift.f,
								key_shift.g, key_shift.z, key_shift.x, key_shift.c,
								key_shift.v, key_shift.b);

}

void DT7::offlineCallback() {
//	logger_printf("DT7 offline!\r\n");

	is_online_ = false;
	rc_current_ = RCData{};//清空目前的所有遥控器数据
	if (uart_instance_) uart_instance_->startRecv();//遥控器尝试重新接收数据
	//如果遥控器掉线后还需要做些别的就在这里加
}
bool DT7::checkRockerValid() {
	auto in_range = [](int16_t val) {
		return val >= RC_CH_VALUE_MIN - RC_CH_VALUE_OFFSET && val <= RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET;
	};

	return in_range(rc_current_.rc.rocker_l_h) &&
			in_range(rc_current_.rc.rocker_l_v) &&
			in_range(rc_current_.rc.rocker_r_h) &&
			in_range(rc_current_.rc.rocker_r_v);
}
