#include "remote_VT13.h"
#include "logger.h"

// 反射型 CCITT-16（等价表驱动），init=0xFFFF，xorout=0x0000
static inline uint16_t Crc16Reflected(const uint8_t* p, uint16_t n) {
	uint16_t crc = 0xFFFF;
	while (n--) {
		crc ^= *p++;
		for (int i = 0; i < 8; ++i)
			crc = (crc & 0x0001) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
	}
	return crc; // 不做 ~crc
}

// ================= 单例 =================
VT13& VT13::getInstance() {
	static VT13 inst;
	return inst;
}

// ================ Public Init ================
bool VT13::init() {
	UARTInstance::Config cfg{};
	cfg.handle = &huart7;
	cfg.tx_type = UARTInstance::DMA;
	cfg.rx_type = UARTInstance::IT_IDLE;
	cfg.rx_size = FRAME_SIZE;
	cfg.rx_cbk  = rxCallback;
	// 波特率在 CubeMX 里配置为 921600 / 8N1 / 无校验 / 无流控（见手册）
	return getInstance().doInit(cfg);
}
bool VT13::init(const UARTInstance::Config& config) { return getInstance().doInit(config); }
bool VT13::init(UART_HandleTypeDef* huart) {
	if (!huart) return false;
	UARTInstance::Config cfg{};
	cfg.handle = huart;
	cfg.tx_type = UARTInstance::DMA;
	cfg.rx_type = UARTInstance::IT_IDLE;
	cfg.rx_size = FRAME_SIZE;
	cfg.rx_cbk  = rxCallback;
	return getInstance().doInit(cfg);
}

bool VT13::doInit(const UARTInstance::Config& cfg) {
	if (inited_) return false;
	uart_.emplace(cfg);
	daemon_.emplace(DaemonInstance::Config{
			.reload_count = 10,                         // 超时100ms认为遥控器丢失
			.init_count   = 10,                         // 可选：上电初始延时，按需调整
			.callback     = [this]() { this->offlineCallback(); }
	});
	uart_->startRecv();
	inited_ = true;
	return true;
}

void VT13::start() {
	if (!inited_) return;
	uart_->startRecv();
}
void VT13::stop() {
	if (!inited_) return;
	is_online_ = false;
	frame_valid_ = false;
	uart_->stopRecv();
}

// ================ RX 路由 ================
void VT13::rxCallback(uint8_t* data, uint16_t size) {
	getInstance().parseFrame(data, size);
}


// ================ Bit 读取小工具 ================
static inline uint32_t rd_bits(const uint8_t* buf, uint16_t bit_off, uint8_t bit_len) {
	// 从 bit_off 开始读取 bit_len 位（最多 32 位），LSB-first 按自然位拼接
	uint32_t v = 0;
	for (uint8_t i = 0; i < bit_len; ++i) {
		uint16_t bit = bit_off + i;
		uint8_t  byte_idx = bit >> 3;
		uint8_t  bit_idx  = bit & 0x7;
		uint8_t  bit_val  = (buf[byte_idx] >> bit_idx) & 0x1;
		v |= (static_cast<uint32_t>(bit_val) << i);
	}
	return v;
}

// ================ 解析主逻辑 ================
void VT13::parseFrame(uint8_t* data, uint16_t size) {
	if (size != FRAME_SIZE) { frame_valid_ = false; return; }

	// Header 校验（0..7=0xA9, 8..15=0x53）
	if (data[0] != 0xA9 || data[1] != 0x53) { frame_valid_ = false; return; }

	const uint16_t rx_crc = static_cast<uint16_t>(data[19] | (static_cast<uint16_t>(data[20]) << 8));
	const uint16_t calc   = Crc16Reflected(data, 19);
	if (rx_crc != calc) { frame_valid_ = false; return; }

	// 帧通过 -> 喂狗、在线
	daemon_->reload();
	is_online_ = true;

	// ------- 根据手册的 bit 布局读取域（偏移单位：bit） -------
	// ch0..ch3 每个 11bit，范围 364/1024/1684
	// ch0: 16, ch1:27, ch2:38, ch3:49
	int16_t ch0 = static_cast<int16_t>(rd_bits(data, 16, 11));
	int16_t ch1 = static_cast<int16_t>(rd_bits(data, 27, 11));
	int16_t ch2 = static_cast<int16_t>(rd_bits(data, 38, 11));
	int16_t ch3 = static_cast<int16_t>(rd_bits(data, 49, 11));

	// Mode switch(2bit, 60), Pause(1bit,62), Left btn(1bit,63), Right btn(1bit,64)
	uint8_t mode_sw  = static_cast<uint8_t>(rd_bits(data, 60, 2));
	uint8_t pause    = static_cast<uint8_t>(rd_bits(data, 62, 1));
	uint8_t btn_l    = static_cast<uint8_t>(rd_bits(data, 63, 1));
	uint8_t btn_r    = static_cast<uint8_t>(rd_bits(data, 64, 1));

	// Dial 11bit @65
	int16_t dial     = static_cast<int16_t>(rd_bits(data, 65, 11));

	// Trigger 1bit @76
	uint8_t trigger  = static_cast<uint8_t>(rd_bits(data, 76, 1));

	// Mouse X/Y/Z：有符号 16bit（80/96/112）
	auto s16 = [](uint32_t u16) -> int16_t { return static_cast<int16_t>(u16 & 0xFFFFu); };
	int16_t mx = s16(rd_bits(data, 80, 16));
	int16_t my = s16(rd_bits(data, 96, 16));
	int16_t mz = s16(rd_bits(data,112, 16));

	// Mouse buttons: left/right/middle 各 2bit（128/130/132）
	uint8_t mb_l = static_cast<uint8_t>(rd_bits(data,128,2)) & 0x1;
	uint8_t mb_r = static_cast<uint8_t>(rd_bits(data,130,2)) & 0x1;
	uint8_t mb_m = static_cast<uint8_t>(rd_bits(data,132,2)) & 0x1;

	// Keyboard 16bit @136：bit0..15 -> W,S,A,D,Shift,Ctrl,Q,E,R,F,G,Z,X,C,V,B
	uint16_t k_raw = static_cast<uint16_t>(rd_bits(data, 136, 16));

	// ---------- 填充 rc_current_ ----------
	// 通道：保留“中心偏移为 0”的语义，便于上层复用（与 DT7 风格一致）
	rc_current_.rc.rocker_r_h = ch0 - RC_CEN;
	rc_current_.rc.rocker_r_v = ch1 - RC_CEN;
	rc_current_.rc.rocker_l_v = ch2 - RC_CEN;
	rc_current_.rc.rocker_l_h = ch3 - RC_CEN;
	rc_current_.rc.dial       = dial - RC_CEN;

	rc_current_.rc.mode_switch= mode_sw;
	rc_current_.rc.pause      = pause;
	rc_current_.rc.btn_left   = btn_l;
	rc_current_.rc.btn_right  = btn_r;
	rc_current_.rc.trigger    = trigger;

	rc_current_.mouse.x = mx;
	rc_current_.mouse.y = my;
	rc_current_.mouse.z = mz;

	// 鼠标键：PRESS
	rc_current_.mouse.press_l[MOUSE_PRESS] = mb_l;
	rc_current_.mouse.press_r[MOUSE_PRESS] = mb_r;
	rc_current_.mouse.press_m[MOUSE_PRESS] = mb_m;
	// 鼠标键：CLICK（上升沿）
	rc_current_.mouse.press_l[MOUSE_CLICK] =
			(rc_current_.mouse.press_l[MOUSE_PRESS] == 1 && rc_last_.mouse.press_l[MOUSE_PRESS] == 0) ? 1 : 0;
	rc_current_.mouse.press_r[MOUSE_CLICK] =
			(rc_current_.mouse.press_r[MOUSE_PRESS] == 1 && rc_last_.mouse.press_r[MOUSE_PRESS] == 0) ? 1 : 0;
	rc_current_.mouse.press_m[MOUSE_CLICK] =
			(rc_current_.mouse.press_m[MOUSE_PRESS] == 1 && rc_last_.mouse.press_m[MOUSE_PRESS] == 0) ? 1 : 0;

	// 键盘：PRESS
	rc_current_.key[KEY_PRESS].keys = k_raw;
	// 键盘：CLICK = 本帧按下 & 上一帧未按
	rc_current_.key[KEY_CLICK].keys = rc_current_.key[KEY_PRESS].keys & (~rc_last_.key[KEY_PRESS].keys);
	// 键盘：组合键视图（保留与 DT7 同风格）
	if (rc_current_.key[KEY_PRESS].ctrl)  rc_current_.key[KEY_PRESS_WITH_CTRL]  = rc_current_.key[KEY_PRESS];
	else                                   rc_current_.key[KEY_PRESS_WITH_CTRL]  = VT13::Key_t{};
	if (rc_current_.key[KEY_PRESS].shift) rc_current_.key[KEY_PRESS_WITH_SHIFT] = rc_current_.key[KEY_PRESS];
	else                                   rc_current_.key[KEY_PRESS_WITH_SHIFT] = VT13::Key_t{};

	// 有效帧标记
	frame_valid_ = true;
	// 保存历史（供下一帧做上升沿）
	rc_last_ = rc_current_;
}

// ================ Offline ================
void VT13::offlineCallback() {
	is_online_ = false;
	rc_current_ = RCData{};           // 清空当前数据
	if (uart_) uart_->startRecv(); // 试图恢复接收
}

// ================ Debug ================
void VT13::debugPrintKeys() {
	const auto &k = rc_current_.key[KEY_PRESS];
	logger_printf("[VT13 Keys] W=%d S=%d A=%d D=%d Shift=%d Ctrl=%d Q=%d E=%d R=%d F=%d G=%d Z=%d X=%d C=%d V=%d B=%d\r\n",
								k.w,k.s,k.a,k.d,k.shift,k.ctrl,k.q,k.e,k.r,k.f,k.g,k.z,k.x,k.c,k.v,k.b);
}
void VT13::debugPrintComboKeys() {
	const auto &kc = rc_current_.key[KEY_PRESS_WITH_CTRL];
	const auto &ks = rc_current_.key[KEY_PRESS_WITH_SHIFT];
	logger_printf("[VT13 Keys+Ctrl] W=%d S=%d A=%d D=%d Q=%d E=%d R=%d F=%d G=%d Z=%d X=%d C=%d V=%d B=%d\r\n",
								kc.w,kc.s,kc.a,kc.d,kc.q,kc.e,kc.r,kc.f,kc.g,kc.z,kc.x,kc.c,kc.v,kc.b);
	logger_printf("[VT13 Keys+Shift] W=%d S=%d A=%d D=%d Q=%d E=%d R=%d F=%d G=%d Z=%d X=%d C=%d V=%d B=%d\r\n",
								ks.w,ks.s,ks.a,ks.d,ks.q,ks.e,ks.r,ks.f,ks.g,ks.z,ks.x,ks.c,ks.v,ks.b);
}
