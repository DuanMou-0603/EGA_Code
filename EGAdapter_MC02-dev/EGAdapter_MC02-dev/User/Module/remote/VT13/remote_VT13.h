//
// Created by An on 2025/9/29.
//

#ifndef REMOTE_VT13_H_
#define REMOTE_VT13_H_

#pragma once
#include <optional>
#include <cstdint>
#include <cstring>
#include "driver_usart.h"
#include "daemon.h"

// 默认串口huart7
// 参考：VT13 手册 - 串口参数 921600/8N1/无校验/无流控；每 14ms 输出 21 字节帧；见"Remote Control Data" 章节
// 帧格式(位偏移)：Header0(0..7)=0xA9, Header1(8..15)=0x53, CRC 在 152..167；其余见 .cpp 解析表

namespace VT13Key {
	static constexpr uint8_t W = 0;
	static constexpr uint8_t S = 1;
	static constexpr uint8_t A = 2;
	static constexpr uint8_t D = 3;
	static constexpr uint8_t SHIFT = 4;
	static constexpr uint8_t CTRL = 5;
	static constexpr uint8_t Q = 6;
	static constexpr uint8_t E = 7;
	static constexpr uint8_t R = 8;
	static constexpr uint8_t F = 9;
	static constexpr uint8_t G = 10;
	static constexpr uint8_t Z = 11;
	static constexpr uint8_t X = 12;
	static constexpr uint8_t C = 13;
	static constexpr uint8_t V = 14;
	static constexpr uint8_t B = 15;
}

class VT13 {
 public:
	// 通道原始范围（来自手册）
	static constexpr int16_t RC_MIN = 364;
	static constexpr int16_t RC_CEN = 1024;
	static constexpr int16_t RC_MAX = 1684;

	// 21 字节帧大小
	static constexpr uint16_t FRAME_SIZE = 21;

	// 物理拨位：C/N/S -> 0/1/2
	enum ModeSwitch : uint8_t { MODE_C = 0, MODE_N = 1, MODE_S = 2 };

	// 键鼠数组索引
	static constexpr uint8_t MOUSE_PRESS = 0;
	static constexpr uint8_t MOUSE_CLICK = 1;

	static constexpr uint8_t KEY_PRESS = 0;
	static constexpr uint8_t KEY_CLICK = 1;
	static constexpr uint8_t KEY_PRESS_WITH_CTRL = 2;
	static constexpr uint8_t KEY_PRESS_WITH_SHIFT = 3;

	struct Key_t {
		union {
			struct {
				uint16_t w: 1, s: 1, a: 1, d: 1, shift: 1, ctrl: 1, q: 1, e: 1,
						r: 1, f: 1, g: 1, z: 1, x: 1, c: 1, v: 1, b: 1;
			};
			uint16_t keys{0};
		};
	};

	struct RCData {
		struct {
			// 注意：VT13 的通道顺序（相对 DT7/DBUS 有差异）：
			// ch0: 右水平, ch1: 右竖直, ch2: 左竖直, ch3: 左水平
			int16_t rocker_r_h;   // ch0 - 右水平
			int16_t rocker_r_v;   // ch1 - 右竖直
			int16_t rocker_l_v;   // ch2 - 左竖直
			int16_t rocker_l_h;   // ch3 - 左水平
			int16_t dial;         // 拨轮（同样 11 位，范围同上）
			uint8_t mode_switch;  // 0/1/2 -> C/N/S
			uint8_t pause;        // 0/1
			uint8_t btn_left;     // 自定义左 0/1
			uint8_t btn_right;    // 自定义右 0/1
			uint8_t trigger;      // 扳机 0/1
		} rc;

		struct {
			int16_t x;
			int16_t y;
			int16_t z;                 // 滚轮速度（有正负）
			uint8_t press_l[2];        // [PRESS, CLICK]
			uint8_t press_r[2];
			uint8_t press_m[2];
		} mouse;

		Key_t key[4]; // [PRESS, CLICK, PRESS_WITH_CTRL, PRESS_WITH_SHIFT]
	};

 public:
	static VT13 &getInstance();

	// 初始化（默认使用 huart5，可按需改）
	static bool init();
	static bool init(const UARTInstance::Config &config);
	static bool init(UART_HandleTypeDef *huart);

	void start();
	void stop();

	// 数据接口
	const RCData &getData() const { return getInstance().rc_current_; }
	bool isFrameValid() const { return frame_valid_; }
	bool isOnline() const { return is_online_; }

	// 鼠标/键盘 API（Click：查询即清零）
	bool isMouseLeftPressed() const { return (rc_current_.mouse.press_l[MOUSE_PRESS]) & 0x1; }
	bool isMouseRightPressed() const { return (rc_current_.mouse.press_r[MOUSE_PRESS]) & 0x1; }
	bool isMouseMiddlePressed() const { return (rc_current_.mouse.press_m[MOUSE_PRESS]) & 0x1; }

	bool isMouseLeftClicked() {
		bool v = rc_current_.mouse.press_l[MOUSE_CLICK];
		rc_current_.mouse.press_l[MOUSE_CLICK] = 0;
		return v;
	}
	bool isMouseRightClicked() {
		bool v = rc_current_.mouse.press_r[MOUSE_CLICK];
		rc_current_.mouse.press_r[MOUSE_CLICK] = 0;
		return v;
	}
	bool isMouseMiddleClicked() {
		bool v = rc_current_.mouse.press_m[MOUSE_CLICK];
		rc_current_.mouse.press_m[MOUSE_CLICK] = 0;
		return v;
	}

	bool isKeyPressed(int idx) const {
		if (idx < 0 || idx >= 16) return false;
		return (rc_current_.key[KEY_PRESS].keys >> idx) & 0x1;
	}
	bool isKeyClicked(int idx) {
		if (idx < 0 || idx >= 16) return false;
		const uint16_t mask = static_cast<uint16_t>(1u) << idx;
		const bool clicked = (rc_current_.key[KEY_CLICK].keys & mask) != 0;
		rc_current_.key[KEY_CLICK].keys &= static_cast<uint16_t>(~mask);
		return clicked;
	}
	bool isCtrlComboPressed(int idx) const {
		if (idx < 0 || idx >= 16) return false;
		return (rc_current_.key[KEY_PRESS_WITH_CTRL].keys >> idx) & 0x1;
	}
	bool isShiftComboPressed(int idx) const {
		if (idx < 0 || idx >= 16) return false;
		return (rc_current_.key[KEY_PRESS_WITH_SHIFT].keys >> idx) & 0x1;
	}

	// 调试输出
	void debugPrintKeys();
	void debugPrintComboKeys();

 private:
	VT13() = default;
	VT13(const VT13 &) = delete;
	VT13 &operator=(const VT13 &) = delete;

	bool doInit(const UARTInstance::Config &cfg);

	static void rxCallback(uint8_t *data, uint16_t size);
	void parseFrame(uint8_t *data, uint16_t size);

	void offlineCallback();

 private:
	std::optional<UARTInstance> uart_;
	std::optional<DaemonInstance> daemon_;

	bool frame_valid_{false};
	bool is_online_{false};
	bool inited_{false};

	RCData rc_current_{};
	RCData rc_last_{}; // 用于 click 上升沿检测
};

#endif //SIXFRICHERO_USER_MODULE_REMOTE_VT13_REMOTE_VT13_H_
