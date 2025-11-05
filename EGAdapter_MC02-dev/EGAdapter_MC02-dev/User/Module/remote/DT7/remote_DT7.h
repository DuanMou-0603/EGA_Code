#pragma once

#include <optional>
#include "driver_usart.h"
#include "daemon.h"


/**
 * CubeMX设置：
 * UART5，异步模式
 * Baud Rate:		100000Bit/s
 * Word Length:	9Bits(including Parity)
 * Parity:			Even
 * stop Bits:		1
 *
 * 本驱动为单例设计，不支持多实例（因为UART不支持对同一个外设多次初始化）
 * DT7遥控器发送频率为大约14ms一次
 */

// 键盘按键索引
namespace DT7Key {
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

class DT7 {
 public:
	// 摇杆偏置
	static constexpr int16_t RC_CH_VALUE_MIN = 364;
	static constexpr int16_t RC_CH_VALUE_OFFSET = 1024;
	static constexpr int16_t RC_CH_VALUE_MAX = 1684;
	static constexpr float RC_CH_VALUE_RANGE_ABS = 660.0f;
	// 拨杆状态
	static constexpr uint8_t RC_SW_UP = 1;
	static constexpr uint8_t RC_SW_MID = 3;
	static constexpr uint8_t RC_SW_DOWN = 2;
	//鼠标状态索引
	static constexpr uint8_t MOUSE_PRESS = 0;
	static constexpr uint8_t MOUSE_CLICK = 1;
	// 键盘状态索引
	static constexpr uint8_t KEY_PRESS = 0;
	static constexpr uint8_t KEY_CLICK = 1;
	static constexpr uint8_t KEY_PRESS_WITH_CTRL = 2;
	static constexpr uint8_t KEY_PRESS_WITH_SHIFT = 3;

	//位域结构体,跃鹿说可以提高解析速度
	struct Key_t {
		union {
			struct {
				uint16_t w: 1;
				uint16_t s: 1;
				uint16_t a: 1;
				uint16_t d: 1;
				uint16_t shift: 1;
				uint16_t ctrl: 1;
				uint16_t q: 1;
				uint16_t e: 1;
				uint16_t r: 1;
				uint16_t f: 1;
				uint16_t g: 1;
				uint16_t z: 1;
				uint16_t x: 1;
				uint16_t c: 1;
				uint16_t v: 1;
				uint16_t b: 1;
			};
			uint16_t keys{0};
		};
	};
	// 遥控器完整数据
	struct RCData {
		struct {
			int16_t rocker_r_h;   // 右水平（horizontal）
			int16_t rocker_r_v;   // 右竖直（vertical）
			int16_t rocker_l_h;   // 左水平
			int16_t rocker_l_v;   // 左竖直
			int16_t dial;  // 拨轮
			uint8_t sw_left;
			uint8_t sw_right;
		} rc;

		struct {
			int16_t x;
			int16_t y;
			int16_t z;//鼠标滚轮
			uint8_t press_l[2];
			uint8_t press_r[2];
		} mouse;

		Key_t key[4];//分别为：当前按下的键，上一帧没按但这一帧按了的键，Ctrl组合键，Shift组合键
		//uint8_t key_count[3][16];//统计总按下次数的键，但是看起来没什么用
	};

 private:
	static constexpr uint16_t DBUS_FRAME_SIZE = 18;

 public:
	static DT7 &getInstance();

	static bool init();  // 使用默认配置初始化。绝大部分情况下直接用这个就行
	static bool init(const UARTInstance::Config &config);
	static bool init(UART_HandleTypeDef *huart);

	void stop();
	void start();

	// 数据接口
	const RCData &getData() const { return getInstance().rc_current_; }//获取完整数据
	bool isFrameValid() const { return frame_valid_; }//判断是否有效

	// API。
	uint8_t getLeftSwitch() const { return rc_current_.rc.sw_left; }
	uint8_t getRightSwitch() const { return rc_current_.rc.sw_right; }
	bool isMouseLeftPressed() const {
		return (rc_current_.mouse.press_l[MOUSE_PRESS]) & 0x1;
	}
	bool isMouseLeftClicked() {//注意：查询CLICK事件为了消抖，同一帧内只能查到一次，请勿连续查询。
		bool clicked = rc_current_.mouse.press_l[MOUSE_CLICK];
		rc_current_.mouse.press_l[MOUSE_CLICK] = 0;//防止抖动
		return clicked;
	}
	bool isMouseRightPressed() const {
		return (rc_current_.mouse.press_r[MOUSE_PRESS]) & 0x1;
	}
	bool isMouseRightClicked() {//注意：查询CLICK事件为了消抖，同一帧内只能查到一次，请勿连续查询。
		bool clicked = rc_current_.mouse.press_r[MOUSE_CLICK];
		rc_current_.mouse.press_r[MOUSE_CLICK] = 0;
		return clicked;
	}
	bool isKeyPressed(int key_index) const {
		if (key_index < 0 || key_index >= 16) return false;
		return (rc_current_.key[KEY_PRESS].keys >> key_index) & 0x1;
	}
	bool isKeyClicked(int key_index) {//注意：查询CLICK事件为了消抖，同一帧内只能查到一次，请勿连续查询。
		if (key_index < 0 || key_index >= 16) return false;
		const bool clicked =(rc_current_.key[KEY_CLICK].keys >> key_index) & 0x1;
		const uint16_t mask = static_cast<uint16_t>(1u) << key_index;
		rc_current_.key[KEY_CLICK].keys &= static_cast<uint16_t>(~mask);
		return clicked;
	}
	bool isCtrlComboPressed(int key_index) const {
		if (key_index < 0 || key_index >= 16) return false;
		return (rc_current_.key[KEY_PRESS_WITH_CTRL].keys >> key_index) & 0x1;
	}
	bool isShiftComboPressed(int key_index) const {
		if (key_index < 0 || key_index >= 16) return false;
		return (rc_current_.key[KEY_PRESS_WITH_SHIFT].keys >> key_index) & 0x1;
	}

	bool isOnline()const{
		return is_online_;
	}

	void debugPrintKeys();
	void debugPrintComboKeys();

 private:
	DT7() = default;                       // 使用 default 构造函数
	DT7(const DT7 &) = delete;            //禁止拷贝
	DT7 &operator=(const DT7 &) = delete;

	bool doInit(const UARTInstance::Config &config);

	bool checkRockerValid();

	static void DBUSCallback(uint8_t *data, uint16_t size);
	void DBUSParse(uint8_t *data, uint16_t size); //解析遥控器协议
	void parseKeys(uint16_t raw_keys);//键盘比较特殊，单独拿出来一个函数

	void offlineCallback();

 private:
	// 默认成员初始化（C++11 起支持）
	std::optional<UARTInstance> uart_instance_; //延迟注册
	std::optional<DaemonInstance> daemon_;//延迟注册
	bool frame_valid_{false};//看起来这个标志位不是很有用？
	bool is_online_{false};//标记遥控器是否在线。
	bool inited_{false};

	RCData rc_current_{};
	RCData rc_last_{};
};