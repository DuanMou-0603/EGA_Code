//
// Created by zhangzhiwen on 25-10-29.
//

#ifndef REMOTE_FSI6X_H
#define REMOTE_FSI6X_H
#include <optional>

#include "daemon.h"
#include "driver_usart.h"

class FS_I6X {
 public:
	enum class SwitchStatus : uint8_t {
		HIGH = 0,
		MIDDLE = 1,
		LOW = 2,
	};

	struct RCData {
		int16_t rocker_l_h = 0;
		int16_t rocker_l_v = 0;
		int16_t rocker_r_h = 0;
		int16_t rocker_r_v = 0;
		SwitchStatus switch_A = SwitchStatus::HIGH;
		SwitchStatus switch_B = SwitchStatus::HIGH;
		SwitchStatus switch_C = SwitchStatus::HIGH;
		SwitchStatus switch_D = SwitchStatus::HIGH;
		int16_t knob_l = 0;
		int16_t knob_r = 0;

		bool operator!=(const RCData &other) const {
			if (this->rocker_l_h != other.rocker_l_h ||
					this->rocker_l_v != other.rocker_l_v ||
					this->rocker_r_h != other.rocker_r_h ||
					this->rocker_r_v != other.rocker_r_v ||
					this->switch_A != other.switch_A ||
					this->switch_B != other.switch_B ||
					this->switch_C != other.switch_C ||
					this->switch_D != other.switch_D ||
					this->knob_l != other.knob_l ||
					this->knob_r != other.knob_r) {
				return true;
			}
			return false;
		}
	};

 public:
	static FS_I6X &getInstance();

	static bool init(); // 使用默认配置初始化。绝大部分情况下直接用这个就行
	static bool init(const UARTInstance::Config &config);
	static bool init(UART_HandleTypeDef *huart);

	void stop();
	void start();

	// 数据接口
	const RCData &getData() { return rc_current_; }       //获取完整数据
	[[nodiscard]] bool isFrameValid() const { return frame_valid_; } //判断是否有效
	[[nodiscard]] bool isOnline() const { return is_online_; }

	FS_I6X(const FS_I6X &) = delete; //禁止拷贝
	FS_I6X &operator=(const FS_I6X &) = delete;

 private:
	FS_I6X() = default; // 使用 default 构造函数

	bool doInit(const UARTInstance::Config &config);

	bool checkRockerValid() const;

	static inline void SBUSCallback(uint8_t *data, uint16_t size);

	static inline SwitchStatus getSwitchStatus(int16_t val);
	void SBUSParse(uint8_t *data, uint16_t size); //解析遥控器协议
	// void                   parseKeys(uint16_t raw_keys);            //键盘比较特殊，单独拿出来一个函数

	void offlineCallback();

 private:
	static constexpr uint16_t DBUS_FRAME_SIZE = 25u;
	static constexpr uint16_t RC_CH_VALUE_OFFSET = 1024;
	static constexpr uint16_t FS_I6X_FORMAT = 784; // 遥控器归一化数值
	// 默认成员初始化（C++11 起支持）
	std::optional<UARTInstance> uart_instance_;      //延迟注册
	std::optional<DaemonInstance> daemon_;             //延迟注册
	bool frame_valid_{false}; //看起来这个标志位不是很有用？
	bool is_online_{false};   //标记遥控器是否在线。
	bool inited_{false};

	RCData rc_current_{};
	RCData rc_last_{};
};

#endif //REMOTE_FSI6X_H
