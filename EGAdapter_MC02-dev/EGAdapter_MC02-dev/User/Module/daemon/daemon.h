//
// Author: An
// Date: 25-10-03
//

#ifndef MODULE_DAEMON_HPP
#define MODULE_DAEMON_HPP

#include <cstdint>
#include <array>
#include <functional>
#include <algorithm>


/**
 * @brief 守护进程（Daemon）实例，用于监控模块是否卡死或离线。
 *        每个模块可注册一个实例，在RTOS任务中周期性调用Tick()。
 */
class DaemonInstance {
 public:
	static constexpr size_t MX_DAEMON_NUM =32;// 最大监控对象数量
	using OfflineCallback = std::function<void()>;  // 去除void*参数

 private:
	// ==== 静态全局管理 ====
	static std::array<DaemonInstance*, MX_DAEMON_NUM> instances_;
	static uint8_t instance_count_;

 private:
	// ==== 每个实例的成员 ====
	uint16_t reload_count_;    // 计数重载值
	uint16_t temp_count_;      // 当前剩余计数
	OfflineCallback callback_; // 离线回调函数

 public:
	struct Config {
		uint16_t reload_count = 100;  // 超时时间计数（默认100）
		uint16_t init_count = 100;    // 初始计数（可用于上电延迟）
		OfflineCallback callback = nullptr; // 离线时的回调函数，可以使用lambda运算符在回调中传入父类，例如.callback = [this]() { this->OnOffline(); } // 捕获this
	};

 public:
	explicit DaemonInstance(const Config& cfg);
	~DaemonInstance();

	// “喂狗”——重载计数
	void reload();
	// 是否在线
	bool isOnline() const;

	// 静态：周期任务，用于所有实例计数递减与离线检测
	static void tick();
};



#endif // MODULE_DAEMON_HPP
