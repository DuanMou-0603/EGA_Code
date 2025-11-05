#include "daemon.h"
#include "main.h"

// =============================== 静态成员初始化 ===============================
std::array<DaemonInstance*, DaemonInstance::MX_DAEMON_NUM> DaemonInstance::instances_{};
uint8_t DaemonInstance::instance_count_ = 0;

// =============================== 构造函数 ===============================
DaemonInstance::DaemonInstance(const Config& cfg)
		: reload_count_(cfg.reload_count == 0 ? 100 : cfg.reload_count),
			temp_count_(cfg.init_count == 0 ? 100 : cfg.init_count),
			callback_(cfg.callback)
{
	if (instance_count_ >= MX_DAEMON_NUM) {
		// 实例超出上限
		Error_Handler();
	}

	// 注册到静态实例表
	instances_[instance_count_++] = this;
}

// =============================== 析构函数 ===============================
DaemonInstance::~DaemonInstance() {
	// 注销自身，保持数组连续
	for (uint8_t i = 0; i < instance_count_; ++i) {
		if (instances_[i] == this) {
			instances_[i] = instances_[instance_count_ - 1];
			instances_[instance_count_ - 1] = nullptr;
			--instance_count_;
			break;
		}
	}
}

// =============================== 公共成员函数 ===============================
void DaemonInstance::reload() {
	temp_count_ = reload_count_;
}

bool DaemonInstance::isOnline() const {
	return temp_count_ > 0;
}

// =============================== 静态任务函数 ===============================
void DaemonInstance::tick() {
	for (uint8_t i = 0; i < instance_count_; ++i) {
		DaemonInstance* ins = instances_[i];
		if (!ins) continue;

		if (ins->temp_count_ > 0) {
			ins->temp_count_--;
		} else {
			if (ins->callback_) {
				ins->callback_();
			}
		}
	}
}
