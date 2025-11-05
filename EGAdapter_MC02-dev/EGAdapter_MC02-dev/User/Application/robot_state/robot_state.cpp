//
// Created by An on 2025/10/24.
//

#include "robot_state.h"
#include "logger.h"

RobotState &RobotState::getInstance() {
	static RobotState instance; // 局部静态变量，C++11保证线程安全
	return instance;
}

bool RobotState::init(const RobotState::Config &config) {
	RobotState &instance = getInstance();
	instance.remote_type_ = config.remote_type;
	bool is_init_success;
	switch (instance.remote_type_) {
		case RemoteType::VT13:
			is_init_success = VT13::init();
			break;
		case RemoteType::DT7:
			is_init_success = DT7::init();
			break;
		case RemoteType::FS_I6X:
			is_init_success = FS_I6X::init();
			break;
		default:
			break;
	}

	//	//实例化守护进程
//	DaemonInstance::Config daemon_cfg = {
//			.reload_count=10,//超时100ms认为机器人掉线
//			.init_count=10,
//			.callback=[]() { getInstance().offlineCallback(); }
//	};
//	getInstance().daemon_.emplace(daemon_cfg);

	if (is_init_success) {
		instance.inited_ = true;
		return true;
	} else {
		return false;
	}
}

void RobotState::update() {
	RobotState &instance = getInstance();
	// 如果未初始化，直接返回
	if (!instance.isInitialized()) {
		instance.protect_mode_ = ProtectMode::ON;
		return;
	}
	// 根据遥控器类型调用不同的update函数
	switch (instance.remote_type_) {
		case RemoteType::VT13: {
			VT13 &remote = VT13::getInstance(); // 只调用一次getInstance()
			instance.protect_mode_ = remote.isOnline() ? ProtectMode::OFF : ProtectMode::ON;
			if (remote.isOnline()) {
				instance.update(remote);
			}
			break;
		}
		case RemoteType::DT7: {
			DT7 &remote = DT7::getInstance(); // 只调用一次getInstance()
			instance.protect_mode_ = remote.isOnline() ? ProtectMode::OFF : ProtectMode::ON;
			if (remote.isOnline()) {
				instance.update(remote);
			}
			break;
		}
		case RemoteType::FS_I6X: {
			FS_I6X &remote = FS_I6X::getInstance(); // 只调用一次getInstance()
			instance.protect_mode_ = remote.isOnline() ? ProtectMode::OFF : ProtectMode::ON;
			if (remote.isOnline()) {
				instance.update(remote);
			}
			break;
		}
		default:
			break;
	}
}

void RobotState::update(DT7 &dt7) {
	if (!dt7.isOnline()) {
		return;
	}
	auto &data = dt7.getData();

	//修改云台状态变量 lyh
	//todo

	//修改发射机构状态变量 lyh
	//todo

	//修改底盘状态变量 sxb
	chassis_state_.speed_x = float(data.rc.rocker_l_v) / DT7::RC_CH_VALUE_RANGE_ABS;//左摇杆竖直
	chassis_state_.speed_y = -float(data.rc.rocker_l_h) / DT7::RC_CH_VALUE_RANGE_ABS;//左摇杆水平，坐标系调整为右手系
	chassis_state_.speed_z = -float(data.rc.rocker_r_h) / DT7::RC_CH_VALUE_RANGE_ABS;//右摇杆水平，坐标系调整为右手系
	switch (data.rc.sw_right) {
		case DT7::RC_SW_UP:
			chassis_state_.work_mode = Chassis::WorkMode::STOP;
			break;
		case DT7::RC_SW_MID:
			chassis_state_.work_mode = Chassis::WorkMode::FOLLOW;
			break;
		case DT7::RC_SW_DOWN:
			chassis_state_.work_mode = Chassis::WorkMode::SPIN;
			break;
		default:
			//理论上不可能进到这里
			break;
	}
	switch (data.rc.sw_left) {
		case DT7::RC_SW_UP:
			chassis_state_.power_mode = Chassis::PowerMode::RUSH;
			break;
		case DT7::RC_SW_MID:
			chassis_state_.power_mode = Chassis::PowerMode::NORMAL;
			break;
		case DT7::RC_SW_DOWN:
			chassis_state_.power_mode = Chassis::PowerMode::SLOW;
			break;
		default:
			//理论上不可能进到这里
			break;
	}


	//输出调试信息
	// logger_printf("%f,%f,%f\r\n",chassis_state_.speed_x,chassis_state_.speed_y,chassis_state_.speed_z);
	// logger_printf("DT7 state updated!\r\n");

	//重载守护进程
//	daemon_->reload();
}
void RobotState::update(VT13 &vt13) {
	if (!vt13.isOnline()) {
		return;
	}
	//修改云台状态变量 lyh
	//todo

	//修改发射机构状态变量 lyh
	//todo

	//修改底盘状态变量 sxb
	//todo

	//输出调试信息
	logger_printf("VT13 state updated!\r\n");

	//重载守护进程
//	daemon_->reload();
}

void RobotState::update(FS_I6X &fsi6x) {

}

//void RobotState::offlineCallback() {
//	//logger_printf("Robot State Loss!\r\n");
//}
