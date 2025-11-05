//
// Created by An on 2025/10/24.
//

#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include "remote_DT7.h"
#include "remote_VT13.h"
#include "remote_FS_I6X.h"
#include "led_on_board.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "minipc.h"

/**
 * @brief 机器人状态类
 * @details 用于获取和更新机器人的状态变量，包括底盘云台目标速度位置、是否小陀螺等
 */
class RobotState {
	// 获取遥控器数据，解析出机器人的状态变量（底盘云台目标速度位置、是否小陀螺等）
 public:
	/**
	 * @brief 遥控器类型枚举
	 * @details 用于表示不同的遥控器类型，包括新遥控器VT13、旧遥控器DT7、第三方遥控器FS_I6X
	 */
	enum class RemoteType : uint8_t {
		VT13,
		DT7,
		FS_I6X,
	};
	/**
	 * @brief 控制模式枚举
	 * @details 用于表示不同的控制模式，包括遥控器模式（这里不区分遥控器种类）、键盘模式
	 */
	enum class ControlMode : uint8_t {
		REMOTE,        // 遥控器模式（这里不区分遥控器种类）
		KEY,          // 键盘模式
	};
	/**
	 * @brief 保护模式枚举
	 * @details 用于表示不同的保护模式，供电机类决定是否使能，防止遥控器丢控导致电机疯转
	 */
	enum class ProtectMode : bool {
		OFF = false,
		ON = true,
	};
	/**
	 * @brief 自瞄模式枚举
	 * @details 表示是否开启自瞄
	 */
	enum class AutoAimMode : bool {
		OFF = false,
		ON = true,
	};

 public:
	//使用结构体定义机器人运行所需最小状态变量
	//云台状态变量
	struct GimbalState {
		float speed_yaw;                //yaw方向速度		归一化成-1到1的浮点数
		float speed_pitch;              //pitch方向速度	归一化成-1到1的浮点数

		//Gimbal::ServoState servo_state = Gimbal::ServoState::NONE; //默认无舵机
	};
	//发射机构状态变量
	struct ShootState {
		float rammer_channel;          //拨弹“力度”，归一化为-1到1的浮点数

		Shoot::FricMode fric_mode = Shoot::FricMode::OFF; //是否开启摩擦轮

		//拨弹许可（发射许可之一）（可能需要改为Shoot类内变量？）
		Shoot::RammerPermission rammer_permission = Shoot::RammerPermission::FORBID;
	};
	//底盘状态变量
	struct ChassisState {
		float speed_x;                  //x方向速度 						归一化成-1到1的浮点数
		float speed_y;                  //y方向速度 						归一化成-1到1的浮点数
		float speed_z;                  //z方向速度（小陀螺速度）	归一化成-1到1的浮点数

		Chassis::WorkMode work_mode;    //工作模式
		Chassis::PowerMode power_mode;  //功率选择
	};

	//由用户定义的初始化配置
	struct Config {
		RemoteType remote_type = RemoteType::VT13;//默认新遥控器
	};

 public:
	//构造与析构
	static RobotState &getInstance();
	static bool init(const Config &config);

	/**
	 * @brief 更新机器人状态
	 * @details 根据当前遥控器类型和遥控器数据，更新机器人的状态变量
	 * 设计为静态函数，因为只需要根据当前遥控器类型和遥控器数据更新一次状态变量，不需要实例化
	 * 其他函数保持为非静态函数，保证只读
	 */

	static void update();

	//getter
	[[nodiscard]] ControlMode getControlMode() const { return control_mode_; };
	[[nodiscard]] AutoAimMode getAutoAimMode() const { return auto_aim_mode_; };

	[[nodiscard]] GimbalState getGimbalState() const { return gimbal_state_; };
	[[nodiscard]] ShootState getShootState() const { return shoot_state_; };
	[[nodiscard]] ChassisState getChassisState() const { return chassis_state_; };

	[[nodiscard]] bool isInitialized() const { return inited_; };
	[[nodiscard]] bool isProtectMode() const { return protect_mode_ == ProtectMode::ON; };
	[[nodiscard]] bool isAutoAimMode() const { return auto_aim_mode_ == AutoAimMode::ON; };

 private:
	//隐式构造函数，防止外部实例化
	RobotState()=default;
	~RobotState() = default;
	//每种遥控器单独的update重载
	void update(DT7 &dt7);
	void update(VT13 &vt13);
	void update(FS_I6X &fsi6x);
	//键盘处理函数，在update中根据是否键盘控制来选择性调用
	//void update_from_key(DT7 &dt7);
	//void update_from_key(VT13 &vt13);

	//守护进程callback
	//void offlineCallback();

 private:
	bool inited_ = false; //是否初始化完成

	RemoteType remote_type_ = RemoteType::VT13;//默认VT13，构造函数中根据config修改
	ControlMode control_mode_ = ControlMode::REMOTE;//默认使用遥控器模式，在update中根据键位选择是否启用键盘模式。
	ProtectMode protect_mode_ = ProtectMode::ON; //整车是否进入保护模式，默认开启。根据遥控器是否在线判断是否进入保护模式
	AutoAimMode auto_aim_mode_ = AutoAimMode::OFF; //自瞄是否开启。因为在多个APP中都要使用，所以声明为单独的私有变量

	GimbalState gimbal_state_{};
	ShootState shoot_state_{};
	ChassisState chassis_state_{};

	//守护进程，确保遥控器失效时进入保护模式，延迟注册
//	std::optional<DaemonInstance> daemon_;
};

#endif //STATE_H_