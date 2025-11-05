//
// Created by An on 2025/9/30.
//

#ifndef MOTOR_H_
#define MOTOR_H_

#include <optional>
#include "driver_can.h"
#include "controller.h"
#include "daemon.h"

class Motor {
 public:
	static constexpr float RAD_2_DEGREE = 57.2957795f;// 180.0f/3.1415f
	static constexpr float DEGREE_2_RAD = 1.0f / RAD_2_DEGREE;
 public:
	using FeedbackSourceFunc = std::function<float()>;

 public:
	enum class WorkMode : uint8_t { //控制目标，即定位置，定速度，定扭矩
		NONE = 0,
		OPEN = 1, //开环，输入值直接作为输出值
		POSITION = 2, // deg
		VELOCITY = 3, // deg/s
		TORQUE = 4,// N·m (扭矩),在某些情况下也可以理解为CURRENT //有点别扭 //主要给达妙使用
	};
	enum class Direction : uint8_t { //正反标记位
		NORMAL = 0,
		REVERSE = 1
	};
	enum class LoopType : uint8_t { //闭环选择，可以或运算
		OPEN = 0,//开环，输入直接加到输出上
		POSITION = 0b0001,
		VELOCITY = 0b0010,
		CURRENT = 0b0100, //for DJI

		//only for check
		POSITION_AND_VELOCITY = 0b0011,
		ALL_THREE = 0b0111,
	};
	enum class FeedbackType : uint8_t {
		MOTOR_FEED, //电机自身反馈
		OTHER_FEED  //其它反馈源
	};

	enum class MotorType {
		OTHER = 0,
		//DJI电机
		GM6020,
		M3508,
		M2006,
		//达妙电机
		DM_MOTOR
		//其它电机

	};

	struct MeasureBase {// 同一单位：度，度每秒，牛米
		float position_rotor = 0.0f; // 【转子】位置。根据电机不同，可能不止360度
		float total_position_rotor = 0.0f; // 【转子】上电后经过的完整角度
		float total_position = 0.0f; // 【输出轴】上电后经过的完整角度
		float velocity_rotor = 0.0f; // 【转子】速度 度每秒
		float velocity = 0.0f; // 【输出轴】速度 度每秒
		float torque = 0.0f; // 等效力矩
		int32_t round = 0; // 圈数
	};

	//反馈类型结构体
	struct FeedbackSetting{
		FeedbackType type=FeedbackType::MOTOR_FEED;
		Direction direction=Direction::NORMAL;
		FeedbackSourceFunc source_func = nullptr;
	};

	/* 电机控制设置,包括工作模式闭环类型,反转标志和反馈来源 */
	struct MotorSetting {
		WorkMode work_mode = WorkMode::POSITION;//默认为位置控制。同时work mode将成为三环运算的最外环
		LoopType loop_type = LoopType::POSITION_AND_VELOCITY;//三环控制，可或运算。DM电机在切换至外部反馈源时也需要设置。
		Direction motor_direction = Direction::NORMAL; //电机正反转标志

		//外部反馈源相关设置
		FeedbackSetting position_feedback_setting{};
		FeedbackSetting velocity_feedback_setting{};

		//前馈相关设置
		//todo

	};

	//这些Config只用于记录初始化配置，其实例应该由各电机自己生成
	struct Config {
		//电机基本设置
		MotorType motor_type = MotorType::OTHER;
		MotorSetting motor_setting;
		//三环控制器设置
		Controller::Config position_controller_config;
		Controller::Config velocity_controller_config;
		Controller::Config current_controller_config;

		//驱动和控制器设置
		CANInstance::Config can_config;

		//各品牌电机所需专有的config
		//大疆电机专有的初始化配置
		struct DJIMotorConfig {
			uint8_t motor_id = 0; //从1到8，必须提供
			float reduction_radio = 0.0f;//减速比。不提供时默认按照电机默认设置来。无减速箱的写1：1
		} dji_motor_config;

		//达妙电机专有的初始化配置
		struct DMMotorConfig {
			//两种控制模式：自带的MIT，或者软件串级PID只输出TORQUE
			bool use_mit = true;//默认使用mit模式控制，否则使用三环控制。有外部参考源时也强制使用三环控制

			float kp = 0.0f; //如果不提供，默认值为0
			float kd = 0.0f;
			float p_max_abs = 12.5f;//每款达妙电机的参数不一样，使用前请一定连接到上位机确认参数，否则会造成速度或者力矩输出错误！
			float v_max_abs = 45.0f;
			float t_max_abs = 18.0f;
			float reduction_radio = 1.0f;//减速比。达妙电机绝大多数能直接输出输出轴信息，默认减速比为1
		} dm_motor_config;

	};

	//接口函数
 public:
	//所有电机的总控制循环
	static void controlLoop();
	//使能所有电机
	static void enableAll();
	//禁用所有电机
	static void disableAll();
	//查询是否存在失能电机
	[[nodiscard]] static bool hasDisabledMotor();

	//工厂函数，方便上层创建电机
	static std::unique_ptr<Motor> create(const Config &config);

 public:
	virtual ~Motor() = default;

	// 数据解析接口，纯虚
	virtual void decodeData(const uint8_t *data, uint8_t len) = 0;

	/* 核心工作函数 */
	// 目标值更新，Application层主要调用setRef。
	virtual void setRef(float ref) { ref_ = ref; }
	// 计算输出，纯虚
	virtual void updateControl() = 0; //计算电机的输出值，但是不发送
	// 发送控制命令
	virtual void sendCommand() {}; //默认什么都不做，但是电机应该自己实现。不写成纯虚函数是因为沟槽的DJI电机必须分组发送

	/* 调整设置 */
	// 调整工作模式
	virtual void setWorkMode(WorkMode mode) { work_mode_ = mode; }
	// 调整电机旋转方向
	virtual void setMotorDirection(Direction direction) { motor_direction_ = direction; };
	// 调整反馈方向
	//virtual void setFeedbackDirection(Direction direction) { feedback_direction_ = direction; };

	// 启用/禁用电机（通用）
	virtual void enable() { is_enabled_ = true; }
	virtual void disable() { is_enabled_ = false; }

	// 查询状态
	[[nodiscard]] virtual bool isEnabled() const { return is_enabled_; }
	[[nodiscard]] virtual bool isOnline() const { return is_online_; }

	// getter函数
	[[nodiscard]] MeasureBase getMeasure()const {return measure_base_;}

 protected:
	void initParams(const Config &config) {//初始化所有基类参数，但是为了稳妥还是在派生类构造函数里调用吧
		/* ===== 初始化基类参数 ===== */
		motor_type_ = config.motor_type;
		work_mode_ = config.motor_setting.work_mode;
		loop_type_ = config.motor_setting.loop_type;
		motor_direction_ = config.motor_setting.motor_direction;

		position_feedback_type_ = config.motor_setting.position_feedback_setting.type;//位置环反馈数据来源
		position_feedback_direction_ = config.motor_setting.position_feedback_setting.direction; //反馈量正反转标志
		position_feedback_source_func_ = config.motor_setting.position_feedback_setting.source_func; //反馈量获取函数

		velocity_feedback_type_ = config.motor_setting.velocity_feedback_setting.type;//速度环反馈数据来源
		velocity_feedback_direction_ = config.motor_setting.velocity_feedback_setting.direction; //反馈量正反转标志
		velocity_feedback_source_func_ = config.motor_setting.velocity_feedback_setting.source_func; //反馈量获取函数

	};
	// 电机掉线回调
	virtual void offlineCallback() {};

 protected:
	float ref_ = 0.0f; //根据Mode不同，所代表含义也可能不同
	bool is_online_ = false;//电机是否在线,根据是否能接到回调决定。
	bool is_enabled_ = false;//电机是否使能,根据用户意图决定是否使用电机。
	MeasureBase measure_base_{};

	//电机设置
	MotorType motor_type_;

	//控制设置
	WorkMode work_mode_;
	LoopType loop_type_;
	Direction motor_direction_ = Direction::NORMAL; //电机正反转标志


	//外部反馈源相关设置
	FeedbackType position_feedback_type_ = FeedbackType::MOTOR_FEED;//位置环反馈数据来源
	Direction position_feedback_direction_ = Direction::NORMAL; //反馈量正反转标志
	FeedbackSourceFunc position_feedback_source_func_ = nullptr;

	FeedbackType velocity_feedback_type_ = FeedbackType::MOTOR_FEED;//速度环反馈数据来源
	Direction velocity_feedback_direction_ = Direction::NORMAL; //反馈量正反转标志
	FeedbackSourceFunc velocity_feedback_source_func_ = nullptr;

	//电机内部维护实例，均使用optional延迟到派生类再注册
	std::optional<CANInstance> can_instance_;
	std::optional<DaemonInstance> daemon_;

	//三环控制器，大部分情况下使用前两环
	std::unique_ptr<Controller> position_controller_;
	std::unique_ptr<Controller> velocity_controller_;
	std::unique_ptr<Controller> current_controller_;

};

#endif //MOTOR_H_
