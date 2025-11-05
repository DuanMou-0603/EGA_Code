//
// Created by An on 2025/10/29.
//

#pragma once

class Chassis {
 public:
	enum class WorkMode : uint8_t {
		STOP = 0,
		FOLLOW,	//跟随模式
		SPIN,		//自旋模式（小陀螺）
	};

	enum class PowerMode : uint8_t {
		NORMAL,	//正常移动
		RUSH, 	//冲刺,冲!
		SLOW,		//缓步模式
	};

};
