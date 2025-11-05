//
// Created by An on 2025/9/22.
//

#pragma once

#include <cstdint>

class Gimbal{
 public:
	// 舵机状态枚举，用于早期上供弹步兵开盖
	enum class ServoState: uint8_t {
		NONE,					//无舵机
		OFF,         	// 关闭
		ON          	// 开启
	};
};



