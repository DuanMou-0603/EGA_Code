//
// Created by An on 2025/10/24.
//

#pragma once

class Shoot {
 public:
	// 拨弹轮许可状态枚举
	enum class RammerPermission {
		FORBID = false,      // 禁止
		PERMIT = true       // 允许
	};

	//摩擦轮状态
	enum class FricMode {
		OFF,          // 摩擦轮关闭
		ON,          // 摩擦轮开启
		//LOW_SPEED,	// 低射速模式，用于（可能出现的）不同射速情况
		//HIGH_SPEED, // 高射速模式，用于（可能出现的）不同射速情况
		//ADAPT_SPEED,// 用户可调射速模式，用于（可能出现的）不同射速情况
	};

};