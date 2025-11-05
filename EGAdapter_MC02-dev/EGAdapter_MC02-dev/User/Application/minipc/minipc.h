//
// Created by An on 2025/10/30.
//

#pragma once


class MiniPC{
 public:
	//小电脑开火许可。 发射许可之一。
	enum class FirePermission{
		FORBID,
		PERMIT,
	};

	//小电脑状态变量
	struct MiniPCState {
		//todo 小电脑传来的值，需要根据视觉协议进行修改
		float offset_x;
		float offset_y;
		//...

		//开火许可 （发弹许可条件之一）
		MiniPC::FirePermission fire_permission = MiniPC::FirePermission::FORBID;
	};

	[[nodiscard]] MiniPCState getMiniPCState() const { return mini_pc_state_; };
 private:
		MiniPCState mini_pc_state_;
};