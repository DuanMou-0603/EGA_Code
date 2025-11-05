//
// Author: An
// Date: 25-9-25
//

#ifndef DRIVER_CAN_HPP
#define DRIVER_CAN_HPP
// =============================== 调用库 ===============================
#include <sys/_stdint.h>
#include <memory>
#include <array>
#include <functional>

#include "fdcan.h"

#define MX_CAN_MSG_LEN 8   // 单帧最大数据长度,单位字节

// =============================== 变量区 ===============================

// =============================== 函数声明 ===============================

// =============================== 类声明 ===============================
struct can_msg_t {
	uint8_t data[MX_CAN_MSG_LEN] = {0};
	uint8_t length = MX_CAN_MSG_LEN; //默认情况下为8，必要时可自行修改
};

class CANInstance {
	// 简化类型定义
 public:
	static constexpr size_t MX_CAN_INS_NUM =6;// 单条总线最大负载
	static constexpr size_t CAN_DEV_NUM =3;// 芯片支持的CAN外设数量，对于H723来说有3个CAN外设
	using CANRxCallback = std::function<void(const uint8_t *data, uint8_t len)>;
	using CANTxCallback = std::function<void()>;

 private:
	// 静态共享指针数组,用于存储所有CAN实例,用于中断回调函数
	// 每个 FDCAN 外设一组定长表 + 计数，避免运行期堆分配
	static std::array<CANInstance*, MX_CAN_INS_NUM> instance_[CAN_DEV_NUM];
	static uint8_t instance_count_[CAN_DEV_NUM];
	// 用于分配过滤器,每次添加新的实例时,会分配一个过滤器并递增此变量
	static uint8_t can1_filter_idx_, can2_filter_idx_, can3_filter_idx_;

	//每个instance的私有成员
 private:
	FDCAN_HandleTypeDef *can_handle_;// can句柄
	//发送时自动设置
	FDCAN_TxHeaderTypeDef tx_header_{};  // CAN报文发送配置

	//tx
	uint32_t tx_id_;                // 发送id
	uint8_t tx_len_{};              // 疑似没什么用，这里假定发送的数据都是8位
	CANTxCallback tx_callback_;     // 传输完成回调函数
	uint8_t tx_buff_[MX_CAN_MSG_LEN]{};            // 发送缓存

	//rx
	uint32_t rx_id_;                // 接收id
	uint8_t rx_buff_[MX_CAN_MSG_LEN]{};            // 接收缓存,最大消息长度为8
	uint8_t rx_len_{};                // 接收长度,可能为0-8
	CANRxCallback rx_callback_;

 private:
	//构建实例时为接收ID设置过滤规则
	void addFilter();
	//启动CAN服务和接收中断
	void serviceInit();

 public:
	struct Config {
		FDCAN_HandleTypeDef *can_handle;              // can句柄
		uint32_t tx_id = 0;
		CANTxCallback tx_callback = nullptr;// 不提供。 日后需要写发送回调时才有用
		uint32_t rx_id = 0;
		CANRxCallback rx_callback = nullptr;// 回调函数：参数 (id, data, len)
	};

	CANInstance(const Config &config);
	~CANInstance();

	void setDataLength(uint8_t length);
	FDCAN_HandleTypeDef *getCanHandle() const {return can_handle_;}

	bool transmit(const can_msg_t &msg, uint16_t block_timeout_us = 150);//如果FIFO满，允许超时150us

	//测试用的方法，严格来说该方法会破坏实例的封装性
	/**
	 * @brief 直接使用硬件FIFO发送CAN消息
	 *
	 * @param target_id 目标ID
	 * @param data 数据指针
	 * @param length 数据长度(0-8)
	 * @param block_timeout_us 阻塞超时时间(微秒)
	 * @return true 发送成功
	 * @return false 发送失败
	 */
	bool transmit(uint32_t target_id, const uint8_t *data, uint8_t length, uint16_t block_timeout_us = 300);

	// 回调设置,需要在外部调用所以设置为static
	static void CANRxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
};

#endif //DRIVER_CAN_HPP