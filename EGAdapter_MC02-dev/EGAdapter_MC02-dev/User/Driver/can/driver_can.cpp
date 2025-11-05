//
// Author: Breezeee
// Date: 25-5-28
//

// =============================== 引入头文件 ===============================
#include <cstdint>
#include "memory.h"
#include "fdcan.h"
#include "driver_can.h"
#include "driver_dwt.h"


// =============================== 宏定义区 ===============================

// =============================== 变量区 ===============================
std::array<CANInstance*, CANInstance::MX_CAN_INS_NUM> CANInstance::instance_[CAN_DEV_NUM]{};
uint8_t CANInstance::instance_count_[CAN_DEV_NUM] = {0, 0, 0};
uint8_t CANInstance::can1_filter_idx_ = 0;
uint8_t CANInstance::can2_filter_idx_ = 0;
uint8_t CANInstance::can3_filter_idx_ = 0;

// =============================== 函数实现 ===============================

CANInstance::CANInstance(const Config &config) {
	//config合法性检查
	if (config.can_handle == nullptr) {
		//等到添加日志模块后，写报错信息
		Error_Handler();
	}
	if (config.tx_id > 0x7FF || config.rx_id > 0x7FF || config.tx_id == config.rx_id || !config.tx_id || !config.rx_id) {
		//等到添加日志模块后，写报错信息
		//暂时注释掉，允许非法的id
		//Error_Handler();
	}
	if (config.rx_callback == nullptr) {
		//等到添加日志模块后，写报错信息
	}

	can_handle_ = config.can_handle;
	tx_id_ = config.tx_id;
	rx_id_ = config.rx_id;
	//tx_callback_ = config.tx_callback;
	rx_callback_ = config.rx_callback;

	tx_header_.Identifier = tx_id_;
	tx_header_.IdType = FDCAN_STANDARD_ID;
	tx_header_.TxFrameType = FDCAN_DATA_FRAME;
	tx_header_.DataLength = FDCAN_DLC_BYTES_8; //默认长度为8，在发送时可更改
	tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header_.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	tx_header_.MessageMarker = 0;

	// 在第一个实例被创建后，统一启动三个fdcan服务
	if (can1_filter_idx_ == 0 && can2_filter_idx_ == 0 && can3_filter_idx_ == 0) {
		serviceInit();
	}

	addFilter();

	uint8_t can_dev = can_handle_->Instance == FDCAN1 ? 0 :
										(can_handle_->Instance == FDCAN2 ? 1 : 2);

	// 加入实例列表（尾部插入）
	if (instance_count_[can_dev] < MX_CAN_INS_NUM) {
		instance_[can_dev][instance_count_[can_dev]++] = this;
	}
}

CANInstance::~CANInstance() {
	if (!can_handle_) return;
	uint8_t can_dev = can_handle_->Instance == FDCAN1 ? 0 :
										(can_handle_->Instance == FDCAN2 ? 1 : 2);

	for (uint8_t i = 0; i < instance_count_[can_dev]; ++i) {
		if (instance_[can_dev][i] == this) {
			// 用最后一个覆盖当前项，收缩计数，避免“空洞”
			instance_[can_dev][i] = instance_[can_dev][instance_count_[can_dev] - 1];
			instance_[can_dev][instance_count_[can_dev] - 1] = nullptr;
			--instance_count_[can_dev];
			break;
		}
	}
}

void CANInstance::addFilter() {
	// 如果超出CubeMX中设置的滤波器数量上限就报错
	if (can1_filter_idx_ > hfdcan1.Init.StdFiltersNbr ||
			can2_filter_idx_ > hfdcan2.Init.StdFiltersNbr ||
			can3_filter_idx_ > hfdcan3.Init.StdFiltersNbr) {
		Error_Handler();
	}

	FDCAN_FilterTypeDef filter_cfg;

	filter_cfg.IdType = FDCAN_STANDARD_ID;
	filter_cfg.FilterIndex = can_handle_->Instance == FDCAN1 ? (can1_filter_idx_++) :
													 (can_handle_->Instance == FDCAN2 ? (can2_filter_idx_++) :
														(can3_filter_idx_++));
	filter_cfg.FilterType = FDCAN_FILTER_MASK;
	filter_cfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter_cfg.FilterID1 = rx_id_ << 5;  // STDID放在低11位，需要左移5位
	filter_cfg.FilterID2 = 0x7FF << 5;   // 掩码：匹配所有位

	if (HAL_FDCAN_ConfigFilter(can_handle_, &filter_cfg) != HAL_OK) {
		Error_Handler();
	}
}

void CANInstance::serviceInit() {
	// 中断设置
	uint32_t FDCAN_ActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;

	// 初始化三个FDCAN外设
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_ActiveITs, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_ActiveITs, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_ActiveITs, 0);
}

void CANInstance::setDataLength(uint8_t length) {
	if (length > 8 || length == 0) {
		// 长度错误处理
		Error_Handler();
	}
	tx_header_.DataLength = length;
}

//常规调用方法，发送和接收id依赖实例生成时的配置
bool CANInstance::transmit(const can_msg_t &msg, uint16_t block_timeout_us) {
	if (can_handle_ == nullptr || msg.data == nullptr || msg.length > MX_CAN_MSG_LEN) {
		return false;
	}

	memcpy(tx_buff_, msg.data, msg.length);
	tx_header_.DataLength = msg.length;

	// 4. 检查硬件FIFO状态，如果满则等待
	if (DWTInstance::isInitialized()) {
		uint32_t start_time = DWTInstance::getTimeline_us();
		while (HAL_FDCAN_GetTxFifoFreeLevel(can_handle_) == 0) {
			if ((DWTInstance::getTimeline_us() - start_time) > block_timeout_us) {
				return false; // 超时
			}
		}
	}


	// 5. 发送消息
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(can_handle_, &tx_header_, tx_buff_);
	if (status != HAL_OK) {
		return false;
	}

	return true;
}

//临时发送用的
bool CANInstance::transmit(uint32_t target_id, const uint8_t *data, uint8_t length, uint16_t block_timeout_us) {
	// 1. 参数检查
	if (can_handle_ == nullptr || data == nullptr || length > MX_CAN_MSG_LEN) {
		return false;
	}

	// 2. 构造临时发送头（基于默认配置）
	FDCAN_TxHeaderTypeDef temp_header = tx_header_;
	temp_header.Identifier = target_id;
	temp_header.DataLength = length;

	// 3. 复制数据到发送缓冲区
	uint8_t temp_buff[MX_CAN_MSG_LEN];
	if (length > 0) {
		memcpy(temp_buff, data, length);
	}

	// 4. 检查 FIFO 状态，如果满则等待
	if (DWTInstance::isInitialized()) {
		uint32_t start_time = DWTInstance::getTimeline_us();
		while (HAL_FDCAN_GetTxFifoFreeLevel(can_handle_) == 0) {
			if ((DWTInstance::getTimeline_us() - start_time) > block_timeout_us) {
				return false; // 超时
			}
		}
	}

	// 5. 发送消息
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(can_handle_, &temp_header, temp_buff);
	if (status != HAL_OK) {
		return false;
	}

	return true;
}

void CANInstance::CANRxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo) {
	uint8_t can_dev = hfdcan->Instance == FDCAN1 ? 0 :
										(hfdcan->Instance == FDCAN2 ? 1 : 2);

	// 处理RX FIFO中的所有新消息
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[MX_CAN_MSG_LEN];

	while (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) == HAL_OK) {
		// 遍历当前 CAN 外设的有效实例
		for (uint8_t i = 0; i < instance_count_[can_dev]; ++i) {
			CANInstance* instance = instance_[can_dev][i];
			if (instance && instance->rx_id_ == rx_header.Identifier) {
				memcpy(instance->rx_buff_, rx_data, rx_header.DataLength);
				instance->rx_len_ = rx_header.DataLength;
				if (instance->rx_callback_ != nullptr) {
					//回调函数要求同时获取id，data，len
//					instance->rx_callback_(rx_header.Identifier,
//																 instance->rx_buff_,
//																 instance->rx_len_);
					//回调函数要求data和len，因为实例化时已经指定了rx_id这里不再加了
					instance->rx_callback_(instance->rx_buff_,
																 instance->rx_len_);
				}
				break; // 保持你当前“命中一个就退出”的语义不变
			}
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		CANInstance::CANRxFifoCallback(hfdcan, FDCAN_RX_FIFO0);
	}
}