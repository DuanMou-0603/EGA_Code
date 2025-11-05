#include "driver_usart.h"
#include <cstring>

/// 静态成员初始化
std::array<UARTInstance*, UART_MX_INS_NUM> UARTInstance::instance_list_{};
uint8_t UARTInstance::instance_count_ = 0;

UARTInstance::UARTInstance(const Config &config) {
	// 参数检查
	if (config.handle == nullptr) {
		//ERROR("[UARTInstance] handle is nullptr");
	}
	if (config.tx_type == BLOCK && config.use_fifo) {
		//ERROR("[UARTInstance] BLOCK 模式不能启用 FIFO");
	}
	if (config.use_fifo && config.queue_mx_size == 0) {
		//ERROR("[UARTInstance] FIFO 队列大小为 0");
	}
	if (config.rx_size == 0) {
		//WARNING("[UARTInstance] rx_size 设置为 0");
	}
	if (config.rx_cbk == nullptr) {
		//WARNING("[UARTInstance] rx_cbk 为空指针");
	}
	//检测重复实例
	for (auto ins : instance_list_) {
		if (ins->handle_ == config.handle) {
			//ERROR("UARTInstance for this handle already exists!");
			handle_ = nullptr;
			return;
		}
}
	// 成员初始化
	handle_ = config.handle;
	rx_size_ = config.rx_size;
	rx_type_ = config.rx_type;
	rx_cbk_ = config.rx_cbk;

	tx_use_fifo_ = config.use_fifo;
	tx_queue_mx_size_ = tx_use_fifo_ ? config.queue_mx_size : 1;
	tx_type_ = config.tx_type;
	tx_cbk_ = config.tx_cbk;

	// 加入实例列表
	if (instance_count_ < UART_MX_INS_NUM) {
		instance_list_[instance_count_++] = this;
	}

	if (rx_cbk_ != nullptr &&
			(rx_type_ == DMA_IDLE ||
					rx_type_ == IT_IDLE ||
					rx_type_ == DMA_NUM ||
					rx_type_ == IT_NUM)) {
		startRecv();
	}

}

UARTInstance::~UARTInstance() {
	for (uint8_t i = 0; i < instance_count_; i++) {
		if (instance_list_[i] == this) {
			// 用最后一个覆盖当前元素
			instance_list_[i] = instance_list_[instance_count_ - 1];
			instance_list_[instance_count_ - 1] = nullptr;
			--instance_count_;
			break;
		}
	}
}
//
//UARTInstance::UARTInstance(UARTInstance&& other) noexcept {
//	// 直接接管资源
//	handle_ = other.handle_;
//	rx_size_ = other.rx_size_;
//	rx_type_ = other.rx_type_;
//	rx_cbk_ = other.rx_cbk_;
//
//	tx_use_fifo_ = other.tx_use_fifo_;
//	tx_queue_mx_size_ = other.tx_queue_mx_size_;
//	tx_type_ = other.tx_type_;
//	tx_cbk_ = other.tx_cbk_;
//
//	memcpy(rx_buff_, other.rx_buff_, UART_MX_RX_BUFFER_SIZE);
//
//	// 注册到实例列表
//	if (instance_count_ < UART_MX_INS_NUM) {
//		instance_list_[instance_count_++] = this;
//	}
//
//	// 把 old 标记成空
//	other.handle_ = nullptr;
//}
//
//UARTInstance& UARTInstance::operator=(UARTInstance&& other) noexcept {
//	if (this != &other) {
//		// 先把自己从实例列表移除
//		for (uint8_t i = 0; i < instance_count_; i++) {
//			if (instance_list_[i] == this) {
//				instance_list_[i] = instance_list_[instance_count_ - 1];
//				instance_list_[instance_count_ - 1] = nullptr;
//				--instance_count_;
//				break;
//			}
//		}
//
//		// 接管资源
//		handle_      = other.handle_;
//		rx_size_     = other.rx_size_;
//		rx_type_     = other.rx_type_;
//		rx_cbk_      = std::move(other.rx_cbk_);
//
//		tx_use_fifo_     = other.tx_use_fifo_;
//		tx_queue_mx_size_ = other.tx_queue_mx_size_;
//		tx_type_         = other.tx_type_;
//		tx_cbk_          = std::move(other.tx_cbk_);
//
//		memcpy(rx_buff_, other.rx_buff_, UART_MX_RX_BUFFER_SIZE);
//
//		// 注册到实例列表（类似 push_back）
//		if (instance_count_ < UART_MX_INS_NUM) {
//			instance_list_[instance_count_++] = this;
//		}
//
//		// 清空 old
//		other.handle_ = nullptr;
//	}
//	return *this;
//}

void UARTInstance::popSend() {
	if (tx_queue_.size()) {
		uint8_t *data = tx_queue_.front();
		uint16_t len = tx_len_queue_.front();

		if (tx_type_ == DMA) {
			if (HAL_UART_Transmit_DMA(handle_, data, len) != HAL_OK) {
				//ERROR("[PopSend] HAL_UART_Transmit_DMA failed");
			}
		} else if (tx_type_ == IT) {
			if (HAL_UART_Transmit_IT(handle_, data, len) != HAL_OK) {
				//ERROR("[PopSend] HAL_UART_Transmit_IT failed");
			}
		} else {
			//ERROR("[PopSend] BLOCK 模式不支持 FIFO");
		}
	} else {
		//WARNING("[PopSend] tx_queue_ 为空");
	}
}

UARTInstance::Tx_State UARTInstance::send(uint8_t *data, uint16_t size, uint32_t timeout) {
	if (data == nullptr || size == 0) {
		//ERROR("[UARTSend] 参数错误: data=nullptr 或 size=0");
	}

	if (tx_type_ == BLOCK) {
		if (HAL_UART_Transmit(handle_, data, size, timeout) == HAL_TIMEOUT)
			return BLOCK_TIMEOUT;
		else
			return BLOCK_FINISH;
	}

	if ((tx_use_fifo_ && tx_queue_.size() < tx_queue_mx_size_) || !tx_use_fifo_) {
		tx_queue_.push(data);
		tx_len_queue_.push(size);
		if (tx_queue_.size() == 1) {
			popSend();
			return ONGOING;
		} else {
			return WAITING;
		}
	} else {
		//WARNING("[UARTSend] 队列已满");
		return BUFF_FULL;
	}
}

UARTInstance::Tx_State UARTInstance::sendCopy(const uint8_t *data, uint16_t size, uint32_t timeout) {
	if (data == nullptr || size == 0 || size > COPY_BLOCK_SIZE) {
		//ERROR("[UARTSendCopy] 参数错误: data=nullptr 或 size 超过限制");
		return ERROR_STATE;
	}

	// 找一个空闲块
	int free_idx = -1;
	for (uint8_t i = 0; i < COPY_POOL_BLOCKS; i++) {
		if (!copy_pool_used_[i]) {
			free_idx = i;
			break;
		}
	}

	if (free_idx < 0) {
		// 没有空闲块
		return BUFF_FULL;
	}

	// 拷贝数据到内部缓冲池
	memcpy(copy_pool_[free_idx], data, size);
	copy_pool_used_[free_idx] = true;

	// 入队发送，仍然走原来的 Send 流程，但传指针是内部缓冲
	Tx_State state = send(copy_pool_[free_idx], size, timeout);

	// 注意：释放时机在发送完成回调里
	if (state == ONGOING || state == WAITING) {
		// 在 HAL_UART_TxCpltCallback 中检测完成后释放 copy_pool_used_[idx]
	} else {
		// 如果发送失败，立刻释放
		copy_pool_used_[free_idx] = false;
	}

	return state;
}

void UARTInstance::setSendType(Tx_Type type, bool use_fifo, uint8_t queue_mx_size) {
	if (type == BLOCK && use_fifo) {
		//ERROR("[UARTSetSendType] BLOCK 模式不能使用 FIFO");
	}
	if (use_fifo && queue_mx_size == 0) {
		//ERROR("[UARTSetSendType] FIFO 队列大小不能为 0");
		queue_mx_size = 1;
	}

	tx_type_ = type;
	tx_use_fifo_ = use_fifo;
	HAL_UART_AbortTransmit(handle_);
	tx_queue_.clear();
	tx_len_queue_.clear();
	tx_queue_mx_size_ = queue_mx_size;
}

uint16_t UARTInstance::recv(uint8_t *data, uint16_t target_size, uint32_t timeout) {
	if (data == nullptr || target_size == 0) {
		//ERROR("[UARTRecv] 参数错误: data=nullptr 或 target_size=0");
	}

	if (rx_type_ == BLOCK_NUM) {
		if (HAL_UART_Receive(handle_, data, target_size, timeout) == HAL_OK)
			return target_size;
	} else if (rx_type_ == BLOCK_IDLE) {
		uint16_t real_rx_size = 0;
		if (HAL_UARTEx_ReceiveToIdle(handle_, data, target_size, &real_rx_size, timeout) == HAL_OK)
			return real_rx_size;
	} else {
		//ERROR("[UARTRecv] 当前模式不是阻塞接收");
	}
	return 0;
}

void UARTInstance::stopRecv() {
	HAL_UART_AbortReceive(handle_);
	memset(rx_buff_, 0, UART_MX_RX_BUFFER_SIZE);
	//WARNING("[UARTStopRecv] 接收已停止");
}

void UARTInstance::startRecv() {
	switch (rx_type_) {
		case DMA_IDLE: HAL_UARTEx_ReceiveToIdle_DMA(handle_, rx_buff_, rx_size_);
			__HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
			break;
		case IT_IDLE: HAL_UARTEx_ReceiveToIdle_IT(handle_, rx_buff_, rx_size_);
			break;
		case DMA_NUM: HAL_UART_Receive_DMA(handle_, rx_buff_, rx_size_);
			__HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
			break; // ✅ 修复缺失 break
		case IT_NUM: HAL_UART_Receive_IT(handle_, rx_buff_, rx_size_);
			break;
		default:
			//ERROR("[UARTRestartRecv] rx_type_ 非法");
			break;
	}
}

void UARTInstance::setRecvType(Rx_Type type, uint16_t size) {
	rx_type_ = type;
	rx_size_ = size;
	stopRecv();
	if (type != BLOCK_IDLE && type != BLOCK_NUM)
		startRecv(); // 非阻塞模式,重启接收服务
}

// ---------------- HAL 回调 ----------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	for (auto ins : UARTInstance::instance_list_) {
		if (ins->handle_ == huart && ins->rx_cbk_ != nullptr) {
			ins->rx_cbk_(ins->rx_buff_, Size);
			ins->startRecv();
			break;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	for (uint8_t i = 0; i < UARTInstance::instance_count_; i++) {
		auto ins =  UARTInstance::instance_list_[i];
		if (ins && ins->handle_ == huart) {
			// 出队当前已经完成发送的数据
			if (ins->tx_use_fifo_ && ins->tx_queue_.size()) {
				uint8_t* finished_ptr = ins->tx_queue_.front();
				ins->tx_queue_.pop();
				ins->tx_len_queue_.pop();

				// 如果 finished_ptr 来自内部 copy_pool，标记为空闲
				for (uint8_t i = 0; i < UARTInstance::COPY_POOL_BLOCKS; i++) {
					if (finished_ptr == ins->copy_pool_[i]) {
						ins->copy_pool_used_[i] = false;
						break;
					}
				}

				// 如果还有剩余数据，继续发送
				if (ins->tx_queue_.size()) {
					ins->popSend();
				}
			}

			// 执行用户设置的发送完成回调
			if (ins->tx_cbk_ != nullptr) {
				ins->tx_cbk_();
			}
			return;
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	for (auto ins : UARTInstance::instance_list_) {
		if (ins->handle_ == huart) {
			//WARNING("[UARTErrCbk] UART 错误");
			if (ins->rx_type_ != UARTInstance::BLOCK_IDLE &&
					ins->rx_type_ != UARTInstance::BLOCK_NUM) {
				ins->startRecv();
			}
		}
	}
}
