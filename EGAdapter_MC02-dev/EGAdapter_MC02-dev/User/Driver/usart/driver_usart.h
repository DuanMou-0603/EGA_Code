#pragma once
// HAL
#include "usart.h"
// STL
#include <functional>
// Custom
#include "loop_queue.h"
#include "bit_locker.h"

#define UART_MX_INS_NUM 6           ///< 最大支持的 UART 实例数，实例需要与外设一一对应
#define UART_MX_RX_BUFFER_SIZE 256  ///< 接收缓冲区大小，默认256字节

/**
 * @brief UART 驱动实例类
 */
class UARTInstance {
	// HAL 回调函数声明为友元，以便访问私有变量
	friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
	friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
	friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
	friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

 public:
	using UARTRecvCallback = std::function<void(uint8_t *, uint16_t)>; ///< 接收回调
	using UARTTransCallback = std::function<void()>;                   ///< 发送回调

/**
 * @brief UART 发送状态枚举
 */
	enum Tx_State {
		ERROR_STATE,         ///< 发送失败
		BLOCK_FINISH,  ///< 阻塞发送完成
		BLOCK_TIMEOUT, ///< 阻塞发送超时
		ONGOING,       ///< DMA/IT 正在发送
		WAITING,       ///< 已加入队列等待发送
		BUFF_FULL,     ///< 队列已满
	};

/**
 * @brief UART 发送模式枚举
 */
	enum Tx_Type {
		DMA,   ///< DMA 发送
		IT,    ///< 中断发送
		BLOCK, ///< 阻塞发送
	};

/**
 * @brief UART 接收模式枚举
 */
	enum Rx_Type {
		BLOCK_NUM,  ///< 阻塞接收，固定字节数
		DMA_NUM,    ///< DMA 接收固定字节数
		IT_NUM,     ///< 中断接收固定字节数
		BLOCK_IDLE, ///< 阻塞接收，空闲中断返回
		DMA_IDLE,   ///< DMA 接收+空闲中断
		IT_IDLE,    ///< 中断接收+空闲中断
	};


	/**
	 * @brief UART 初始化配置
	 */
	struct Config {
		UART_HandleTypeDef *handle = nullptr;

		Tx_Type tx_type = DMA;
		UARTTransCallback tx_cbk = nullptr;

		Rx_Type rx_type = IT_IDLE;
		UARTRecvCallback rx_cbk = nullptr;
		uint16_t rx_size = 0;

		bool use_fifo = true;
		uint8_t queue_mx_size = 4;
	};

 private:
	///  所有类实例的指针列表,注意为共享的,不要在外部修改
	static std::array<UARTInstance *, UART_MX_INS_NUM> instance_list_;
	static uint8_t instance_count_;

	UART_HandleTypeDef *handle_;                  ///< HAL UART 句柄
	uint16_t rx_size_;                            ///< 接收数据长度
	uint8_t rx_buff_[UART_MX_RX_BUFFER_SIZE]{0};     ///< 接收缓冲区
	Rx_Type rx_type_;                      ///< 接收模式
	UARTRecvCallback rx_cbk_;                     ///< 接收回调


	bool tx_use_fifo_;                            ///< 是否启用 FIFO 队列
	uint8_t tx_queue_mx_size_;                    ///< 队列最大长度
	loop_queue<uint8_t *> tx_queue_;              ///< 发送数据指针队列 // 注意此处只保存指针,module需自行维护数据块
	loop_queue<uint16_t> tx_len_queue_;           ///< 发送数据长度队列
	Tx_Type tx_type_;                      ///< 发送模式
	UARTTransCallback tx_cbk_;                    ///< 发送回调

	static constexpr uint8_t COPY_POOL_BLOCKS = 4; ///< 用于复制发送模式，可连续调用最多4次
	static constexpr uint16_t COPY_BLOCK_SIZE = 128;
	uint8_t copy_pool_[COPY_POOL_BLOCKS][COPY_BLOCK_SIZE]{0};  ///< 内部发送缓冲池
	bool copy_pool_used_[COPY_POOL_BLOCKS] = {false};       ///< 使用标记

	void popSend(); ///< 队列出队并启动一次发送

 public:
	explicit UARTInstance(const Config &config);
	~UARTInstance();///< 析构函数，自动清理 instance_list_

	// 禁止拷贝
	UARTInstance(const UARTInstance &) = delete;
	UARTInstance &operator=(const UARTInstance &) = delete;

	// 禁止移动
	UARTInstance(UARTInstance &&other) = delete;
	UARTInstance &operator=(UARTInstance &&other) = delete;

	// 允许移动
//	UARTInstance(UARTInstance&& other) noexcept;
//	UARTInstance& operator=(UARTInstance&& other) noexcept;

	/**
 * @brief 发送函数,BLOCK/IT/DMA三种模式,使用前需先设置发送模式(或初始化时传入)
 *
 * @attention timeout参数仅在[UART_TX_BLOCK]模式下生效
 * @attention 若为IT/DMA,需保证data在离开调用者的作用域前不会被修改且离开后不会被释放!!!
 * @attention 若使用非阻塞的队列发送,需保证这些数据块不被修改/释放!!!
 *
 * @param data 发送数据指针,注意在发送完成前不要修改/释放
 * @param size 发送数据长度,以byte计
 * @param timeout 发送超时时间,单位ms,仅在[UART_TX_BLOCK]模式下生效,注意默认参数值为无限等待
 * @return UART_Tx_State_e 返回发送状态
 */
	Tx_State send(uint8_t *data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
	Tx_State sendCopy(const uint8_t *data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);

	/**
 * @brief 修改发送设置,若使用发送队列请保证修改前的数据已经发送完成
 *
 * @attention 此函数会终止当前的发送并清空队列(若启用了fifo)
 *
 * @param type UART_Tx_Type_e的枚举值,分别为DMA/IT/BLOCK
 * @param use_fifo 是否使用发送队列,默认为false
 * @param queue_mx_size 发送队列最大长度,默认为0不使用
 */
	void setSendType(Tx_Type type, bool use_fifo = false, uint8_t queue_mx_size = 0);
	/**
 * @brief 在阻塞模式下于timeout时间内接收一定量的数据
 *
 * @param data 接收数据缓冲区指针
 * @param target_size 期望接收的数据量,以byte计
 * @param timeout 超时时间,默认为永久
 * @return uint16_t 实际接收到的数据量,若超时则返回0
 */
	uint16_t recv(uint8_t *data, uint16_t target_size, uint32_t timeout = HAL_MAX_DELAY);
	/**
 * @brief 修改接收设置,请保证缓冲区数据解析完毕/转移到他处
 *        此函数会停止正在进行的接收,并清空接收缓冲区
 *
 * @param type UART_Rx_Type_e的枚举值,分别为分别为 DMA/IT/BLOCK + NUM/IDLE
 * @param size 期望接收一包的数据量,以byte计
 */
	void setRecvType(Rx_Type type, uint16_t size);
	/**
 * @brief 重启接收服务,仅用于IT和DMA模式.
 *        若使用BLOCK模式,请使用Recv函数,否则此函数进入死循环
 */
	void startRecv();
	/**
 * @brief 停止当前正在进行的接收并清空缓冲区,BLOCK/IT/DMA三种模式通用
 *        恢复接收请使用UARTRestartRecv函数(仅用于IT和DMA模式)
 *        BLOCK接收重新通过UARTRecv函数发起
 */
	void stopRecv();
};
