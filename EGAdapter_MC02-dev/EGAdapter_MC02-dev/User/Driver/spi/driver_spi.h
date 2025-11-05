//
// Created by zhangzhiwen on 25-10-10.
//

#ifndef DRIVER_SPI_H
#define DRIVER_SPI_H

// HAL
#include "main.h"
#include "spi.h"
// STL
#include <functional>
#include <array>
// Custom
#include "bit_locker.h"

#define SPI_MX_INS_NUM 2 ///< 最大支持的 SPI 实例数，实例需要与外设一一对应
#define SPI_MX_BUFFER_SIZE 64///< 缓冲区大小，默认64字节

// SPI允许一个物理外设对应多个实实例，通过片选引脚来区分
// SPI通信一般没有串口那样多处同时调用一个外设实例的情况，所以没有使用环形缓冲器
/**
 * @brief SPI 驱动实例类
 */
class SPIInstance
{
    // HAL 回调函数声明为友元，以便访问私有变量
    friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi);
    friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);
    friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);

public:
    using SPIRecvCallback  = std::function<void(uint8_t*, uint16_t)>; ///< 接收回调
    using SPITransCallback = std::function<void()>;                   ///< 发送回调

    /**
         * @brief SPI 传输状态枚举
         */
    enum TxRx_State
    {
        ERROR_STATE,   ///< 传输失败
        BLOCK_FINISH,  ///< 阻塞传输完成
        BLOCK_TIMEOUT, ///< 阻塞传输超时
        ONGOING,       ///< 正在传输
    };

    /**
     * @brief SPI 传输模式枚举
     */
    enum TxRx_Type
    {
        DMA,   ///< DMA 传输
        IT,    ///< 中断传输
        BLOCK, ///< 阻塞传输
    };

    /**
     * @brief SPI 工作模式配置枚举
     */
    enum TxRx_Mode
    {
        FULL_DUPLEX, ///< 全双工
        HALF_DUPLEX, ///< 半双工
        TX_ONLY,     ///< 仅发送
        RX_ONLY,     ///< 仅接收
    };

    /**
         * @brief SPI 初始化配置
         */
    struct Config
    {
        SPI_HandleTypeDef* handle            = nullptr;
        GPIO_TypeDef*      cs_port           = nullptr;
        uint16_t           cs_pin            = 0;
        GPIO_PinState      effect_pin_state_ = GPIO_PIN_RESET;
        TxRx_Type          type              = BLOCK;
        TxRx_Mode          mode              = FULL_DUPLEX;
        bool               is_master         = true; ///< 默认为主机
        uint16_t           tx_size           = 0;
        uint16_t           rx_size           = 0;
        SPITransCallback   tx_cbk            = nullptr;
        SPIRecvCallback    rx_cbk            = nullptr; ///< 接收回调（仅在 RX_ONLY 或全双工时有效）
    };

private:
    /// 所有类实例的指针列表，注意为共享的，不要在外部修改
    static std::array<SPIInstance*, SPI_MX_INS_NUM> instance_list_;
    static uint8_t                                  instance_count_;

    SPI_HandleTypeDef* handle_; ///< HAL SPI 句柄
    GPIO_TypeDef*      cs_port_;
    uint16_t           cs_pin_;
    GPIO_PinState      effect_pin_state_;
    TxRx_Type          type_;      ///< 当前传输模式
    TxRx_Mode          mode_;      ///< 工作模式
    bool               is_master_; ///< 是否为主机 true: 主机 false: 从机
    SPITransCallback   tx_cbk_;    ///< 传输完成回调
    SPIRecvCallback    rx_cbk_;    ///< 接收回调（用于 RX_ONLY 或全双工）

    // 发送缓冲区
    uint8_t  tx_buff_[SPI_MX_BUFFER_SIZE]{0};
    uint16_t tx_size_{0};
    // 接收缓冲区
    uint8_t  rx_buff_[SPI_MX_BUFFER_SIZE]{0};
    uint16_t rx_size_{0};

    // 当前正在传输的数据（用于回调识别）
    uint8_t* current_tx_data_ = nullptr;
    uint8_t* current_rx_data_ = nullptr;
    uint16_t current_size_    = 0;

public:
    explicit SPIInstance(const Config& config); // 构造函数，不会启动接收，需要用的时候再启动
    ~SPIInstance();                             ///< 析构函数，自动从 instance_list_ 中移除

    // 禁止拷贝
    SPIInstance(const SPIInstance&)            = delete;
    SPIInstance& operator=(const SPIInstance&) = delete;

    // 禁止移动
    SPIInstance(SPIInstance&& other)            = delete;
    SPIInstance& operator=(SPIInstance&& other) = delete;

    /**
     * @brief 收发数据（根据 type 和 mode 决定行为）
     *
     * @attention timeout 仅在 BLOCK 模式下生效
     * @attention IT/DMA 模式下，data 和 rx_data（如提供）必须在传输完成前保持有效！
     *
     * @param tx_data 发送数据指针
     * @param timeout 超时时间（ms），仅 BLOCK 模式有效，默认无限等待
     * @return Tx_State 传输状态
     */
    TxRx_State transRecv(uint8_t* tx_data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    TxRx_State transRecv(uint8_t* tx_data, uint8_t* rx_data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    // TX_ONLY
    TxRx_State trans(uint8_t* tx_data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);
    // RX_ONLY
    TxRx_State recv(uint16_t size, uint32_t timeout = HAL_MAX_DELAY);

    // 最有用的函数,需要在外部手动设置片选信号
    // TODO:DMA和IT实现
    void    writeReg(uint8_t reg, uint8_t byte); // 向地址写一字节数据
    uint8_t readReg(uint8_t reg);                // 从地址读一字节数据
    uint8_t byte(uint8_t byte);                  // 写一字节，读一字节

    /**
     * @brief 修改传输模式
     */
    void setTransRecvType(TxRx_Type type, uint16_t size);

    /**
     * @brief 停止当前传输（适用于所有模式）
     */
    void stop();

    // 片选信号，但是实际上永远置有效电平也可以（因为一个SPI外设只和一个从机通信）
    inline void cs_enable() const
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, effect_pin_state_);
    };

    inline void cs_disable() const
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, effect_pin_state_ == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    };
};


#endif //DRIVER_SPI_H
