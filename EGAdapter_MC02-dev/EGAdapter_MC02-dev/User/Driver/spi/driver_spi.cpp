//
// Created by zhangzhiwen on 25-10-10.
//

#include "driver_spi.h"

#include "driver_usart.h"
#include "gpio.h"
/// 静态成员初始化
std::array<SPIInstance*, SPI_MX_INS_NUM> SPIInstance::instance_list_{};
uint8_t                                  SPIInstance::instance_count_ = 0;

SPIInstance::SPIInstance(const Config& config)
{
    // 参数检查
    if (config.handle == nullptr)
    {
        // ERROR("[SPIInstance] handle is nullptr");
    }
    if (config.cs_port == nullptr)
    {
        // ERROR("[SPIInstance] csPort is nullptr");
    }
    if (config.rx_size == 0)
    {
        //WARNING("[UARTInstance] rx_size 设置为 0");
    }
    if (config.rx_cbk == nullptr)
    {
        //WARNING("[UARTInstance] rx_cbk 为空指针");
    }
    //检测重复实例
    for (auto ins : instance_list_)
    {
        if (ins->handle_ == config.handle
            && ins->cs_port_ == config.cs_port
            && ins->cs_pin_ == config.cs_pin)
        {
            //ERROR("SPIInstance for this handle already exists!");
            handle_ = nullptr;
            return;
        }
    }
    // 成员初始化
    this->handle_           = config.handle;
    this->cs_port_          = config.cs_port;
    this->cs_pin_           = config.cs_pin;
    this->effect_pin_state_ = config.effect_pin_state_;
    this->type_             = config.type;
    this->mode_             = config.mode;
    this->is_master_        = config.is_master;
    this->tx_size_          = config.tx_size;
    this->rx_size_          = config.rx_size;
    this->rx_cbk_           = config.rx_cbk;
    this->tx_cbk_           = config.tx_cbk;
    // 加入实例列表
    if (instance_count_ < SPI_MX_INS_NUM)
    {
        instance_list_[instance_count_++] = this;
    }
    // CS_ENABLE();
}

SPIInstance::~SPIInstance()
{
    for (uint8_t i = 0; i < instance_count_; i++)
    {
        if (instance_list_[i] == this)
        {
            // 用最后一个覆盖当前元素
            instance_list_[i]                   = instance_list_[instance_count_ - 1];
            instance_list_[instance_count_ - 1] = nullptr;
            --instance_count_;
            break;
        }
    }
}

SPIInstance::TxRx_State SPIInstance::transRecv(uint8_t* tx_data, uint16_t size, uint32_t timeout)
{
    if (mode_ == TX_ONLY || mode_ == RX_ONLY) { return ERROR_STATE; }
    if (tx_data == nullptr || size == 0)
    {
        // ERROR("[SPIInstance::transRecv] tx_data is nullptr or rx_data is nullptr");
        return ERROR_STATE;
    }
    tx_size_ = size;
    rx_size_ = size;
	cs_enable();
    switch (this->type_)
    {
    case BLOCK:
        {
            if (HAL_SPI_TransmitReceive(handle_, tx_data, rx_buff_, size, timeout) == HAL_TIMEOUT)
            {
							cs_disable();
                return BLOCK_TIMEOUT;
            }
            while (handle_->State != HAL_SPI_STATE_READY)
            {
            }
					cs_disable();
            return BLOCK_FINISH;
            break;
        }
    case DMA:
        {
            if (HAL_SPI_TransmitReceive_DMA(handle_, tx_data, rx_buff_, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_DMA failed");
            }
            return ONGOING;
        }
    case IT:
        {
            if (HAL_SPI_TransmitReceive_IT(handle_, tx_data, rx_buff_, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_IT failed");
            }
            return ONGOING;
        }
    default:
        {
            return ERROR_STATE;
        }
    }
}

SPIInstance::TxRx_State SPIInstance::transRecv(uint8_t* tx_data, uint8_t* rx_data, uint16_t size, uint32_t timeout)
{
    if (mode_ == TX_ONLY || mode_ == RX_ONLY) { return ERROR_STATE; }
    if (tx_data == nullptr || rx_data == nullptr || size == 0)
    {
        // ERROR("[SPIInstance::transRecv] tx_data is nullptr or rx_data is nullptr");
        return ERROR_STATE;
    }
    tx_size_ = size;
    rx_size_ = size;
	cs_enable();
    switch (this->type_)
    {
    case BLOCK:
        {
            if (HAL_SPI_TransmitReceive(handle_, tx_data, rx_data, size, timeout) == HAL_TIMEOUT)
            {
							cs_disable();
                return BLOCK_TIMEOUT;
            }
            while (handle_->State != HAL_SPI_STATE_READY)
            {
            }
					cs_disable();
            return BLOCK_FINISH;
            break;
        }
    case DMA:
        {
            if (HAL_SPI_TransmitReceive_DMA(handle_, tx_data, rx_data, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_DMA failed");
            }
            return ONGOING;
        }
    case IT:
        {
            if (HAL_SPI_TransmitReceive_IT(handle_, tx_data, rx_data, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_IT failed");
            }
            return ONGOING;
        }
    default:
        {
            return ERROR_STATE;
        }
    }
}

SPIInstance::TxRx_State SPIInstance::trans(uint8_t* tx_data, uint16_t size, uint32_t timeout)
{
    if (mode_ == RX_ONLY) { return ERROR_STATE; }
    if (tx_data == nullptr || size == 0)
    {
        return ERROR_STATE;
    }
    tx_size_ = size;
	cs_enable();
    switch (this->type_)
    {
    case BLOCK:
        {
            if (HAL_SPI_Transmit(handle_, tx_data, size, timeout) == HAL_TIMEOUT)
            {
							cs_disable();
                return BLOCK_TIMEOUT;
            }
            while (handle_->State != HAL_SPI_STATE_READY)
            {
            }
					cs_disable();
            return BLOCK_FINISH;
            break;
        }
    case DMA:
        {
            if (HAL_SPI_Transmit_DMA(handle_, tx_data, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_DMA failed");
            }
            return ONGOING;
        }
    case IT:
        {
            if (HAL_SPI_Transmit_IT(handle_, tx_data, size) != HAL_OK)
            {
                //ERROR("[transRecv] HAL_SPI_TransmitReceive_IT failed");
            }
            return ONGOING;
        }
    default:
        {
            return ERROR_STATE;
        }
    }
}

SPIInstance::TxRx_State SPIInstance::recv(uint16_t size, uint32_t timeout)
{
    if (mode_ == TX_ONLY) { return ERROR_STATE; }
    if (size == 0) { return ERROR_STATE; }
    rx_size_ = size;
	cs_enable();
    switch (this->type_)
    {
    case BLOCK:
        {
            if (HAL_SPI_Receive(handle_, rx_buff_, size, timeout) == HAL_TIMEOUT)
            {
							cs_disable();
                return BLOCK_TIMEOUT;
            }
            while (handle_->State != HAL_SPI_STATE_READY)
            {
            }
					cs_disable();
            return BLOCK_FINISH;
        }
    case DMA:
        {
            if (HAL_SPI_Receive_DMA(handle_, rx_buff_, size) != HAL_OK)
            {
                ;
            }
            return ONGOING;
        }
    case IT:
        {
            if (HAL_SPI_Receive_IT(handle_, rx_buff_, size) != HAL_OK)
            {
                ;
            }
            return ONGOING;
        }
    default:
        {
            return ERROR_STATE;
        }
    }
}

void SPIInstance::writeReg(uint8_t reg, uint8_t byte)
{
    tx_buff_[0] = reg;
    tx_buff_[1] = byte;
    HAL_SPI_TransmitReceive(handle_, tx_buff_, rx_buff_, 2, HAL_MAX_DELAY);
    while (handle_->State != HAL_SPI_STATE_READY);
}

uint8_t SPIInstance::readReg(uint8_t reg)
{
    tx_buff_[0] = reg;
    tx_buff_[1] = 0x55;
    HAL_SPI_TransmitReceive(handle_, tx_buff_, rx_buff_, 2, HAL_MAX_DELAY);
    while (handle_->State != HAL_SPI_STATE_READY)
    {
    }
    return rx_buff_[1];
}

uint8_t SPIInstance::byte(uint8_t byte)
{
    tx_buff_[0] = byte;
    HAL_SPI_TransmitReceive(handle_, tx_buff_, rx_buff_, 1, HAL_MAX_DELAY);
    while (handle_->State != HAL_SPI_STATE_READY)
    {
    }
    return rx_buff_[0];
}


void SPIInstance::setTransRecvType(TxRx_Type type, uint16_t size)
{
    stop();
    type_    = type;
    tx_size_ = size;
    rx_size_ = size;
}

void SPIInstance::stop()
{
    switch (type_)
    {
    case BLOCK:
        {
            HAL_SPI_Abort(handle_);
            break;
        }
    case IT:
        {
            HAL_SPI_Abort_IT(handle_);
            break;
        }
    case DMA:
        {
            HAL_SPI_DMAPause(handle_);
            break;
        }
    default:
        {
            ;
        }
    }
	cs_disable();
}

// for TX_ONLY
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    for (auto& ins : SPIInstance::instance_list_)
    {
        if (ins->handle_ == hspi && ins->tx_cbk_ != nullptr)
        {
            ins->tx_cbk_();
					ins->cs_disable();
        }
    }
}

// for RX_ONLY
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    for (auto& ins : SPIInstance::instance_list_)
    {
        if (ins->handle_ == hspi && ins->rx_cbk_ != nullptr)
        {
            ins->rx_cbk_(ins->rx_buff_, ins->rx_size_);
					ins->cs_disable();
        }
    }
}

// for Duplex
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    for (auto& ins : SPIInstance::instance_list_)
    {
        if (ins->handle_ == hspi && ins->rx_cbk_ != nullptr)
        {
            ins->rx_cbk_(ins->rx_buff_, ins->rx_size_);
					ins->cs_disable();
        }
    }
}
