//
// Created by zhangzhiwen on 25-10-13.
//

#include "bmi088.h"

#include "cmsis_os.h"

#include "logger.h"

BMI088::BMI088(const Config& config) : IMU(),
                                       error_code_(BMI088_NO_ERROR),
                                       accel_sensitivity_(BMI088_ACCEL_6G_SEN),
                                       gyro_sensitivity_(BMI088_GYRO_2000_SEN),
                                       accel_scale_(1.0f),
                                       temperature_(0.0f)
{
    //初始化基类参数
    initParams(config);

    //todo 目前强制使用默认外设，之后改成可以用config提供
    //初始化加速度计外设
    SPIInstance::Config accel_spi_cfg{
        .handle = &hspi2,
        .cs_port = ACC_CS_GPIO_Port,
        .cs_pin = ACC_CS_Pin,
        .effect_pin_state_ = GPIO_PIN_RESET,
        .type = SPIInstance::BLOCK,
        .mode = SPIInstance::FULL_DUPLEX,
        .is_master = true,
        .tx_size = 8,
        .rx_size = 8,
        .tx_cbk = nullptr,
        .rx_cbk = nullptr,
    };
    spi_accel_.emplace(accel_spi_cfg);

    //初始化陀螺仪外设
    SPIInstance::Config
        gyro_spi_cfg({
            .handle = &hspi2,
            .cs_port = GYRO_CS_GPIO_Port,
            .cs_pin = GYRO_CS_Pin,
            .effect_pin_state_ = GPIO_PIN_RESET,
            .type = SPIInstance::BLOCK,
            .mode = SPIInstance::FULL_DUPLEX,
            .is_master = true,
            .tx_size = 8,
            .rx_size = 8,
            .tx_cbk = nullptr,
            .rx_cbk = nullptr,
        });
    spi_gyro_.emplace(gyro_spi_cfg);
    //固定参数初始化滤波器 todo 考虑将其设置为带默认值的config

    //选择姿态融合滤波器类型
    switch (filter_type_)
    {
    case FilterType::EKF:
        {
            ekf_filter_.emplace(quaternion_, sample_time_);
        }
    case FilterType::MADGWICK:
        {
            madgwick_filter_.emplace();
        }
    }

    //清零偏置
    memset(gyro_offset_, 0, sizeof(gyro_offset_));
}

BMI088::Error BMI088::init()
{
    error_code_ = BMI088_NO_ERROR;
    error_code_ = accelInit();
    error_code_ = static_cast<Error>(static_cast<uint8_t>(error_code_) | static_cast<uint8_t>(gyroInit()));

    return error_code_;
}

void BMI088::calibrate()
{
    constexpr uint16_t kSampleCount = 6000;
    float              gyro_sum[3]  = {0.0f};

    for (uint16_t i = 0; i < kSampleCount; ++i)
    {
        readGyro();
        for (uint8_t j = 0; j < 3; ++j)
            gyro_sum[j] += gyro_data_(j, 0);
        osDelay(1);
    }
    for (uint8_t j      = 0; j < 3; ++j)
        gyro_offset_[j] = gyro_sum[j] / kSampleCount;
}

void BMI088::update()
{
    readAccel();
    readGyro();
    readTemperature();
}

void BMI088::compute()
{
    if (filter_type_ == FilterType::EKF)
    {
        ekf_filter_->updateIMU(accel_data_, gyro_data_, quaternion_);
        eulerAngles_ = quaternion_.ToEuler();
        //logger_printf("%f,%f,%f,%f\r\n",quaternion_.w,quaternion_.x,quaternion_.y,quaternion_.z);
    }
    else if (filter_type_ == FilterType::MADGWICK)
    {
        madgwick_filter_->updateIMU(accel_data_, gyro_data_, quaternion_);
        eulerAngles_ = quaternion_.ToEuler();
    }
}

// =============================================================
// 私有函数实现
// =============================================================
BMI088::Error BMI088::accelInit()
{
    if (!spi_accel_.has_value())
        return BMI088_NO_SENSOR;

    accelReadReg(BMI088_ACC_CHIP_ID);
    osDelay(10);
    accelReadReg(BMI088_ACC_CHIP_ID);
    osDelay(10);
    accelWriteReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(100);
    accelReadReg(BMI088_ACC_CHIP_ID);
    osDelay(10);
    uint8_t chip_id = accelReadReg(BMI088_ACC_CHIP_ID);
    //	logger_printf("accel chip id: %x\r\n", chip_id);
    // if (chip_id != BMI088_ACC_CHIP_ID_VALUE && chip_id != 0x16)
    //     return BMI088_NO_SENSOR;

    uint16_t res;
    // set accel sonsor config and check
    for (int write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        accelWriteReg(BMI088_Accel_Init_Table[write_reg_num][0], BMI088_Accel_Init_Table[write_reg_num][1]);
        osDelay(50);

        res = accelReadReg(BMI088_Accel_Init_Table[write_reg_num][0]);
        //		logger_printf("%d,%x,%x\r\n", write_reg_num, BMI088_Accel_Init_Table[write_reg_num][1], res);

        osDelay(10);

        if (res != BMI088_Accel_Init_Table[write_reg_num][1])
        {
            // write_reg_num--;
        }
    }
    return BMI088_NO_ERROR;
}

BMI088::Error BMI088::gyroInit()
{
    if (!spi_gyro_.has_value())
        return BMI088_NO_SENSOR;

    gyroReadReg(BMI088_GYRO_CHIP_ID);
    osDelay(10);
    gyroReadReg(BMI088_GYRO_CHIP_ID);
    osDelay(10);
    accelWriteReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET);
    osDelay(100);
    gyroReadReg(BMI088_GYRO_CHIP_ID);
    osDelay(10);
    uint8_t chip_id = gyroReadReg(BMI088_GYRO_CHIP_ID);
    //	logger_printf("gyro chip id: %x\r\n", chip_id);
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    uint8_t res;
    // set gyro sonsor config and check
    for (int write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        gyroWriteReg(BMI088_Gyro_Init_Table[write_reg_num][0], BMI088_Gyro_Init_Table[write_reg_num][1]);
        osDelay(10);
        res = gyroReadReg(BMI088_Gyro_Init_Table[write_reg_num][0]);
        //		logger_printf("%d,%x,%x\r\n", write_reg_num, BMI088_Gyro_Init_Table[write_reg_num][1], res);
        osDelay(10);

        if (res != BMI088_Gyro_Init_Table[write_reg_num][1])
        {
            write_reg_num--;
        }
    }
    return BMI088_NO_ERROR;
}

void BMI088::readAccel()
{
    uint8_t buf[6];
    accelReadRegs(BMI088_ACCEL_XOUT_L, buf, 6);
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    // logger_printf("accel raw data: %d,%d,%d\r\n",raw_x,raw_y,raw_z);
    accel_data_(0, 0) = raw_x * accel_sensitivity_ * accel_scale_;
    accel_data_(1, 0) = raw_y * accel_sensitivity_ * accel_scale_;
    accel_data_(2, 0) = raw_z * accel_sensitivity_ * accel_scale_;
}

void BMI088::readGyro()
{
    uint8_t buf[8];
    gyroReadRegs(BMI088_GYRO_CHIP_ID, buf, 8);
    int16_t raw_x = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_y = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t raw_z = (int16_t)((buf[7] << 8) | buf[6]);

    // logger_printf("gyro raw data: %d,%d,%d\r\n",raw_x,raw_y,raw_z);
    gyro_data_(0, 0) = (float)raw_x * gyro_sensitivity_ - gyro_offset_[0];
    gyro_data_(1, 0) = (float)raw_y * gyro_sensitivity_ - gyro_offset_[1];
    gyro_data_(2, 0) = (float)raw_z * gyro_sensitivity_ - gyro_offset_[2];
}

void BMI088::readTemperature()
{
    uint8_t buf[2];
    accelReadRegs(BMI088_TEMP_M, buf, 2);
    int16_t raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw_temp > 1023)
        raw_temp -= 2048;
    temperature_ = (float)raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

// ------------------------
// 底层读写封装（代替宏）
// ------------------------
inline void BMI088::accelWriteReg(uint8_t reg, uint8_t data)
{
    spi_accel_->cs_enable();
    spi_accel_->writeReg(reg, data);
    spi_accel_->cs_disable();
}

inline uint8_t BMI088::accelReadReg(uint8_t reg)
{
    spi_accel_->cs_enable();
    spi_accel_->byte(reg | 0x80);
    spi_accel_->byte(reg | 0x80);
    uint8_t val = spi_accel_->byte(0x55);
    spi_accel_->cs_disable();
    return val;
}

inline void BMI088::accelReadRegs(uint8_t reg, uint8_t* data, uint16_t len)
{
    spi_accel_->cs_enable();
    spi_accel_->byte(reg | 0x80);
    spi_accel_->byte(reg | 0x80);
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = spi_accel_->byte(0x55);
    }
    spi_accel_->cs_disable();
}

inline void BMI088::gyroWriteReg(uint8_t reg, uint8_t data)
{
    spi_gyro_->cs_enable();
    spi_gyro_->writeReg(reg, data);
    spi_gyro_->cs_disable();
}

inline uint8_t BMI088::gyroReadReg(uint8_t reg)
{
    spi_gyro_->cs_enable();
    uint8_t val;
    val = spi_gyro_->readReg(reg | 0x80);
    spi_gyro_->cs_disable();
    return val;
}

inline void BMI088::gyroReadRegs(uint8_t reg, uint8_t* data, uint16_t len)
{
    spi_gyro_->cs_enable();
    spi_gyro_->byte(reg | 0x80);
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = spi_gyro_->byte(0x55);
    }
    spi_gyro_->cs_disable();
}
