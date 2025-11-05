//
// Created by zhangzhiwen on 25-10-13.
//

#ifndef BMI088_H
#define BMI088_H

#include <optional>
#include "imu.h"
#include "driver_spi.h"
#include "madgwick_filter.h"
#include "bmi088reg.h"

class BMI088 : public IMU
{
public:
    enum Error
    {
        BMI088_NO_ERROR                     = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
        BMI088_ACC_PWR_CONF_ERROR           = 0x02,
        BMI088_ACC_CONF_ERROR               = 0x03,
        BMI088_ACC_SELF_TEST_ERROR          = 0x04,
        BMI088_ACC_RANGE_ERROR              = 0x05,
        BMI088_INT1_IO_CTRL_ERROR           = 0x06,
        BMI088_INT_MAP_DATA_ERROR           = 0x07,
        BMI088_GYRO_RANGE_ERROR             = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
        BMI088_GYRO_LPM1_ERROR              = 0x0A,
        BMI088_GYRO_CTRL_ERROR              = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,
        BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
        BMI088_NO_SENSOR                    = 0xFF,
    };

public:
    //    BMI088();
    //    explicit BMI088(SPIInstance::Config& cfg_accel, SPIInstance::Config& cfg_gyro);
    explicit BMI088(const Config& config);
    ~BMI088() override = default;

    // IMU 基类接口实现
    Error init();
    void  update() override;
    void  calibrate() override;
    void  compute() override;

    // 辅助函数
    inline Error getError() const { return error_code_; }
    float        getTemperature() const { return temperature_; };

private:
    // 内部功能函数
    Error accelInit();
    Error gyroInit();
    void  readAccel();
    void  readGyro();
    void  readTemperature();

    // === 底层读写函数===
    inline void    accelWriteReg(uint8_t reg, uint8_t data);
    inline uint8_t accelReadReg(uint8_t reg);
    inline void    accelReadRegs(uint8_t reg, uint8_t* data, uint16_t len);

    inline void    gyroWriteReg(uint8_t reg, uint8_t data);
    inline uint8_t gyroReadReg(uint8_t reg);
    inline void    gyroReadRegs(uint8_t reg, uint8_t* data, uint16_t len);

private:
    std::optional<SPIInstance> spi_accel_;
    std::optional<SPIInstance> spi_gyro_;
    // std::optional<MadgwickFilter> filter_;
    Error error_code_;

    float accel_sensitivity_;
    float gyro_sensitivity_;
    float accel_scale_;
    float gyro_offset_[3];
    float temperature_;


    const uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {
            BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
            BMI088_INT1_IO_CTRL_ERROR
        },
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
    };

    const uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {
            BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
            BMI088_GYRO_INT3_INT4_IO_CONF_ERROR
        },
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
    };
};


#endif //BMI088_H
