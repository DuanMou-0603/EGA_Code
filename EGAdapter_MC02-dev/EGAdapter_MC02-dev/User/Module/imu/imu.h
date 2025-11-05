//
// Created by zhangzhiwen on 25-10-12.
//

#ifndef IMU_H
#define IMU_H

#include "matrix_f32.h"
#include "quaternion_f32.h"
#include <cstdint>
#include <optional>
#include "driver_spi.h"
#include "madgwick_filter.h"
#include "quaternionEKF.h"

// =============================================================
// 通用 IMU 基类 —— 可派生出不同具体型号 (如 BMI088 / MPU6050)
// =============================================================
class IMU
{
public:
    // IMU 设备类型枚举
    enum class Type
    {
        UNKNOWN = 0,
        BMI088,
        DM_IMU,

        MPU6050,
        ICM42688,

        CUSTOM
    };

    enum class CalibrationStatus
    {
        ONLINE,  // 在线校准
        OFFLINE, // 使用固定值
    };

    enum class FilterType
    {
        EKF,
        MADGWICK,
    };

    struct BMI088Config
    {
        SPIInstance::Config spi_accel;
        SPIInstance::Config spi_gyro;
    };

    struct Config
    {
        Type       type        = Type::BMI088; //默认为板载的BMI088
        FilterType filter_type = FilterType::EKF;
        float      sample_time = 0.001;

        //各种陀螺仪的配置
        BMI088Config bmi088_config{};
        //待添加
    };

    //静态
public:
    static std::unique_ptr<IMU> create(const Config& config); //工厂函数，供上层调用
    //static void update();//静态循环，调用所有注册过的imu并更新数据，方便放在task里面循环跑（是否有实现的必要？）

public:
    virtual ~IMU() = default;

    // ============ 接口函数（可被派生类实现） ============
    // virtual Status Init() = 0;   // 初始化传感器,由于所需参数不同，在子类定义
    virtual void update() = 0; // 更新原始数据（加速度、角速度）
    virtual void calibrate()
    {
    }; // 可选：零偏/校准

    // ============ 公共功能接口 ============
    // 由 Update() 后调用，用于融合或解算姿态
    virtual void compute() = 0;

    // 获取接口
    [[nodiscard]] inline const QuaternionF32&   getQuaternion() const { return quaternion_; }
    [[nodiscard]] inline const MatrixF32<3, 1>& getEulerAngles() const { return eulerAngles_; }
    [[nodiscard]] inline const MatrixF32<3, 1>& getGyro() const { return gyro_data_; }
    [[nodiscard]] inline const MatrixF32<3, 1>& getAccel() const { return accel_data_; }

    [[nodiscard]] inline Type GetType() const { return type_; }

protected:
    void initParams(const Config& config)
    {
        type_        = config.type;
        filter_type_ = config.filter_type;
        sample_time_ = config.sample_time;
    }

protected:
    Type type_ = Type::BMI088;

    FilterType                    filter_type_ = FilterType::EKF;
    std::optional<QuaternionEKF>  ekf_filter_;
    std::optional<MadgwickFilter> madgwick_filter_;

    float           sample_time_ = 0.01; // 采样时间
    QuaternionF32   quaternion_;         // 当前姿态（四元数）
    MatrixF32<3, 1> eulerAngles_;        // 当前姿态（欧拉角：roll pitch yaw）
    MatrixF32<3, 1> gyro_data_;          // 角速度 (rad/s)
    MatrixF32<3, 1> accel_data_;         // 加速度 (m/s²)
};

#endif // IMU_H
