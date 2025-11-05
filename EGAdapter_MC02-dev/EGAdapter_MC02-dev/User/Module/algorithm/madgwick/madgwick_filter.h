//
// Created by zhangzhiwen on 25-10-14.
//

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "matrix_f32.h"
#include "quaternion_f32.h"

class MadgwickFilter
{
public:
    MadgwickFilter(float beta = 0.1f, float samplePeriod = 0.001f);

    // 主更新函数（只使用陀螺仪 + 加速度计）
    void updateIMU(MatrixF32<3, 1> accel, MatrixF32<3, 1> gyro, QuaternionF32& q);

private:
    float beta_;         // 滤波增益系数
    float samplePeriod_; // 采样周期
};

#endif //MADGWICK_FILTER_H
