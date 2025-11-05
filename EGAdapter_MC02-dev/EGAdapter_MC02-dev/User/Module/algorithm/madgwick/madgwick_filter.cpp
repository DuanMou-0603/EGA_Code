//
// Created by zhangzhiwen on 25-10-14.
//

#include "madgwick_filter.h"

#include "logger.h"

MadgwickFilter::MadgwickFilter(float beta, float samplePeriod)
    : beta_(beta), samplePeriod_(samplePeriod)
{
}

void MadgwickFilter::updateIMU(MatrixF32<3, 1> accel, MatrixF32<3, 1> gyro, QuaternionF32& q)
{
    // 步骤1：归一化加速度
    accel.normalize();
    if (!accel.zero())
    {
//        logger_printf("accel zero\r\n");
        return;
    }
    float32_t q0_ = q.w, q1_ = q.x, q2_ = q.y, q3_ = q.z;

    float32_t ax = accel(0, 0), ay = accel(1, 0), az = accel(2, 0);
    float32_t gx = gyro(0, 0),  gy = gyro(1, 0),  gz = gyro(2, 0);

    // 步骤2：梯度下降算法修正项
    float f1 = 2.0f * (q1_ * q3_ - q0_ * q2_) - ax;
    float f2 = 2.0f * (q0_ * q1_ + q2_ * q3_) - ay;
    float f3 = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_ - az;

    float J_11or24 = 2.0f * q2_;
    float J_12or23 = 2.0f * q3_;
    float J_13or22 = 2.0f * q0_;
    float J_14or21 = 2.0f * q1_;
    float J_32 = 2.0f * J_14or21;
    float J_33 = 2.0f * J_11or24;

    float s0 = -J_14or21 * f2 + J_11or24 * f1;
    float s1 = J_13or22 * f2 + J_12or23 * f1 - J_32 * f3;
    float s2 = J_12or23 * f2 - J_13or22 * f1 - J_33 * f3;
    float s3 = J_14or21 * f1;

    // QuaternionF32 s(s0, s1, s2, s3);
    // s.Normalize();
    float normS;
    arm_sqrt_f32(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3,&normS);
    s0 /= normS;
    s1 /= normS;
    s2 /= normS;
    s3 /= normS;

    // 步骤3：四元数微分方程
    // QuaternionF32 dq = q.ToDq(gyro) - s * beta_;
    float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz) - beta_ * s0;
    float qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy) - beta_ * s1;
    float qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx) - beta_ * s2;
    float qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx) - beta_ * s3;

    // 步骤4：积分更新
    // q += dq * samplePeriod_;
    q0_ += qDot1 * samplePeriod_;
    q1_ += qDot2 * samplePeriod_;
    q2_ += qDot3 * samplePeriod_;
    q3_ += qDot4 * samplePeriod_;

    q.w = q0_;
    q.x = q1_;
    q.y = q2_;
    q.z = q3_;
    // 步骤5：归一化四元数
    q.Normalize();
    // float normQ = std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    // q0_ /= normQ;
    // q1_ /= normQ;
    // q2_ /= normQ;
    // q3_ /= normQ;
}
