//
// Created by zhangzhiwen on 25-10-12.
//

#ifndef MATRIX_F32_H
#define MATRIX_F32_H

#include "arm_math.h"
#include <cmath>
#include <cstring>

// =============================================================
//  基于 arm_math 的 Eigen 风格静态矩阵模板类
// =============================================================
template <uint16_t ROWS, uint16_t COLS>
class MatrixF32
{
public:
    float                   data_[ROWS * COLS]{0};
    arm_matrix_instance_f32 mat_{0};

public:
    // ----------- 构造函数 -----------
    MatrixF32()
    {
        arm_mat_init_f32(&mat_, ROWS, COLS, data_);
        setZero();
    }

    explicit MatrixF32(const float* src)
    {
        arm_mat_init_f32(&mat_, ROWS, COLS, data_);
        memcpy(data_, src, sizeof(float) * ROWS * COLS);
    }

    // ----------- 基础访问 -----------
    inline float&       operator()(const uint16_t r, const uint16_t c) { return data_[r * COLS + c]; }
    inline const float& operator()(const uint16_t r, const uint16_t c) const { return data_[r * COLS + c]; }

    inline float*       getData() { return data_; }
    inline const float* getData() const { return data_; }

    inline arm_matrix_instance_f32*       getMat() { return &mat_; }
    inline const arm_matrix_instance_f32* getMat() const { return &mat_; }

    static inline uint16_t rows() { return ROWS; }
    static inline uint16_t cols() { return COLS; }

    void setZero() { memset(data_, 0, sizeof(data_)); }

    void setIdentity()
    {
        setZero();
        for (uint16_t i   = 0; i < ROWS && i < COLS; i++)
            (*this)(i, i) = 1.0f;
    }

    // ----------- 矩阵运算 -----------
    MatrixF32 operator+(const MatrixF32& rhs) const
    {
        MatrixF32 result;
        arm_mat_add_f32(&mat_, &rhs.mat_, &result.mat_);
        return result;
    }

    MatrixF32 operator-(const MatrixF32& rhs) const
    {
        MatrixF32 result;
        arm_mat_sub_f32(&mat_, &rhs.mat_, &result.mat_);
        return result;
    }

    template <uint16_t RHS_COLS>
    MatrixF32<ROWS, RHS_COLS> operator*(const MatrixF32<COLS, RHS_COLS>& rhs) const
    {
        MatrixF32<ROWS, RHS_COLS> result;
        arm_mat_mult_f32(&mat_, &rhs.mat_, &result.mat_);
        return result;
    }

    MatrixF32 operator*(float scalar) const
    {
        MatrixF32 result;
        for (uint16_t i     = 0; i < ROWS * COLS; i++)
            result.data_[i] = data_[i] * scalar;
        return result;
    }

    MatrixF32 operator/(float scalar) const
    {
        MatrixF32 result;
        for (uint16_t i     = 0; i < ROWS * COLS; i++)
            result.data_[i] = data_[i] / scalar;
        return result;
    }

    MatrixF32& operator+=(const MatrixF32& rhs)
    {
        *this = *this + rhs;
        return *this;
    }

    MatrixF32& operator-=(const MatrixF32& rhs)
    {
        *this = *this - rhs;
        return *this;
    }

    MatrixF32& operator*=(float scalar)
    {
        for (uint16_t i = 0; i < ROWS * COLS; i++)
            data_[i] *= scalar;
        return *this;
    }

    MatrixF32& operator/=(float scalar)
    {
        for (uint16_t i = 0; i < ROWS * COLS; i++)
            data_[i] /= scalar;
        return *this;
    }

    MatrixF32<COLS, ROWS> transpose() const
    {
        MatrixF32<COLS, ROWS> result;
        arm_mat_trans_f32(&mat_, &result.mat_);
        return result;
    }

    MatrixF32 inverse() const
    {
        static_assert(ROWS == COLS, "Matrix must be square to invert");
        MatrixF32 result;
        arm_mat_inverse_f32(&mat_, &result.mat_);
        return result;
    }

    // 判断是否全零
    bool zero() const
    {
        for (uint16_t i = 0; i < ROWS * COLS; i++)
        {
            if (data_[i] <= 1e-6f && data_[i] >= -1e-6f)
                return false;
        }
        return true;
    }

    // ----------- 数学函数 -----------
    float trace() const
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < ROWS && i < COLS; i++)
            sum += (*this)(i, i);
        return sum;
    }

    float norm() const
    {
        float sum = 0.0f;
        for (uint16_t i = 0; i < ROWS * COLS; i++)
            sum += data_[i] * data_[i];
        return sqrtf(sum);
    }

    void normalize()
    {
        float n = norm();
        if (n > 1e-6f)
        {
            for (uint16_t i = 0; i < ROWS * COLS; i++)
                data_[i] /= n;
        }
    }

    float dot(const MatrixF32& rhs) const
    {
        static_assert(ROWS == 1 || COLS == 1, "Dot only for vectors");
        float res = 0.0f;
        for (uint16_t i = 0; i < ROWS * COLS; i++)
            res += data_[i] * rhs.data_[i];
        return res;
    }

    MatrixF32<3, 1> cross(const MatrixF32<3, 1>& rhs) const
    {
        static_assert(ROWS == 3 && COLS == 1, "Cross only for 3x1 vectors");
        MatrixF32<3, 1> res;
        res(0, 0) = (*this)(1, 0) * rhs(2, 0) - (*this)(2, 0) * rhs(1, 0);
        res(1, 0) = (*this)(2, 0) * rhs(0, 0) - (*this)(0, 0) * rhs(2, 0);
        res(2, 0) = (*this)(0, 0) * rhs(1, 0) - (*this)(1, 0) * rhs(0, 0);
        return res;
    }
};

#endif //MATRIX_F32_H
