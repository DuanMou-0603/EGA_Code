/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"
#include "quaternion_f32.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

// 为了避免冲突，定义放在"kalman_filter.h"了
// struct KalmanFilter_t;
// struct QEKF_INS_t
// {
//     uint8_t        Initialized;
//     KalmanFilter_t IMU_QuaternionEKF;
//     uint8_t        ConvergeFlag;
//     uint8_t        StableFlag;
//     uint64_t       ErrorCount;
//     uint64_t       UpdateCount;
//
//     float q[4];        // 四元数估计值
//     float GyroBias[3]; // 陀螺仪零偏估计值
//
//     float Gyro[3];
//     float Accel[3];
//
//     float OrientationCosine[3];
//
//     float accLPFcoef;
//     float gyro_norm;
//     float accl_norm;
//     float AdaptiveGainScale;
//
//     float Roll;
//     float Pitch;
//     float Yaw;
//
//     float YawTotalAngle;
//
//     float Q1; // 四元数更新过程噪声
//     float Q2; // 陀螺仪零偏过程噪声
//     float R;  // 加速度计量测噪声
//
//     float dt; // 姿态更新周期
//     mat   ChiSquare;
//     float ChiSquare_Data[1];      // 卡方检验检测函数
//     float ChiSquareTestThreshold; // 卡方检验阈值
//     float lambda;                 // 渐消因子
//
//     int16_t YawRoundCount;
//
//     float YawAngleLast;
// };

void IMU_QuaternionEKF_Init(QEKF_INS_t& QEKF_INS, float* init_quaternion, float process_noise1, float process_noise2,
                            float       measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(QEKF_INS_t& QEKF_INS, float gx, float gy, float gz, float ax, float ay, float az,
                              float       dt);

class QuaternionEKF
{
public:
    QuaternionEKF(QuaternionF32 init_q,
                  float         sample_time,
                  float         process_noise1 = 10,
                  float         process_noise2 = 0.001,
                  float         measure_noise  = 1000000,
                  float         lambda         = 1,
                  float         lpf            = 0
    ):
        process_noise1_(process_noise1), process_noise2_(process_noise2),
        measure_noise_(measure_noise), lambda_(lambda), lpf_(lpf), sample_time_(sample_time)
    {
        float init_quaternion[4]{init_q.w, init_q.x, init_q.y, init_q.z};
        IMU_QuaternionEKF_Init(QEKF_INS, init_quaternion,
                               process_noise1_, process_noise2_,
                               measure_noise_, lambda_, lpf_);
    }

    ~QuaternionEKF() = default;

    void updateIMU(MatrixF32<3, 1> accel, MatrixF32<3, 1> gyro, QuaternionF32& q)
    {
        IMU_QuaternionEKF_Update(QEKF_INS,
                                 gyro(0, 0), gyro(1, 0), gyro(2, 0),
                                 accel(0, 0), accel(1, 0), accel(2, 0), sample_time_);
        q.w = QEKF_INS.q[0];
        q.x = QEKF_INS.q[1];
        q.y = QEKF_INS.q[2];
        q.z = QEKF_INS.q[3];
    }

private:
    QEKF_INS_t QEKF_INS{0};
    float      sample_time_;
    float      process_noise1_;
    float      process_noise2_;
    float      measure_noise_;
    float      lambda_;
    float      lpf_;
};


#endif
