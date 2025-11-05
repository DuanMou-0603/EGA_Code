/**
 ******************************************************************************
 * @file    kalman filter.h
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// cortex-m4 DSP lib
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // define in arm_math.h
*/

#include "main.h"
#include "arm_math.h"
//#include "dsp/matrix_functions.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

struct QEKF_INS_t;

struct KalmanFilter_t
{
    float* FilteredValue;
    float* MeasuredVector;
    float* ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t* MeasurementMap;        // 量测与状态的关系 how measurement relates to the state
    float*   MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float*   MatR_DiagonalElements; // 量测方差 variance for each measurement
    float*   StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t* temp;

    // uint8_t MeasurementMap[3];
    // float   MeasurementDegree[3];
    // float   MatR_DiagonalElements[3];
    // float   StateMinVariance[6];
    // uint8_t temp[3];

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(QEKF_INS_t& QEKF_INS, struct KalmanFilter_t* kf);
    void (*User_Func1_f)(QEKF_INS_t& QEKF_INS, struct KalmanFilter_t* kf);
    void (*User_Func2_f)(QEKF_INS_t& QEKF_INS, struct KalmanFilter_t* kf);
    void (*User_Func3_f)(QEKF_INS_t& QEKF_INS, struct KalmanFilter_t* kf);
    void (*User_Func4_f)(struct KalmanFilter_t* kf);
    void (*User_Func5_f)(struct KalmanFilter_t* kf);
    void (*User_Func6_f)(struct KalmanFilter_t* kf);

    // 矩阵存储空间指针
    float *xhat_data, *xhatminus_data;
    float* u_data;
    float* z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float* B_data;
    float *H_data, *HT_data;
    float* Q_data;
    float* R_data;
    float* K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;

    // 矩阵存储空间数组，不使用动态分配
    // float xhat_data[6], xhatminus_data[6];
    // float u_data[3];
    // float z_data[3];
    // float P_data[36], Pminus_data[36];
    // float F_data[36], FT_data[36];
    // float B_data[18];
    // float H_data[18], HT_data[18];
    // float Q_data[36];
    // float R_data[9];
    // float K_data[18];
    // float S_data[36];
    // float temp_matrix_data[36], temp_matrix_data1[36];
    // float temp_vector_data[6], temp_vector_data1[6];
};

struct QEKF_INS_t
{
    uint8_t        Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t        ConvergeFlag;
    uint8_t        StableFlag;
    uint64_t       ErrorCount;
    uint64_t       UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat   ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
};


extern uint16_t sizeof_float, sizeof_double;

void   Kalman_Filter_Init(KalmanFilter_t* kf, uint8_t xhatSize = 6, uint8_t uSize = 0, uint8_t zSize = 3);
void   Kalman_Filter_Measure(KalmanFilter_t* kf);
void   Kalman_Filter_xhatMinusUpdate(KalmanFilter_t* kf);
void   Kalman_Filter_PminusUpdate(KalmanFilter_t* kf);
void   Kalman_Filter_SetK(KalmanFilter_t* kf);
void   Kalman_Filter_xhatUpdate(KalmanFilter_t* kf);
void   Kalman_Filter_P_Update(KalmanFilter_t* kf);
float* Kalman_Filter_Update(QEKF_INS_t& QEKF_INS,KalmanFilter_t* kf);


#endif //__KALMAN_FILTER_H
