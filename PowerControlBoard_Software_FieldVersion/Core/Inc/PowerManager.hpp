#pragma once

#include "Calibration.hpp"
#include "Interface.hpp"
#include "Communication.hpp"

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "hrtim.h"
#include "stdint.h"
#include "Config.hpp"
#include "Utility.hpp"

#include "dac.h"
#include "opamp.h"
#include "comp.h"



//#define ADC_BUFFER_SIZE     8U
//#define ADC_RANK_NUM        4U
#define ADC4_BUFFER_SIZE    2U

#define ADC_ISENSE_FILTER_ALPHA 0.9f

#define HRTIM_INT_SCALER    4U


struct RxData;
struct TxData;

struct SystemData
{
    uint32_t vTick = 0;//主时钟计数器，提供全局时间基准和各种定时功能
    bool systemInited = false;//系统初始化完成标志
    //- main函数启动 ：在 main() 函数中完成STM32外设初始化后调用 systemStart()
    //- 初始化阶段 ： systemStart() 调用 init() 函数执行系统初始化
    //- 初始化完成 ：在 init() 函数最后执行 sysData.systemInited = true ，标志系统初始化完成
    //- 进入主循环 ：随后进入 loop() 无限循环
    uint32_t hardwareUID[3] = {0};
    uint16_t buttonCnt = 0;//按键按下时间计数器 | 短按功能（1-2秒） ：错误恢复,恢复manual和auto | 长按功能（>3秒） ：强制系统重启
    bool buttonPressed = 0;
    bool buttonPressedLast = 0;

    uint8_t lfLoopIndex = 0;//用于实现低频任务的时分复用调度
    //- Case 0 : 系统基础任务（虚拟时钟递增、蜂鸣器更新、错误处理）
    //- Case 1 : 通信和接口任务（CAN数据发送、超时检测、按键状态更新）
    //- Case 2 : 电源管理任务（电容容量估算、ADC低频更新）
    //- Case 3 : 保护和特殊功能（低电压检测、蜂鸣器序列更新、WPT无线充电相关处理）
    
};

enum DCDCMode{BUCK, BUCKBOOST, BOOSTBUCK, BOOST, CALIBRATION_A, CALIBRATION_B};
enum PCMMode{IB_VALLEY, IA_PEAK};

struct PowerStageData
{
    bool timerEnabled = 0;//控制HRTIM的运行状态 为true时，才允许AB和E输出
    bool outputABEnabled = 0;//指示HRTIM定时器的输出AB通道是否已启用
    //- 电容容量估算 ：仅当输出AB启用时才执行电容容量估算算法
    //- 无线充电控制 ：作为WPT（无线功率传输）功能启用和禁用的前提条件
    //- 功率控制循环 ：仅在输出启用时执行中频功率控制循环和电感电流设置
    //- 短路保护检测 ：仅在输出启用时进行短路保护检测
    bool outputEEnabled = 0;//用于指示HRTIM定时器的输出E通道是否已启用
    //- 功率监测 ：仅当输出E启用时才监测WPT功率，当WPT功率低于3W时递增低功率计数器
    //- 过压保护 ：当输出E启用且电容电压超过最大值的101%时，触发WPT错误状态
    //- 电压调节 ：仅在输出E启用时执行WPT电压闭环控制，调节占空比以维持29.5V的WPT电压
    //- 底盘功率计算 ：在CAN通信中，当输出E启用时，底盘功率 = 总功率 - WPT功率；未启用时，底盘功率 = 总功率
    //- 功率分配监控 ：为上位机提供准确的功率分配信息，区分底盘用电和无线充电用电
    //bool chargePumpEnabled = true;
    bool allowEnableOutput = 1;//只有当 timerEnabled 和 allowEnableOutput 都为true时才允许启用输出AB
    //电源恢复控制 ：在 powerOnOffControl 函数中，电压恢复时需要同时满足电压条件、错误等级条件和 allowEnableOutput 条件才能重新启用输出


    float dutyByVoltage = 0.0f;//输出电压（vB，电容侧）与输入电压（vA，源侧）的比值
    //- BUCK模式 (降压)：当 dutyByVoltage > 0.84 时切换到 BUCKBOOST
    //- BUCKBOOST模式 (混合)：
    //- < 0.80 时回到 BUCK
    //- > 1.02 时进入 BOOSTBUCK
    //- BOOSTBUCK模式 (混合)：
    //- < 0.82 时回到 BUCK
    //- < 0.98 时回到 BUCKBOOST
    //- > 1.25 时进入 BOOST
    //- BOOST模式 (升压)：
    //- < 0.82 时回到 BUCK
    //- < 1.19 时回到 BOOSTBUCK
    float efficiency = 1.0f;//计算双向功率传输效率，预留的保护

    float dutyE = 0.0f; //E侧占空比，专门用于控制无线充电功率级的开关管占空比
    float dutyEMin = 0.0f; //E侧最小占空比
    

    uint8_t softStartCnt = SOFT_START_TIME;//初始化延时 ：系统启动或输出重新启用时，计数器被设置为8，提供软启动缓冲时间
    //当检测到输入电压低于 REFEREE_UVLO_LIMIT 时，禁用输出并重置计数器为8
    //裁判系统功率控制 ：在 updateMFLoop 函数中，只有当 softStartCnt 为0时才允许执行正常的裁判系统功率控制
    //保护模式切换 ：计数器非零时系统运行在保护模式下，使用固定的6W功率目标
    //错误记录 ：在短路保护检测中，将 softStartCnt 的值记录到 errorData.errorVoltage 中，用于故障诊断
    
    float iLLimit = MAX_INDUCTOR_CURRENT;//电感电流的上下限保护阈值
    

    DCDCMode dcdcMode = BUCK;//- BUCK ：降压模式| BUCKBOOST ：降压-升压过渡模式 | BOOSTBUCK ：升压-降压过渡模式| BOOST ：升压模式| CALIBRATION_A ：A侧校准模式| CALIBRATION_B ：B侧校准模式
    //- 在 `modeStateMachine` 函数中，根据dutyByVoltage（输出电压与输入电压的比值）自动切换工作模式 | 具体切换见上
    //- 每种模式对应不同的HRTIM比较器设置，控制A侧和B侧的占空比限制
    //- BUCK模式 ：A侧可变控制（0.5%-94%），B侧常开
    //- BUCKBOOST模式 ：A侧可变控制（5%-94%），B侧固定84%占空比
    //- BOOSTBUCK模式 ：A侧固定84%占空比，B侧可变控制（25%-94%）
    //- BOOST模式 ：A侧常开，B侧可变控制（25%-94%）
 
    PCMMode pcmMode = IB_VALLEY;//- IB_VALLEY ：B侧电流谷值控制模式，控制B侧开关管的电流谷值，适用于降压和降压-升压过渡模式 | IA_PEAK ：A侧电流峰值控制模式，控制A侧开关管的电流峰值，适用于升压和升压-降压过渡模式
    //在 `modeStateMachine` 函数中，根据不同的DCDC工作模式自动设置相应的PCM模式
    //- 在BUCK/BUCKBOOST模式下，A侧为主控制开关，B侧为同步整流，使用IB_VALLEY控制
    //- 在BOOSTBUCK/BOOST模式下，B侧为主控制开关，A侧为同步整流，使用IA_PEAK控制
    float iLTarget = 0.0f;//iLTarget是系统的电感电流目标值，经过各种控制算法（裁判系统功率控制、电容电压限制、电流限制等）计算后的结果
    //- 裁判系统功率控制 ：通过 `updateMFLoop` 函数中的功率PID控制器计算目标电流
    //- 电容电压限制 ：当电容电压接近最大值时，通过 dIL_VCap_Max 限制电流增量
    //- 电流限制保护 ：通过 dIL_IB_Positive 和 dIL_IB_Negative 限制正负向电流
    //- 电容充电控制 ：根据电容电压状态决定是否允许充电
    //在136kHz的 `HRTIM1_Master_IRQHandler` 中断中实时更新
    float IRQload = 0.0f;//监测中断处理函数的负载率 emm看不懂。。。

};

struct ADCData
{
    bool adcInitialized = 0;//预留的状态标志变量，用于跟踪ADC初始化状态
    uint32_t rawData12[4 * HRTIM_INT_SCALER];//双ADC同步采样 ：通过 HAL_ADCEx_MultiModeStart_DMA(&hadc1, adcData.rawData12, 4 * HRTIM_INT_SCALER) 实现ADC1和ADC2的同步采样
    //- 每4个元素为一组 ：对应一次完整的ADC采样周期
    //- ADC1通道 ：iA（A相电流）、iR（参考电流）、vA（A相电压）、vWPT（无线充电电压）具体可看updateADCmf函数
    //- ADC2通道 ：iB（B相电流）、iB（重复）、vB（B相电压）、iWPT（无线充电电流）
    //- 采样倍频 ：通过 HRTIM_INT_SCALER=4 实现4倍过采样
    uint32_t sumData[4];//sumData[0] = adcData.rawData12[0] + adcData.rawData12[4] + adcData.rawData12[8] + adcData.rawData12[12];
    //具体可看updateADCmf函数

#ifdef CALIBRATION_MODE
    float tempData[7];
#endif    
    uint32_t rawData4[ADC4_BUFFER_SIZE];                //ADC4 原始数据，用于NTC和辅助电源监测
    float iA = 0.0f, iB = 0.0f, iR = 0.0f, iCap;   //电流
    float vA = 0.0f, vB = 0.0f, vCap = 0.0f;              //电压
    float iChassis = 0.0f;              //电容电流，电压
    float pReferee, pChassis;            //裁判系统功率，电容功率，底盘功率，无线充电功率
    float iCaplf = 0.0f, vCaplf = 0.0f; //电容电压，电压
    float pRefereelf = 0.0f, pChassislf = 0.0f;

    float vWPT, iWPT, pWPT, pWPTlf, vWPTlf;//vWPT - 原始实时电压值 |vWPTlf - 低通滤波后的电压值

    float vAux;//监测辅助电源电压，正常工作范围1.3V-1.5V，电压测量范围：0-2.9V
};



enum LimitFactor
{   
    REFEREE_POWER,
    CAPARR_VOLTAGE_MAX,
    CAPARR_VOLTAGE_NORMAL,
    IB_POSITIVE,//电容最大充电电流
    IB_NEGATIVE,//电容最大放电电流
};

enum ErrorLevel
{
    NO_ERROR = 0,               // 无错误
    ERROR_RECOVER_AUTO = 1,     // 错误，可通过自动恢复
    ERROR_RECOVER_MANUAL = 2,   // 错误，可通过发信息恢复
    ERROR_UNRECOVERABLE = 3,    // 错误，不可恢复
    WARNING                     // 警告
};

enum WPTStatus
{
    WPT_ERROR = 0,      // 非无线充电硬件，或发生错误
    WPT_OFF = 1,        // 无线充电关闭
    WPT_CHARGING = 2,   // 无线充电中
    WPT_FINISHED = 3    // 无线充电完成(电压>98%, 能量大于96%)
};

struct ControlData
{   
    struct RefereeData
    {
        float kP = 1.0f, kI=0.04f, kD=1.5f; //积分增益
        uint16_t lastError = 0.0f; //上次误差
        float integral = 0.0f; //积分值
        int16_t error = 0U;
        uint32_t lastTimestamp = 0;//记录裁判系统数据的最后接收时间戳 | 当连接状态为真且超时时触发断连处理 
        float pRefereeBias = 0.0f;//pRefereeBias = kP*error + kI*integral + kD*(error - lastError)
        //pRefereeTarget = CLAMP(pRefereeBias + rd.refereePowerLimit, 5.0f, 135.0f)
        bool isConnected = 0;//标识与裁判系统的连接状态
        bool useNewFeedbackMessage = 0;//控制CAN通信中反馈消息的格式选择 0为旧格式
    };
    
    LimitFactor limitFactor;
    
    RefereeData refLoop;
    
    float pRefereeTarget = REFEREE_DEFUALT_POWER;

    float vCapArrNormal = CAPARR_MAX_VOLTAGE;

    bool allowCharge = false; //是否允许充电

    WPTStatus wptStatus = WPT_OFF; //无线充电状态
    
};

struct LoopControlData
{
    IncreasementPID iRPID {0.1f, 0.2f, 0.10f, 0.01f};
    //IncreasementPID vCapPID {0.0f, 0.0f, 0.02f, 0.0f};

    float currentLimitKI = 0.8f;
    float voltageLimitKI = 0.01f;
    float burstKI = 2.0f;

    float vWPTTarget = 26.2f; //无线充电目标电压
    float wptVoltageKI = 0.001f;

    float deltaIL;//电感电流目标值的增量调整量
    float dIL_VCap_Max;//电容电压限制控制的电流增量调整量
    //float dIL_VCap_MaxBurst;
    float dIL_IB_Positive;
    float dIL_IB_Negative;

    float dIL_recoverBurst;//突发功率恢复控制的电流增量调整量


    float deltaDutyE2 = 0.0f;//无线充电占空比控制的辅助调整量，暂未使用


};


struct CAPARRStatus
{
    struct CapacityEstimateData
    {
        float dQ = 0.0f;//累计电荷变化量
        float lastVCap = 0.0f;//上次记录的电容电压值
        uint32_t lastTick = 0;//上次估算的时间戳
        float dQtodV = CAPARR_DEFUALT_CAPACITY;//电荷变化对电压变化的比值，即电容容量估算值
        float dVtodQ = (1.0f / CAPARR_DEFUALT_CAPACITY);
        
        float maxIB = 0.0f;//估算周期内B侧电流的最大值
        float minIB = 0.0f;
    };
    
    float maxOutCurrent = 2.0f;
    float maxInCurrent = 2.0f;
    CapacityEstimateData capEstData;

    uint16_t warningCnt = 0; //警告计数
};

struct ErrorData
{
    uint16_t errorCode = 0;//详见Config.hpp
    uint16_t shortCircuitCnt = 0;
    uint16_t overVoltageCnt = 0;
    uint16_t overCurrentCnt = 0;
    uint8_t lowBattery = 0;
    uint16_t lowBatteryCnt = 0; //低电压计数
    ErrorLevel errorLevel     = NO_ERROR;       // 错误等级
    uint32_t powerOffCnt = 0; //关机计数

    float errorVoltage = 0.0f;//错误发生时的电压值
    float errorCurrent = 0.0f;//错误发生时的电流值
};



extern SystemData sysData;
extern ErrorData errorData;
extern ControlData ctrlData;
extern LoopControlData mfLoop;
extern ADCData adcData; 
extern CAPARRStatus capStatus;             //电容状态
extern PowerStageData psData;


namespace HRTIM
{

void startTimer();

void stopTimer();

bool enableOutputAB();

bool enableOutputE(float dutyE);

void disableOutputAB();

void disableOutputE();


void setOutputAB();//暂未被使用

} // namespace HRTIM

namespace ADC
{

void initAnalog();//初始化运放(OPAMP1-4)、DAC、比较器等模拟前端电路

void initADC();//校准并启动ADC1/ADC2(同步采样)和ADC4(温度监测)，配置DMA传输

void processData();//仅声明未实现

void processADC4Data();//仅声明未实现

void updateADClf();//处理辅助电源电压监测，在系统低频循环中调用

void updateADCmf();//在HRTIM中断(136kHz/8≈17kHz)中实时处理ADC数据
//计算电流(iA、iB、iR、iWPT)、电压(vA、vB、vWPT)、功率等关键参数，并进行低通滤波

}

namespace PowerControl
{

void updateMFLoop();//高频功率控制环路（在HRTIM中断中调用，136kHz/8采样）
    //- 执行裁判系统功率PID控制
    //- 计算电感目标电流
    //- 实现多重限制保护（电压限制、电流限制等）
    //- 处理超级电容充放电控制

void setInductorCurrent();//- 设置电感电流（在HRTIM中断中调用）
    //- 通过DAC生成锯齿波控制电感电流
    //- 实现双向电流控制（正负电流）

void powerOnOffControl();//- 电源开关控制（在主循环中调用）
    //- 监控输入电压，实现欠压保护和恢复
    //- 软启动控制
    //- 输出使能/禁用管理

void updateRefereePower(const RxData &rd, const uint32_t& currentTick);
    //- 裁判系统功率更新（接收数据时调用）
    //- 处理裁判系统能量缓冲区PID控制
    //- 计算功率偏置和目标功率
    //- 维护裁判系统连接状态
void checkRxDataTimeout(const uint32_t& currentTick);
    //数据超时检查（在主循环中调用）
    //- 检测裁判系统通信超时
    //- 超时时恢复默认参数
    //- 重置控制状态
} // namespace PowerControl


namespace Protection
{

void errorCheckHF();//有声明未实现
void errorCheckLF();//有声明未实现

void checkShortCircuit();
    //- 短路保护检测（在HRTIM中断中调用）
    //- 检测A侧（裁判系统/底盘端）和B侧（电容/无线充电端）短路
    //- 通过电压和电流阈值判断短路状态
    //- 累计计数器机制防止误触发
    //- 触发时立即禁用输出并设置错误代码
void errorHandlerLF();
    //- 低频错误处理（在主循环中调用）
    //- 管理警告状态的自动恢复
    //- 监测长时间掉电情况并自动清除错误
    //- 逐渐减少错误计数器
void hrtimFaultHandler();
    //- HRTIM硬件故障处理（在故障中断中调用）
    //- 处理5种硬件故障：vA过压、iA过流、iR过流、vB过压、iB过流
    //- 所有硬件故障均设为自动恢复级别
    //- 故障触发时立即禁用输出

void configAWDG();
    //- 配置模拟看门狗（系统初始化时调用）
    //- 设置ADC阈值监测
    //- 配置过压保护阈值
    //- 使能HRTIM故障中断

void checkHardwareUID();
    //- 硬件UID验证（系统初始化时调用）
    //- 验证硬件唯一标识符
    //- 防止固件在错误硬件上运行
    //- 不匹配时进入安全模式（蜂鸣器报警）

void checkEfficiency();
    //- 效率监测（在HRTIM中断中调用）
    //- 实时计算双向功率传输效率
    //- 根据电流方向选择不同的效率计算公式
void autoClearError();
    //- 自动错误清除
    //- 清除可自动恢复的错误（如过压、过流）
    //- 重新使能输出
void manualClearError();
    //- 手动错误清除
    //- 清除需要手动干预的错误（如短路）
    //- 重置相关计数器

void checkLowBattery();
    //- 低电压保护（在主循环中调用）
    //- 检测电池电压过低情况
    //- 提供低电压恢复机制
    //- 防止系统在低电压下工作

} // namespace Protection


namespace CAPARR
{

void updateMaxCurrent();
    //- 根据电容电压动态调整最大输入/输出电流
    //- 高电压时允许最大电流，低电压时线性降低电流限制
    //- 保护电容在低电压状态下不被过度放电

void estimateCapacity(const uint32_t& _currentTick);
    //- 通过电压变化和电荷累积实时估算电容容量
    //- 监测电容健康状态，检测容量衰减
    //- 当容量异常时触发警告计数和蜂鸣器报警

void updateCurrentforEstimation();
    //- 记录电流的最大值和最小值
    //- 为容量估算提供电流变化范围数据

uint16_t getMaxPowerFeedback();
    //- 根据电容电压计算可提供的最大功率
    //- 为裁判系统提供功率限制反馈
    //- 确保系统不超过电容的安全工作范围

} // namespace CAPARR
