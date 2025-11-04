#include "main.h"
#include "hrtim.h"
#include "stdint.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "math.h"

#include "Calibration.hpp"
#include "PowerManager.hpp"
#include "Utility.hpp"
#include "Communication.hpp"
#include "Config.hpp"
#include "Interface.hpp"


// uint16_t deadTime = 50;
// uint32_t dtxr = deadTime | (deadTime << 16);
// hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].DTxR = dtxr;

void tickCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) // 4kHz tick
    {
        // 以下每个任务以1kHz运行
        // HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);
        switch (sysData.lfLoopIndex)//通过 sysData.lfLoopIndex 实现4个任务的轮询调度，每个任务以1kHz频率运行
        {
        case 0://系统基础服务
            sysData.vTick++;
            Buzzer::update();
            //WS2812::update();
            Protection::errorHandlerLF();//低频错误处理
            sysData.lfLoopIndex++;
            break;
        case 1://通信和用户接口
            if(sysData.systemInited)
            {
                CANcomm::sendSCData();// 发送超级电容数据
                PowerControl::checkRxDataTimeout(sysData.vTick);//// 检查接收数据超时
                Interface::updateButtonState();// 更新按钮状态
            }
            sysData.lfLoopIndex++;
            break;
        case 2://ADC和电容管理
            if(psData.outputABEnabled)
            {
                CAPARR::estimateCapacity(sysData.vTick);//估算电容容量
            }
            ADC::updateADClf();//更新低频ADC数据
            sysData.lfLoopIndex++;
            break;
        case 3://保护和WPT控制
            Protection::checkLowBattery();// 检查低电压保护
            Interface::updateBuzzerSequence();//更新蜂鸣器序列

            #ifdef WPT_HARDWARE
            
            if(psData.outputEEnabled)
            {   //功率监控和ASK通信控制
                askData.lastPowerOnTime = sysData.vTick;
                if(adcData.pWPTlf < 3.0f)
                    askData.lowPowerCnt++;
                else
                    askData.lowPowerCnt = 0U;
            }
            else 
            {
                askData.enableASK = 0;
                askData.lowPowerCnt = 0;
                if(adcData.vWPT < adcData.vB + 0.5f && adcData.vCaplf > CAPARR_CUTOFF_VOLTAGE)
                    askData.allowRestart = 1U; // 允许重新启动WPT
            }
            
            #endif // WPT_HARDWARE

            sysData.lfLoopIndex = 0;
            break;
        default:
            break;
        }
        
        // 以下每个任务以4kHz运行
        
        if(sysData.systemInited)
        {
            PowerControl::powerOnOffControl();// 电源开关控制
            
            #ifdef WPT_HARDWARE  // WPT实时控制逻辑

            if(psData.outputEEnabled)
            {
                  // 电压保护和功率控制
                if(adcData.vCaplf > (CAPARR_MAX_VOLTAGE * 1.01f))//严重过压保护
                {
                    askData.enableASK = 0;// 禁用ASK通信
                    askData.powerRequirement = 0U;// 停止功率需求

                    ctrlData.wptStatus = WPT_ERROR; // 设置错误状态
                }
                // 充电完成检测
                else if(adcData.vCaplf > (CAPARR_MAX_VOLTAGE *0.99f))//正常充电完成
                {
                    askData.powerRequirement = 0U;// 停止功率需求
                    ctrlData.wptStatus = WPT_FINISHED;// 设置充电完成状态
                }
                else
                {
                    if(adcData.vCap * capStatus.maxInCurrent > adcData.pRefereelf + 120.0f)//电容电压 × 最大输入电流 > 参考功率 + 120W 时请求功率
                        askData.powerRequirement = 1U;// 需要更多功率
                    else
                        askData.powerRequirement = 0U;// 功率充足
                    ctrlData.wptStatus = WPT_CHARGING;//无线充电
                }
                
                if(adcData.pWPTlf > 145.0f)//WPT功率超过145W时触发过功率保护
                {
                    askData.enableASK = 0;
                    askData.powerRequirement = 0U;
                    ctrlData.wptStatus = WPT_ERROR;
                } 

                if(!psData.outputABEnabled || adcData.vWPT < adcData.vB || askData.lowPowerCnt > 150U) //adcData.pWPT < 3.0f //!psData.outputABEnabled || 
                {
                    HRTIM::disableOutputE();
                    askData.enableASK = 0;
                    askData.lowPowerCnt = 0U;
                    ctrlData.wptStatus = WPT_OFF;
                }
            }
            else
            {  
                if(psData.outputABEnabled && (adcData.vWPT > 29.55f)) // && sysData.vTick - askData.lastPowerOnTime > 400U//(adcData.vWPT > adcData.vCap + 0.8f && askData.allowRestart) || 
                {
                    HRTIM::enableOutputE(0.96f);//以96%占空比启动WPT输出
                    askData.allowRestart = 0U;
                    
                    askData.powerRequirement = 0U;
                    askData.enableASK = 1;
                    ctrlData.wptStatus = WPT_CHARGING;//初始化ASK通信和充电状态
                }
            } 
            ASKcomm::askLoop();// ASK通信循环
            #endif // WPT_HARDWARE
        }
        
    }
}

void init()
{
    Protection::configAWDG();//配置模拟看门狗，用于电压监控和保护。

    ADC::initAnalog();//配置运放、比较器等模拟电路
    ADC::initADC();

    CANcomm::init();
    //WS2812::init();
    Buzzer::init();
    
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, tickCallback);
    HAL_TIM_Base_Start_IT(&htim6);//4kHz系统滴答定时器，驱动主要任务调度
    
    HAL_TIM_Base_Start(&htim16);
    
    #ifdef WPT_HARDWARE

    ASKcomm::init();

    #endif // WPT_HARDWARE

    Protection::checkHardwareUID();
    Buzzer::play(800, 150);//系统启动音
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 + HRTIM_OUTPUT_TB2);
    //确保TA1、TA2、TB1、TB2通道初始状态为关闭
    psData.outputABEnabled = 0;
    HRTIM::startTimer();

    HAL_Delay(400);
    sysData.systemInited = true;
}


static void loop()
{
    while (true)
    {
        HAL_Delay(1);
        // WS2812::blink(0, COLOR_BLANK);
        // WS2812::blink(1, COLOR_BLANK);
        // WS2812::blink(2, COLOR_BLANK);
    }
}

extern "C"
{
    void systemStart()
    {
        init();
        loop();
    }
}
