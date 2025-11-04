#include "Interface.hpp"
#include "string.h"

//extern uint32_t vTick;

InterfaceStatus interfaceStatus;

typedef struct
{
    RGB rgbs[LED_NUM];
    unsigned char updatedFlag;//// 更新标志，标记是否有新的颜色数据需要发送
    unsigned char txFlag;
} RGBStatus;




namespace WS2812
{

static RGBStatus rgbStatus;
static int ws2812_isInit = 0;

void PWM_DMA_TransmitFinshed_Callback(TIM_HandleTypeDef *htim)
{
    if (htim == &WS2812_TIM)
    {
        HAL_TIM_PWM_Stop_DMA(htim, WS2812_TIM_CHANNEL);//停止TIM2通道3的PWM DMA传输
        rgbStatus.txFlag = 0;
    }
}

static uint32_t CCRDMABuff[LED_NUM * sizeof(RGB) * 8 + 1];//- 每个字节需要8个PWM脉冲来表示（WS2812协议要求）
        //- + 1 ：额外的复位脉冲（低电平，用于结束传输）

void init()
{
    if (ws2812_isInit)
        return;
    ws2812_isInit = 1;
    HAL_TIM_RegisterCallback(&WS2812_TIM, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, PWM_DMA_TransmitFinshed_Callback);//注册回调函数
}

void update()
{
    if (!rgbStatus.updatedFlag)//更新检查 ： updatedFlag 为0时直接返回，避免无效传输
        return;
    if (rgbStatus.txFlag)//传输冲突检查 ： txFlag 为1时表示正在传输，防止重复启动
        return;
    rgbStatus.txFlag = 1;//标记开始传输
    unsigned int data;

    /*Pack the RGB Value*/
    for (unsigned int i = 0; i < LED_NUM; i++)//转换后格式： 0x00GGRRBB (24位有效数据)
    {
        data = *(unsigned volatile int *)(&rgbStatus.rgbs[i]);
        for (unsigned int j = 0; j < sizeof(RGB) * 8; j++)
            CCRDMABuff[i * sizeof(RGB) * 8 + j] = ((1UL << (23 - j)) & data) ? BIT1_WIDTH : BIT0_WIDTH;
    }
    CCRDMABuff[LED_NUM * sizeof(RGB) * 8] = 0;

    /*Transmit DMA*/
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_CHANNEL, CCRDMABuff, LED_NUM * sizeof(RGB) * 8 + 1);
    rgbStatus.updatedFlag = 0;
}

void blink(uint8_t index, uint8_t r, uint8_t g, uint8_t b)//分量设置
{
    if (index >= LED_NUM || rgbStatus.txFlag)
        return;
    rgbStatus.updatedFlag = 1;
    rgbStatus.rgbs[index].blue = b;
    rgbStatus.rgbs[index].green = g;
    rgbStatus.rgbs[index].red = r;
    //直接将RGB分量值写入对应LED的颜色结构体。
}

void blink(uint8_t index, uint32_t colorCode)//颜色码设置，colorCode 格式： 0x00RRGGBB (24位RGB颜色)
{
    if (index >= LED_NUM || rgbStatus.txFlag)
        return;
    rgbStatus.updatedFlag = 1;
    rgbStatus.rgbs[index].red = (colorCode >> 16) & 0xFF;//提取高8位（红色分量）
    rgbStatus.rgbs[index].green = (colorCode >> 8) & 0xFF;//提取中8位（绿色分量）
    rgbStatus.rgbs[index].blue = colorCode & 0xFF;//提取低8位（蓝色分量）
}


}  // namespace WS2812


namespace Buzzer
{

uint32_t stopTime = 0;

void init()//初始化蜂鸣器PWM定时器
{
    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 10000000U / 2500U - 1);//设置2500Hz的默认频率
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0U);//初始状态下占空比为0%，蜂鸣器静音
    HAL_TIMEx_PWMN_Start(&BUZZER_TIM, BUZZER_TIM_CHANNEL);//启动定时器的互补输出通道
}

void stop()//停止蜂鸣器播放
{
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0U);
}

void play(uint16_t freq, uint16_t duration)
{
    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 10000000U / freq - 1);
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 5000000U / freq);
    stopTime = sysData.vTick + duration;//定时控制
}

void update()//检查播放时间并自动停止
{
    if (sysData.vTick >= stopTime)
    {
        stop();
    }
}

} // namespace Buzzer

namespace Interface
{

void updateButtonState()
{
    if(HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))//读取按钮GPIO引脚状态（高电平表示按钮未按下）
    {
        if(sysData.buttonPressedLast && (sysData.buttonCnt > 1000 && sysData.buttonCnt < 2000))//短按功能 （1-2秒）
        {
            if(errorData.errorLevel == ERROR_RECOVER_MANUAL)//清除手动恢复错误（如短路保护）
            {
                Protection::manualClearError();
            }
            else if(errorData.errorLevel == ERROR_RECOVER_AUTO)//清除自动恢复错误（如过流、过压）
            {
                Protection::autoClearError();
            }
        }
        sysData.buttonPressed = 0;
        sysData.buttonCnt = 0;
    }
    else
    {
        sysData.buttonPressed = 1;
        sysData.buttonCnt++;
        if(sysData.buttonCnt > 3000)//长按检测 （>3秒）
        {
            HRTIM::disableOutputAB();//禁用AB侧输出
            __disable_irq();//禁用全局中断
            while (true)
                NVIC_SystemReset();//强制系统重启
        }
    }
    sysData.buttonPressedLast = sysData.buttonPressed;
}


void updateBuzzerSequence()
{
    interfaceStatus.isWarningLast = interfaceStatus.isWarning;
    if(errorData.errorLevel)
    {
        interfaceStatus.isWarning = 1;
        if(!interfaceStatus.isWarningLast)//首次进警告状态
        {
            interfaceStatus.buzzerSequenceCnt = 0;
            switch (errorData.errorLevel)
            {
            case ERROR_UNRECOVERABLE:
                if(errorData.errorCode & ERROR_POWERSTAGE)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_Unrecoverable, sizeof(buzzerWS_Unrecoverable));
                break;
            case ERROR_RECOVER_MANUAL:
                if(errorData.errorCode & ERROR_SCP_B)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_SCPB, sizeof(buzzerWS_SCPB));
                else
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_SCPA, sizeof(buzzerWS_SCPA));
                break;
            case ERROR_RECOVER_AUTO:
                if(errorData.errorCode & ERROR_OCP_A)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_OCPA, sizeof(buzzerWS_OCPA));
                else if(errorData.errorCode & ERROR_OCP_B)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_OCPB, sizeof(buzzerWS_OCPB));
                else if(errorData.errorCode & ERROR_OCP_R)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_OCPR, sizeof(buzzerWS_OCPR));
                else if(errorData.errorCode & ERROR_OVP_A)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_OVPA, sizeof(buzzerWS_OVPA));
                else
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_OVPB, sizeof(buzzerWS_OVPB));

                break;
            case WARNING:
                if(errorData.errorCode & WARNING_LOWBATTERY)
                    memcpy(interfaceStatus.buzzerNote, buzzerWS_LowBattery, sizeof(buzzerWS_LowBattery));
                break;
            default:
                break;
            }
        }
    }   
    else
    {
        interfaceStatus.isWarning = 0;
        interfaceStatus.buzzerSequenceCnt = 0;
        interfaceStatus.noteIndex = 0;
        return;
    }
    
    if(interfaceStatus.buzzerSequenceCnt == interfaceStatus.buzzerNote[interfaceStatus.noteIndex].startTime)//检查当前时间是否到达音频的开始时间
    {
        Buzzer::play(interfaceStatus.buzzerNote[interfaceStatus.noteIndex].freq, 
        interfaceStatus.buzzerNote[interfaceStatus.noteIndex].duration);
        interfaceStatus.noteIndex++;
    }

    interfaceStatus.buzzerSequenceCnt ++;
    if(interfaceStatus.buzzerSequenceCnt >= WARNING_PERIOD)//达到警告周期（5秒）
    {
        Protection::autoClearError();//尝试清除自动恢复错误
        interfaceStatus.buzzerSequenceCnt = 0;
        interfaceStatus.noteIndex = 0;
    }    
}


} // namespace Interface