#pragma once




/*-------- HARDWARE CONFIG --------*/
#define HW_VSENSE_RATIO     16.0f   // 33k:2.2k， 电压分压比例
#define HW_ISENSE_RATIO     25.0f   // INA240A1 1/(20*0.002)，电流检测总增益
#define ADC_VREF            2.9f   // VREFBUF

#define ADC_VSENSE_RES      ADC_VREF * HW_VSENSE_RATIO / 4096.0f
#define ADC_ISENSE_RES      ADC_VREF * HW_ISENSE_RATIO / 4096.0f
#define ADC_ISENSE_OFFSET   ADC_VREF * HW_ISENSE_RATIO / 2.0f
//2.9V × 25 / 2 = 36.25A，用于双向电流检测的零点偏移


#define HW_RSENSE           0.002f//电流检测电阻值0.002Ω（2mΩ）
#define HW_IAMP_GAIN        20.0f//INA240A1电流放大器增益20倍





#define ADC_ISENSE_ALPHA    0.8f
#define ADC_VSENSE_ALPHA    0.8f
//电流和电压ADC采样的 低通滤波系数
#define ADC_CALI_ALPHA      0.001f//ADC校准数据的 极慢滤波系数
#define MF_TO_LF_ALPHA      0.092f//中频到低频功率转换的滤波系数


#define HRTIM_PERIOD        21760U//HRTIM的周期值，实际频率 = HRTIM时钟频率 / 21760
#define VB_LIMIT_BY_DUTY        29.8f//电容电压上限
#define VWPT_LIMIT_BY_DUTY      30.8f//WPT电压上限

/*-------- PROTECTION --------*/
// 过压保护阈值
#define OVP_A                   29.0f//第一级过压保护阈值（29.0V），触发警告或轻度保护
#define OVP_B                   30.5f//第二级过压保护阈值（30.5V），触发严重保护

// 过流保护阈值
#define OCP_CAPARR              25.5f//电容组过流保护阈值
#define OCP_CHASSIS             20.0f//底盘系统过流保护阈值
#define OCP_REFEREE             6.5f//裁判系统过流保护阈值
// // Over Temperature Protection
// #define OTP_LIMIT               80.0f
// #define OTP_RECOVERY            70.0f
// 短路保护（SCP - Short Circuit Protection）阈值
#define SCP_VOLTAGE             5.0f
#define SCP_CURRENT             5.0f
#define SCP_RECOVER_TIME        1000
// 裁判系统欠压关断
#define REFEREE_UVLO_LIMIT      18.0f //狗腿特殊阈值15V，正常阈值18V
#define REFEREE_UVLO_RECOVERY   20.0f
// Low Efficiency Protection
#define LOW_EFFICIENCY_RATIO    0.75f

#define BATTERY_LOW_LIMIT       20.92f
#define BATTERY_LOW_RECOVERY    21.6f

#define MAX_INDUCTOR_CURRENT    25.0f//电感电流限制
#define SOFT_START_TIME         8//缓启动时间


/*-------- DEFUALT --------*/
#define REFEREE_DEFUALT_POWER   37.0f//裁判系统默认功率限制（37.0W）
#define REFEREE_ENERGY_BUFFER   57U//裁判系统能量缓冲区（57J）
#define REFEREE_POWER_BIAS_LIMIT    15.0f//功率偏差限制阈值（15.0W）
#define REFEREE_POWER_BIAS_WARNING  10.0f//功率偏差警告阈值（10.0W）
#define RXDATA_TIMEOUT          500U//接收数据超时时间

/*-------- SuperCapacitor Array --------*/
// 电容组异常保护阈值
#define CAPARR_DEFUALT_CAPACITY 4.4f//电容组默认容量
#define CAPARR_CAPACITY_HT      10.0f//电容容量上限阈值
#define CAPARR_CAPACITY_LT      0.2f//电容容量下限阈值
// 电容组内阻补偿
#define CAPARR_DCR              0.1f//电容组等效串联电阻（0.1Ω）
// 电容组低电量限流
#define CAPARR_CUTOFF_VOLTAGE   5.0f//电容截止电压（5.0V），低于此值停止放电
#define CAPARR_LOW_VOLTAGE      10.0f//电容低电压阈值（10.0V），触发低电量保护
#define CAPARR_MAX_VOLTAGE      28.8f//电容组最大电压
#define CAPARR_MAX_CURRENT      15.0f//电容组最大电流
#define CM01_CURRENT_LIMIT      15.0f//CM01电流限制


// ERROR_UNRECOVERABLE
#define ERROR_POWERSTAGE        0b0000000000000001//功率级错误
#define ERROR_CAPARR            0b0000000000000010//电容组严重故障
// ERROR_RECOVER_MANUAL
#define ERROR_SCP_A             0b0000000000000100//A侧短路保护
#define ERROR_SCP_B             0b0000000000001000//B侧短路保护
// ERROR_RECOVER_AUTO
#define ERROR_OCP_A             0b0000000000010000
#define ERROR_OCP_B             0b0000000000100000//A侧/B侧过流保护
#define ERROR_OCP_R             0b0000000001000000//裁判系统过流保护
#define ERROR_OVP_A             0b0000000010000000
#define ERROR_OVP_B             0b0000000100000000//A侧/B侧过压保护
// WARNING
#define WARNING_LOWBATTERY      0b0000001000000000//低电池电压警告
#define REFEREE_INACCURATE      0b0000010000000000//裁判系统数据不准确警告




#define WPT_CUTOFF_VOLTAGE      29.4f//- 无线功率传输截止电压（29.4V），当电压达到此值时停止WPT充电，防止过充
