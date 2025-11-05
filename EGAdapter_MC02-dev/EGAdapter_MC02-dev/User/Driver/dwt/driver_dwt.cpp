#include "driver_dwt.h"
#include "cmsis_os2.h"
#include "bit_locker.h"

void DWTInstance::DWT_CNT_Update(void)
{
	static bit_locker_h7 bit_lock;
	static uint32_t CYCCNT_LAST;

	// 如果获取锁失败，直接返回（非阻塞）
	if (!bit_lock.try_lock()) {
		return;
	}

	// 临界区开始
	volatile uint32_t cnt_now = DWT->CYCCNT;
	if (cnt_now < CYCCNT_LAST)
		cyc_round_cnt_++;
	CYCCNT_LAST = cnt_now;

	bit_lock.unlock();  // 临界区结束，释放锁
}

void DWTInstance::DWT_SysTimeUpdate(void)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

	DWT_CNT_Update();

	CYCCNT64 = (uint64_t)cyc_round_cnt_ * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
	CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
	CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
	dwt_time_.s = CNT_TEMP1;
	dwt_time_.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
	CNT_TEMP3 = CNT_TEMP2 - dwt_time_.ms * CPU_FREQ_Hz_ms;
	dwt_time_.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

void DWTInstance::init(uint32_t CPU_Freq_mHz)
{
	DWTInstance& instance = getInstance();

	/* 使能DWT外设 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	/* DWT CYCCNT寄存器计数清0 */
	DWT->CYCCNT = (uint32_t)0u;
	/* 使能Cortex-M DWT CYCCNT寄存器 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	instance.CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
	instance.CPU_FREQ_Hz_ms = instance.CPU_FREQ_Hz / 1000;
	instance.CPU_FREQ_Hz_us = instance.CPU_FREQ_Hz / 1000000;
	instance.cyc_round_cnt_ = 0;

	instance.DWT_CNT_Update();

	instance.initialized_ = true;
}

float DWTInstance::getTimeline_s(void)
{
	DWTInstance& instance = getInstance();
	instance.DWT_SysTimeUpdate();

	float DWT_Timelinef32 = instance.dwt_time_.s + instance.dwt_time_.ms * 0.001f + instance.dwt_time_.us * 0.000001f;

	return DWT_Timelinef32;
}

float DWTInstance::getTimeline_ms(void)
{
	DWTInstance& instance = getInstance();
	instance.DWT_SysTimeUpdate();

	float DWT_Timelinef32 = instance.dwt_time_.s * 1000 + instance.dwt_time_.ms + instance.dwt_time_.us * 0.001f;

	return DWT_Timelinef32;
}

uint64_t DWTInstance::getTimeline_us(void)
{
	DWTInstance& instance = getInstance();
	instance.DWT_SysTimeUpdate();
	return instance.dwt_time_.s * 1000000 + instance.dwt_time_.ms * 1000 + instance.dwt_time_.us;
}

void DWTInstance::delay(float Delay)
{
	DWTInstance& instance = getInstance();
	uint32_t tickstart = DWT->CYCCNT;
	float wait = Delay;

	while ((DWT->CYCCNT - tickstart) < wait * (float)instance.CPU_FREQ_Hz)
		;
}

float DWTInstance::getDeltaT()
{
	DWTInstance& instance = getInstance();
	volatile uint32_t cnt_now = DWT->CYCCNT;
	float dt = ((uint32_t)(cnt_now - instance.dwt_cnt_)) / ((float)(instance.CPU_FREQ_Hz));
	instance.dwt_cnt_ = cnt_now;

	instance.DWT_CNT_Update();

	return dt;
}

double DWTInstance::getDeltaT64()
{
	DWTInstance& instance = getInstance();
	volatile uint32_t cnt_now = DWT->CYCCNT;
	double dt = ((uint32_t)(cnt_now - instance.dwt_cnt_)) / ((double)(instance.CPU_FREQ_Hz));
	instance.dwt_cnt_ = cnt_now;

	instance.DWT_CNT_Update();

	return dt;
}