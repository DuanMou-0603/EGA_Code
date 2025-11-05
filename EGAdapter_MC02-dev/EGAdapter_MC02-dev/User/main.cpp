//
// Author: Breezeee
// Date: 25-5-28
//
// =============================== 引入头文件 ===============================
#include "main.h"

#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"

#include "task_manager.h"

void MPU_Config(void);
/**
 * @brief 主函数
 * @details 初始化系统，配置外设，创建FreeRTOS任务，进入主循环
 * @attention CubeMX更新代码以后一定要检查一下主函数。
 * 						如果你的Core文件夹是由STM32CubeMX自动生成的全新框架
 * 						请手动在main.h添加对时钟树函数的声明：
 * 						void SystemClock_Config(void);
 * 						并将MPU_Config函数复制过来。
 * 						如果MPU有更新，需要手动更新MPU_Config函数。
 */
int main() {
	/* MPU Configuration*/
	MPU_Config();

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_FDCAN1_Init();
	MX_SPI6_Init();
	MX_USART1_UART_Init();
	MX_FDCAN2_Init();
	MX_FDCAN3_Init();
	MX_UART5_Init();
	MX_UART7_Init();
	MX_SPI2_Init();
	MX_TIM3_Init();

	/* 初始化FreeRTOS任务 */
	Task_Init();

	/* 正常来说代码不会进到这里 */
	while (1) {
	}
}

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	*/
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}




