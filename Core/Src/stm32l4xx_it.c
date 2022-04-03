#include "main.h"
#include "stm32l4xx_it.h"

extern struct Flags Systems_f;
extern uint16_t DMA_ADC_buffer[];

static uint8_t jerking_time = RESET;


void DMA1_Channel1_IRQHandler(void)	{
	Systems_f.DMA_ADC_f = SET;
//	GPIOE-> ODR ^= (GPIO_ODR_OD8);
	DMA1 -> IFCR |= DMA_IFCR_CTCIF1;
}

void EXTI2_IRQHandler(void)	{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2))	{
		interrupts_handling(RESET);
		Systems_f.Joystick_f = RIGHT;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
	}
}

void EXTI9_5_IRQHandler(void)	{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5))	{
		interrupts_handling(RESET);
		Systems_f.Joystick_f = DOWN;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
	}
}

void EXTI3_IRQHandler(void)	{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3))	{
		interrupts_handling(RESET);
		Systems_f.Joystick_f = UP;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
	}
}

void TIM7_IRQHandler(void)	{
	jerking_time ++;
	if(jerking_time >= EXPIRED_JERKING)	{
		interrupts_handling(SET);
		jerking_time = RESET;
		TIM7 -> CR1 &= ~TIM_CR1_CEN;
		TIM7 -> SR &= ~TIM_SR_UIF;
//		TIM6 -> CR1 &= ~TIM_CR1_CEN;									//	???
	}
	GPIOE-> ODR ^= (GPIO_ODR_OD8);
}

void NMI_Handler(void)	{
	while (1)	{}
}

void HardFault_Handler(void)	{
	while (1)	{}
}

void MemManage_Handler(void)	{
	while (1)	{}
}

void BusFault_Handler(void)	{
	while (1)	{}
}

void UsageFault_Handler(void)	{
	while (1)	{}
}

void SVC_Handler(void)	{}

void DebugMon_Handler(void)	{}

void PendSV_Handler(void)	{}

void SysTick_Handler(void)	{
	HAL_IncTick();
}
