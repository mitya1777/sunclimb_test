#include "main.h"
#include "stm32l4xx_it.h"



extern struct Flags Systems_f;


void DMA1_Channel1_IRQHandler(void)
{
	Systems_f.DMA_ADC_f = SET;
	DMA1 -> IFCR |= DMA_IFCR_CTCIF1;
}


void EXTI9_5_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5))
	{
		Systems_f.Joystick_f = DOWN;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14))
	{
		Systems_f.Joystick_f = UP;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
	}

	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15))
	{
		Systems_f.Joystick_f = RIGHT;
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
	}

	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
}


void NMI_Handler(void)
{
  while (1)
  {  }
}


void HardFault_Handler(void)
{
  while (1)
  {  }
}


void MemManage_Handler(void)
{
  while (1)
  {  }
}


void BusFault_Handler(void)
{
  while (1)
  {  }
}


void UsageFault_Handler(void)
{
  while (1)
  {  }
}


void SVC_Handler(void)
{}


void DebugMon_Handler(void)
{}


void PendSV_Handler(void)
{}


void SysTick_Handler(void)
{
  HAL_IncTick();
}
