#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l476g_discovery_glass_lcd.h"


void interrupts_handling(uint8_t);


struct Flags {
	uint8_t DMA_ADC_f,
			LCD_f,
			Joystick_f;
};


void Error_Handler(void);

#define JOY_DOWN_Pin LL_GPIO_PIN_5
#define JOY_UP_Pin LL_GPIO_PIN_15
#define JOY_RIGHT_Pin LL_GPIO_PIN_2

#define UP 			0x01
#define DOWN 		0x02
#define RIGHT 		0x04

#define PWM			0x05
#define U_IN		0x06
#define I_IN		0x07
#define P_IN		0x08



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
