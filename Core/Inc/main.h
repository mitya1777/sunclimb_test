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


struct Flags {
	uint8_t DMA_ADC_f,
			LCD_f,
			Joystick_f;
};


void Error_Handler(void);

#define JOY_DOWN_Pin LL_GPIO_PIN_5
#define JOY_UP_Pin LL_GPIO_PIN_14
#define JOY_RIGHT_Pin LL_GPIO_PIN_14

#define JOY_DOWN_GPIO_Port GPIOA
#define JOY_UP_RIGHT_GPIO_Port GPIOC

#define UP 			0x01
#define DOWN 		0x02
#define RIGHT 		0x03


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
