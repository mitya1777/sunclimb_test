#include "main.h"

LCD_HandleTypeDef hlcd;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void TIM6_init(void);
static void DMA_init(void);

static void calculaion(void);
static void LCD_depiction(uint8_t info_to_output);
static char convert_digit_to_ascii (uint8_t);

struct Flags Systems_f = {RESET};
static uint16_t DMA_ADC_buffer[2] = {RESET};
static float Uin, U2, Iin, Pin = RESET;
static float Rsh = 0.1;

static float duty_cycle = RESET;
static char dc_d, dc_u, dc_f = RESET;
static char Uin_u, Uin_f1, Uin_f2 = RESET;
static char  Iin_u, Iin_f1, Iin_f2 = RESET;
static char Pin_u, Pin_f1, Pin_f2 = RESET;

static uint8_t anchor = RESET;


char letters[9] = {'P', 'W', 'M', '8', 'U', 'I', 'P', 'i', 'n'};
char digits[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

static uint8_t button_up, button_down = RESET;
static uint8_t button_right = 0x03;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  DMA_init();
  BSP_LCD_GLASS_Init();
//  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  TIM6_init();

  BSP_LCD_GLASS_Clear();
  Systems_f.Joystick_f = 0x03;

  TIM6 -> CR1 |= TIM_CR1_CEN;

  while (1)
  {
	  if(Systems_f.DMA_ADC_f != RESET)				//		ADC data is ready: checking condition
	  {
		  calculaion();	  	  	  	  	  	  	  	//		Power and current calculation
		  Systems_f.DMA_ADC_f = RESET;
		  LCD_depiction(anchor);
	  }

	  switch (Systems_f.Joystick_f)
	  {
		case UP:
			duty_cycle += 0.25;
			if(duty_cycle >= 99.75)
			{
				duty_cycle = 99.75;
			}
			TIM1 -> CCR2 = duty_cycle * 4;

			if(button_right == PWM)
			{
				LCD_depiction(PWM);
			}

			Systems_f.Joystick_f = RESET;
			break;

		case DOWN:
			duty_cycle -= 0.25;
			if(duty_cycle <= 1)
			{
				duty_cycle = 1;
			}
			TIM1 -> CCR2 = duty_cycle * 4;

			if(button_right == PWM)
			{
				LCD_depiction(PWM);
			}

			Systems_f.Joystick_f = RESET;
			break;

		case RIGHT:
			button_right ++;
			if(button_right > P_IN)
			{
				button_right = PWM;
			}

			LCD_depiction(button_right);

			Systems_f.Joystick_f = RESET;
			break;

		default:
			Systems_f.Joystick_f = RESET;
			break;
	}
  }
}



static void calculaion(void)
{
	Uin = 3.0 * DMA_ADC_buffer[0]/4096;
	U2 = 3.0 * DMA_ADC_buffer[1]/4096;
	Iin = (Uin - U2) / Rsh;
	Pin = Uin * Iin;
}


static void LCD_depiction(uint8_t info_to_output)
{
	BSP_LCD_GLASS_Clear();

	anchor = info_to_output;
	switch (info_to_output)
	{
		case PWM:
			if(duty_cycle >= 10)
			{
				dc_d = convert_digit_to_ascii((uint8_t) duty_cycle / 10);
				dc_u = convert_digit_to_ascii((uint8_t) duty_cycle % 10);
				dc_f = convert_digit_to_ascii((uint8_t) (duty_cycle * 10) % 10);
			}
			else
			{
				dc_d = convert_digit_to_ascii((uint8_t) duty_cycle / 10);
				dc_u = convert_digit_to_ascii((uint8_t) duty_cycle);
				dc_f = convert_digit_to_ascii((uint8_t) (duty_cycle * 10) % 10);
			}

			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[0]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[1]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[2]), POINT_ON, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_3);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &dc_d, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &dc_u, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &dc_f, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
			BSP_LCD_GLASS_DisplayBar(LCD_BAR_0);
			break;

		case U_IN:
			Uin_u = convert_digit_to_ascii((uint8_t) Uin);
			Uin_f1 = convert_digit_to_ascii((uint8_t) (Uin * 10) % 10);
			Uin_f2 = convert_digit_to_ascii((uint8_t) (Uin * 100) % 10);

			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[4]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[7]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[8]), POINT_ON, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_3);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Uin_u, POINT_ON, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Uin_f1, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Uin_f2, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
			BSP_LCD_GLASS_DisplayBar(LCD_BAR_1);
			break;

		case I_IN:
			Iin_u = convert_digit_to_ascii((uint8_t) Iin);
			Iin_f1 = convert_digit_to_ascii((uint8_t) (Iin * 10) % 10);
			Iin_f2 = convert_digit_to_ascii((uint8_t) (Iin * 100) % 10);

			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[5]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[7]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[8]), POINT_ON, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_3);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Iin_u, POINT_ON, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Iin_f1, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Iin_f2, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
			BSP_LCD_GLASS_DisplayBar(LCD_BAR_2);
			break;

		case P_IN:
			Pin_u = convert_digit_to_ascii((uint8_t) Pin);
			Pin_f1 = convert_digit_to_ascii((uint8_t) (Pin * 10) % 10);
			Pin_f2 = convert_digit_to_ascii((uint8_t) (Pin * 100) % 10);

			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[6]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[7]), POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_2);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) ((uint32_t) &letters[8]), POINT_ON, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_3);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Pin_u, POINT_ON, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_4);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Pin_f1, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_5);
			BSP_LCD_GLASS_DisplayChar((uint8_t *) &Pin_f2, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
			BSP_LCD_GLASS_DisplayBar(LCD_BAR_3);
			break;

		default:
			break;
	}
}

static char convert_digit_to_ascii(uint8_t dgt)
{
	char ascii = digits[dgt];
	return ascii;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_PLLSAI1_ConfigDomain_ADC(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLSAI1R_DIV_2);
  LL_RCC_PLLSAI1_EnableDomain_ADC();
  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(40000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN5
  PA1   ------> ADC1_IN6
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_0|LL_GPIO_PIN_1);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM6_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV8;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

  ADC1 -> CR |= ADC_CR_ADEN;
  ADC1 -> CR |= ADC_CR_ADSTART;
}


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 400;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_NC);
  LL_TIM_DisableExternalClock(TIM1);
  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  /**TIM1 GPIO Configuration
  PE11   ------> TIM1_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}


static void TIM6_init(void)
{
	TIM6 -> CR1 |= TIM_CR1_ARPE;
	TIM6 -> PSC |= (10000 << TIM_PSC_PSC_Pos);
	TIM6 -> ARR |= (1000  << TIM_ARR_ARR_Pos);
	TIM6 -> CNT |= (1     << TIM_CNT_CNT_Pos);
	TIM6 -> CR2 |= TIM_CR2_MMS2_1;
}


/**
  * Enable DMA controller clock
  */
//static void MX_DMA_Init(void)
//{
//
//  /* Init with LL driver */
//  /* DMA controller clock enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
//
//  /* DMA interrupt init */
//  /* DMA1_Channel1_IRQn interrupt configuration */
//  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
//
//}


static void DMA_init(void)
{
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	ADC1 -> CFGR |= ADC_CFGR_DMAEN;											//	enable DMA on ADC
	ADC1 -> CFGR |= ADC_CFGR_DMACFG;										//	circular DMA mode configuration
	DMA1_Channel1-> CPAR = (uint32_t) (&(ADC1 -> DR));						//	peripheral address setting (ADC1_DR register pointer)
	DMA1_Channel1 -> CMAR = (uint32_t) (&DMA_ADC_buffer);					//	memory address setting (ADC_DMA_buffer)
	DMA1_Channel1 -> CNDTR = 0x02;											//	total data to be transferred number configuration
	DMA1_Channel1 -> CCR &= ~(DMA_CCR_PL_0 | DMA_CCR_PL_1);					//	channel low priority configuration
	DMA1_Channel1 -> CCR &= ~(DMA_CCR_DIR | DMA_CCR_PINC);					//	data transfer direction and peripheral pointer increment configuration
	DMA1_Channel1 -> CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; 				//	16 bit memory and peripheral size configuration
	DMA1_Channel1 -> CCR |= DMA_CCR_MINC;									//	memory address pointer increment configuration
	DMA1_Channel1 -> CCR |= DMA_CCR_CIRC;									//	circular mode configuration
	DMA1_Channel1 -> CCR |= DMA_CCR_TCIE;									//	transfer complete interrupt enable
	DMA2_Channel1 -> CCR |= DMA_CCR_EN;
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE5);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE2);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE15);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_5;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_15;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOE, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinMode(GPIOE, LL_GPIO_PIN_15, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_2, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);

  GPIOE-> MODER &= ~(GPIO_MODER_MODE8);										//	Zeroing mode register
  GPIOE-> MODER |= (GPIO_MODER_MODE8_0);									// Setting desired mode of port
  GPIOE-> OTYPER &= ~(GPIO_OTYPER_OT_8);									// Zeroing mode register
  GPIOE-> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);								//	Zeroing speed register
  GPIOE-> OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1);	// Setting desired speed of port`s clock cycles
  GPIOE-> PUPDR &= ~(GPIO_PUPDR_PUPD8);										// Zeroing pull(up/down) register

  GPIOE-> ODR &= ~(GPIO_ODR_OD8);											// Set "Off" desired pin of port

  NVIC_SetPriority(EXTI2_IRQn, 0x00);
  NVIC_SetPriority(EXTI9_5_IRQn, 0x00);
  NVIC_SetPriority(EXTI15_10_IRQn, 0x00);
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
