/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   This file provides targets hardware configuration.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/

#include "hw_config.h"
#include "board.h"

#include "gd32f4xx.h"
#include "stm32f4xx_it.h"  //for SysTickCntr



#define RT485_2_Pin GPIO_PIN_1  //PB

#define RE485_1_Pin GPIO_PIN_8 //PA

#define RE483_3_Pin GPIO_PIN_12 //PC

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

void EnableBkpSram(void)
{
	rcu_periph_clock_enable(RCU_PMU);
	rcu_periph_clock_enable(RCU_BKPSRAM);
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);  //RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_BKPSRAM); //	RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;

	/* Allow access to backup domain */
	pmu_backup_write_enable(); //  LL_PWR_EnableBkUpAccess();  // 1
	pmu_backup_ldo_config(PMU_CS_BLDOON); //  LL_PWR_EnableBkUpRegulator();//2
}

void LL_Init(void)
{

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
//  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
//  {
//  Error_Handler();
//  }
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);


//  LL_RCC_HSE_Enable();
//   /* Wait till HSE is ready */
//  while(LL_RCC_HSE_IsReady() != 1)
//  {
//  }
	//^^^^^^^^^^^ this is done in SystemInit ^^^^^^^^^^^^^^^^^------------------



  rcu_osci_on(RCU_IRC32K);
  /* wait till IRC32K is ready */
  while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)){
  }
//  LL_RCC_LSI_Enable();
   /* Wait till LSI is ready */
//  while(LL_RCC_LSI_IsReady() != 1)
//  {
//  }


//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);

//  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//  }
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
 // LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//  }
  //^^^^^^^^^^^^^^^^^^^^ this is done in SystemInit ^^^^^^^^^^^^^^^^^^-------------------

  SysTick_Config(168000000/(20*1000)); // ===>> See stm32f4xx_it.c SysTick_Handler!!!!!!!!!!!!!!!!!!!!!!!!!!
  systick_clksource_set(SYSTICK_CLKSOURCE_HCLK);
 // LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK); done above in SysTick_Config(168000000/(20*1000));

  SystemCoreClockUpdate(); //must be == LL_SetSystemCoreClock(168000000);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,OPPSysTick_IRQn);
}

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);
void ADC012_Init(void)
{
	  adc_clock_config(ADC_ADCCK_PCLK2_DIV4); //pclk==apb2=ahb/2; ahb=sysclk ;sysclk=168(240max) =>84/4=21mhz

	rcu_periph_clock_enable(RCU_ADC0);
	rcu_periph_clock_enable(RCU_ADC1);
	rcu_periph_clock_enable(RCU_ADC2);

	adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT); //ADC_ALL_INSERTED_PARALLEL);

	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
}
/* ADC1 init function */
void MX_ADC1_Init(void)
{

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOB);


  /**ADC1 GPIO Configuration
  PA4   ------> ADC1_IN4
  PC5   ------> ADC1_IN15
  PB0   ------> ADC1_IN8
  */
  gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_4);
  gpio_mode_set(GPIOC,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_5);
  gpio_mode_set(GPIOB,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_0);


  adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
  adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);
  adc_channel_length_config(ADC0,ADC_INSERTED_CHANNEL,3);

  adc_inserted_channel_config(ADC0,0,ADC_CHANNEL_4,ADC_SAMPLETIME_56); //pa4
  adc_inserted_channel_config(ADC0,1,ADC_CHANNEL_8,ADC_SAMPLETIME_56); //pb0
  adc_inserted_channel_config(ADC0,2,ADC_CHANNEL_15,ADC_SAMPLETIME_56); //pc5

  adc_software_trigger_enable(ADC0,ADC_INSERTED_CHANNEL);

  /* enable ADC interface */
  adc_enable(ADC0);
  /* wait for ADC stability */
  uint32_t dd=SysTickCntr+11; while(SysTickCntr>dd){;}//delay_1ms(1);
  /* ADC calibration and reset calibration */
  adc_calibration_enable(ADC0);
  dd=SysTickCntr+2; while(SysTickCntr>dd){;}//delay_01ms(.1);
//  ADC_CTL1(ADC0) |= (uint32_t)ADC_CTL1_SWICST;  //start from irq  ADC1->CR2 |= ADC_CR2_JSWSTART;
 }

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  rcu_periph_clock_enable(RCU_GPIOA);
  gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /**ADC2 GPIO Configuration
  PA5   ------> ADC2_IN5
  PA6   ------> ADC2_IN6
  PA7   ------> ADC2_IN7
  */
  adc_data_alignment_config(ADC1,ADC_DATAALIGN_RIGHT);
  adc_special_function_config(ADC1,ADC_SCAN_MODE,ENABLE);
  adc_channel_length_config(ADC1,ADC_INSERTED_CHANNEL,3);

  /**Configure Injected Channel
    */
  adc_inserted_channel_config(ADC1,0,ADC_CHANNEL_5,ADC_SAMPLETIME_56); //pa5
  adc_inserted_channel_config(ADC1,1,ADC_CHANNEL_6,ADC_SAMPLETIME_56); //pa6
  adc_inserted_channel_config(ADC1,2,ADC_CHANNEL_7,ADC_SAMPLETIME_56); //pa7

  adc_software_trigger_enable(ADC1,ADC_INSERTED_CHANNEL);

  /* enable ADC interface */
  adc_enable(ADC1);
  /* wait for ADC stability */
  uint32_t dd=SysTickCntr+11; while(SysTickCntr>dd){;}//delay_1ms(1);
  /* ADC calibration and reset calibration */
  adc_calibration_enable(ADC1);
   dd=SysTickCntr+2; while(SysTickCntr>dd){;}//delay_01ms(.1);

  //  ADC_CTL1(ADC1) |= (uint32_t)ADC_CTL1_SWICST;  //start from irq  ADC2->CR2 |= ADC_CR2_JSWSTART;

}

/* ADC3 init function */
void MX_ADC3_Init(void)
{

	  rcu_periph_clock_enable(RCU_GPIOC);
	  gpio_mode_set(GPIOC,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_2|GPIO_PIN_3);

  /**ADC3 GPIO Configuration
  PC2   ------> ADC3_IN12
  PC3   ------> ADC3_IN13
  */
	  adc_data_alignment_config(ADC2,ADC_DATAALIGN_RIGHT);
	  adc_special_function_config(ADC2,ADC_SCAN_MODE,ENABLE);
	  adc_channel_length_config(ADC2,ADC_INSERTED_CHANNEL,2);

  adc_inserted_channel_config(ADC2,0,ADC_CHANNEL_13,ADC_SAMPLETIME_56); //pc3
  adc_inserted_channel_config(ADC2,1,ADC_CHANNEL_12,ADC_SAMPLETIME_56); //pc2

  adc_software_trigger_enable(ADC2,ADC_INSERTED_CHANNEL);

  /* enable ADC interface */
  adc_enable(ADC2);
  /* wait for ADC stability */
  uint32_t dd=SysTickCntr+11; while(SysTickCntr>dd){;}//delay_1ms(1);
  /* ADC calibration and reset calibration */
  adc_calibration_enable(ADC2);
  dd=SysTickCntr+2; while(SysTickCntr>dd){;}//delay_01ms(.1);

  //  ADC_CTL1(ADC2) |= (uint32_t)ADC_CTL1_SWICST; //start from irq  ADC3->CR2 |= ADC_CR2_JSWSTART;

}

/* CRC init function */
void MX_CRC_Init(void)
{

  // Peripheral clock enable
  rcu_periph_clock_enable(RCU_CRC);
}

/* IWDG init function */
void MX_IWDG_Init(void)
{

    rcu_osci_on(RCU_IRC32K);

    /* wait till IRC32K is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)){
    }
    //clock is 32768 internal rc
    fwdgt_config(2*50,FWDGT_PSC_DIV32); // 32 ~ 1 KHz

    fwdgt_enable();
/*

  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);  // ~1ms resolution
  LL_IWDG_SetReloadCounter(IWDG, 100);            // 100*1ms timeout
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }
  LL_IWDG_ReloadCounter(IWDG);
*/
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;


    rcu_periph_clock_enable(RCU_GPIOB);

    /*Configure PB5 (TIMER2 CH1) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_5);

    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_5);

    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2); //mul4???

    timer_deinit(TIMER2);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 50;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_AUTORELOAD_DEF_1200;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* CH1,CH2 and CH3 configuration in PWM mode */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);

    /* CH2 configuration in PWM mode1,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,1); //1-minimum
    timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0); //pwm 1?
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* auto-reload preload enable */
    timer_enable(TIMER2);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;


    rcu_periph_clock_enable(RCU_GPIOB);

    /*Configure PB6 (TIMER3 CH0) as alternate function*/
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);

    gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_6);

    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2); //mul4???

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 50;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_AUTORELOAD_DEF_1200;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* CH1,CH2 and CH3 configuration in PWM mode */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER3,TIMER_CH_0,&timer_ocintpara);

    /* CH2 configuration in PWM mode1,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_0,1); //1-minimum
    timer_channel_output_mode_config(TIMER3,TIMER_CH_0,TIMER_OC_MODE_PWM0); //pwm 1?
    timer_channel_output_shadow_config(TIMER3,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
}

// TIM2 init function Initialize Timer in 4X Encoder mode
static void MX_TIM2_Init(void)
{
	timer_ic_parameter_struct timer_icintpara;
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;


    rcu_periph_clock_enable(RCU_GPIOA);

    /*Configure PA0 (TIMER1 CH0) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_0);
    /*Configure PA1 (TIMER1 CH1) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_1);

    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2); //mul4???

    timer_deinit(TIMER1);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xffffffff;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    timer_icintpara.icpolarity       = TIMER_IC_POLARITY_RISING;
    timer_icintpara.icselection      = TIMER_IC_SELECTION_DIRECTTI;
    timer_icintpara.icprescaler      = TIMER_IC_PSC_DIV1;
    timer_icintpara.icfilter         = 0x0b;//1011 FILTER_FDIV16_N6
    timer_input_capture_config(TIMER1,TIMER_CH_0 , &timer_icintpara);
    timer_input_capture_config(TIMER1,TIMER_CH_1 , &timer_icintpara);

    timer_quadrature_decoder_mode_config(TIMER1, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING); //x4??

    timer_enable(TIMER1);

}


void InitEncoderInterfaceTim2PA0PA1(void)
{
	 MX_TIM2_Init();
}


/* USART0 init function */
void MX_USART0_UART_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /**USART1 GPIO Configuration
    PA9   ------> USART0_TX
    PA10   ------> USART0_RX
    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);
    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set( GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_10);


    usart_deinit(USART0);
    usart_word_length_set(USART0,8);
    usart_stop_bit_set(USART0,1);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_oversample_config(USART0, USART_OVSMOD_16);
    usart_baudrate_set(USART0,115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_DISABLE); //???
    usart_enable(USART0);

//!!!USART1 from STM32 is USART0 in GD32
  NVIC_SetPriority(USART0_IRQn, OPPUSART1_IRQn);  //prio from h2_config.h from
  NVIC_EnableIRQ(USART0_IRQn);
//! it is slave usart, responds to display
  usart_interrupt_enable(USART0, USART_INT_RBNE);
  LoRCV_Enbl_485_0;
}

#if ENCODER_USED==0
/* USART2 init function */
void MX_USART2_UART_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_USART2);

  /**USART3 GPIO Configuration
  PC10   ------> USART3_TX
  PC11   ------> USART3_RX
  */
    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOC, GPIO_AF_7, GPIO_PIN_10);
    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOC, GPIO_AF_7, GPIO_PIN_11);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_10);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_10);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set( GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11);


    usart_deinit(USART2);
    usart_word_length_set(USART2,8);
    usart_stop_bit_set(USART2,1);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_oversample_config(USART2, USART_OVSMOD_16);
    usart_baudrate_set(USART2,115200U);
    usart_receive_config(USART2, USART_RECEIVE_DISABLE); //????
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);

  //!!!USART3 from STM32 is USART2 in GD32
  NVIC_SetPriority(USART2_IRQn, OPPUSART3_IRQn);  //prio from h2_config.h from
  NVIC_EnableIRQ(USART2_IRQn);
//!it is a master usart, send comands to ext board
//  usart_interrupt_enable(USART2, USART_INT_RBNE);
  LoRCV_Enbl_485_2;

}


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

  //USART2 GPIO Configuration
//  PA2   ------> USART2_TX
//  PA3   ------> USART2_RX

     gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_2);
     /* connect port to USARTx_Rx */
     gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_3);

     /* configure USART Tx as alternate function push-pull */
     gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_2);
     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);

     /* configure USART Rx as alternate function push-pull */
     gpio_mode_set( GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_3);
     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_3);

     usart_deinit(USART1);
     usart_word_length_set(USART1,8);
     usart_stop_bit_set(USART1,1);
     usart_parity_config(USART1, USART_PM_NONE);
     usart_oversample_config(USART1, USART_OVSMOD_16);
     usart_baudrate_set(USART1,115200U);
     usart_receive_config(USART1, USART_RECEIVE_DISABLE); //????
     usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
     usart_enable(USART1);

     //!!!USART2 from STM32 is USART1 in GD32
  NVIC_SetPriority(USART1_IRQn, OPPUSART2_IRQn);  //prio from h2_config.h from
  NVIC_EnableIRQ(USART1_IRQn);

  //  usart_interrupt_enable(USART1, USART_INT_RBNE);
    LoRCV_Enbl_485_1;

}

#endif



void MX_GPIO_Init(void)
{

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOH);

    gpio_bit_reset(GPIOA, GPIO_PIN_ALL);
    gpio_bit_reset(GPIOB, GPIO_PIN_ALL);
    gpio_bit_reset(GPIOC, GPIO_PIN_ALL);
    gpio_bit_reset(GPIOD, GPIO_PIN_ALL);

//inputs
  /**/
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
    		GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);// 8,9,10 for usart 1
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_11|GPIO_PIN_12);//!!PULLUP optocupulers
  /**/
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
  /**/
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5); //!! PULLUP OPTOCUPULERS
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE,GPIO_PIN_2|GPIO_PIN_3);

//ouputs
  /**/

  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,RE485_1_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,RE485_1_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /**/
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RT485_2_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,RT485_2_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
  /**/
  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|RE483_3_Pin);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|RE483_3_Pin);
  /**/
  gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_2);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
