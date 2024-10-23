/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_TwoBoards_FullDuplex_IT/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to send/receive bytes over SPI IP using
  *          the STM32F4xx SPI LL API.
  *          Peripheral initialization done using LL unitary services functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_i2c.h"


#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_angle_sensor.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_global_define.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_switch_IGBT.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_current_sensor.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_throttle.h"
/** @addtogroup STM32F4xx_LL_Examples
  * @{
  */

/** @addtogroup SPI_TwoBoards_FullDuplex_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
#define MASTER_BOARD

/* Private variables ---------------------------------------------------------*/
struct motor_t srm_narrow = {
		.switch_sequence = {
	  /*0x38 0xCF [0]*/		{.phase_name = ST1G, .i2c_pair = {.add = I2C_LEFT_SIDE_ADDR, .dr = PCF8574A_P4_P5_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} },
	  /*0x3C 0xFC [1]*/		{.phase_name = ST2G, .i2c_pair = {.add = I2C_RIGHT_SIDE_ADDR, .dr = PCF8574A_P0_P1_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} },
	  /*0x38 0xF3 [2]*/		{.phase_name = ST1Y, .i2c_pair = {.add = I2C_LEFT_SIDE_ADDR, .dr = PCF8574A_P2_P3_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} },
	  /*0x3C 0xCF [3]*/		{.phase_name = ST2Y, .i2c_pair = {.add = I2C_RIGHT_SIDE_ADDR, .dr = PCF8574A_P4_P5_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} },
      /*0x38 0xFC [4]*/		{.phase_name = ST1R, .i2c_pair = {.add = I2C_LEFT_SIDE_ADDR, .dr = PCF8574A_P0_P1_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} },
	  /*0x3C 0xF3 [5]*/		{.phase_name = ST2R, .i2c_pair = {.add = I2C_RIGHT_SIDE_ADDR, .dr = PCF8574A_P2_P3_SET_0, .byte_msg_lenght = 1, .sended_byte = 0xFF} }
						   },
		.ptr_to_switch = &srm_narrow.switch_sequence[ST2Y],     // start condition - ST1R
		.i2c_send_pair = &srm_narrow.switch_sequence[2].i2c_pair, // take from default phase (start condition)

		.i2c_pair_current_sensor_low = {.add = I2C_DAC_LOW_TRESHOLD, .dr = INIT_VAL_CURRENT_SENSOR, .byte_msg_lenght = 2},
		.i2c_pair_current_sensor_high = {.add = I2C_DAC_HIGH_TRESHOLD, .dr = INIT_VAL_CURRENT_SENSOR, .byte_msg_lenght = 2},

		.status.bit.dac_low_set = 0, .status.bit.dac_high_set  = 0,
		.status.bit.phase_dead_time_timer_set = 0, // not set  - start condition for dac
		.status.bit.i2c_Tx_req = 0,
		.status.bit.command = ENGINE_STOP,
		.status.bit.direction = ROTATION_STOP,
		.status.bit.mode = MODE_STAND_BY,
		//.status.bit.phase_on = 0,

		.switch_par.phase_pulse_count = 0,
		.switch_par.phase_switches_count = 0,
		.switch_par.tst_max_pulse_count = 1,
		.switch_par.phase_worked_time = 0, // will make default value
		.switch_par.PWM_filling = PWM_DEFAULT_FILLING, // 50 - symmetrical PWM
		.switch_par.PWM_max_filling = PWM_DEFAULT_MAX_FILLING,
		.switch_par.phase_working_time = TIM3_DEFAULT_PERIOD,
		.switch_par.phase_pulse_count_before_moving = 0,

		.angle.prev_spi_val = INIT_ANGLE_VALUE,
		.angle.spi_error_value = 0,
		.angle.spi_command = 0xFFFF,
		.angle.ptr_tmp_aligned_tbl = &srm_narrow.angle.aligned_tbl[0]

};

struct tst_current_cmp_t tst_cmp = {.st1_irq_low = 0, .st1_irq_high = 0, .st2_irq_low = 0, .st2_irq_high = 0};
//unsigned short spi_rx_buf;
//unsigned short btn_press_count = 0;
unsigned i2c_return_sign = 0, spi_sr = 0,
		 i2c_irq_cnt = 0, i2c_irq_fb = 0, i2c_irq_add = 0, i2c_irq_Tx = 0, i2c_irq_else = 0, i2c_irq_stp = 0;

// i2c reg
unsigned send_dr = 0 , // for i2c transaction
		 i2c1_cr2, i2c1_cr1, i2c1_ccr, i2c_dr, i2c1_oar1, i2c1_sr1, i2c1_sr2, i2c_req_cnt = 0, i2c_busy_cnt = 0;


volatile unsigned gpioc_idr = 0, usr_button_prs = 0,
		//gpioc_moder, gpioc_pupdr,
		tim2_irq_cnt, tim2_arr, tim2_dier, tim2_prsc, tim2_sr, tim2_cr1;

unsigned tmp_cntr = 0, tim3_ccr1_cnt = 0, tim3_arr_cnt = 0;

unsigned tmp_spi_angle = 0; int tst = 0;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

//void Activate_SPI(void);

void UserButton_Init(void);

//void Configure_I2C1(void);
//unsigned I2C1_Tx(unsigned short msg);
void CurrentCMP_PC10_PC11_Init(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
volatile unsigned exti_cr3, exti_imr = 0, exti_rtsr, tim2_cnt, tim2_tst_cnt = 0;

// tmp variable
unsigned tmpCnt = 0;
int main(void)
{
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();

  //setting of test out pin
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

  /* Initialize LED2 and test pin*/
  //LED_Init();

  /* Configure the SPI1 parameters */
  Configure_SPI1();

  Configure_I2C1();

  Configure_TIM4();
  TIM4->CR1 |= 0x1; // Enable I2C arbiter timer

  // work time counter configuration
  Configure_TIM3();

  // preset dac value
  DAC_set(Convert_currentA_to_DAC_task(INIT_VAL_CURRENT_SENSOR), Convert_currentA_to_DAC_task(5)); // replace first argument


  // border of phase current work
  CurrentCMP_PC10_PC11_Init();

  // SPI Tx timer configuration
  Configure_TIM2();
  //TIM2->CR1 |= 0x1; // CEN - clock enable, SPI Tx timer


//#ifdef MASTER_BOARD
  /* Initialize User push-button in EXTI mode */

  UserButton_Init();

  Tmp_aligned_tbl_Init(&srm_narrow.angle.aligned_tbl[0], sizeof(srm_narrow.angle.aligned_tbl) / 4);

  while (1)
  {
	  gpioc_idr = GPIOC->IDR;

	  // GPIOA->BSRR |= (0x1 << 22);
  }
}

/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None
  * @retval None
  */
void UserButton_Init(void)
{
  // Enable the BUTTON Clock
  // USER_BUTTON_GPIO_CLK_ENABLE();
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  // Configure GPIO for BUTTON
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);

  // Connect External Line to the GPIO
  // USER_BUTTON_SYSCFG_SET_EXTI();
  do{
	  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
  }while(0);

  // Enable a falling trigger External line 13 Interrupt
  //USER_BUTTON_EXTI_LINE_ENABLE();
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);

  //USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_EnableIRQ(EXTI15_10_IRQn); // 10
  NVIC_SetPriority(EXTI15_10_IRQn, 13); //10
}

void CurrentCMP_PC10_PC11_Init(void)
{
	// set Peripheral AHB1 Enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	// set direction of GPIO - input
	// ... low border (right)
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_DOWN);
	// ... high border (right)
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_DOWN);

	// ... low border (left)
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_12, LL_GPIO_PULL_DOWN);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	// ... high border (left)
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);



	//GPIOC->PUPDR |= (0x2UL << 22);

	// connect external GPIO to EXTI10(PC10)
	// ...APB2 for SYSCFG need to clock too !!!!!
	do{
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE10); // PC10
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE11); // PC11

		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE12); // PC12
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE15); // PA15

	}while(0);
	// enable interrupt
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10); // PC10
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_11); // PC11

	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_12); // PC12
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_15); // PA15

	// type of interrupt
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_10);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_11);

	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_12);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_15);


	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 13); // 3
	// configure NVIC for
	//NVIC->ISER[0] |= (0x1 << 10); // NVIC_EnableIRQ(LL_EXTI_LINE_10); //
	//NVIC->
	//NVIC_SetPriority((0x1 << (10)), 0x04); //

}

void SystemClock_Config(void)
{
  /* Enable HSE oscillator */
  LL_RCC_HSE_EnableBypass(); // RCC_CR.HSEBYP = 1;
  LL_RCC_HSE_Enable();       // RCC_CR.HSEON = 1;
  while(LL_RCC_HSE_IsReady() != 1) // wait RCC_CR.HSERDY = 1;
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3); // default 3

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 400, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4); // 4 //2
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms */
  SysTick_Config(100000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 100000000;
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
  * @brief  Function to manage User push-button
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
	{
	//NextCounterClockWisePhase(&srm_narrow);
	srm_narrow.i2c_send_pair = &srm_narrow.ptr_to_switch->i2c_pair;
	// start position in aligned table
	srm_narrow.angle.ptr_tmp_aligned_tbl = &srm_narrow.angle.aligned_tbl[0] + srm_narrow.ptr_to_switch->phase_name;
	//tmp_spi_angle = *srm_narrow.angle.ptr_tmp_aligned_tbl;
	/*
	srm_narrow.status.bit.command = (srm_narrow.status.bit.command == ENGINE_STOP) ? ENGINE_SWITCH_ON:
																					ENGINE_STOP;
		srm_narrow.status.bit.i2c_Tx_req = 1;
*/
	unsigned tmp = srm_narrow.status.bit.command;
	(void)tmp;
	 // reset previous angle set state
	srm_narrow.angle.max_match_angle_cnt = 0;
	srm_narrow.angle.match_angle_cnt = 0;

	srm_narrow.switch_par.phase_worked_time = TIM3->ARR;

	//usr_button_prs = (usr_button_prs > 0 )? 0:1;
	if (usr_button_prs == 0)
	{
		usr_button_prs = 1;
		srm_narrow.status.bit.mode = MODE_SET_AMGLE; // MODE_INFINITY_CONST_PWM;

	}
	else // need to move to I2C switch off sequence
		usr_button_prs = 0;
	}

}


/**
  * @brief  Function called from SPI1 IRQ Handler when TXE flag is set
  *         Function is in charge  to transmit byte on SPI lines.
  * @param  None
  * @retval None
  */
void  SPI1_Tx_Callback(void)
{
  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */
  //LL_SPI_TransmitData8(SPI1, aTxBuffer[ubTransmitIndex++]);

  /*if( !(SPI1->SR & (1UL << 7UL)) )  // if not busy
  { */
	 // SPI1->DR = 0xFFFF;

  //}
}



/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */
