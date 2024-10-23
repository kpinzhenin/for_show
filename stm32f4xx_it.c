/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_TwoBoards_FullDuplex_IT/Src/stm32f4xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_angle_sensor.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_global_define.h"
#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_switch_IGBT.h"

extern struct motor_t srm_narrow;
/** @addtogroup STM32F4xx_LL_Examples
  * @{
  */

/** @addtogroup SPI_TwoBoards_FullDuplex_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles external line 13 interrupt request.
  * @param  None
  * @retval None
  */
extern unsigned tim2_irq_cnt, tim2_tst_cnt;
extern volatile unsigned tim2_cnt, tim2_prsc, tim2_sr, tim2_arr, tim2_dier, tim2_cr1;

void TIM4_IRQHandler(void)
{
	TIM4_CallBack();
}

void TIM3_IRQHandler(void)
{
	TIM3_CallBack();
}

void TIM2_IRQHandler(void)
{
	TIM2_callBack();
	/*
	// expect Update counter interrupt
	if ( !(GPIOA->ODR & 0x40) ) // if before was
	{
		GPIOA->BSRR |= (0x1 << 6); // set PA6

	}else
		GPIOA->BSRR |= (0x1 << 22); // reset PA6

	// check Pending and reset
	if (TIM2->SR & 0x1) // check UIF flag
	{
		TIM2->SR &= 0x0; // reset UIF
		//tim2_tst_cnt++;
	}

	tim2_cnt = TIM2->CNT;
	*/
/*

	tim2_irq_cnt++;
	tim2_sr = TIM2->SR;
	tim2_prsc = TIM2->PSC;
	tim2_arr = TIM2->ARR;
	tim2_dier = TIM2->DIER;
	tim2_cr1 = TIM2->CR1;
*/
}

extern unsigned tmp_cntr;
extern struct tst_current_cmp_t tst_cmp;

void USER_BUTTON_IRQHANDLER(void) // EXTI15_10_IRQHandler
{
  /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

    tmp_cntr++;
    /* Manage code in main.c */
    UserButton_Callback();
  }

  // for LOW border(right side) PC10 pin
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {


	  if (srm_narrow.status.bit.st2_dac_high_falling != 1
			  && srm_narrow.i2c_send_pair->add == I2C_RIGHT_SIDE_ADDR
				&& (TIM3->CR1 & 0x1) == 1
				&& (TIM3->CNT > 50) // WTF!

				//&& TIM3->CCR1 == TIM3->ARR + CCR1_SHIFT_CASE_END_CCR1
				&& srm_narrow.status.bit.command == ENGINE_SWITCH_ON
				&& srm_narrow.i2c_send_pair->sended_byte == srm_narrow.ptr_to_switch->i2c_pair.dr)
				//&& TIM3->CNT > TIM4->ARR )
	  {

			if ( !(GPIOA->ODR & 0x40)  ) // if before was
				GPIOA->BSRR |= (0x1 << 6); // set PA6
			else
				GPIOA->BSRR |= (0x1 << 22); // set PA6

		tst_cmp.st2_irq_high++; // expect falling edge

	  		srm_narrow.status.bit.st2_dac_high_falling = 1;

	 		if (srm_narrow.status.bit.mode == MODE_SET_AMGLE)
	 		{
	 			if (srm_narrow.switch_par.PWM_filling == PWM_DEFAULT_FILLING)
	 			{
	 				srm_narrow.switch_par.phase_pulse_count = srm_narrow.switch_par.tst_max_pulse_count;

	 			}
	 			else
	 			{
	 				srm_narrow.switch_par.PWM_max_filling = srm_narrow.switch_par.PWM_filling;
	 				PWM_FILLING_STEP_DOWN(srm_narrow.switch_par.PWM_filling, 2);
	 			}

	 		}
			// engine will switches off, when call TIM3 update
			srm_narrow.switch_par.phase_worked_time = TIM3->CNT; // save time to rich

			// engine switch off (not good construction)
/*			srm_narrow.status.bit.command = ENGINE_STOP;
			I2C1_Tx();
*/
	  }
	  	  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
	  //UserButton_Callback();
  }
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
	  if (srm_narrow.status.bit.st2_dac_low_rising != 1
			  && srm_narrow.i2c_send_pair->add == I2C_RIGHT_SIDE_ADDR
			  && srm_narrow.status.bit.command == ENGINE_STOP)
	  {
		  tst_cmp.st2_irq_low++; // expect rising edge
		  srm_narrow.status.bit.st2_dac_low_rising = 1;
	  	  srm_narrow.status.bit.st2_dac_high_falling = 0;
	  }

	  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
  }
//--------------------------------------------ST1
  // high border of current sensor
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET) // D14 Saleale
  {

  	if (srm_narrow.status.bit.st1_dac_high_falling != 1
  			&& srm_narrow.i2c_send_pair->add == I2C_LEFT_SIDE_ADDR

			&& (TIM3->CR1 & 0x1) == 1
			&& (TIM3->CNT > 50) // WTF!

			&& srm_narrow.status.bit.command == ENGINE_SWITCH_ON
			&& srm_narrow.i2c_send_pair->sended_byte == srm_narrow.ptr_to_switch->i2c_pair.dr)


  	{

  		if ( !(GPIOA->ODR & 0x40)  ) // if before was
			GPIOA->BSRR |= (0x1 << 6); // set PA6
		else
			GPIOA->BSRR |= (0x1 << 22); // set PA6

	tst_cmp.st1_irq_high++; // expect falling edge

  		srm_narrow.status.bit.st1_dac_high_falling = 1;
  		//srm_narrow.status.bit.st1_dac_low_rising = 0;

 		if (srm_narrow.status.bit.mode == MODE_SET_AMGLE)
 		{
 			if (srm_narrow.switch_par.PWM_filling == PWM_DEFAULT_FILLING)
 			{
 				srm_narrow.switch_par.phase_pulse_count = srm_narrow.switch_par.tst_max_pulse_count;

 			}
 			else
 			{
 				srm_narrow.switch_par.PWM_max_filling = srm_narrow.switch_par.PWM_filling;
 				PWM_FILLING_STEP_DOWN(srm_narrow.switch_par.PWM_filling, 2);
 			}

 		}
		// engine will switches off, when call TIM3 update
		srm_narrow.switch_par.phase_worked_time = TIM3->CNT; // save time to rich

		// engine switch off (not good construction)
/*		srm_narrow.status.bit.command = ENGINE_STOP;
		I2C1_Tx();
*/
        //TIM3->EGR |= 1; // generate UIF interrupt

		//TIM3->CCR1 = TIM3->CNT + 10; // work_time + 10
  	}

  	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);

  }
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {

  	  if (srm_narrow.status.bit.st1_dac_low_rising != 1
  			  && srm_narrow.i2c_send_pair->add == I2C_LEFT_SIDE_ADDR
  			  && srm_narrow.status.bit.command == ENGINE_STOP)
  	  {

  		  tst_cmp.st1_irq_low++; // expect rising edge

  		  srm_narrow.status.bit.st1_dac_low_rising = 1;

  		  /* if(srm_narrow.status.bit.mode != MODE_SET_AMGLE) */
  		  srm_narrow.status.bit.st1_dac_high_falling = 0;
  	  }
  	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
  }
}

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
    /* Call function Slave Reception Callback */
    SPI1_Rx_Callback();
  }
  /* Check RXNE flag value in ISR register */
  else if(LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    /* Call function Slave Reception Callback */
    SPI1_Tx_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if(LL_SPI_IsActiveFlag_OVR(SPI1))
  {
    /* Call Error function */
    SPI1_TransferError_Callback();
  }
}

extern unsigned short i2c_irq_cnt, i2c_irq_fb, i2c_irq_add, i2c1_cr1,
					  i2c_irq_else, i2c_irq_Tx, send_dr, i2c1_sr1, i2c1_cr2, i2c_irq_stp,
					  i2c_busy_cnt;

extern unsigned usr_button_prs, tim3_cnt;

void I2C1_EV_IRQHandler(void)
{
	i2c_irq_cnt++;
	// stage EV5. Time to send address
	  if(LL_I2C_IsActiveFlag_SB(I2C1))
	  {
		i2c_irq_fb++;
	    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a read request */
	    //LL_I2C_TransmitData8(I2C1, (0x38 << 1)); // set ADDRESS and !W bit to read
		LL_I2C_TransmitData8(I2C1, ( (srm_narrow.i2c_send_pair->add) << 1)); // set ADDRESS and !W bit to read

		TIM2->CR1 &= 0xFFFFFFFE; // disable SPI transmit for sync start with TIM3
	  }
	  else if (LL_I2C_IsActiveFlag_ADDR(I2C1)) //I2C_SR1.ADDR = 1
	  {
		  i2c_irq_add++;
		  // prepare to load transmit data
		  LL_I2C_EnableIT_BUF(I2C1); // I2C_CR2.ITBUFEN = 1; Enable TxE interrupt
		  LL_I2C_ClearFlag_ADDR(I2C1); // I2C_SR1.ADDR = 0;
	  }
	  // I2C_SR1.TXE = 1 && I2C_SR1.BTF(byte transfer finished)
	  else if(LL_I2C_IsActiveFlag_TXE(I2C1) && !LL_I2C_IsActiveFlag_BTF(I2C1))
	  {
		  i2c_irq_Tx++;

		  if ( ( (srm_narrow.i2c_send_pair->byte_msg_lenght) - (send_dr)) > 0)    // address byte transmitted
		  {
			  send_dr++;
			  if (srm_narrow.i2c_send_pair->byte_msg_lenght == 1)
			  {
				  srm_narrow.i2c_send_pair->sended_byte = I2C_TRANSMITE_IN_PROGRESS;
				  if (srm_narrow.status.bit.command == ENGINE_SWITCH_ON)
				  {

					I2C1->DR = srm_narrow.i2c_send_pair->dr;
					//TIM3->CR1 |= 0x1; 	// run TIM3 ???? it start before in

					//unsigned tmp_cr = TIM3->CR1;
					//(void)tmp_cr;
				  }
				  else
				  {
					  I2C1->DR = 255; // switch off
					  //TIM2->CR1 &= 0xFFFFFFFE; // disable SPI transmit for sync start with TIM3
					  TIM3->CR1 &= 0xFFFFFFFE; // disable cooldawn timer
				  }

			  }
			  else
			  {
				  I2C1->DR = (char)(srm_narrow.i2c_send_pair->dr >> 8UL*(srm_narrow.i2c_send_pair->byte_msg_lenght - send_dr));
			  }

		  }

	  }
	  else if (LL_I2C_IsActiveFlag_BTF(I2C1) && LL_I2C_IsActiveFlag_TXE(I2C1) )
	  {
		  LL_I2C_DisableIT_BUF(I2C1); // I2C_CR2.ITBUFEN = 0
		  LL_I2C_GenerateStopCondition(I2C1); // I2C_CR1.STOP = 1
		  I2C1->CR1 &= 0xFFFFFFFE; // I2C_CR1.PE = 0

		  if (srm_narrow.i2c_send_pair->byte_msg_lenght == 1)
		  {
			  if (srm_narrow.status.bit.command == ENGINE_STOP)
					 //&& ++i2c_busy_cnt / 2 == 1) // each stop condition of engine stop command()
			  {
				  srm_narrow.i2c_send_pair->sended_byte = 0xFF;

				  srm_narrow.switch_par.phase_pulse_count++;

				  i2c_busy_cnt = 0;

				  // resave angle
				  //srm_narrow.angle.prev_spi_val = srm_narrow.angle.spi_val;
				  if (srm_narrow.angle.spi_val == srm_narrow.angle.prev_spi_val)
				  {

					  ++(srm_narrow.angle.match_angle_cnt);
					  // if after max not moved pulses rotor not moved switch the phase
					  if (srm_narrow.status.bit.direction == ROTATION_STOP)

					  {
						  if( srm_narrow.angle.match_angle_cnt == MAX_NOT_MOVED_PULSE_COUNT)
						  // stop switching
							  srm_narrow.switch_par.phase_pulse_count = srm_narrow.switch_par.tst_max_pulse_count;
					  }
					  else	// after start rotation count the pulse on the angle position
					  {
						  if(srm_narrow.angle.match_angle_cnt > srm_narrow.angle.max_match_angle_cnt)
						  {

							  srm_narrow.angle.aligned_val = srm_narrow.angle.prev_spi_val;
						  	  srm_narrow.angle.max_match_angle_cnt = srm_narrow.angle.match_angle_cnt;

						  }
					  }
				  }
				  // case than angle was change
				  else
				  {
					  srm_narrow.angle.match_angle_cnt = 0;
					  if (srm_narrow.status.bit.direction == ROTATION_STOP)
					  {
						  srm_narrow.status.bit.direction = DirectionDefinder(srm_narrow.angle.prev_spi_val,
						 																	  srm_narrow.angle.spi_val, 50);
						  // made another 10 pulse after start rotation
						  if (srm_narrow.status.bit.direction != ROTATION_STOP)
						  {
							 // save count of pulse before count
							  srm_narrow.switch_par.phase_pulse_count_before_moving =
									  	  	  	  srm_narrow.switch_par.phase_pulse_count;


							  srm_narrow.angle.prev_spi_val = srm_narrow.angle.spi_val; // ?
						  }
					  }
					  else
						  srm_narrow.angle.prev_spi_val = srm_narrow.angle.spi_val;
				  }
				  // it can be MACRO
				  TIM3->CCR1 = PWM_InActiveHalfPeriod(srm_narrow.switch_par.phase_working_time,
						  	  	  	  	  	  	  	  TIM3->CCR1);

				  srm_narrow.status.bit.st1_dac_high_falling = 0;
				  srm_narrow.status.bit.st2_dac_high_falling = 0;

				  if (usr_button_prs == 0) srm_narrow.status.bit.mode = MODE_STAND_BY;
			  }

			  // reset counter of timer at end of packet transaction
			  if (srm_narrow.status.bit.command == ENGINE_SWITCH_ON) // && ++i2c_busy_cnt / 2 == 1
			  {
				  srm_narrow.i2c_send_pair->sended_byte = srm_narrow.ptr_to_switch->i2c_pair.dr;

				  // reset "cooldawn" timer (for correct time count)
				 // TIM3->CNT = 0;

			  }

			  // update CCR1 (EGR) and start cooldown timer
			  TIM3->EGR |= 1; // generate UIF interrupt
			  TIM3->CR1 |= 1;


		  }
		  else // mean lenght = 2. DAC set border
		  {
			  TIM3->CR1 &= 0xFFFFFFFE; 		// CEN = 0. Disable timer if cmp_irq

			  if (srm_narrow.i2c_send_pair == &srm_narrow.i2c_pair_current_sensor_low)
			  {
				  srm_narrow.status.bit.dac_low_set = 1;
			  }
			  if (srm_narrow.i2c_send_pair == &srm_narrow.i2c_pair_current_sensor_high)
			  {
				  srm_narrow.status.bit.dac_high_set = 1;
				  TIM4->CR1 &= 0xFFFFFFFE;
			  }

			  // start SPI transfer after first DAC set
			  if ((TIM2->CR1 & 0x1) == 0
					  && srm_narrow.status.bit.dac_high_set == 1)
			  {
				  TIM2->CR1 |= 0x1; // CEN - clock enable, SPI Tx timer
			  }
		  }

		  // new start of SPI transmit
		  TIM2->EGR |=0x1;
		  TIM2->CR1 |=0x1;

		  send_dr = 0; // reset for next transaction

	  }
}
/**
  * @}
  */

/**
  * @}
  */
