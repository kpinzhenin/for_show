/*
 * cm_angle_sensor.c
 *
 *  Created on: Mar 7, 2023
 *      Author: const
 */

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_i2c.h"

#include "cm_angle_sensor.h"
#include "cm_srm_global_define.h"
#include "cm_srm_switch_IGBT.h"


extern struct motor_t srm_narrow;
extern unsigned tst_cnntr;

void Configure_TIM2(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	// looks like - UDIS (Update Disable)
	TIM2->CR1 |= (0x1 << 7); // set ARPE = 1, that means - reload direct to shadow reg of cnt task
	//TIM2->CR1 |= (0x1 << 3); // set OPM(one pulse mode) = 1,

	TIM2->CR2 = 0x0; // CR2 only about slave and synch mode(use default)
	TIM2->SMCR = 0x0; // Slave mode control register
	TIM2->DIER = 0x1; // only UIE (Update Interrupt Enable) set

	TIM2->PSC = 32; // frequency of timer clock  = Fck_psc / 64
	TIM2->ARR = 55; // means period = 1000us, equal 1kHz / 100

	//TIM2->CR1 = (0x1 << 2); // set URS = 1, Update request source (only from overflow/ underflow)

	// generate update event (otherwise PSC and ARR not working)
	TIM2->EGR |= 0x1;

	NVIC_SetPriority(SPI1_IRQn, 10); // 0
	// Enable TIM2_IRQ
	NVIC_EnableIRQ(TIM2_IRQn);
}
extern unsigned usr_button_prs;

void TIM2_callBack(void)
{

	static short unsigned TIM2_frst_start = 0;
	// start SPI request
	if (TIM2_frst_start == 1)
	{
		LL_SPI_Enable(SPI1);
		if( !(SPI1->SR & (1UL << 7UL)) )  // if not busy(previous transaction must be complete)
			SPI1->DR = srm_narrow.angle.spi_command;
		else
			LL_SPI_Disable(SPI1);
	}
	else TIM2_frst_start = 1;


	//static unsigned default_period = TIM3_DEFAULT_PERIOD;// + 250; // TIM4->ARR + 250
	static unsigned TIM3_start = 0; // flag from first start of TIM3
/*// uncomment for infinity mode
	if (usr_button_prs == 0 && TIM3_start == 1 )
		TIM3_start = 0;
*/
	// make decision about pulse on phase
	if ( (srm_narrow.status.bit.mode == MODE_SET_AMGLE || srm_narrow.status.bit.mode == MODE_INFINITY_CONST_PWM )
			&& TIM3_start == 0
			// cooldawn was over
			&& (TIM3->CR1 & 0x1) == 0
			// stop command was compete send
			&& srm_narrow.status.bit.command == ENGINE_STOP
			&& srm_narrow.i2c_send_pair->sended_byte == 0xFF) // !!!
			// made pulses less than task
			//&& srm_narrow.switch_par.phase_pulse_count < srm_narrow.switch_par.tst_max_pulse_count)
	{

		// set next working time
		TIM3->CCR1 = srm_narrow.switch_par.phase_working_time;
		//TIM3->CNT = 0; // ???????????????????????????????
		// for observe window of I2C send
		if ( !(GPIOA->ODR & 0x40)  ) // if before was
			GPIOA->BSRR |= (0x1 << 6); // set PA6
		else
		    GPIOA->BSRR |= (0x1 << 22); // set PA6

		// first start, to start TIM3
		srm_narrow.status.bit.command = ENGINE_SWITCH_ON;

		unsigned tmp_com = srm_narrow.status.bit.command;
		(void) tmp_com;

		I2C1_Tx();
		TIM3_start = 1;
	}

	if(usr_button_prs > 0 && srm_narrow.status.bit.mode == MODE_TEST_IGBT)
	{
		usr_button_prs = 0;


		if (srm_narrow.status.bit.command == ENGINE_STOP &&
				srm_narrow.i2c_send_pair->sended_byte == 0xFF)
		{
			srm_narrow.status.bit.command = ENGINE_SWITCH_ON;
		}

		if (srm_narrow.status.bit.command == ENGINE_SWITCH_ON &&
				srm_narrow.i2c_send_pair->sended_byte != 0xFF)
		{
			srm_narrow.status.bit.command = ENGINE_STOP;
		}

		I2C1_Tx();
	}

	// check Pending and reset
	if (TIM2->SR & 0x1) // check UIF flag
	{
		TIM2->SR &= 0x0; // reset UIF
	}
	unsigned tim2_cnt = TIM2->CNT;
	(void)tim2_cnt;

}

/**
  * @brief  This function configures SPI1.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI1 pins.
  *        -2- Configure NVIC for SPI1.
  *        -3- Configure SPI1 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_SPI1(void)
{
  /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // RCC_AHB1ENR.GPIOBEN = 1;

  /* Configure SCK Pin connected to CN10-31 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); //LL_GPIO_PULL_NO

  /* Configure MISO Pin connected to CN10-27 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_5);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO); // LL_GPIO_PULL_NO

  /* Configure MOSI Pin connected to CN7-29 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); // LL_GPIO_PULL_NO

  /* Configure NSS Pin PA4, CN7-32 mb need to switch to open-drain type*/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_4, LL_GPIO_AF_5);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_UP); //LL_GPIO_PULL_NO

  /* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
  /* Set priority for SPI1_IRQn */
  NVIC_SetPriority(SPI1_IRQn, 9); // 0
  /* Enable SPI1_IRQn           */
  NVIC_EnableIRQ(SPI1_IRQn);

  /* (3) Configure SPI1 functional parameters ********************************/

  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1); // RCC_APB2ENR.SPI1EN = 1;

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV128); //SPI1_CR1.BR = 111; F_per_ckock / 256
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX); // SPI1_CR1.RXONLY & BIDIOE & BIDIMODE = 0,
  //LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE); // SPI1_CR1.CPHA = 0
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE); // SPI1_CR1.CPHA = 1
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW); // SPI1_CR1.CPOL = 0
  //LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH); // SPI1_CR1.CPOL = 1
  /* Reset value is LL_SPI_MSB_FIRST */
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT); // SPI1_CR1.DFF = 1

  //LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT); // SPI1_CR1.SMM = 1; SPI1_CR2.SSOE = 0?????;
  MODIFY_REG(SPI1->CR1, SPI_CR1_SSM,  0UL << 9); // SSM = 0
  MODIFY_REG(SPI1->CR2, SPI_CR2_SSOE,  1UL << 2); // SSOE = 1

  //MODIFY_REG(SPI1->CR1, SPI_CR1_SSI,  1UL << 8); // SSI = 1 (CS - high)
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER); // SPI1_CR1.MSTR = 1; SPI1_CR1_SSI = 1;

  /* Configure SPI1 transfer interrupts */
  /* Enable RXNE  Interrupt             */
  LL_SPI_EnableIT_RXNE(SPI1); // generate interrupt when Rx buffer not empty. SPI1_SR.RXNE = 1
  /* Enable TXE   Interrupt             */
 // LL_SPI_EnableIT_TXE(SPI1);  // generate interrupt when Tx buffer empty. SPI1_SR.TXE = 1
  /* Enable Error(any error) Interrupt             */
  LL_SPI_EnableIT_ERR(SPI1);

  unsigned spi_cr1 = SPI1->CR1;
  (void)spi_cr1;
}


/**
  * @brief  Function called from SPI1 IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte from SPI lines.
  * @param  None
  * @retval None
  */
void  SPI1_Rx_Callback()
{
  /* Read character in Data register.
  RXNE flag is cleared by reading data in DR register */
  //WRITE_REG(GPIOA->BSRR, (1UL << 6));	// set PA6

  unsigned dr = SPI1->DR; // must be include to motor structure
 //if ( !(dr & 0xC000) )   // except packet with error (14-bit)
  if (dr != 0xFFFF)
  {
	  srm_narrow.angle.spi_val = dr & 0x3FF0;//0x3FE0; // except 5 low bit of received msg 0xFFE0
	  srm_narrow.angle.spi_command = 0xFFFF;
  }
  else
  {
  	  srm_narrow.angle.spi_error_value++;
  	  srm_narrow.angle.spi_command = 0x4001; // reset error

  }

  if (srm_narrow.angle.prev_spi_val == INIT_ANGLE_VALUE
		  || srm_narrow.angle.prev_spi_val == 0)
	  srm_narrow.angle.prev_spi_val = srm_narrow.angle.spi_val;

  LL_SPI_Disable(SPI1); // for set CS(NSS) to high state
}

void SPI1_TransferError_Callback(void)
{
  /* Disable RXNE  Interrupt             */
  LL_SPI_DisableIT_RXNE(SPI1);

  /* Disable TXE   Interrupt             */
  LL_SPI_DisableIT_TXE(SPI1);

  /* Set LED2 to Blinking mode to indicate error occurs */
  //LED_Blinking(LED_BLINK_ERROR);
}


unsigned DirectionDefinder(unsigned prev_angle, unsigned angle, unsigned rotation_sign)
{
	/* 1 LSB of AS5048A = 0.02 degree
	 * deside - sign of rotation it's change value more than 20 LSB - 0.4 degree.
	 * If position will change more than 20 - deside about rotation
	 */
	    //unsigned delta_clockwise = 16383 - prev_spi_angle;

    unsigned clockwise_delta, counterclockwise_delta;

    // clockwise rotation delta
    if (angle >= prev_angle)
    {
        clockwise_delta = angle - prev_angle;
        counterclockwise_delta = (16383 - angle) + prev_angle;
    }
    else
    {
        clockwise_delta = (16383 - prev_angle) + angle;
        counterclockwise_delta = prev_angle - angle;
    }

    //printf("\n prev_angle = %u angle = %u clockwise_delta = %u, counterclockwise = %u \n", prev_angle, angle, clockwise_delta, counterclockwise_delta);
    if (clockwise_delta <= counterclockwise_delta)
    {
        if (clockwise_delta > rotation_sign) { prev_angle = angle; return ROTATION_CLOCKWISE;}
        else return ROTATION_STOP;
    }
    else
    {
        if (counterclockwise_delta > rotation_sign) { prev_angle = angle; return ROTATION_COUNTERCLOCKWISE;}
        else return ROTATION_STOP;
    }
}

void Tmp_aligned_tbl_Init(union aligned_pos *tbl, unsigned tbl_size)
{
	for (int i = 0; i < tbl_size; i++)
	{
		//*tbl++ = INIT_ANGLE_VALUE;
		//tbl->bit.phase_name = i;
		tbl++->bit.val = INIT_ANGLE_VALUE;

	}
}
/*
void  SPI1_Tx_Callback(void)
{

}
*/
