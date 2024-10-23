/*
 * switch_IGBT.c
 *
 *  Created on: Mar 10, 2023
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

#include "cm_srm_global_define.h"
#include "cm_srm_switch_IGBT.h"

extern struct motor_t srm_narrow;
extern unsigned tst_cnntr;

// function prototype
void I2C1_Tx();

void Configure_TIM4(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	// looks like - UDIS (Update Disable)
	TIM4->CR1 |= (0x1 << 7); // set ARPE = 1, that means - reload direct to shadow reg of cnt task
	//TIM2->CR1 |= (0x1 << 3); // set OPM(one pulse mode) = 1,

	TIM4->CR2 = 0x0; // CR2 only about slave and synch mode(use default)
	TIM4->SMCR = 0x0; // Slave mode control register
	TIM4->DIER = 0x1; // only UIE (Update Interrupt Enable) set

	TIM4->PSC = 43; // frequency of timer clock  = Fck_psc / 32
	TIM4->ARR = 150; // means period = 40us, equal 1kHz

	// generate update event (otherwise PSC and ARR not working)
	TIM4->EGR |= 0x1;

	NVIC_SetPriority(TIM4_IRQn, 10); // previous default
	// Enable TIM2_IRQ
	NVIC_EnableIRQ(TIM4_IRQn);
}

extern unsigned i2c_req_cnt;
void TIM4_CallBack(void) // i2c Tx timer
{

	// for observe window of I2C send
/*	if ( !(GPIOA->ODR & 0x40)  ) // if before was
		GPIOA->BSRR |= (0x1 << 6); // set PA6
	else
	    GPIOA->BSRR |= (0x1 << 22); // set PA6
*/

	if (srm_narrow.status.bit.dac_low_set == 0)
	{
		srm_narrow.i2c_send_pair = &srm_narrow.i2c_pair_current_sensor_low;
		//srm_narrow.status.bit.dac_low_set = 1;

		srm_narrow.status.bit.i2c_Tx_req = 1;
	}
	else if(srm_narrow.status.bit.dac_high_set == 0)
	{
		srm_narrow.i2c_send_pair = &srm_narrow.i2c_pair_current_sensor_high;
		//srm_narrow.status.bit.dac_high_set = 1;

		srm_narrow.status.bit.i2c_Tx_req = 1;
	}

/*
	  srm_narrow.i2c_send_pair = &srm_narrow.i2c_pair_current_sensor_high;
 */

    // looks like legacy
	if (srm_narrow.status.bit.i2c_Tx_req == 1)
	{
		//if(srm_narrow.switch_par.phase_pulse_count < MAX_PULSE_COUNT)
		if((srm_narrow.status.bit.command == ENGINE_SWITCH_ON && (TIM3->CR1 & 0x1) == 0)
				|| srm_narrow.status.bit.command == ENGINE_STOP)
		{
			/*
			if ( !(GPIOA->ODR & 0x80)  ) // if before was
				  GPIOA->BSRR |= (0x1 << 7); // set PA7
			  else
				  GPIOA->BSRR |= (0x1 << 23); // set PA7
			*/
			I2C1_Tx();
			// reset request after execution
			srm_narrow.status.bit.i2c_Tx_req = 0;
		}
	}


	// check Pending and reset
	if (TIM4->SR & 0x1) // check UIF flag
	{
		TIM4->SR &= 0x0; // reset UIF
		//tim2_tst_cnt++;
	}

	unsigned tim4_cnt = TIM4->CNT;
	(void)tim4_cnt;
}

void Configure_TIM3(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	// looks like - UDIS (Update Disable)
	TIM3->CR1 |= (0x1 << 7); // set ARPE = 1, that means - reload direct to shadow reg of cnt task

	TIM3->CR2 = 0x0; // CR2 only about slave and synch mode(use default)
	TIM3->SMCR = 0x0; // Slave mode control register
	// UIE (Update Interrupt Enable), CC1IE and CC2IE (Capture/Compare interrupt enable) set
	TIM3->DIER |= (0x1) | (1 << 1) | (1 << 2);

	TIM3->PSC = 43; // frequency of timer clock  = Fck_psc / 32 //43 // 86
	TIM3->ARR = MAX_UPDATE_TIME; // means period = 1000us, equal 1kHz

	//TIM3->CCR1 = TIM3->ARR + CCR1_SHIFT_CASE_END_CCR1; // means, that it's work only after ARR
	//TIM3->CCR1 = TIM4->ARR + 200;
	TIM3->CCR1 = TIM3_DEFAULT_PERIOD;
	TIM3->CCMR1 |= (1 << 4); // OC1M = 1 (OC1 high when CNT = CCR1)

	TIM3->CCR2 = MAX_PHASE_WORKING_TIME_US;
	TIM3->CCMR2 |= (1 << 4); // OC2M = 1 (OC2 high when CNT = CCR2)

	TIM3->CCER |= 0x1;       // CC1 Capture Enable
	TIM3->CCER |= (1 << 4);  // CC2 Capture Enable

	// generate update event (otherwise PSC and ARR not working)
	TIM3->EGR |= 0x1;

	NVIC_SetPriority(SPI1_IRQn, 12);
	// Enable TIM2_IRQ
	NVIC_EnableIRQ(TIM3_IRQn);

}


extern unsigned tim3_ccr1_cnt, tim3_arr_cnt, usr_button_prs;

void TIM3_CallBack(void)
{
	if(TIM3->SR & 0x1) // UIF underflow/update interrupt force CNT = ARR
	{

		TIM3->SR &= 0x0; // reset UIF
	}

	else if(TIM3->SR & 0x2) // CC1IF Capture Compare 1 interrupt force CNT = CCR1
	{

		tim3_ccr1_cnt++;
		// end of "cooldawn" after CCR1 || CMP || ARR interrupt
		if(srm_narrow.status.bit.command == ENGINE_STOP
				&& srm_narrow.i2c_send_pair->sended_byte == 0xFF)
		{

			TIM3->CR1 &= 0xFFFFFFFE; 		// CEN = 0. Disable timer if cmp_irq

			unsigned tmp2 = TIM3->CR1;
			(void)tmp2;

			if (srm_narrow.status.bit.mode == MODE_SET_AMGLE)
				EngineAngleMode();
			if (srm_narrow.status.bit.mode == MODE_INFINITY_CONST_PWM)
				EngineInfinityConstPwm();


		} // for normal switching off
		else if (srm_narrow.status.bit.command == ENGINE_SWITCH_ON
				&& srm_narrow.i2c_send_pair->sended_byte != 0xFF)
		{
			TIM3->CR1 &= 0xFFFFFFFE; 		// CEN = 0. Disable timer if cmp_irq

			srm_narrow.status.bit.command = ENGINE_STOP;

			// NOP
			unsigned tmp_com = srm_narrow.status.bit.command;
			(void)tmp_com;

			//srm_narrow.status.bit.i2c_Tx_req = 1;
			/*
			 if ( !(GPIOA->ODR & 0x80)  ) // if before was
				  GPIOA->BSRR |= (0x1 << 7); // set PA7
			  else
				  GPIOA->BSRR |= (0x1 << 23); // set PA7
			*/
			//TIM4->EGR |= 0x1; // async i2c transmit
			I2C1_Tx();

		}
		//TIM3->CNT = 0; // ?????????????????????????????

		TIM3->SR &= 0x0; 				// reset CC1IF
	}
	else if (TIM3->SR & 0x4) // CC2IF Capture Compare 2 interrupt force CNT = CCR2 max working time reached
	{
		srm_narrow.status.bit.command = ENGINE_STOP;

		// NOP
		unsigned tmp_com = srm_narrow.status.bit.command;
		(void)tmp_com;

		I2C1_Tx();

		TIM3->SR &= 0x0; 				// reset CC2IF
	}
}

void Configure_I2C1(void)
{
	// Start from config the GPIO
	if ( !LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOB) )
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // RCC_AHB1ENR.GPIOBEN = 1;

	if (!LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOA))
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	// configure I2C1_SCL
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_4);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

	// configure I2C1_SDA
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_4);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);


	// Configuration the peripheral I2C
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	NVIC_SetPriority(I2C1_EV_IRQn, 11); // 0
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	/* Configure Error IT:
	*  - Set priority for I2C1_ER_IRQn
	*  - Enable I2C1_ER_IRQn
	*/
	NVIC_SetPriority(I2C1_ER_IRQn, 11); // 0
	NVIC_EnableIRQ(I2C1_ER_IRQn);

	// set peripheral clock freq I2C_CR2.FREQ
	MODIFY_REG(I2C1->CR2, 0x3F, 0x2); // 2MHz
	//i2c1_cr2 = I2C1->CR2;

	// set enable the interrupt I2C_CR2.ITEVTEN = 1
	I2C1->CR2 |= (0x1 << 9);

	// set Thigh = Tlow of pulses by equation I2C_CCR.CCR
	MODIFY_REG(I2C1->CCR, 0x7FF, 0x32); // mean 100 * T peripheral clock
	//i2c1_ccr = I2C1->CCR;

	// set rise time I2C_TRISE.TRISE
	MODIFY_REG(I2C1->TRISE, 0x3F, 0x3); // it should by default (1000 ns / 50ns + 1 of Tpclk)

}

extern unsigned send_dr;

void I2C1_Tx()
{
	// set address for slave mode (add need move to function parameter)
	// MODIFY_REG(I2C1->OAR1, (0x7F << 1), I2C_LEFT_SIDE_ADDR | 0x1 );
	//i2c1_oar1 = I2C1->OAR1;

	// reset status previous transaction
	send_dr = 0;

	// pe enable I2C_CR1.PE
	MODIFY_REG(I2C1->CR1, 0x1, 0x1);

	// reset stop
	CLEAR_BIT(I2C1->CR1, (0x1 << 9)); // I2C_CR1.STOP = 0;

	// start bit in cr reg
	MODIFY_REG(I2C1->CR1, (0x1 << 8), (0x1 << 8)); // I2C_CR1.START = 1

	// set dr reg I2C_DR.DR
	//i2c_dr = I2C1->DR = (char)msg;

	//i2c1_sr2 = I2C1->SR2;
	//i2c1_sr1 = I2C1->SR1;

	//i2c1_cr2 = I2C1->CR2;
	//return 1;
}

void NextClockWisePhase(struct motor_t *m)
{
	m->ptr_to_switch = m->ptr_to_switch->phase_name == ST2R ? m->ptr_to_switch - 5: m->ptr_to_switch + 1;
}

void NextCounterClockWisePhase(struct motor_t *m)
{
	//m->ptr_to_switch = m->ptr_to_switch->phase_name == "ST1G\0" ? m->ptr_to_switch + 5: m->ptr_to_switch - 1;
	if (m->ptr_to_switch->phase_name == ST1G)
		m->ptr_to_switch += 5;
	else
		m->ptr_to_switch -= 1;
}

void NextClockWisePhase_S1(struct motor_t *m)
{
	if (m->ptr_to_switch->phase_name == ST1R)
		m->ptr_to_switch -= 4;
	else
		m->ptr_to_switch += 2;
}
void NextCounterClockWisePhase_S1 (struct motor_t *m)
{
	if (m->ptr_to_switch->phase_name == ST1G)
			m->ptr_to_switch += 4;
		else
			m->ptr_to_switch -= 2;
}

unsigned PWM_ActiveHalfPeriod(unsigned period, unsigned filling)
{
	return (filling > 50 && filling < 100 ) ? (period * filling) / 50 : period;
}

unsigned PWM_InActiveHalfPeriod(unsigned period, unsigned activePeriod)
{
	return (2 * period) - activePeriod;
}

unsigned PWM_InActiveHalfPeriod_constActive(unsigned period, unsigned filling)
{
	return (period * (100 - filling)) / filling;
}

unsigned Delta_angles(unsigned a1, unsigned a2) // need add signature to header
{
	unsigned clockwise_delta, counterclockwise_delta;

    // clockwise rotation delta
    if (a1 >= a2)
    {
        clockwise_delta = a1 - a2;
        counterclockwise_delta = (16383 - a1) + a2;
    }
    else
    {
        clockwise_delta = (16383 - a2) + a1;
        counterclockwise_delta = a2 - a1;
    }

	return (clockwise_delta > counterclockwise_delta)? counterclockwise_delta: clockwise_delta;
}

union aligned_pos tmp_alig;

void WriteAngleTo_Aligned_Tbl(struct motor_t *m)
{
	/* check delta_angles if it x < 448 & x > 480 (if aligned position not set)
	 * 	swap tmp_direction to know, which previous angle value
	 * 	move ptr_tbl pointer to previous table value
	 * 	get value of previous angle
	 * 	if previous value - INIT value - just write to current position angle value (it 1 execution)
	 * 		and switch to next phase
	 * 	if previous value not INIT value
	 * 		check the delta_angle
	 * 		if _calc delta_angles x < 448 & x > 480 not correct
	 * 			swap real direction
	 * 		else
	 * 			good value -> save it
	 *
	 * swap real direction
	 *
	 * use field err for stop rotation(!)
	 */
	int swap_direction = (m->status.bit.direction == ROTATION_CLOCKWISE) ? ROTATION_COUNTERCLOCKWISE : ROTATION_CLOCKWISE;
	// need calc all ptr
	unsigned *prev_angle_tbl = NextShiftedElement(ROTATION_SIGN_TO_SHIFT(swap_direction), // * 2,
					(unsigned*)(m->angle.ptr_tmp_aligned_tbl),
					(unsigned*)(m->angle.aligned_tbl), sizeof(m->angle.aligned_tbl) / 4);

	unsigned *next_angle_tbl = NextShiftedElement(ROTATION_SIGN_TO_SHIFT(m->status.bit.direction), // * 2,
			(unsigned*)(m->angle.ptr_tmp_aligned_tbl),
			(unsigned*)(m->angle.aligned_tbl), sizeof(m->angle.aligned_tbl) / 4);

	unsigned previos_angle_val = ((union aligned_pos*)prev_angle_tbl)->bit.val;

	// check delta of current table position
	if(m->angle.ptr_tmp_aligned_tbl->bit.phase_delta < 224 || // 447
			m->angle.ptr_tmp_aligned_tbl->bit.phase_delta > 256) // 480
	// current position don't have right value
	{
		// if previous angle was INIT dosn't make a sense calc delta
		if (previos_angle_val == INIT_ANGLE_VALUE)
		{	// just save current position
			m->angle.ptr_tmp_aligned_tbl->bit.val = m->angle.aligned_val;     //-
			m->angle.ptr_tmp_aligned_tbl->bit.phase_name = m->ptr_to_switch->phase_name;

			// move pointer to next position
			m->angle.ptr_tmp_aligned_tbl = (union aligned_pos*)next_angle_tbl;

		}
		else // ok we have previous value - calc delta
		{
			unsigned delta = Delta_angles(previos_angle_val, m->angle.aligned_val);
			if (delta >= 223 /* 447*/ && delta <= 256 /*480*/) // if delta was ok, write angle and delta
			{
				m->angle.ptr_tmp_aligned_tbl->bit.val = m->angle.aligned_val;
				m->angle.ptr_tmp_aligned_tbl->bit.phase_delta = delta;
				m->angle.ptr_tmp_aligned_tbl->bit.phase_name = m->ptr_to_switch->phase_name;

				// move pointer to next position
				m->angle.ptr_tmp_aligned_tbl = (union aligned_pos*)next_angle_tbl;

				// check next element (stop rotation condition)
				if ( ((union aligned_pos*)next_angle_tbl)->bit.phase_delta >= 224 &&
					((union aligned_pos*)next_angle_tbl)->bit.phase_delta <= 256 )
				{
					// compete rotation condition
					m->angle.ptr_tmp_aligned_tbl->bit.phase_name |= (1 << 4);

					  if ( !(GPIOA->ODR & 0x80)  ) // if before was
						  GPIOA->BSRR |= (0x1 << 7); // set PA7
					  else
						  GPIOA->BSRR |= (0x1 << 23); // set PA7
				}
			}
			else
			{
				// write current val for wrong reverse write
				m->angle.ptr_tmp_aligned_tbl->bit.val = m->angle.aligned_val;
				m->angle.ptr_tmp_aligned_tbl->bit.phase_name |= (1 << 5);

				// and need reverse direction and tbl pointer
				m->status.bit.direction = swap_direction;
				m->angle.ptr_tmp_aligned_tbl = (union aligned_pos*)prev_angle_tbl;

				// compete rotation condition


				  if ( !(GPIOA->ODR & 0x80)  ) // if before was
					  GPIOA->BSRR |= (0x1 << 7); // set PA7
				  else
					  GPIOA->BSRR |= (0x1 << 23); // set PA7
			}
		}
	}
	else // current delta ok, - mean last phase switching for repeating
	{
		// and need reverse direction and tbl pointer
		m->status.bit.direction = swap_direction;
		m->angle.ptr_tmp_aligned_tbl = (union aligned_pos*)prev_angle_tbl;
	}
}

extern unsigned tmp_spi_angle;
void EngineAngleMode()
{
	// condition for switching phase
	if(srm_narrow.status.bit.direction != ROTATION_STOP &&
	   srm_narrow.switch_par.phase_pulse_count_before_moving > 0 &&
			(srm_narrow.switch_par.phase_pulse_count == MAX_PULSE_COUNT_PER_PHASE ||
				(	srm_narrow.angle.max_match_angle_cnt >= MAX_NOT_MOVED_PULSE_COUNT  / 2 &&
				    srm_narrow.angle.max_match_angle_cnt >= srm_narrow.switch_par.phase_pulse_count_before_moving
				)
			)
	//add ROTATION STOP condition

	)
	{
		// save start condition for 1 turn rotation
	/*	static union aligned_pos *ptr_tbls_begin = &srm_narrow.angle.aligned_tbl[0];
		if (srm_narrow.switch_par.phase_switches_count == 0)
			ptr_tbls_begin = srm_narrow.angle.ptr_tmp_aligned_tbl;
		*/
		// save angle
		WriteAngleTo_Aligned_Tbl(&srm_narrow);

		// need set ptr only after success angle writing (check the angle value after first phase)
		if (++srm_narrow.switch_par.phase_switches_count < MAX_PHASE_SWITCHES &&
				// while end rotation flags will not set
				(srm_narrow.angle.ptr_tmp_aligned_tbl->bit.phase_name & ( (1 << 5) | (1 << 4) ) ) == 0)
		{
		//	if (srm_narrow.angle.ptr_tmp_aligned_tbl != ptr_tbls_begin)
		//	{

				// check the table pointer made a turn
			//switch phase
			srm_narrow.ptr_to_switch = NextSwitchedPhase(ROTATION_SIGN_TO_SHIFT(srm_narrow.status.bit.direction),// * 2,
					srm_narrow.ptr_to_switch,
					srm_narrow.switch_sequence,
					sizeof(srm_narrow.switch_sequence) / sizeof(srm_narrow.switch_sequence[0]));
/*
				if (srm_narrow.status.bit.direction == ROTATION_CLOCKWISE) NextClockWisePhase_S1(&srm_narrow);
				else if (srm_narrow.status.bit.direction == ROTATION_COUNTERCLOCKWISE) NextCounterClockWisePhase_S1(&srm_narrow);
*/
				// set I2C transmit
				srm_narrow.i2c_send_pair = &srm_narrow.ptr_to_switch->i2c_pair;

				// reset switch_par state
				srm_narrow.switch_par.phase_pulse_count = 0;
				srm_narrow.angle.match_angle_cnt = 0;
				srm_narrow.angle.max_match_angle_cnt = 0;

				srm_narrow.switch_par.phase_pulse_count_before_moving = 0;

				srm_narrow.switch_par.PWM_filling = PWM_DEFAULT_FILLING; // change to constant
				srm_narrow.switch_par.PWM_max_filling = PWM_DEFAULT_MAX_FILLING; // change to constant

				// reset rotation sign for new finding of fill
				srm_narrow.status.bit.direction = ROTATION_STOP;
		//	}
		}
	}

	// expect than srm_narrow was global variable
	// when rotation dosn't start
	if(srm_narrow.status.bit.direction == ROTATION_STOP)
	{
		// case with complete filling
		/* when condition of filling was complete and have enough pulse in this state */
		if (srm_narrow.switch_par.PWM_filling == srm_narrow.switch_par.PWM_max_filling
			&& (srm_narrow.switch_par.phase_pulse_count % PWM_PULSE_COUNT_PER_FILLING) == (PWM_PULSE_COUNT_PER_FILLING - 1) )
		{
			// set default filling setting
			srm_narrow.switch_par.PWM_filling = PWM_DEFAULT_FILLING; // change to constant
			srm_narrow.switch_par.PWM_max_filling = PWM_DEFAULT_MAX_FILLING; // change to constant

			PWM_INCRISE_PERIOD(srm_narrow.switch_par.phase_working_time, 10); // us

		}
	}
	// update filling after some pulses
	if (srm_narrow.switch_par.PWM_filling < srm_narrow.switch_par.PWM_max_filling //80 60 54 for half aligned state
			  && srm_narrow.switch_par.phase_pulse_count % PWM_PULSE_COUNT_PER_FILLING == 0)

	{
		PWM_FILLING_STEP_UP(srm_narrow.switch_par.PWM_filling, 2);
	}


//	if (srm_narrow.switch_par.phase_pulse_count < srm_narrow.switch_par.tst_max_pulse_count)
	if( (srm_narrow.status.bit.direction == ROTATION_STOP &&
			srm_narrow.switch_par.phase_pulse_count < MAX_NOT_MOVED_PULSE_COUNT)
			||
		(srm_narrow.status.bit.direction != ROTATION_STOP &&
			// max pulse in phase without switch
			(srm_narrow.switch_par.phase_pulse_count < MAX_PULSE_COUNT_PER_PHASE &&
				// count of match angle must be less than
				(srm_narrow.angle.max_match_angle_cnt < srm_narrow.switch_par.phase_pulse_count_before_moving ||
						srm_narrow.angle.max_match_angle_cnt < MAX_NOT_MOVED_PULSE_COUNT  / 2
					/*	( srm_narrow.angle.max_match_angle_cnt > srm_narrow.switch_par.phase_pulse_count_before_moving &&
						srm_narrow.angle.max_match_angle_cnt > MAX_NOT_MOVED_PULSE_COUNT  / 2) */
				)
			)
	    )
	   )
	{
		TIM3->CCR1 = PWM_ActiveHalfPeriod(srm_narrow.switch_par.phase_working_time,
											srm_narrow.switch_par.PWM_filling);

		srm_narrow.status.bit.command = ENGINE_SWITCH_ON;
		I2C1_Tx();
	}


}

void EngineInfinityConstPwm()
{
	if (usr_button_prs > 0 )
			//&& srm_narrow.switch_par.phase_pulse_count < srm_narrow.switch_par.tst_max_pulse_count)
	{
		TIM3->CCR1 = PWM_ActiveHalfPeriod(srm_narrow.switch_par.phase_working_time,
											srm_narrow.switch_par.PWM_filling);

		srm_narrow.status.bit.command = ENGINE_SWITCH_ON;
		//try without NOP
/*		unsigned tmp_com = srm_narrow.status.bit.command;
		(void)tmp_com;
*/
		TIM3->CR1 |= 0x1; 	// run TIM3
		I2C1_Tx();
	}
}

unsigned * NextShiftedElement(int shift, unsigned *ptr_a, unsigned a[], int size)
{
    // check next and preious element
    if (shift > 0)
    {
        for(int i = 0; i < shift; i++)
            if (ptr_a++ == &a[size - 1])
                ptr_a = &a[0];
    }
    else
    {
        for (int i = 0; i > shift; i--)
            if(ptr_a-- == &a[0])
                ptr_a =&a[71];
    }
    return ptr_a;
}


struct phase_t* NextSwitchedPhase(int shift, struct phase_t *p_ph, struct phase_t arr_ph[], int size)
{
    // return pointer to next phase swithes
        if (shift > 0)
    {
        for(int i = 0; i < shift; i++)
            if (p_ph++ == &arr_ph[size - 1])
                p_ph = &arr_ph[0];
    }
    else
    {
        for (int i = 0; i > shift; i--)
            if(p_ph-- == &arr_ph[0])
                p_ph =&arr_ph[size-1];
    }
    return p_ph;
}

