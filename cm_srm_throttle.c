/*
 * cm_srm_throttle.c
 *
 *  Created on: 20 мар. 2023 г.
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

unsigned Convert_currentA_to_DAC_task(unsigned current_task)
{
	/* max DAC's task = 99 * 10 * 4 = 3960LSB(4.752V).
	that match to 24.9 * 4.752 = 118A max scope current. (calc from sensors measurment)
	so, in this case we have eq (118 / current_task = 3960 / dac_task)
	hence, dac_task = (3960 * current_task) / 118, for sefety 3960 / 118 = 33
	*/
	return (current_task > 124) ? 4092 : current_task * 33;

}

void DAC_set(unsigned task, unsigned offset)
{
	// task must be in range of 0...4095
	if (task > 4095) task = 4095;

	srm_narrow.i2c_pair_current_sensor_high.dr = (offset > task) ? offset : task;
	srm_narrow.i2c_pair_current_sensor_low.dr = (offset > task) ? 0 : task - offset;

}
