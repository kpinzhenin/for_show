/*
 * cm_srm_current_sensor.c
 *
 *  Created on: Mar 14, 2023
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

