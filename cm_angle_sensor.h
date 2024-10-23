/*
 * cm_angle_sensor.h
 *
 *  Created on: Mar 7, 2023
 *      Author: const
 */

#ifndef EXAMPLE_USER_CM_SRM_GLOBAL_DEFINE_H_

#include "C:\Users\const\STM32CubeIDE\workspace_1.7.0\SPI_TwoBoards_FullDuplex_IT\SW4STM32\STM32F411-Nucleo\Example\User\cm_srm_global_define.h"
#endif

#ifndef EXAMPLE_USER_CM_ANGLE_SENSOR_H_
#define EXAMPLE_USER_CM_ANGLE_SENSOR_H_

void Configure_TIM2(void);
void TIM2_callBack(void);

void Configure_SPI1(void);
void SPI1_Rx_Callback(void);

void SPI1_TransferError_Callback(void);

unsigned DirectionDefinder(unsigned prev_angle, unsigned angle, unsigned rotation_sign);
void Tmp_aligned_tbl_Init(union aligned_pos *tbl, unsigned tbl_size);

#endif /* EXAMPLE_USER_CM_ANGLE_SENSOR_H_ */
