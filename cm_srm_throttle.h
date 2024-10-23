/*
 * cm_srm_throttle.h
 *
 *  Created on: 20 мар. 2023 г.
 *      Author: const
 */

#ifndef EXAMPLE_USER_CM_SRM_THROTTLE_H_
#define EXAMPLE_USER_CM_SRM_THROTTLE_H_

unsigned Convert_currentA_to_DAC_task(unsigned current_task);
void DAC_set(unsigned task, unsigned offset);


#endif /* EXAMPLE_USER_CM_SRM_THROTTLE_H_ */
