/*
 * switch_IGBT.h
 *
 *  Created on: Mar 10, 2023
 *      Author: const
 */

#ifndef EXAMPLE_USER_CM_SRM_SWITCH_IGBT_H_
#define EXAMPLE_USER_CM_SRM_SWITCH_IGBT_H_

#ifndef EXAMPLE_USER_CM_SRM_GLOBAL_DEFINE_H_
#include "cm_srm_global_define.h"
#endif


//#define nextClockwisePhase(X) ( ( X->phase_name == "ST2R\0" ) ? X - 5 : X + 1)
//#define nextCounterClockwisePhase(X) ( ( X->phase_name == "ST1G\0") ? X + 5 : X - 1)

void Configure_I2C1(void);
void I2C1_Tx();

//phase working time
void Configure_TIM3(void);
void TIM3_CallBack(void);

// additional start timer (for arbitrary I2C bus )
void Configure_TIM4(void);
void TIM4_CallBack(void);

void NextClockWisePhase(struct motor_t *m);
void NextCounterClockWisePhase(struct motor_t *m);

void NextClockWisePhase_S1(struct motor_t *m);
void NextCounterClockWisePhase_S1 (struct motor_t *m);

// calc PWM parameters
unsigned PWM_ActiveHalfPeriod(unsigned period, unsigned filling);
unsigned PWM_InActiveHalfPeriod(unsigned period, unsigned activePeriod);

#define PWM_FILLING_STEP_UP(F,S) ( ( ((F) + (S)) < 80) ?  /* max pwm filling - 80%*/ \
								(F) += (S) \
								: ( (F) = 80))

#define PWM_FILLING_STEP_DOWN(F,S) ( ( ((F) - (S)) > 50) ? /*min pwm filling - 50%*/\
									(F) -= (S) \
									: ( (F) = 50))

// Need add limits constant ( 200 need to change to MAX_PHASE_WORKING_TIME_US)
#define PWM_INCRISE_PERIOD(PERIOD,STEP) ( ((PERIOD) + (STEP) < 300) ? \
											(PERIOD) += (STEP) \
											: ( (PERIOD) = 300))

#define ROTATION_SIGN_TO_SHIFT(RS) ( (RS) - 2 ) // (-2 + (RS) )

// Engine mode
void EngineAngleMode();
void EngineInfinityConstPwm();

// set struct parametr
void WriteAngleTo_Aligned_Tbl(struct motor_t *m);

// switch to next clock/counterclock element
unsigned * NextShiftedElement(int shift, unsigned *ptr_a, unsigned a[], int size);

struct phase_t* NextSwitchedPhase(int shift, struct phase_t *p_ph, struct phase_t arr_ph[], int size);

#endif /* EXAMPLE_USER_CM_SRM_SWITCH_IGBT_H_ */
