/*
 * cm_srm_global_define.h
 *
 *  Created on: Mar 9, 2023
 *      Author: const
 */

#ifndef EXAMPLE_USER_CM_SRM_GLOBAL_DEFINE_H_
#define EXAMPLE_USER_CM_SRM_GLOBAL_DEFINE_H_

// PCF8474A address
#define I2C_LEFT_SIDE_ADDR 0x38
#define I2C_RIGHT_SIDE_ADDR 0x3C //

// mcp4725 address
#define I2C_DAC_LOW_TRESHOLD 0x60
#define I2C_DAC_HIGH_TRESHOLD 0x61

// switch on both hands - high and low
#define PCF8574A_P4_P5_SET_0 207 //0xCF
#define PCF8574A_P0_P1_SET_0 252 //0xFC
#define PCF8574A_P2_P3_SET_0 243 //0xF3
#define I2C_TRANSMITE_IN_PROGRESS 0xFFF //
#define I2C_TRANSACTION_TIME 150

#define ENGINE_STOP 0			// stop command to switch off IGBT
#define ENGINE_SWITCH_ON 1		// start command to switch on IGBT

// default value of variable
#define INIT_VAL_CURRENT_SENSOR 50 // 25 Amper it no current, it DAC value
#define INIT_ANGLE_VALUE 17000
/*
// determine previous event of TIM3
#define CCR1_SHIFT_INIT_STATE 100    // shift for the CCR1 Reg, previous state - IRQ from CMP
#define CCR1_SHIFT_CASE_END_CCR1 150 // shift for the CCR1 reg, previous state - IRQ from TIM3.CCR1
#define CCR1_SHIFT_CASE_CMP_IRQ 200  // shift for the CCR1 Reg, previous state - IRQ from CMP
#define CCR1_SHIFT_CASE_END_ARR 10   // negative shift in and of ARR
*/
#define TIM3_DEFAULT_PERIOD 90 //100      // start period of PWM (TIM3->CCR1)
#define PWM_PULSE_COUNT_PER_FILLING 10 // pulse in one filling state
#define PWM_DEFAULT_FILLING 50        // start filling of pulse
#define PWM_DEFAULT_MAX_FILLING 72 //80    // max filling of pulse

// limits
#define MAX_NOT_MOVED_PULSE_COUNT 200 //150 // max pulse per phase
#define MAX_PULSE_COUNT_PER_PHASE 4800// max pulse for finding aligned position
#define MAX_PHASE_SWITCHES 80          // max changes of phase
#define MAX_UPDATE_TIME 1000		  // max time when TIM3 will update
#define MAX_PHASE_WORKING_TIME_US 600 // max switching on time // 600
#define MAX_ANGLE_MATCH_IN_ALIGNED_POSITION 300 // sign of reaching the aligned position

// current status of shaft
#define ROTATION_STOP 0x2
#define ROTATION_CLOCKWISE 0X3
#define ROTATION_COUNTERCLOCKWISE 0X1

#define MODE_STAND_BY 0
#define MODE_RUN 1
#define MODE_SET_AMGLE 2
#define MODE_TEST_IGBT 3
#define MODE_INFINITY_CONST_PWM 4

// phase name
#define ST1G 0
#define ST2G 1
#define ST1Y 2
#define ST2Y 3
#define ST1R 4
#define ST2R 5

typedef struct
{
	unsigned add;
	unsigned dr;
	unsigned const byte_msg_lenght;

	unsigned sended_byte;
}i2c_pair_t;

struct phase_t
{
	const unsigned phase_name;
	i2c_pair_t i2c_pair;

	//const unsigned add_i2c;
	//unsigned switch_on_command;

};

typedef struct
{
	unsigned val :16; 				   // aligned position val
	unsigned phase_delta :10; 		   // delta between previous phase
	unsigned phase_name :6;			   // aligned phase name

}angle_aligned; // aligned_pos

union aligned_pos
{
	angle_aligned bit;
	unsigned all;
};

typedef struct
{
	unsigned spi_val;                  // value received from SPI
	unsigned prev_spi_val;             // previous save angle's value

	// next section for determine aligned position and safe lookup table
	unsigned match_angle_cnt; 		   // count of match between a previous and current SPI value
	unsigned max_match_angle_cnt;      // max count of pulse on such position (aligned point have more pulses without moving then other)
	unsigned aligned_val;              // value of SPI in point which have a "max_match_angle_cnt"

	unsigned spi_error_value;
	unsigned short spi_command;

	unsigned tmp_aligned_tbl[72];
	union aligned_pos aligned_tbl[72];
	union aligned_pos *ptr_tmp_aligned_tbl; // need to change to ptr_aligned_tbl

	int shift_to_lowest_el_in_tbl;          // shift in aligned table to lowest element for searching phases num
	//unsigned phase_delta; 				// delta angle between phase on stator

}angle_t;


typedef struct
{
	unsigned dac_low_set :1;                // start prepare
	unsigned dac_high_set :1;               // start prepare
	unsigned phase_dead_time_timer_set :1;  // configuration complete for timer which control phase working time
	unsigned i2c_Tx_req :1; 				// request to i2c msg transmit
	//unsigned phase_ON:1;                    // wait before new phases pulse
	unsigned command:3;						// command to engine
	unsigned st1_dac_low_rising:1;
	unsigned st1_dac_high_falling:1;
	unsigned st2_dac_low_rising:1;
	unsigned st2_dac_high_falling:1;
	unsigned mode:3;						// engine's mode
	unsigned direction:3;					// direction of rotation
	//unsigned phase_on:1;					// current working phase is switch on
	unsigned reserv :16;
}status_word_t;

union u_status_word_t{
	status_word_t bit;
	unsigned all;
};

typedef struct
{
	unsigned phase_pulse_count;            // current pulse count
	unsigned phase_switches_count;         // max phases switches per turn

	unsigned phase_worked_time;            // worked time of current phase
	unsigned phase_pulse_count_before_moving;// count of pulse before start rotation
	volatile unsigned tst_max_pulse_count;

	unsigned PWM_filling; 				   // percent of PWM's filling
	unsigned PWM_max_filling;			   // max filling of current PWM's cycle
	unsigned phase_working_time;           // time of switch on phase
}switches_par_t;

struct motor_t
{
	struct phase_t switch_sequence[6]; // determine the order of phase switching

	struct phase_t *ptr_to_switch;     // pointer to current switch
	i2c_pair_t *i2c_send_pair;         // i2c address and data for send

	angle_t angle;					   // contain a field, which describe angle manipulation

	i2c_pair_t i2c_pair_current_sensor_low;
	i2c_pair_t i2c_pair_current_sensor_high;

	switches_par_t switch_par;

	union u_status_word_t status;
};

struct tst_current_cmp_t
{
	unsigned st1_irq_low;
	unsigned st1_irq_high;

	unsigned st2_irq_low;
	unsigned st2_irq_high;
};

#endif /* EXAMPLE_USER_CM_SRM_GLOBAL_DEFINE_H_ */
