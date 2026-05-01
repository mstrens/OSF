#pragma once

#include "cybsp.h"
#include "SEGGER_RTT.h"
#include "main.h"



// copied from TSDZ2
// motor states
#define BLOCK_COMMUTATION 			            0
#define SINEWAVE_INTERPOLATION_60_DEGREES 	    0x80

//Hall calibration
#define HALL_TO_CALIBRATE 0
#define HALL_CALIBRATING    1
#define HALL_MEASURED       2
#define HALL_CALIBRATION_CANCELED 3
#define HALL_CALIBRATION_ERROR    4
#define HALL_CALIBRATED     5

#define ID_IQ_COUNTER (64)

// power variables
extern volatile uint8_t ui8_adc_battery_current_filtered;
extern volatile uint16_t ui16_hall_counter_total;    
extern volatile uint16_t ui16_hall_calib_cnt[6];
extern uint8_t ui8_hall_ref_angles[8];  // was 6 in tsdz2
extern const uint8_t ui8_hall_counter_offsets[8]; // was 6 in tsdz2
extern volatile uint8_t ui8_hall_sensors_state;

// Sensors
extern volatile uint16_t ui16_adc_torque;


// added by mstrens because defined in ebike_app.c and used in motor.c
extern uint8_t ui8_adc_battery_overcurrent;

// added by mstrens to debug

#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
extern uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
extern uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
#endif

void CCU80_0_IRQHandler(); // called when ccu8 Slice 4 reaches 840  counting UP (= 1/4 of 19mhz cycles)
void CCU80_1_IRQHandler(); // called when ccu8 Slice 4 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)
void POSIF0_1_IRQHandler(); // called when posif generate a SR 1 ( used currently to debug)

uint32_t getHallPosition(void);
void posif_init_position();
void update_shadow_pattern(uint8_t current_pattern);

void motor_enable_pwm(void) ;
void motor_disable_pwm(void) ;

void get_curr_hall_pattern();

void set_rotor_angle( uint8_t angle, uint8_t duty_cycle);

void check_current_during(uint32_t during_ms, uint16_t max_A);

void log_hall_sensor_position();

uint16_t get_current_adc_10bits();

uint32_t calculate_average_angle(uint8_t pattern);

void update_foc_pid();

void update_foc_optimiser(void);

//__RAM_FUNC static inline void calculate_id_part1();

//__RAM_FUNC static inline void calculate_id_part2();

void hall_calibrate();
void hall_positions_init();

void capture_3_phase_current_offset();


void pll_init(void);
void pll_on_pwm_tick(void);
void pll_on_hall_event(uint16_t dt_us, uint16_t hall_phase_q8_8, uint16_t ui16_angle_between_2_hall_fronts_q8_8,bool seq_ok);
uint16_t pll_get_angle_q8_8(void);
uint16_t pll_get_erps(void);
uint32_t pll_get_velocity(void);
uint32_t pll_get_rpm(void);

//extern volatile uint16_t ui16_g_foc_angle_q8_8; // not used anymore with optimised lead angle in systick.c
extern uint32_t ui32_hall_velocity_q8_8X1024;

extern volatile int32_t i32_id_sum ;
extern volatile int32_t i32_iq_sum ;
extern volatile uint8_t ui8_id_iq_counter ;

// for security checks ; shared with systicks
extern volatile uint32_t ui32_Iu_rms_2_filt;
extern volatile uint32_t ui32_Iv_rms_2_filt;
extern volatile uint32_t ui32_Iw_rms_2_filt;
extern volatile uint32_t ui32_Imotor_rms_2_filt;
// Flags fault shared with ebike_app.c
extern volatile bool fault_phase_current_peak;
extern volatile bool fault_idc_fast;



extern volatile int32_t debug_id ;
extern volatile int32_t debug_iq ;

extern volatile int32_t debug_Iu;
extern volatile int32_t debug_Iv;
extern volatile int32_t debug_Iw;
extern volatile int32_t debug_Iuvw;

extern volatile int32_t debug_va ; // to debug
extern volatile int32_t debug_vb ; // to debug
extern volatile int32_t debug_vc ;  // to debug
extern volatile int32_t debug_Ialpha;
extern volatile int32_t debug_Ibeta;
extern volatile int32_t debug_angle;

extern volatile int32_t debug_foc; 

extern int32_t debug_raw_id;
extern int32_t debug_raw_iq;

