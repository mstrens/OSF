/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

//#include <stdint.h>
#include "main.h"

// startup boost mode
#define CADENCE					0
#define SPEED						1

// for oem display
extern volatile uint8_t ui8_system_state;

// cadence sensor
extern uint16_t ui16_cadence_ticks_count_min_speed_adj;

// ADC motor phase current max
extern volatile uint16_t ui16_adc_motor_phase_current_max;

// Motor enabled
extern uint8_t ui8_motor_enabled;


extern uint8_t ui8_pwm_duty_cycle_max;
typedef struct  _configuration_variables
{
  //uint8_t ui8_motor_power_x10; // not used
  uint32_t version;                 // added by mstrens to check the validity of configuration in flash; 32bits to use readWord
  uint8_t ui8_battery_current_max; // from  ebike_app.c
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_wheel_speed_max;
  // for oem display
  uint8_t ui8_torque_sensor_estimated;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_est_x100;
  uint8_t ui8_assist_without_pedal_rotation_enabled;
  uint8_t ui8_assist_with_error_enabled;
  uint8_t ui8_battery_SOC_percentage_8b;
  uint8_t ui8_set_parameter_enabled;
  uint8_t ui8_street_mode_enabled;
  uint8_t ui8_riding_mode;
  uint8_t ui8_lights_configuration;
  uint8_t ui8_startup_boost_enabled;
  uint8_t ui8_auto_display_data_enabled;
  uint8_t ui8_torque_sensor_adv_enabled; 
  uint8_t ui8_soc_percent_calculation;
} struct_configuration_variables;

extern uint8_t ui8_test_mode_flag ;
extern volatile uint32_t ui32_adc_battery_current_1_rotation_15b; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
extern uint8_t hall_reference_angle ; 

void new_torque_sample();

/*
extern uint8_t ui8_best_ref_angles1 ;
extern uint8_t ui8_best_ref_angles2 ;
extern uint8_t ui8_best_ref_angles3 ;
extern uint8_t ui8_best_ref_angles4 ;
extern uint8_t ui8_best_ref_angles5 ;
extern uint8_t ui8_best_ref_angles6 ;
*/

extern uint8_t ui8_adc_battery_current_max;

void ebike_app_controller(void);
struct_configuration_variables* get_configuration_variables(void);

void ebike_app_init(void);

uint16_t read_battery_soc(void);

//static void calc_oem_wheel_speed(void);
//static void ebike_control_lights(void);

// used only in VLCD5 version
void fillRxBuffer();
void uart_receive_package();
void uart_send_package();


#endif /* _EBIKE_APP_H_ */


