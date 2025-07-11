/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

#include <stdint.h>
#include "cybsp.h"
#include "cy_utils.h"

#include "SEGGER_RTT.h"
#include "config_tsdz8.h"


// riding modes
#define OFF_MODE                                  0 // not used
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
#define TORQUE_SENSOR_CALIBRATION_MODE            8								   


// walk assist
//#define WALK_ASSIST_THRESHOLD_SPEED_X10           80  // 80 -> 8.0 kph, this is the maximum speed limit from which walk assist can be activated

// cruise
//#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated

// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

// cadence sensor
//#define STANDARD_MODE                             0
//#define ADVANCED_MODE                             1
//#define CALIBRATION_MODE                          2




uint16_t map_ui16(uint16_t in, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
uint8_t map_ui8(uint8_t in, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
uint8_t ui8_max(uint8_t value_a, uint8_t value_b);
uint8_t ui8_min(uint8_t value_a, uint8_t value_b);
uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha);
void crc16(uint8_t ui8_data, uint16_t *ui16_crc);


void lights_set_state(uint8_t ui8_state) ; // moved by mstrens from another tsdz2 file

// added by mstrens
extern volatile uint32_t system_ticks ;
extern struct_config m_config ;


bool take_action(uint32_t index, uint32_t interval);
bool take_action_250ms(uint32_t index, uint32_t interval);

//void wait_ms(uint32_t time);
#endif /* COMMON_COMMON_H_ */
