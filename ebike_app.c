/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"
#include "ebike_app.h"
#include "main.h"
#include "motor.h"
#include "common.h"
#include "eeprom.h"
#include "adc.h"
#include "systick.h"

volatile struct_configuration_variables m_configuration_variables;

// display menu
static uint8_t ui8_assist_level = ECO;
static uint8_t ui8_assist_level_temp = ECO;
static uint8_t ui8_assist_level_5_flag = 0;
static uint8_t ui8_riding_mode_temp = 0;
static uint8_t ui8_lights_flag = 0;
static uint8_t ui8_lights_on_5s = 0;
static uint8_t ui8_menu_flag = 0;
static uint8_t ui8_menu_index = 0;
static uint8_t ui8_data_index = 0;
static uint8_t ui8_menu_counter = 0;
static uint8_t ui8_display_function_code = 0;
static uint8_t ui8_display_function_code_temp = 0;
static uint8_t ui8_menu_function_enabled = 0;
static uint8_t ui8_display_data_enabled = 0;
static uint16_t ui16_display_data = 0;
static uint16_t ui16_data_value = 0;
static uint8_t ui8_auto_display_data_flag = 0;
static uint8_t ui8_auto_display_data_status = 0;
 uint8_t ui8_auto_data_number_display;// init in config.c : AUTO_DATA_NUMBER_DISPLAY;
static uint16_t ui16_display_data_factor = 0;
 uint8_t ui8_delay_display_function; // = DELAY_MENU_ON;
 uint8_t ui8_display_data_on_startup; // = DATA_DISPLAY_ON_STARTUP;
 uint8_t ui8_set_parameter_enabled_temp; // = ENABLE_SET_PARAMETER_ON_STARTUP;
 uint8_t ui8_auto_display_data_enabled_temp;// = ENABLE_AUTO_DATA_DISPLAY;
 uint8_t ui8_street_mode_enabled_temp; // = ENABLE_STREET_MODE_ON_STARTUP;
 uint8_t ui8_torque_sensor_adv_enabled_temp; // = TORQUE_SENSOR_ADV_ON_STARTUP;
 uint8_t ui8_assist_without_pedal_rotation_temp; // = MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION;
 uint8_t ui8_walk_assist_enabled_array[2]; // = {ENABLE_WALK_ASSIST,STREET_MODE_WALK_ENABLED};
// uint8_t ui8_display_battery_soc_flag = 0;
static uint8_t ui8_display_riding_mode = 0;
static uint8_t ui8_display_lights_configuration = 0;
static uint8_t ui8_display_alternative_lights_configuration = 0;
// uint8_t ui8_display_torque_sensor_flag_1 = 0;
// uint8_t ui8_display_torque_sensor_flag_2 = 0;
// uint8_t ui8_display_torque_sensor_value_flag = 0;
// uint8_t ui8_display_torque_sensor_step_flag = 0;
static uint8_t ui8_display_function_status[3][5];
 uint8_t ui8_lights_configuration_2;// = LIGHTS_CONFIGURATION_2;
 uint8_t ui8_lights_configuration_3;// = LIGHTS_CONFIGURATION_3;
 uint8_t ui8_lights_configuration_temp;// = LIGHTS_CONFIGURATION_ON_STARTUP;

// system
static uint8_t ui8_riding_mode_parameter = 0;
volatile uint8_t ui8_system_state = NO_ERROR;
 uint8_t ui8_motor_enabled = 1; 
 uint8_t ui8_assist_without_pedal_rotation_threshold;// = ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD;
static uint8_t ui8_lights_state = 0;
static uint8_t ui8_lights_button_flag = 0;
static uint8_t ui8_field_weakening_erps_delta = 0;
 //uint8_t ui8_optional_ADC_function = OPTIONAL_ADC_FUNCTION;
static uint8_t ui8_walk_assist_level = 0;

// battery
static uint16_t ui16_battery_voltage_filtered_x10 = 0;
static uint16_t ui16_battery_voltage_calibrated_x10 = 0;
volatile uint16_t ui16_battery_voltage_soc_filtered_x10 = 0;
static uint16_t ui16_battery_power_x10 = 0;															  
static uint16_t ui16_battery_power_filtered_x10 = 0;
 uint16_t ui16_actual_battery_capacity; // = (uint16_t)(((uint32_t) m_config.target_max_battery_capacity * m_config.actual_battery_capacity_percent ) / 100);
static uint32_t ui32_wh_x10 = 0;
static uint32_t ui32_wh_sum_x10 = 0;
volatile uint32_t ui32_wh_offset_x10 = 0;
static uint32_t ui32_wh_since_power_on_x10 = 0;
volatile uint16_t ui16_battery_SOC_percentage_x10 = 0;
//volatile uint8_t ui8_battery_SOC_init_flag = 0;
static uint8_t ui8_battery_state_of_charge = 0;

// power control
static uint8_t ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;        // 194
static uint8_t ui8_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
static uint8_t ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;         //73
static uint8_t ui8_duty_cycle_ramp_down_inverse_step_default = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
static uint16_t ui16_battery_voltage_filtered_x1000 = 0;
static uint16_t ui16_battery_no_load_voltage_filtered_x10 = 0;
static uint8_t ui8_battery_current_filtered_x10 = 0;
static uint16_t ui16_adc_battery_voltage_filtered = 0;
uint8_t ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX; // In tdsz2 it was 112 = 18A; it is updated by the program based on riding mode parameters
static uint8_t ui8_adc_battery_current_target = 0;
static uint8_t ui8_duty_cycle_target = 0;
static uint16_t ui16_duty_cycle_percent = 0;
volatile uint16_t ui16_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX; // In tdsz2 it was 187
static uint8_t ui8_error_battery_overcurrent = 0;
static uint8_t ui8_error_battery_overcurrent_counter = 0;
uint8_t ui8_adc_battery_overcurrent = (uint8_t)(ADC_10_BIT_BATTERY_CURRENT_MAX + ADC_10_BIT_BATTERY_EXTRACURRENT); //In tdsz2 it was 112 + 50
static uint8_t ui8_adc_battery_current_max_temp_1 = 0;
static uint8_t ui8_adc_battery_current_max_temp_2 = 0;
static uint32_t ui32_adc_battery_power_max_x1000_array[2];

// Motor ERPS
static uint16_t ui16_motor_speed_erps = 0;

// cadence sensor
 uint16_t ui16_cadence_ticks_count_min_speed_adj = CADENCE_SENSOR_CALC_COUNTER_MIN; //4270 
static uint8_t ui8_pedal_cadence_RPM = 0;
 // added by mstrens to allow faster torque decrease 
uint8_t ui8_pedal_cadence_RPM_previous = 0;
int16_t i16_pedal_cadence_RPM_decrease_ratio = 0 ;

 uint8_t ui8_motor_deceleration ;//= MOTOR_DECELERATION;

// torque sensor
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100; // = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100;  // 67
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_calc_x100;// = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;  //
 uint16_t ui16_adc_pedal_torque_offset;// = PEDAL_TORQUE_ADC_OFFSET;      // 150
 uint16_t ui16_adc_pedal_torque_offset_init;// = PEDAL_TORQUE_ADC_OFFSET; // 150
 //uint16_t ui16_adc_pedal_torque_offset_cal;// not used anymore
 uint16_t ui16_adc_pedal_torque_offset_set ;  // mstrens : added to use the same fields as 860c; init in ebike_init
  uint16_t ui16_adc_pedal_torque_offset_min;// = PEDAL_TORQUE_ADC_OFFSET - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; //150-30
 uint16_t ui16_adc_pedal_torque_offset_max;// = PEDAL_TORQUE_ADC_OFFSET + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; // 150 + 30
 uint8_t ui8_adc_pedal_torque_offset_error = 0;
 uint8_t ui8_adc_pedal_torque_range_adj = 0; // mstrens : added to have code similar to 860c; init in ebike_init()
 uint16_t ui16_adc_pedal_torque_range = 160;// mstrens : added to have code similar to 860c; init in ebike_app_init()

//volatile uint16_t ui16_adc_coaster_brake_threshold = 0; // mstrens : not used in vlcd5 version
static uint16_t ui16_adc_pedal_torque = 0;
static uint16_t ui16_adc_pedal_torque_max_temp = 0;
static uint16_t ui16_adc_pedal_torque_delta = 0;
static uint16_t ui16_adc_pedal_torque_delta_temp = 0;
static uint16_t ui16_adc_pedal_torque_delta_no_boost = 0;
static uint16_t ui16_human_power_x10 = 0;
static uint16_t ui16_human_power_filtered_x10 = 0;
uint8_t ui8_torque_sensor_calibrated;// = TORQUE_SENSOR_CALIBRATED;
static uint16_t ui16_pedal_weight_x100 = 0;
static uint16_t ui16_pedal_torque_step_temp = 0;
 static uint8_t ui8_torque_sensor_calibration_flag = 0;
 static uint8_t ui8_torque_sensor_calibration_flag_1 = 0;
 static uint8_t ui8_torque_sensor_calibration_flag_2 = 0;
 static uint8_t ui8_torque_sensor_calibration_flag_3 = 0;
 static uint8_t ui8_display_torque_sensor_calibration_value_flag = 0;
 static uint8_t ui8_display_torque_sensor_calibration_step_flag = 0;
 static uint8_t ui8_torque_sensor_calibration_with_weight_flag = 0;
 static uint8_t ui8_torque_sensor_value_display_counter = 0;
 static uint8_t ui8_torque_sensor_value_display_flag = 0;
 static uint16_t ui16_torque_sensor_value_to_display = 0;
 static uint8_t ui8_torque_sensor_step_display_counter = 0;
 static uint8_t ui8_torque_sensor_step_display_flag = 0;
 static uint8_t ui8_torque_sensor_step_to_display = 0;
 static uint8_t ui8_pedal_torque_per_10_bit_ADC_step_detected_x100 = 0;
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_advanced_x100 ; // mstrens : init is done in ebike_app_init()
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100_array[2];
 uint8_t ui8_eMTB_based_on_power; //= eMTB_BASED_ON_POWER;

// wheel speed sensor
static uint16_t ui16_wheel_speed_x10 = 0;
 uint8_t ui8_wheel_speed_max_array[2] ; //= {WHEEL_MAX_SPEED,STREET_MODE_SPEED_LIMIT};
uint8_t ui8_wheel_speed_simulate = 0;  //added by mstrens to simulate a fixed speed whithout having a speed sensor 

// wheel speed display
static uint8_t ui8_display_ready_flag = 0;
static uint8_t ui8_startup_counter = 0;
static uint8_t ui8_startup_flag = 0;
static  uint16_t ui16_oem_wheel_speed_time = 0;
static  uint8_t ui8_oem_wheel_diameter = 0;
 uint32_t ui32_odometer_compensation_mm = ZERO_ODOMETER_COMPENSATION;

// throttle control
static uint8_t ui8_adc_throttle_assist = 0;
static uint8_t ui8_throttle_adc_in = 0;
 uint8_t ui8_throttle_mode_array[2]; //initialisation moved to config.c
 //uint8_t ui8_throttle_mode_array[2] = {THROTTLE_MODE,STREET_MODE_THROTTLE_MODE}; // this variable is never updated (so based only on config)

// cruise control
 uint8_t ui8_cruise_threshold_speed_x10_array[2] ;  //initialisation moved to config.c
 //uint8_t ui8_cruise_threshold_speed_x10_array[2] = {CRUISE_OFFROAD_THRESHOLD_SPEED_X10,CRUISE_STREET_THRESHOLD_SPEED_X10}; initialisation moved to config.c
 static uint8_t ui8_cruise_button_flag = 0;

// walk assist
static uint8_t ui8_walk_assist_flag = 0;
static uint8_t ui8_walk_assist_speed_target_x10 = 0;
static uint8_t ui8_walk_assist_duty_cycle_counter = 0;
static uint8_t ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MIN;
static uint8_t ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_MIN;
static uint8_t ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
static uint16_t ui16_walk_assist_wheel_speed_counter = 0;
static uint16_t ui16_walk_assist_erps_target = 0;
static uint16_t ui16_walk_assist_erps_min = 0;
static uint16_t ui16_walk_assist_erps_max = 0;
static uint8_t ui8_walk_assist_speed_flag = 0;

// startup boost
 uint8_t ui8_startup_boost_at_zero ;// = STARTUP_BOOST_AT_ZERO;
 static uint8_t ui8_startup_boost_flag = 0;
 uint8_t ui8_startup_boost_enabled_temp;// = STARTUP_BOOST_ON_STARTUP;
 static uint16_t ui16_startup_boost_factor_array[120];

// smooth start
static uint8_t ui8_smooth_start_flag = 0;
static uint8_t ui8_smooth_start_counter = 0;
static uint8_t ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
static int8_t ui8_smooth_start_counter_set_temp = SMOOTH_START_RAMP_DEFAULT;

// startup assist
static uint8_t ui8_startup_assist_flag = 0;
static uint8_t ui8_startup_assist_adc_battery_current_target = 0;

// motor temperature control
static uint16_t ui16_adc_motor_temperature_filtered = 0;
static uint16_t ui16_motor_temperature_filtered_x10 = 0;
// uint8_t ui8_motor_temperature_max_value_to_limit_array[2] = {MOTOR_TEMPERATURE_MAX_VALUE_LIMIT, (uint8_t)(MOTOR_TEMPERATURE_MAX_VALUE_LIMIT + 50)};
// uint8_t ui8_motor_temperature_min_value_to_limit_array[2] = {MOTOR_TEMPERATURE_MIN_VALUE_LIMIT, (uint8_t)(MOTOR_TEMPERATURE_MIN_VALUE_LIMIT + 50)};

// controller temperature control  added by mstrens
uint8_t ui8_temp_celcius = 0;


// UART
// function to get data from rxfifo and set them in rxBuffer; set a flag when a valid frame is received
// is called only when at least one byte is present in fifo
// UART
#define UART_RX_BUFFER_LEN   			7
#define RX_CHECK_CODE					(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN				9
#define TX_CHECK_CODE					(UART_TX_BUFFER_LEN - 1)
#define TX_STX							0x43
#define RX_STX							0x59
//uint8_t ui8_state_machine = 0; // 0= not yet a start byte , 1 = accumulate 
uint32_t rxIdx = 0;
//uint32_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
//uint8_t ui8_rx_counter = 0;
//uint8_t ui8_received_package_flag = 0; // become 1 when buffer is filled with a frame
//uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;

// uart send
volatile uint8_t ui8_working_status = 0;
volatile uint8_t ui8_display_fault_code = 0;

// array for oem display
 uint8_t ui8_data_index_array[DATA_INDEX_ARRAY_DIM];// = {DISPLAY_DATA_1,DISPLAY_DATA_2,DISPLAY_DATA_3,DISPLAY_DATA_4,DISPLAY_DATA_5,DISPLAY_DATA_6};
 uint8_t ui8_delay_display_array[DATA_INDEX_ARRAY_DIM];// = {DELAY_DISPLAY_DATA_1,DELAY_DISPLAY_DATA_2,DELAY_DISPLAY_DATA_3,DELAY_DISPLAY_DATA_4,DELAY_DISPLAY_DATA_5,DELAY_DISPLAY_DATA_6};

// array for riding parameters
 uint8_t  ui8_riding_mode_parameter_array[8][5];
//	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
//	{TORQUE_ASSIST_LEVEL_0, TORQUE_ASSIST_LEVEL_1, TORQUE_ASSIST_LEVEL_2, TORQUE_ASSIST_LEVEL_3, TORQUE_ASSIST_LEVEL_4},
//	{CADENCE_ASSIST_LEVEL_0, CADENCE_ASSIST_LEVEL_1, CADENCE_ASSIST_LEVEL_2, CADENCE_ASSIST_LEVEL_3, CADENCE_ASSIST_LEVEL_4},
//	{EMTB_ASSIST_LEVEL_0, EMTB_ASSIST_LEVEL_1, EMTB_ASSIST_LEVEL_2, EMTB_ASSIST_LEVEL_3, EMTB_ASSIST_LEVEL_4},
//	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
//	{CRUISE_TARGET_SPEED_LEVEL_0, CRUISE_TARGET_SPEED_LEVEL_1, CRUISE_TARGET_SPEED_LEVEL_2, CRUISE_TARGET_SPEED_LEVEL_3, CRUISE_TARGET_SPEED_LEVEL_4},
//	{WALK_ASSIST_LEVEL_0, WALK_ASSIST_LEVEL_1, WALK_ASSIST_LEVEL_2, WALK_ASSIST_LEVEL_3, WALK_ASSIST_LEVEL_4},
//	{0, 0, 0, 0, 0}
		
// to debug
uint16_t debug1 =0;
uint16_t debug2 =0;
uint16_t debug3 =0;
uint16_t debug4 =0;
uint16_t debug5 =0;
uint16_t debug6 =0;
uint16_t debug7 =0;
uint16_t debug8 =0;
uint16_t debug9 =0;

// added by mstrens to optimise hall positions
extern volatile uint8_t ui8_best_ref_angles[8];

uint16_t ui16_adc_pedal_torque_delta_to_remap = 0;
uint16_t ui16_adc_pedal_torque_delta_remapped = 0;
int i32_adc_pedal_torque_delta_expo = 0;

uint16_t ui16_adc_throttle;
// added by mstrens in order to allow changes in ucPorobe for testing with different speed  and same load
uint8_t ui8_pwm_duty_cycle_max = PWM_DUTY_CYCLE_MAX;
// added by mstrens for using testing mode
uint8_t ui8_test_mode_flag = DEFAULT_TEST_MODE_FLAG ; // can be changed in uc_probe
uint8_t ui8_battery_current_target_testing = DEFAULT_BATTERY_CURRENT_TARGET_TESTING_A ; // value is in A ; this is a default value that can be changed with uc_probe
uint8_t ui8_duty_cycle_target_testing = DEFAULT_DUTY_CYCLE_TARTGET_TESTING; // max is 245, this is a default value that can be changed with uc_probe
#define AVERAGING_BITS 6
#define AVERAGING_CNT (1<<AVERAGING_BITS) // 25 msec per cycle; 64 = 1,5 sec
uint32_t ui32_battery_current_mA_acc =0;
uint32_t ui32_battery_current_mA_cnt = AVERAGING_CNT;
uint32_t ui32_battery_current_mA_avg = 0;


uint32_t ui32_current_1_rotation_ma = 0; // average current over 1 electric rotation
	

// system functions
static void get_battery_voltage(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);

static void ebike_control_lights(void);
static void ebike_control_motor(void);
static void check_system(void);

static void set_motor_ramp(void);
static void apply_startup_boost(void);
static void apply_smooth_start(void);

static void apply_power_assist(void);
static void apply_torque_assist(void);
static void apply_cadence_assist(void);
static void apply_emtb_assist(void);
static void apply_hybrid_assist(void);
static void apply_cruise(void);
static void apply_walk_assist(void);
static void apply_throttle(void);
//static void apply_temperature_limiting(void);
static void apply_speed_limit(void);

// functions for oem display
static void calc_oem_wheel_speed(void);
static void apply_torque_sensor_calibration(void);

// battery soc percentage x10 calculation
static void calc_watt_hours_used(void);
static void check_battery_soc(void);
uint16_t read_battery_soc(void);
uint16_t calc_battery_soc_x10(uint16_t ui16_battery_soc_offset_x10, uint16_t ui16_battery_soc_step_x10, uint16_t ui16_cell_volts_max_x100, uint16_t ui16_cell_volts_min_x100);

	

// ********************* init used only in VLCD5 version******************************
void ebike_app_init(void)
{
	// minimum value for these displays
	// to compensate for the delay of the lights button
	//#if ENABLE_VLCD6 || ENABLE_850C || ENABLE_EKD01
	if ((m_config.enable_vlcd6) || (m_config.enable_850c) || (m_config.enable_ekd01)){//#if ENABLE_VLCD6 || ENABLE_850C
		if (ui8_delay_display_function < 70) {
			ui8_delay_display_function = 70;
		}
	} // endif
	
	// set low voltage cutoff (16 bit) ; 39V => 390*100/87= 448adcfor 48V battery
	ui16_adc_voltage_cut_off = ((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 * 100U) / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
	
	// check if assist without pedal rotation threshold is valid (safety)
	if (ui8_assist_without_pedal_rotation_threshold > 100) {
		ui8_assist_without_pedal_rotation_threshold = 100;
	}
	// set duty cycle ramp up inverse step default
	ui8_duty_cycle_ramp_up_inverse_step_default = map_ui8((uint8_t) MOTOR_ACCELERATION,
				(uint8_t) 0,
				(uint8_t) 100,
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
	
	// set duty cycle ramp down inverse step default
	ui8_duty_cycle_ramp_down_inverse_step_default = map_ui8((uint8_t) MOTOR_DECELERATION,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
	
	// Smooth start counter set ; counter is e.g. about 160
	ui8_smooth_start_counter_set = map_ui8((uint8_t) SMOOTH_START_SET_PERCENT,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) 255,
                (uint8_t) SMOOTH_START_RAMP_MIN);
	
	// set pedal torque per 10_bit DC_step x100 advanced (calibrated) or default(not calibrated)
	ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_DEFAULT] = m_config.pedal_torque_per_10_bit_adc_step_x100; // PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100; // 67
	if (ui8_torque_sensor_calibrated) {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = m_config.pedal_torque_per_10_bit_adc_step_adv_x100;//  PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100; //34
	}
	else {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = m_config.pedal_torque_per_10_bit_adc_step_x100;//  PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100;
	}
	// added by mstrens : it is not clear why ????
	ui8_pedal_torque_per_10_bit_ADC_step_advanced_x100 = m_config.pedal_torque_per_10_bit_adc_step_adv_x100;
	// parameters status on startup
	// set parameters on startup
	ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
	// auto display data on startup
	ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
	// street mode on startup
	ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
	// startup boost on startup
	ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
	// torque sensor adv on startup
	ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
	// assist without pedal rotation on startup
	ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
	// system error enabled on startup
	ui8_display_function_status[2][TURBO] = m_configuration_variables.ui8_assist_with_error_enabled;
	// riding mode on startup
	ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
	// lights configuration on startup
	ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
	
	// percentage remaining battery capacity x10 at power on
	ui16_battery_SOC_percentage_x10 = ((uint16_t) m_configuration_variables.ui8_battery_SOC_percentage_8b) << 2;
		 
	// battery SOC checked at power on
	if (ui16_battery_SOC_percentage_x10 > 0U) {
		// calculate watt-hours x10 at power on
		ui32_wh_offset_x10 = ((uint32_t)(1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
	}

	// make startup boost array
	ui16_startup_boost_factor_array[0] = STARTUP_BOOST_TORQUE_FACTOR;
	
	uint8_t ui8_i;
	for (ui8_i = 1; ui8_i < 120; ui8_i++)
	{
		//uint16_t ui16_temp = (ui16_startup_boost_factor_array[ui8_i - 1] * STARTUP_BOOST_CADENCE_STEP) >> 8;
		//ui16_startup_boost_factor_array[ui8_i] = ui16_startup_boost_factor_array[ui8_i - 1] - ui16_temp;

		// propose correction to let factor reaches 0 still using intergers
		uint32_t ui32_temp = ( ((uint32_t)(ui16_startup_boost_factor_array[ui8_i - 1]) << 8))
			- (((uint32_t) ui16_startup_boost_factor_array[ui8_i - 1]) * STARTUP_BOOST_CADENCE_STEP);
		ui16_startup_boost_factor_array[ui8_i] = ui32_temp >> 8;
	}
	
	// enable data displayed on startup
	//#if DATA_DISPLAY_ON_STARTUP
	if (m_config.data_display_on_startup) {
		ui8_display_data_enabled = 1;
	}	// endif
	
	// calculate max adc battery current from the received battery current limit
	ui8_adc_battery_current_max_temp_1 = (uint8_t)((uint16_t)(m_configuration_variables.ui8_battery_current_max * 100U)
				/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100);

	// calculate the max adc battery power from the power limit received in offroad mode
	ui32_adc_battery_power_max_x1000_array[OFFROAD_MODE] = (uint32_t)((uint32_t)TARGET_MAX_BATTERY_POWER * 100U * 1000U)
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
	// calculate the max adc battery power from the received power limit in street mode
	ui32_adc_battery_power_max_x1000_array[STREET_MODE] = (uint32_t)((uint32_t)STREET_MODE_POWER_LIMIT * 100U * 1000U)
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
	// set max motor phase current
	uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
	// mstrens changed it in U16
	ui16_adc_motor_phase_current_max = (ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
	// limit max motor phase current if higher than configured hardware limit (safety)
	if (ui16_adc_motor_phase_current_max > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX) {
		ui16_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
	}

	//mstrens : added to use the same fields as in 860C in get_pedal_torque()
	ui16_adc_pedal_torque_offset_set = m_config.pedal_torque_adc_offset;
	ui8_adc_pedal_torque_range_adj = m_config.pedal_torque_adc_range_adj;
 	ui16_adc_pedal_torque_range = m_config.pedal_torque_adc_max - m_config.pedal_torque_adc_offset;
}


void ebike_app_controller(void) // is called every 25ms by main()
{
	// calculate motor ERPS = electrical rotation per sec ; ui16_hall_counter_total is the number of tick (4usec/tick) for a full electric rotation
    // mstrens changed it
	// 0x8000 was the value for TSDZ2; 
	// TSDZ8 should test on a value that is 2 * because there is 4 poles instead of 8 and so it takes more ticks for the same mecanical speed
	//if ((ui16_hall_counter_total >= 0x8000 ) || (ui16_hall_counter_total < 10)) { For TSDZ2
	// > 0X8000 = >32000 ; *4 usec = 0,131 sec per electric rotation ; for TSDZ2 * 8 = 1 sec per rotation = 60 rotations mecanical /sec
	// normally this should not happens because there is already a check in motor.c that set ui16_hall_counter_total = 0xffff when enlapsed time is more than a value
	// So, we should not exceed a uint16_t variable
	
	//if ((ui16_hall_counter_total >= 0xF000 ) || (ui16_hall_counter_total < 10)) {
    //    ui16_motor_speed_erps = 0;  // speed is 0 if number of ticks is to high
    //} else {
        //ui16_motor_speed_erps = (uint16_t)(HALL_COUNTER_FREQ >> 2) / (uint16_t)(ui16_tmp >> 2); // 250000/nrOfTicks; so in sec
	//	ui16_motor_speed_erps = ((uint32_t) HALL_COUNTER_FREQ) / ui16_hall_counter_total; // 250000/nrOfTicks; so rotation in sec
	//}
	ui16_motor_speed_erps = pll_get_erps(); // speed is calculated in PLL based on interval between 2 hall front (usually with pll correction)
	// calculate the wheel speed
	calc_wheel_speed();
	
	// calculate the cadence and set limits from wheel speed
	calc_cadence();

	// Calculate filtered Battery Voltage (mV)
    get_battery_voltage(); // get a ui16_adc_voltage filtered value and convert it in mv with ui16_battery_voltage_filtered_x1000

    // Calculate filtered Battery Current (Ampx10)
    ui8_battery_current_filtered_x10 = (uint8_t)(((uint16_t)(ui8_adc_battery_current_filtered * (uint8_t)BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100)) / 10U);
	
	// get pedal torque ; calculate ui16_pedal_torque_x100 and ui16_human_power_x10 (human power)
	get_pedal_torque();
	
	// send/receive data, ebike control lights, calc oem wheelspeed, 
	// check system, check battery soc, every 4 cycles (25ms * 4)	
    static uint8_t ui8_counter;

	switch (ui8_counter++ & 0x03) {
		case 0: 
			uart_receive_package();
			break;
		case 1:
			ebike_control_lights();
			calc_oem_wheel_speed();
			calc_watt_hours_used();
			break;
		case 2:
			uart_send_package();
			break;
		case 3:
			check_system();
			hall_calibrate();  // calibrate the hall position (using result of data saved by ISR when speed is high enough)

			check_battery_soc();
						/*
			// added by mstrens
			ui8_best_ref_angles[1] = ui8_best_ref_angles1;
			ui8_best_ref_angles[2] = ui8_best_ref_angles2;
			ui8_best_ref_angles[3] = ui8_best_ref_angles3;
			ui8_best_ref_angles[4] = ui8_best_ref_angles4;
			ui8_best_ref_angles[5] = ui8_best_ref_angles5;
			ui8_best_ref_angles[6] = ui8_best_ref_angles6; 			break;
			*/
			break;
	}
	
	// use received data and sensor input to control motor
    ebike_control_motor();

    /*------------------------------------------------------------------------

     NOTE: regarding function call order

     Do not change order of functions if not absolutely sure it will
     not cause any undesirable consequences.

     ------------------------------------------------------------------------*/
	// for debugging
	//debug1 = ui8_best_ref_angles[2];
	//debug2 = ui8_best_ref_angles[3];
	//debug3 = ui8_best_ref_angles[4];
	//debug4 = ui8_best_ref_angles[5];
	//debug5 = ui8_best_ref_angles[6];
	
	#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
	// allow to calculate the regressions for each interval; 
	// first is the duty cycle
	// second value is the previous ticks for 360°
	// next one is the ticks between 5 and 1; then between 1 and 3, between 3 and 2 ...
	if (ticks_intervals_status == 2) { // when all 8 values have been written by irq0 in motor.c
		SEGGER_RTT_printf(0,"%u,%u,%u,%u,%u,%u,%u,%u\r\n", ticks_intervals[7],ticks_intervals[0],ticks_intervals[1],ticks_intervals[3],ticks_intervals[2],ticks_intervals[6],ticks_intervals[4],ticks_intervals[5] );
		ticks_intervals_status = 0; // reset status to allow a new capture
	}
	#endif
}


static void ebike_control_motor(void) // is called every 25ms by ebike_app_controller()
{
    // reset control variables (safety)
    ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
    ui8_adc_battery_current_target = 0;
    ui8_duty_cycle_target = 0;
	
	// field weakening enabled
	//#if FIELD_WEAKENING_ENABLED
	if (m_config.field_weakening_enabled) {
		if ((ui16_motor_speed_erps > MOTOR_SPEED_FIELD_WEAKENING_MIN)
			&& (ui8_adc_battery_current_filtered < ui8_controller_adc_battery_current_target)
			&& (ui8_adc_throttle_assist == 0U) ){
			ui8_field_weakening_erps_delta = ui16_motor_speed_erps - MOTOR_SPEED_FIELD_WEAKENING_MIN;
			ui8_fw_hall_counter_offset_max = ui8_field_weakening_erps_delta >> 5;
			if (ui8_fw_hall_counter_offset_max > FW_HALL_COUNTER_OFFSET_MAX) {
				ui8_fw_hall_counter_offset_max = FW_HALL_COUNTER_OFFSET_MAX;
			}
			ui8_field_weakening_enabled = 1;
		}
		else {
			ui8_field_weakening_enabled = 0;
		}
	} // endif	

	// added by mstrens (test mode flag)
	 // ********************* here the 2 main ways to run the motor (one for test/calibration, the other for normal use) *****************
	if (ui8_test_mode_flag == NORMAL_RUNNING_MODE) {
		// select riding mode and calculate ui8_adc_battery_current_target and ui8_duty_cycle_target is 255 (or 0)
		//        It also adapt the ramp up and down inverse step that has an impact on how fast the motor react to a change.
		switch (m_configuration_variables.ui8_riding_mode) {
			case POWER_ASSIST_MODE: apply_power_assist(); break;
			case TORQUE_ASSIST_MODE: apply_torque_assist(); break;
			case CADENCE_ASSIST_MODE: apply_cadence_assist(); break;
			case eMTB_ASSIST_MODE: apply_emtb_assist(); break;
			case HYBRID_ASSIST_MODE: apply_hybrid_assist(); break;
			case CRUISE_MODE: apply_cruise(); break;
			case WALK_ASSIST_MODE: apply_walk_assist(); break;
			case TORQUE_SENSOR_CALIBRATION_MODE: apply_torque_sensor_calibration(); break;
		}
	} else { // we are in testing mode 
		// for testing, we force the 4 parameters used to control the motor
		ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;     // 194
		ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  //73
		// set current target to the testing value and check with max
		ui8_adc_battery_current_target = (uint16_t) ui8_battery_current_target_testing *100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
		if (ui8_adc_battery_current_target > ADC_10_BIT_BATTERY_CURRENT_MAX)
				ui8_adc_battery_current_target = ADC_10_BIT_BATTERY_CURRENT_MAX;
		// in case of error, force current target to 0 to avoid that motor starts again (with code for motor enable)
		if(	ui8_system_state )  {
			ui8_adc_battery_current_target = 0;
		}	
		// set duty cycle target to the tesing value and check against max
		ui8_duty_cycle_target = ui8_duty_cycle_target_testing;
		if (ui8_duty_cycle_target >= ui8_pwm_duty_cycle_max ) {
			ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;  
		}
		ui8_riding_mode_parameter =  50; // if it is set on 0 , it means that there is no assist and motor stays/goes off in safety checks
		ui8_duty_cycle_ramp_up_inverse_step = DEFAULT_RAMP_UP_INVERSE_TESTING;
		ui8_duty_cycle_ramp_down_inverse_step = DEFAULT_RAMP_DOWN_INVERSE_TESTING;
	}

    // select optional ADC function
	//#if (OPTIONAL_ADC_FUNCTION == THROTTLE_CONTROL)   // in some cases, it can increase the target current and change duty_cyle and ramp up/down
		// note: it never decreases the target current set in a previous function based on the riding mode 
	if (m_config.throttle_mode){ // 0 means that throttle is disabled
		if (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) { //  0 means that Throttle is "disabled"
			apply_throttle();
		}
	}
	//#elif (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL)
	//apply_temperature_limiting();
	//#endif
	
    // speed limit :  reduce ui8_adc_battery_current_target progressively (up to 0) when close to speed limit (or exceed)
    apply_speed_limit();
	
	// Check battery Over-current (read current here in case PWM interrupt for some error was disabled)
	//the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
	// in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
	// In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
	//      SO 1 adc step = 1/24,576 = 0,040A
	// For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
	// For 10 A, TSDZ8 shoud give 10*24,576 steps
	// to convert TSDZ8 steps in the same units as TSDZ2, we shoud take ADC *62/245,76 = 0,25 and divide by 4 (or >>2)
	// current is available in gr0 ch1 result 8 in queue 0 p2.8 and/or in gr0 ch0 result in 12 (p2.8)
	
	// modified by mstrens (a reset was missing; we use the filtered value (based on moving average))
	if (m_config.overcurrent_delay > 0) {  // OVERCURRENT_DELAY
		if ( ui8_adc_battery_current_filtered > ui8_adc_battery_overcurrent){ // 112+50 in tsdz2 (*0,16A) => 26A
			ui8_error_battery_overcurrent_counter++;
		} else {
			ui8_error_battery_overcurrent_counter = 0;
		}
		if (ui8_error_battery_overcurrent_counter >= OVERCURRENT_DELAY) { // OVERCURRENT_DELAY refers to m_config
			ui8_system_state = ERROR_BATTERY_OVERCURRENT;
		}
	} // endif
	// added by mstrens to use also checks done in motor.c (in ISR) that are only to protect controller (does not take care of user parameter in congig or 860c setup)
	if (fault_phase_current_peak || fault_idc_fast) ui8_system_state = ERROR_BATTERY_OVERCURRENT ;

	// for debug
	// calculate an average in mA (to find parameters giving lowest current)
	ui32_current_1_rotation_ma = (ui8_adc_battery_current_filtered * 10 * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) ;
	
	ui32_battery_current_mA_acc += ui32_current_1_rotation_ma;
	ui32_battery_current_mA_cnt--;
	if (ui32_battery_current_mA_cnt == 0){ // so about 1.5 sec
		// calculate avg current of avg of each rotation
		ui32_battery_current_mA_avg = ui32_battery_current_mA_acc >> AVERAGING_BITS;
		//  Var = (SumSq − (Sum × Sum) / n) / (n − 1)  Wikipedia
		ui32_battery_current_mA_cnt = AVERAGING_CNT ;
		ui32_battery_current_mA_acc = 0;		
	}
	
    // reset control parameters if... (safety)
    if ((ui8_brake_state)
	  || (!ui8_motor_enabled)
	  || (ui8_system_state == ERROR_MOTOR_BLOCKED)
	  || (ui8_system_state == ERROR_MOTOR_CHECK)
	  || (ui8_system_state == ERROR_BATTERY_OVERCURRENT)
	  || (ui8_system_state == ERROR_THROTTLE)
	  || (ui8_assist_level == OFF)
	  || (ui8_riding_mode_parameter == 0U)
	  || (ui8_battery_SOC_saved_flag)
	  || ((ui8_system_state != NO_ERROR)&&(!m_configuration_variables.ui8_assist_with_error_enabled))) {
		ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
        ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        ui8_controller_adc_battery_current_target = 0;
        ui8_controller_duty_cycle_target = 0;
    }
	else { // motor can run (no safety issue)
        // limit max current if higher than configured hardware limit (safety) 
        if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) {
            ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
        }
		
		// set limit battery overcurrent
		ui8_adc_battery_overcurrent = ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT;
		
        // limit target current if higher than max value (safety)
        if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }

        // limit target duty cycle if higher than max value
        if (ui8_duty_cycle_target > ui8_pwm_duty_cycle_max) {
            ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
        }

		// limit target duty cycle ramp up inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_up_inverse_step < PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        }

        // limit target duty cycle ramp down inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_down_inverse_step < PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        }
		
        // set duty cycle ramp up in controller
        ui8_controller_duty_cycle_ramp_up_inverse_step = ui8_duty_cycle_ramp_up_inverse_step;

        // set duty cycle ramp down in controller
        ui8_controller_duty_cycle_ramp_down_inverse_step = ui8_duty_cycle_ramp_down_inverse_step;

        // set target battery current in controller
        ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target;
		
		// set target duty cycle in controller
        ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
	}
	
    // check if the motor should be enabled or disabled
	// stop the motor e.g. if erps = 0 and current target and duty cycle are both 0
    if (ui8_motor_enabled
		&& ((ui8_brake_state)
			|| (ui8_system_state == ERROR_MOTOR_BLOCKED)
			|| (ui8_system_state == ERROR_MOTOR_CHECK)
			|| (ui8_system_state == ERROR_BATTERY_OVERCURRENT)
			|| (ui8_system_state == ERROR_THROTTLE)
			|| (ui8_battery_SOC_saved_flag)
			|| ((ui16_motor_speed_erps == 0)
				&& (ui8_adc_battery_current_target == 0U)
				&& (ui16_g_duty_cycle == 0U)))) {
        ui8_motor_enabled = 0;
        motor_disable_pwm();
    }
	else if (!ui8_motor_enabled
			&& (!ui8_brake_state)
			&& (ui16_motor_speed_erps < ERPS_SPEED_OF_MOTOR_REENABLING) // enable the motor only if it rotates slowly or is stopped
			&& (ui8_adc_battery_current_target > 0U)) {
		ui8_motor_enabled = 1;
		motor_enable_pwm(); // other actions are done here like reset pll timeout, reset fw, reset duty cycle
	}
}


// calculate motor ramp depending on speed and cadence
static void set_motor_ramp(void)
{
	uint8_t ui8_tmp;
	if (ui16_wheel_speed_x10 >= 200) {
        ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    }
	else {
        ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, // 10*4 = 40 -> 4 kph
                (uint8_t)50, // 50*4 = 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
        if (ui8_tmp < ui8_duty_cycle_ramp_up_inverse_step) {
            ui8_duty_cycle_ramp_up_inverse_step = ui8_tmp;
		}
        ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, // 10*4 = 40 -> 4 kph
                (uint8_t)50, // 50*4 = 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        if (ui8_tmp < ui8_duty_cycle_ramp_down_inverse_step) {
            ui8_duty_cycle_ramp_down_inverse_step = ui8_tmp;
		}
    }
}


// calculate startup boost & new pedal torque delta
static void apply_startup_boost(void)
{
	// startup boost mode
	switch (ui8_startup_boost_at_zero) {
		case CADENCE:
			ui8_startup_boost_flag = 1;
			break;
		case SPEED:
			if (ui16_wheel_speed_x10 == 0U) {
				ui8_startup_boost_flag = 1;
			}
			else if (ui8_pedal_cadence_RPM > 45) {
				ui8_startup_boost_flag = 0;
			}
			break;
	}
	// pedal torque delta & startup boost
	if (ui8_startup_boost_flag) {
		uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
		ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
	}
}


// calculate smooth start & new pedal torque delta; increase progressively the torque delta value
static void apply_smooth_start(void) 
{
	if ((ui8_pedal_cadence_RPM == 0U)&&(ui16_motor_speed_erps == 0U)) {
		ui8_smooth_start_flag = 1;
		ui8_smooth_start_counter = ui8_smooth_start_counter_set;
	}
	else if (ui8_smooth_start_flag) {
		if (ui8_smooth_start_counter > 0) {
			ui8_smooth_start_counter--;
		}
		else {
			ui8_smooth_start_flag = 0;
		}
		// pedal torque delta & smooth start; temp begin at 100, and go progressively down to 0
		uint16_t ui16_temp = 100 - ((ui8_smooth_start_counter * (uint8_t)100) / ui8_smooth_start_counter_set);
		ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta * ui16_temp) / 100;
	}
}


static void apply_power_assist(void)
{
	//mstrens :here current is based on adc_pedal_torque_delta * cadence * assist_factor * some ratio
	uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;

	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((ui8_pedal_cadence_RPM == 0U) &&
		   (ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 5; // substitute cadence for power formula (it was 1 instead of 5 but 1 gives near-zero at low PAS)
		}
	}

	// startup boost
	if (m_configuration_variables.ui8_startup_boost_enabled) {
		apply_startup_boost();
	}

  if ((ui8_pedal_cadence_RPM > 0U)||(ui8_startup_assist_adc_battery_current_target)) {
	// calculate torque on pedals + torque startup boost
    uint32_t ui32_pedal_torque_x100 = (uint32_t)(ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100);
	
    // calculate power assist by multiplying human power with the power assist multiplier
    uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
					* ui32_pedal_torque_x100) >> 8; // see note below ; IN tsdz2, it is 9; with 8 we can double the current for the same level

    /*------------------------------------------------------------------------

     NOTE: regarding the human power calculation

     (1) Formula: pedal power = torque * rotations per second * 2 * pi
     (2) Formula: pedal power = torque * rotations per minute * 2 * pi / 60
     (3) Formula: pedal power = torque * rotations per minute * 0.1047
     (4) Formula: pedal power = torque * 100 * rotations per minute * 0.001047
     (5) Formula: pedal power = torque * 100 * rotations per minute / 955
     (6) Formula: pedal power * 100  =  torque * 100 * rotations per minute * (100 / 955)
     (7) Formula: assist power * 100  =  torque * 100 * rotations per minute * (100 / 955) * (ui8_power_assist_multiplier_x50 / 50)
     (8) Formula: assist power * 100  =  torque * 100 * rotations per minute * (2 / 955) * ui8_power_assist_multiplier_x50
     (9) Formula: assist power * 100  =  torque * 100 * rotations per minute * ui8_power_assist_multiplier_x50 / 480

     ------------------------------------------------------------------------*/

    // calculate target current
    uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
    // set battery current target in ADC steps
    uint16_t ui16_adc_battery_current_target = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
    // set motor acceleration / deceleration
	set_motor_ramp();
	
    // set battery current target
    if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    }
	else {
        ui8_adc_battery_current_target = (uint8_t)ui16_adc_battery_current_target;
    }
	
	//#if STARTUP_ASSIST_ENABLED
	// set startup assist battery current target
	if (m_config.startup_assist_enabled){
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
  	}//#endif
	
    // set duty cycle target
    if (ui8_adc_battery_current_target) {
			ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
    }
	else {
        ui8_duty_cycle_target = 0;
    }
  }
}


 static void apply_torque_assist(void)
{
	// here current is based on torque * assist_factor
	// smooth start
	//#if SMOOTH_START_ENABLED
	if (m_config.smooth_start_enabled)  {
		apply_smooth_start();
	} // endif
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((ui8_pedal_cadence_RPM == 0U)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}

    // calculate torque assistance
    if (((ui16_adc_pedal_torque_delta > 0U)&&(ui8_pedal_cadence_RPM > 0U))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
        // get the torque assist factor
        uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;

        // calculate torque assist target current
        uint16_t ui16_adc_battery_current_target_torque_assist = (ui16_adc_pedal_torque_delta
                * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;

        // set motor acceleration / deceleration (adapt the ramp up and down inverse step) based on wheel speed and cadence (to react faster when running fast)
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = (uint8_t)ui16_adc_battery_current_target_torque_assist;
        }
		
		//#if STARTUP_ASSIST_ENABLED
		if (m_config.startup_assist_enabled) {
			// set startup assist battery current target
			if (ui8_startup_assist_flag) {
				if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
					ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
				}
				ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
			}
			else {
				ui8_startup_assist_adc_battery_current_target = 0;
			}
		} // endif

		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}// at this point, ui8_adc_battery_current_target is set and ui8_duty_cycle_target is 255 (or 0)

 static void apply_cadence_assist(void)
{
	// here we simulate a torque based on cadence * assist_factor
    if (ui8_pedal_cadence_RPM > 0U) {
		// simulated pedal torque delta ; riding mode parameter is the value based on assist mode and assist level selected on the display
		ui16_adc_pedal_torque_delta = ((uint16_t)ui8_riding_mode_parameter + (uint16_t)ui8_pedal_cadence_RPM) >> 2;
		
		// smooth start (increase progressively adc_pedal_torque_delta from 0 up to the asked value)
		if (ui8_smooth_start_counter_set < SMOOTH_START_RAMP_DEFAULT) {
			 ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
		}
		apply_smooth_start();
		ui8_smooth_start_counter_set = ui8_smooth_start_counter_set_temp;   		
		
        // set cadence assist current target
		uint16_t ui16_adc_battery_current_target_cadence_assist = ui16_adc_pedal_torque_delta;
		
		// restore pedal torque delta
		ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque_delta_temp;
		
		// set motor acceleration / deceleration // calculate motor ramp depending on speed and cadence
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_cadence_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = (uint8_t)ui16_adc_battery_current_target_cadence_assist;
        }
		
		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}


 static void apply_emtb_assist(void)
{
#define eMTB_ASSIST_DENOMINATOR_MIN			10
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((ui8_pedal_cadence_RPM == 0U)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}
	
	if (((ui16_adc_pedal_torque_delta)&&(ui8_pedal_cadence_RPM > 0U))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
		
		// get the eMTB assist denominator torque based
		uint16_t ui16_eMTB_assist_denominator = (508 - (ui8_riding_mode_parameter << 1));
		// get the eMTB assist denominator power based
		if (ui8_eMTB_based_on_power) {
			if (ui16_eMTB_assist_denominator >= ui8_pedal_cadence_RPM) {
				ui16_eMTB_assist_denominator -= ui8_pedal_cadence_RPM;
			}
			else {
				ui16_eMTB_assist_denominator = 0;
			}
		}
		ui16_eMTB_assist_denominator += eMTB_ASSIST_DENOMINATOR_MIN;
		
		// eMTB pedal torque delta calculation (progressive)
		uint16_t ui16_eMTB_adc_pedal_torque_delta = (uint16_t)((uint32_t)((ui16_adc_pedal_torque_delta * ui16_adc_pedal_torque_delta) + ui16_eMTB_assist_denominator)
			/ ui16_eMTB_assist_denominator);
		
		// set eMTB assist target current
		uint16_t ui16_adc_battery_current_target_eMTB_assist = ui16_eMTB_adc_pedal_torque_delta;
		
        // set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = (uint8_t)ui16_adc_battery_current_target_eMTB_assist;
        }
		
		//#if STARTUP_ASSIST_ENABLED
		if (m_config.startup_assist_enabled){
			// set startup assist battery current target
			if (ui8_startup_assist_flag) {
				if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
					ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
				}
				ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
			}
			else {
				ui8_startup_assist_adc_battery_current_target = 0;
			}
		} // endif
		
        // set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}


 static void apply_hybrid_assist(void)
{
	// here current is max of current calculated based on power and on torque assist mode.
	uint16_t ui16_adc_battery_current_target_power_assist;
	uint16_t ui16_adc_battery_current_target_torque_assist;
	uint16_t ui16_adc_battery_current_target;
	
	// smooth start
	//#if SMOOTH_START_ENABLED
	if (m_config.smooth_start_enabled) {
		apply_smooth_start();
	} // endif
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((ui8_pedal_cadence_RPM == 0U)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}	

	if ((ui8_pedal_cadence_RPM > 0U)||(ui8_startup_assist_adc_battery_current_target)) {
		// calculate torque assistance
		if (ui16_adc_pedal_torque_delta > 0U) {
			// get the torque assist factor
			uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter_array[TORQUE_ASSIST_MODE - 1][ui8_assist_level];
		
			// calculate torque assist target current
			ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
		}
		else {
			ui16_adc_battery_current_target_torque_assist = 0;
		}
	
		// calculate power assistance
		// get the power assist multiplier
		uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;
		// calculate torque on pedals //mstrens : seems only in VLCD5 version
		uint16_t ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100;
		// calculate power assist by multiplying human power with the power assist multiplier
		uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
				* ui16_pedal_torque_x100) >> 8; // see note below
	
		// calculate power assist target current x100
		uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
		// calculate power assist target current
		ui16_adc_battery_current_target_power_assist = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
		// set battery current target in ADC steps
		if (ui16_adc_battery_current_target_power_assist > ui16_adc_battery_current_target_torque_assist) {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_power_assist;
		}
		else {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
		}
		// set motor acceleration / deceleration
		set_motor_ramp();
	
		// set battery current target
		if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = (uint8_t)ui16_adc_battery_current_target;
		}
		
		//#if STARTUP_ASSIST_ENABLED
		if (m_config.startup_assist_enabled){
			// set startup assist battery current target
			if (ui8_startup_assist_flag) {
				if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
					ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
				}
				ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
			}
			else {
				ui8_startup_assist_adc_battery_current_target = 0;
			}
		} // endif
		
		// set duty cycle target
		if (ui8_adc_battery_current_target) {
			ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
		}
		else {
			ui8_duty_cycle_target = 0;
		}
	}
}




// when wheel speed is less than predefined, apply cadence mode, once wheel speed is reached, PID adapts duty_cycle to keep the speed 
 static void apply_cruise(void)
{
	#define CRUISE_PID_KP							4
	#define CRUISE_PID_KD							6
	#define CRUISE_PID_KI_X16						10
	#define CRUISE_PID_INTEGRAL_LIMIT				1000
	#define CRUISE_PID_OUTPUT_LIMIT					1000
	static int16_t i16_error;
	static int16_t i16_last_error;
	static int16_t i16_integral;
	static int16_t i16_derivative;
	static int16_t i16_control_output;
	static uint16_t ui16_wheel_speed_target_x10;
	
	static uint8_t ui8_cruise_PID_initialize = 0;
	static uint8_t ui8_cruise_assist_flag = 0;
	static uint8_t ui8_riding_mode_cruise = 0;
	static uint8_t ui8_riding_mode_cruise_temp = 0;
	static uint8_t ui8_cruise_threshold_speed_x10;
	
//#if CRUISE_MODE_ENABLED
	if (m_config.cruise_mode_enabled){
		// set cruise speed threshold
		ui8_cruise_threshold_speed_x10 = ui8_cruise_threshold_speed_x10_array[m_configuration_variables.ui8_street_mode_enabled];
		
		// verify riding mode change
		if (ui8_riding_mode_cruise_temp == ui8_riding_mode_cruise) {
			// enable cruise assist
			ui8_cruise_assist_flag = 1;
		}
		else {
			// for next verify riding mode change
			ui8_riding_mode_cruise_temp = ui8_riding_mode_cruise;
		}

		// mstrens :  I replaced all #if and { by apply_cruise_flag and this one is tetsed just after
		bool apply_cruise_flag = false;
		if (m_config.street_mode_cruise_enabled){
			if (m_config.cruise_mode_enabled) {
				if (m_config.enable_brake_sensor){
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
							&&(ui8_cruise_assist_flag)
							&&((ui8_pedal_cadence_RPM)||(ui8_cruise_button_flag)))   {apply_cruise_flag = true;}	
				} else {
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
						&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))         {apply_cruise_flag = true;}
				}
			} else {
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))             {apply_cruise_flag = true;}
			}
		} else {
			if (m_config.cruise_mode_walk_enabled) {
				if (m_config.enable_brake_sensor) {
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)
					&&((ui8_pedal_cadence_RPM)||(ui8_cruise_button_flag)))             {apply_cruise_flag = true;}
				} else {
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))			   {apply_cruise_flag = true;}
				}
			} else {
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))			   {apply_cruise_flag = true;}
			}
		}
		/*
		#if STREET_MODE_CRUISE_ENABLED
			#if CRUISE_MODE_WALK_ENABLED
				#if ENABLE_BRAKE_SENSOR
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
						&&(ui8_cruise_assist_flag)
						&&((ui8_pedal_cadence_RPM > 0U)||(ui8_cruise_button_flag)))
				#else
					if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
						&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM > 0U))
				#endif
			#else
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM > 0U))
			#endif
		#else
			#if CRUISE_MODE_WALK_ENABLED
				#if ENABLE_BRAKE_SENSOR
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)
					&&((ui8_pedal_cadence_RPM > 0U)||(ui8_cruise_button_flag)))
				#else
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM > 0U))
				#endif
			#else
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(!m_configuration_variables.ui8_street_mode_enabled)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))
			#endif
		#endif
		{ // part of the IF
		*/	
		if (apply_cruise_flag) {
			// for verify riding mode change
			ui8_riding_mode_cruise = CRUISE_MODE;
			
			// initialize cruise PID controller
			if (ui8_cruise_PID_initialize) {
				ui8_cruise_PID_initialize = 0;
				
				// reset PID variables
				i16_error = 0;
				i16_last_error = 0;
				i16_integral = 500; // initialize integral to a value so the motor does not start from zero
				i16_derivative = 0;
				i16_control_output = 0;
				
				// set cruise speed target
				ui16_wheel_speed_target_x10 = (uint16_t) (ui8_riding_mode_parameter_array[CRUISE_MODE - 1][ui8_assist_level] * (uint8_t)10);
			}
			
			// calculate error
			i16_error = (ui16_wheel_speed_target_x10 - ui16_wheel_speed_x10);
			
			// calculate integral
			i16_integral = i16_integral + i16_error;
			
			// limit integral
			if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT) {
				i16_integral = CRUISE_PID_INTEGRAL_LIMIT;
			}
			else if (i16_integral < 0) {
				i16_integral = 0;
			}
			
			// calculate derivative
			i16_derivative = i16_error - i16_last_error;
			
			// save error to last error
			i16_last_error = i16_error;

			// calculate control output ( output =  P I D )
			i16_control_output = (CRUISE_PID_KP * i16_error)
								+ ((CRUISE_PID_KI_X16 * i16_integral) >> 4)
								+ (CRUISE_PID_KD * i16_derivative);
			
			// limit control output to just positive values
			if (i16_control_output < 0) {
				i16_control_output = 0;
			}
			
			// limit control output to the maximum value
			if (i16_control_output > CRUISE_PID_OUTPUT_LIMIT) {
				i16_control_output = CRUISE_PID_OUTPUT_LIMIT;
			}
			
			// set motor acceleration / deceleration
			ui8_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
			ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
			
			// set battery current target
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
			
			// set duty cycle target  |  map the control output to an appropriate target PWM value
			ui8_duty_cycle_target = map_ui8((uint8_t) (i16_control_output >> 2),
				(uint8_t) 0,					// minimum control output from PID
				(uint8_t) 250,					// maximum control output from PID
				(uint8_t) 0,					// minimum duty cycle
				(uint8_t) PWM_DUTY_CYCLE_MAX);	// maximum duty cycle
		}
		else {
			// disable cruise assist
			ui8_cruise_assist_flag = 0;
			ui8_cruise_PID_initialize = 1;
			
			// for verify riding mode change
			ui8_riding_mode_cruise = CADENCE_ASSIST_MODE;
				
			// applies cadence assist up to cruise speed threshold
			ui8_riding_mode_parameter = ui8_riding_mode_parameter_array[CADENCE_ASSIST_MODE - 1][ui8_assist_level];
			apply_cadence_assist();
		}
	} // end cruise mode enabled = endif
}


static void apply_walk_assist(void)
{
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		// get walk assist duty cycle target
		ui8_walk_assist_duty_cycle_target = ui8_riding_mode_parameter + 20;
	}
	else {
		// get walk assist speed target x10
		ui8_walk_assist_speed_target_x10 = ui8_riding_mode_parameter;
		
		// set walk assist duty cycle target
		if ((!ui8_walk_assist_speed_flag)&&(ui16_motor_speed_erps == 0U)) {
			ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_STARTUP;
			ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_STARTUP;
			ui16_walk_assist_wheel_speed_counter = 0;
			ui16_walk_assist_erps_target = 0;
		}
		else if (ui8_walk_assist_speed_flag) {
			if (ui16_motor_speed_erps < ui16_walk_assist_erps_min) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					else {
						ui8_walk_assist_duty_cycle_max++;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_target) {
				ui8_walk_assist_adj_delay = (ui16_motor_speed_erps - ui16_walk_assist_erps_min) * WALK_ASSIST_ADJ_DELAY_MIN;
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = (ui16_walk_assist_erps_max - ui16_motor_speed_erps) * WALK_ASSIST_ADJ_DELAY_MIN;
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) {
						ui8_walk_assist_duty_cycle_target--;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps >= ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) {
						ui8_walk_assist_duty_cycle_target--;
						ui8_walk_assist_duty_cycle_max--;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
		}
		else {
			ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_STARTUP;
			
			if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
				if (ui16_wheel_speed_x10 > 0U) {
					if (ui16_wheel_speed_x10 > WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10) {
						ui8_walk_assist_duty_cycle_target--;
					}
					
					if (ui16_walk_assist_wheel_speed_counter++ >= 10) {
						ui8_walk_assist_duty_cycle_max += 10;
					
						// set walk assist erps target
						ui16_walk_assist_erps_target = ((ui16_motor_speed_erps * ui8_walk_assist_speed_target_x10) / ui16_wheel_speed_x10);
						ui16_walk_assist_erps_min = ui16_walk_assist_erps_target - WALK_ASSIST_ERPS_THRESHOLD;
						ui16_walk_assist_erps_max = ui16_walk_assist_erps_target + WALK_ASSIST_ERPS_THRESHOLD;
					
						// set walk assist speed flag
						ui8_walk_assist_speed_flag = 1;
					}
				}
				else {
					if ((ui8_walk_assist_duty_cycle_max + 10) < WALK_ASSIST_DUTY_CYCLE_MAX) {
						ui8_walk_assist_duty_cycle_target++;
						ui8_walk_assist_duty_cycle_max++;
					}
				}
				ui8_walk_assist_duty_cycle_counter = 0;
			}
		}
	}

	if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) {
		ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX;
	}
	
	// set motor acceleration / deceleration
	ui8_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
	ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
	
	// set battery current target
	ui8_adc_battery_current_target = ui8_min(WALK_ASSIST_ADC_BATTERY_CURRENT_MAX, ui8_adc_battery_current_max);

	// set duty cycle target   // so here duty_cycle_target can be less than 255
	ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;
}


static void apply_torque_sensor_calibration(void)
{
#define PEDAL_TORQUE_ADC_STEP_MIN_VALUE		160 //  20 << 3
#define PEDAL_TORQUE_ADC_STEP_MAX_VALUE		800 // 100 << 3
	
	static uint8_t ui8_step_counter;
	
	if (ui8_torque_sensor_calibration_with_weight_flag) {
		// increment pedal torque step temp
		if (ui8_step_counter++ & 0x01) {
			ui16_pedal_torque_step_temp++;
		}
		if (ui16_pedal_torque_step_temp > PEDAL_TORQUE_ADC_STEP_MAX_VALUE) {
			ui16_pedal_torque_step_temp = PEDAL_TORQUE_ADC_STEP_MIN_VALUE;
		}
		// pedal torque 10 bit ADC step x100 detected
		ui8_pedal_torque_per_10_bit_ADC_step_detected_x100 = ui16_pedal_torque_step_temp >> 3;
		
		// pedal weight (from LCD3 version)
		ui16_pedal_weight_x100 = (uint16_t)(((uint32_t) ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_detected_x100 * 100) / 167);
		//uint16_t ui16_adc_pedal_torque_delta_simulation = 100; // weight 20kg
		//uint16_t ui16_adc_pedal_torque_delta_simulation = 110; // weight 25kg
		//ui16_pedal_weight_x100 = (uint16_t)(((uint32_t) ui8_pedal_torque_per_10_bit_ADC_step_detected_x100 * ui16_adc_pedal_torque_delta_simulation * 100) / 167);
	}
	else {	
		ui16_pedal_torque_step_temp = PEDAL_TORQUE_ADC_STEP_MIN_VALUE;
	}
}


static void apply_throttle(void)
{
	//mstrens : Next line has been moved from motor.c to here to save time in irq 1 ; >>2 because we use 10 bits instead of 12 bits
	//ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0x0FFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
    // changed by mstrens to take care of infineon init
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_POT_RESULT_REG ) & 0x0FFF) >> 2; // throttle gr1 ch7 result 7  in bg  p2.5
	// map adc value from 0 to 255
	// here the 4 values refers to m_config values
    ui8_throttle_adc_in = map_ui8((uint8_t)(ui16_adc_throttle >> 2), // from 10 bits to 8 bits; then remap from 0 to 255
            (uint8_t) ADC_THROTTLE_MIN_VALUE, 
            (uint8_t) ADC_THROTTLE_MAX_VALUE, 
            (uint8_t) ASSIST_THROTTLE_MIN_VALUE,
            (uint8_t) ASSIST_THROTTLE_MAX_VALUE);

	// / set throttle assist
	if (ui8_throttle_adc_in) {  
		ui8_adc_throttle_assist = ui8_throttle_adc_in; // mstrens this could be done in all cases
	}
	else {
		ui8_adc_throttle_assist = 0;
	}
	
	// throttle mode pedaling ; Set assist on 0 when other conditions does not match
	switch (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) {
        case PEDALING: // Throttle assist only when pedaling
			if (ui8_pedal_cadence_RPM == 0U) {
				ui8_adc_throttle_assist = 0;
			}
          break;
        case W_O_P_6KM_H_AND_PEDALING: // Set assist on 0 when when speed is more than 6km/h and not pedaling 
			if ((ui16_wheel_speed_x10 > WALK_ASSIST_THRESHOLD_SPEED_X10) 
				&&(ui8_pedal_cadence_RPM == 0U)) {
				ui8_adc_throttle_assist = 0;
			}
          break;
		default:
          break;  
	}
	
	// map ADC throttle value from 0 to max battery current
    uint8_t ui8_adc_battery_current_target_throttle = map_ui8((uint8_t) ui8_adc_throttle_assist,
            (uint8_t) 0,
            (uint8_t) 255,
            (uint8_t) 0,
            (uint8_t) ui8_adc_battery_current_max);
	
    if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_target) {
        // set motor acceleration / deceleration // 255= 25,5 km/h
        if (ui16_wheel_speed_x10 >= 255) {
            ui8_duty_cycle_ramp_up_inverse_step = THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        }  
		else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        }

		// set battery current target
		if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;
		}
		
		// set duty cycle target
		ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;
    }// end of current_target_trottle > current_target
}

/*
static void apply_temperature_limiting(void)
{
    // next line has bee moved from motor.c to here to save time in irq 1
	//ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5    
	// changed by mstrens to take care of infineon init
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_POT_RESULT_REG ) & 0x0FFF) >> 2; // throttle gr1 ch7 result 7  in bg  p2.5
	
	// get ADC measurement
    uint16_t ui16_temp = ui16_adc_throttle;

    // filter ADC measurement to motor temperature variable
    ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 8);

    // convert ADC value
    ui16_motor_temperature_filtered_x10 = (uint16_t)(((uint32_t) ui16_adc_motor_temperature_filtered * 10000) / 2048);

	#if (TEMPERATURE_SENSOR_TYPE == TMP36)
	if (ui16_motor_temperature_filtered_x10 > 500) {
		ui16_motor_temperature_filtered_x10 = ui16_motor_temperature_filtered_x10 - 500;
	}
	else {
		ui16_motor_temperature_filtered_x10 = 0;
	}
	#endif
				
    // min temperature value can not be equal or higher than max temperature value
    if (ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] >= ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE]) {
        ui8_adc_battery_current_target = 0;
    }
	else {
        // adjust target current if motor over temperature limit
        ui8_adc_battery_current_target = (uint8_t)map_ui16(ui16_motor_temperature_filtered_x10,
				(uint16_t) ((uint8_t)ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] * (uint8_t)10U),
				(uint16_t) ((uint8_t)ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] * (uint8_t)10U),
				ui8_adc_battery_current_target,
				0);
	}
}
*/

static void apply_speed_limit(void)
{
	if (m_configuration_variables.ui8_wheel_speed_max > 0U) {
		uint16_t speed_limit_low  = (uint16_t)((uint8_t)(m_configuration_variables.ui8_wheel_speed_max - 2U) * (uint8_t)10U); // casting literal to uint8_t ensures usage of MUL X,A
		uint16_t speed_limit_high = (uint16_t)((uint8_t)(m_configuration_variables.ui8_wheel_speed_max + 2U) * (uint8_t)10U);
		// mstrens : light diff with 860C version		
		if (ui16_wheel_speed_x10 > speed_limit_high) {
			ui8_duty_cycle_target = 0;
		}
		
        // set battery current target
        ui8_adc_battery_current_target = (uint8_t)map_ui16(ui16_wheel_speed_x10,
                speed_limit_low,
                speed_limit_high,
                ui8_adc_battery_current_target,
                0U);
    }   
}


static void calc_wheel_speed(void)
{
    // calc wheel speed (km/h x10)
	uint16_t ui16_tmp = ui16_wheel_speed_sensor_ticks; // copy the value from isr    
    if (ui16_tmp) {
        // rps = PWM_CYCLES_SECOND / ui16_wheel_speed_sensor_ticks (rev/sec)
        // km/h*10 = rps * ui16_wheel_perimeter * ((3600 / (1000 * 1000)) * 10)
        // !!!warning if PWM_CYCLES_SECOND is not a multiple of 1000
		// mstrens : PWM_CYCLES_SECOND = 19047
        ui16_wheel_speed_x10 = (uint16_t)(((uint32_t) m_configuration_variables.ui16_wheel_perimeter * ((PWM_CYCLES_SECOND/1000)*36U)) / ui16_tmp);
	}
	else {
		ui16_wheel_speed_x10 = 0;
    }
	// addded by mstrens (I think)
	if (ui8_wheel_speed_simulate > 0){ 
		ui16_wheel_speed_x10 = ui8_wheel_speed_simulate * 10;
	}	
}

static void calc_cadence(void)
{
    // get the cadence sensor ticks from the ISR
    uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;

    // adjust cadence sensor ticks counter min depending on wheel speed
    ui16_cadence_ticks_count_min_speed_adj = map_ui16(ui16_wheel_speed_x10,
            40,
            400,
            CADENCE_SENSOR_CALC_COUNTER_MIN / 19,            //4270
            CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED /19); //341

    // calculate cadence in RPM and avoid zero division
    // !!!warning if PWM_CYCLES_SECOND > 21845
	// * 3 because 60 sec in 1 min and 20 magnetic in one pedal rotation
    if (ui16_cadence_sensor_ticks_temp > 0U) {
        ui8_pedal_cadence_RPM = (uint8_t)((PWM_CYCLES_SECOND * 3U) / ui16_cadence_sensor_ticks_temp);
		
		if (ui8_pedal_cadence_RPM > 120) {
			ui8_pedal_cadence_RPM = 120;
		}
	}
	else {
        ui8_pedal_cadence_RPM = 0;
	}
	
	/*-------------------------------------------------------------------------------------------------

     NOTE: regarding the cadence calculation

     Cadence is calculated by counting how many ticks there are between two LOW to HIGH transitions.

     Formula for calculating the cadence in RPM:

     (1) Cadence in RPM = (60 * PWM_CYCLES_SECOND) / CADENCE_SENSOR_NUMBER_MAGNETS) / ticks

     (2) Cadence in RPM = (PWM_CYCLES_SECOND * 3) / ticks

     -------------------------------------------------------------------------------------------------*/
	 // added by mstrens
	 // we also calculate the ratio of decrease of cadence in order to allow faster reaction of assistance when pedal pressure reduce
	// i16_pedal_cadence_RPM_decrease_ratio = 0;
	// if ( ui8_pedal_cadence_RPM > 0){
	//	int16_t i16_pedal_cadence_RPM_difference = (int16_t) ui8_pedal_cadence_RPM_previous - (int16_t) ui8_pedal_cadence_RPM;
	//	if (i16_pedal_cadence_RPM_difference > 0) {
	//		i16_pedal_cadence_RPM_decrease_ratio = (i16_pedal_cadence_RPM_difference << 8) / ui8_pedal_cadence_RPM;
	//	} 
	//} 
	 ui8_pedal_cadence_RPM_previous = ui8_pedal_cadence_RPM;
}



void get_battery_voltage(void)
{
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT   2

    /*---------------------------------------------------------
     NOTE: regarding filter coefficients

     Possible values: 0, 1, 2, 3, 4, 5, 6
     0 equals to no filtering and no delay, higher values
     will increase filtering but will also add a bigger delay.
     ---------------------------------------------------------*/

    static uint16_t ui16_adc_battery_voltage_accumulated;
	
    // low pass filter the voltage readed value, to avoid possible fast spikes/noise
    ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
    ui16_adc_battery_voltage_accumulated += ui16_adc_voltage;
	ui16_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
    ui16_battery_voltage_filtered_x1000 = ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}

//in openTX expo : f(x) = (k*x*x*x/(1024*1024) + x*(256-k) + 128) / 256 ; k is between 0 and 256, x is between 0 and 1024
//expo : f(x) = (k*x*x*x/(256*256) + x*(256-k) + 128) / 256 ; k is between 0 and 256, x is between 0 and 256

// return is in range 0/255
//#define EXTENDED_EXPO 1
// calculate expo only for positive number with positive ratio
unsigned int expou(unsigned int x, unsigned int k)
{
#if defined(EXTENDED_EXPO)
  bool extended;
  if (k > 204) {  // it was 80 when k was in range 0/100 ; so 80* 256 /100 = 204
    extended=true;
  }
  else {
    k += (k>>2);  // use bigger values before extend, because the effect is anyway very very low
    extended=false;
  }
#endif

  //k = calc100to256(k);  // was activated when k was in range 0/100

  uint32_t value = (uint32_t) x*x;
  value *= (uint32_t)k;
  value *= (uint32_t)x;

#if defined(EXTENDED_EXPO)
  if (extended) {  // for higher values do more multiplications to get a stronger expo curve
    value >>= 16;
    value *= (uint32_t)x;
    value >>= 4;
    value *= (uint32_t)x;
  }
#endif

  value >>= 16;
  value += (uint32_t)(256-k) * x + 128;

  return value >> 8;
}

//#define RESXu 1024u for openTX
#define RESXu 256u
// calculate expo for positive or negative parameters
int expo(int x, int k)
{
  if (k == 0) {
    return x;
  }

  int y;
  bool neg = (x < 0);

  if (neg) {
    x = -x;
  }
  if (x > (int)RESXu) {
    x = RESXu;
  }
  if (k < 0) {
    y = RESXu - expou(RESXu-x, -k);
  }
  else {
    y = expou(x, k);
  }
  return neg ? -y : y;
}

// here the code for get_pedal_torque()
#define TOFFSET_CYCLES 120 // 3sec (25ms*120)
static uint8_t toffset_cycle_counter = 0;
// get_pedal_torque has been totally rewrittent for TSDZ8 (do not update based on TSDZ2)

#if  (USE_SPIDER_LOGIC_FOR_TORQUE > (0)) 
static uint16_t ui16_TSamples[21];
static uint8_t ui8_TSamplesNum = 0;
static uint8_t ui8_TSamplesPos = 0;
static uint16_t ui16_TSum = 0;
//static uint8_t ui8_adc_pedal_torque_delta = 0;  // added by mstrens to save the remap torque in uint8_t
#if (USE_SPIDER_LOGIC_FOR_TORQUE == (2))
static uint16_t ui16_TExpected[21];  // expected torque value based on expected in previous rotation and difference in TSamples
static uint16_t ui16_TExpectedNew ;
#endif
static uint16_t ui16_adc_pedal_torque_filtered_noExpo ;
uint16_t ui16_TSampleOld;
// PWM IRQ set ui8_pas_new_transition when a new PAS signal transition is detected.
// 20 transtions/revolution (one every 18 deg)
// @120 rmp: 40 transitions/sec 1 every 25 ms
// on exit, ui16_TSum contains the sum of 20 values (if ui8_TSampleNum = 20 = buffer is full)
void new_torque_sample() {

    if (ui8_pas_new_transition & 0x80) {
    	// Pedal stop or backward rotation -> reset all
        ui8_pas_new_transition = 0;
        ui8_TSamplesNum = 0;
        ui16_TSum = 0;
        ui8_TSamplesPos = 0;
        return;
    }

    ui8_pas_new_transition = 0; // reset the flag saying there is a PAS transition
	uint16_t ui16_TorqueDeltaADC_norm = 0; // will contain the raw remap delta value
	uint16_t ui16_adc_torque = ui16_adc_torque_filtered; // save the value being calculated in motor.c because it can change in irq
	if ( ui16_adc_torque > ui16_adc_pedal_torque_offset) {
    	// map the delta value to max 160.
		ui16_TorqueDeltaADC_norm = ((uint32_t)(ui16_adc_torque - ui16_adc_pedal_torque_offset) * ADC_TORQUE_SENSOR_RANGE_TARGET) /ui16_adc_pedal_torque_range ;
		if (ui16_TorqueDeltaADC_norm > ADC_TORQUE_SENSOR_RANGE_TARGET) { // for safety
			ui16_TorqueDeltaADC_norm = ADC_TORQUE_SENSOR_RANGE_TARGET;
		}
	}
    #if ((USE_SPIDER_LOGIC_FOR_TORQUE == (1)) || (USE_SPIDER_LOGIC_FOR_TORQUE == (2)))
	// when define == 3, we do not reset the buffer
    if (ui16_TorqueDeltaADC_norm == 0) {   // perhaps this could be omitted
    	// torque adc value less than 0 torque reference ADC -> reset all
        ui8_TSamplesNum = 0;
        ui16_TSum = 0;
        ui8_TSamplesPos = 0;
        return;
    }
	#endif
	ui16_TSampleOld = ui16_TSamples[ui8_TSamplesPos] ; // latest value that will be lost (valid only if buffer is full = TSamplesNum == 20)
    ui16_TSamples[ui8_TSamplesPos] = ui16_TorqueDeltaADC_norm; // store the new delta value value
    ui16_TSum += ui16_TorqueDeltaADC_norm; // Add to the average the new sample
    if (ui8_TSamplesNum == 20) { // when buffer is already full
        // Remove from the sum the sample at the same pedal position of the previous pedal stroke (saved in old)
        if (ui16_TSum > ui16_TSampleOld) { 
			ui16_TSum -= ui16_TSampleOld;
		} else {              // this should not happen because the value should already be part of the sum; added for safety
			ui16_TSum = 0; 
		}
		#if (USE_SPIDER_LOGIC_FOR_TORQUE == (2))
		// calculate new expected = old Expected(in the same pedal position) + new torque - old torque (only if result is posiif, else 0)
		ui16_TExpectedNew = ui16_TExpected[ui8_TSamplesPos] + ui16_TorqueDeltaADC_norm;
		if (ui16_TExpectedNew > ui16_TSampleOld) {
			ui16_TExpectedNew -= ui16_TSampleOld;
		} else {
			ui16_TExpectedNew = 0; 
		}
		// save the expected torque; to be used in get_pedal_torque;
		ui16_TExpected[ui8_TSamplesPos] = ui16_TExpectedNew;
		#endif
    } else {
        ui8_TSamplesNum++;
		#if (USE_SPIDER_LOGIC_FOR_TORQUE == (2))
		if (ui8_TSamplesNum == 20){ 
        	// fill expected array with the average
			ui16_TExpectedNew = ui16_TSum / 20;
			for (uint8_t i=0;i<20;i++){
				ui16_TExpected[i] = ui16_TExpectedNew;
    		}
		}
		#endif
	}
	ui8_TSamplesPos++;  // increase the pos
	if (ui8_TSamplesPos >= 20) {
        ui8_TSamplesPos = 0;
	}

}

//( (USE_SPIDER_LOGIC_FOR_TORQUE > 0 (so 1, 2, 3)
#define TORQUE_SENSOR_ADC_REMAP_NORM_DIFF_MAX 100 // max value is 160
static void get_pedal_torque(void) {
	if (toffset_cycle_counter < TOFFSET_CYCLES) {  // less than 3 sec
		ui16_adc_pedal_torque_offset_init = filter(ui16_adc_torque_filtered, ui16_adc_pedal_torque_offset_init , 4) ; // get filtered torque captured in motor.c irq1
		toffset_cycle_counter++;
		if ((toffset_cycle_counter == TOFFSET_CYCLES)&&(ui8_torque_sensor_calibrated)) {
			if ((ui16_adc_pedal_torque_offset_init > ui16_adc_pedal_torque_offset_min)&& 
			(ui16_adc_pedal_torque_offset_init < ui16_adc_pedal_torque_offset_max)) {
				ui8_adc_pedal_torque_offset_error = 0;
			}
			else {
				ui8_adc_pedal_torque_offset_error = 1;
			}
		}
		ui16_adc_pedal_torque = ui16_adc_pedal_torque_offset_init;
	} else { // after 3 sec
		ui16_adc_pedal_torque = ui16_adc_torque_filtered; // ui16_adc_torque_filtered is the value calculated in irq
	}
	ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_set ; // this value is received from the config (in 860C)
	ui16_adc_pedal_torque_delta = 0; // this is the final value to retun 
	uint16_t ui16_TorqueDeltaADC_norm = 0;
	uint16_t ui16_adc_torque = ui16_adc_torque_filtered; // save the value being calculated in motor.c because it can change in irq
	if ( ui16_adc_torque > ui16_adc_pedal_torque_offset) {
		// map the delta value to max 160.
		ui16_TorqueDeltaADC_norm = ((uint32_t)(ui16_adc_torque - ui16_adc_pedal_torque_offset) * ADC_TORQUE_SENSOR_RANGE_TARGET) /ui16_adc_pedal_torque_range ;
		if (ui16_TorqueDeltaADC_norm > ADC_TORQUE_SENSOR_RANGE_TARGET) { // for safety
			ui16_TorqueDeltaADC_norm = ADC_TORQUE_SENSOR_RANGE_TARGET;
		}	
	} else {
		ui16_adc_pedal_torque_filtered_noExpo = 0 ;  // reset filtered no expo when torque is 0
	}	
#if ((USE_SPIDER_LOGIC_FOR_TORQUE == (1)) || (USE_SPIDER_LOGIC_FOR_TORQUE == (3)) )
	// when TSampleNum == 20,
	//                     if difference with previous at the same position,is low, use the average (no filter because already average over one rotation)
	//                     else ; use new value but with filtering
	// else use the new value after some filtering
	if (ui8_TSamplesNum == 20)  { // replace by an average when difference is low
		uint16_t ui16_tmp ;  // difference with previous position
		if (ui16_TorqueDeltaADC_norm > ui16_TSampleOld) {
			ui16_tmp =  ui16_TorqueDeltaADC_norm - ui16_TSampleOld;
		} else {
			ui16_tmp = ui16_TSampleOld - ui16_TorqueDeltaADC_norm;
		}

		if (ui16_tmp < TORQUE_SENSOR_ADC_REMAP_NORM_DIFF_MAX  ) {
			ui16_adc_pedal_torque_filtered_noExpo = ui16_TSum / ((uint8_t)20); // overwrite with avg when difference with previous rotation is low
		} else {      // when difference is big, apply a filter
			ui16_adc_pedal_torque_filtered_noExpo = filter( ui16_TorqueDeltaADC_norm , ui16_adc_pedal_torque_filtered_noExpo , 5);
		}
	} else {   // when buffer is not full
		#if (USE_SPIDER_LOGIC_FOR_TORQUE == (1)) 
		ui16_adc_pedal_torque_filtered_noExpo = filter( ui16_TorqueDeltaADC_norm , ui16_adc_pedal_torque_filtered_noExpo , 5); 
		#else // (USE_SPIDER_LOGIC_FOR_TORQUE == (3) we use the average
		if (ui8_TSamplesNum > 0 && ui16_TorqueDeltaADC_norm > 0) {
			ui16_adc_pedal_torque_filtered_noExpo = ui16_TSum / ui8_TSamplesNum; // partial rotation: avg of 1-19 samples
		} else {
			// No samples yet, or no current torque (foot off pedal) — use filtered ADC.
			// Decays to 0 when rider stops; responds instantly at standstill.
			ui16_adc_pedal_torque_filtered_noExpo = filter(ui16_TorqueDeltaADC_norm,
				ui16_adc_pedal_torque_filtered_noExpo, 5);
		}	
		#endif
	}

#else  // (USE_SPIDER_LOGIC_FOR_TORQUE == (2))
	// when ui8_TSamplesNum == 20, we can compare with previous sample and we use Texpected that has been calculated
	// else, we use current value (remapped to 160) and filtered
	if (ui8_TSamplesNum == 20){
		ui16_adc_pedal_torque_filtered_noExpo = filter( ui16_TExpectedNew, ui16_adc_pedal_torque_filtered_noExpo, 4);
	} else {  // TSampleNum =0/19;
		ui16_adc_pedal_torque_filtered_noExpo = filter( ui16_TorqueDeltaADC_norm , ui16_adc_pedal_torque_filtered_noExpo , 5); 
	}
#endif	
	ui16_adc_pedal_torque_delta =  expo(
				(int) ui16_adc_pedal_torque_filtered_noExpo  ,
				 ((int) ui8_adc_pedal_torque_range_adj - 20) * 12 ); // apply expo ; *12 because expo expect a value in range -256/+256
		
	// here ui16_adc_pedal_torque_delta is known
	ui16_adc_pedal_torque_delta_temp = ui16_adc_pedal_torque_delta;
	
	// for cadence sensor check
	ui16_adc_pedal_torque_delta_no_boost = ui16_adc_pedal_torque_delta;
		
	// calculate torque on pedals // only in 860C the variable is static and used in display interface
	uint16_t ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100;
	
	// calculate human power x10
	// mstrens : always use ui16_pedal_torque_x100 ; discard m_configuration_variables.ui8_torque_sensor_adv_enabled
	ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 96); // see note below
}
#else // NORMAL or KATANA logic : (USE_SPIDER_LOGIC_FOR_TORQUE == (0)) 
      //       so with KATANA logic or Max logic
#if (USE_KATANA1234_LOGIC_FOR_TORQUE == (1))
#define KATANA_BUFFER_LEN 40
uint8_t katana_buffer[KATANA_BUFFER_LEN] ;
uint8_t katana_index = 0;
uint8_t katana_count = 0;
uint16_t katana_sum = 0; 	
#endif
#if (USE_KATANA1234_LOGIC_FOR_TORQUE == (2))
#define KATANA_BUFFER_LEN_MAX (40) // firt tested with 50 and this was not very positive
#define KATANA_BUFFER_LEN_MIN (40) // first used 30
#define KATANA_2_POWER (6)                    // for a size of 64
#define KATANA_MODULO ((1<<KATANA_2_POWER)-1)
uint8_t katana_buffer[1<<KATANA_2_POWER] ; // using a power of 2 allow faster modulo: 64 = 2^6

uint8_t katana_buffer_len = KATANA_BUFFER_LEN_MAX; 
uint8_t katana_next_read = 0;
uint8_t katana_next_write = 0;
uint8_t katana_count = 0;
uint16_t katana_sum = 0; 	
#endif

static uint16_t ui16_adc_pedal_torque_noExpo;

 static void get_pedal_torque(void)
{
			// parameters that could be filled by the user are
			// adc_pedal_torque_offset_adj (range 0/34 in 860C param, default 20,Rx_buf[50] ); Not used because in javaconfigurator range is -20/+20
			//                            value transmitted is display * range/160 when calib is enabled, display value when disabled 
			// adc_pedal_torque_range_adj (range 0/40, default 20, Rx_buf[51]) : is used for expo
			// adc_pedal_torque_angle_adj (range 160/16 based on an index being filled by the user, Rx_buf[52]); so can't be used here
			// ui16_adc_pedal_torque_offset_set (Rx_buf[76-77])= ADC whith no load + a margin (= value as defined in 860C but user has to add the margin
			// in 860C, user can fill an offset_max but this is not transmitted (but used to calculate range)
			// ui8_pedal_torque_per_10_bit_ADC_step_x100 (Tx_buf[83]) is used to calculate human torque.
			//            The transmitted value is filled in torque_sensor_Step (not calibrated) or in torque_sensor_Step_adv (calibrated) 
			// ui8_assist_without_pedal_rotation_threshold (Tx_buf[84]) is transmitted and used
			// 
			// some values are transmitted but can't be filled by user
			// ui16_adc_pedal_torque_range (Rx_buf[78-79])=  range calculated in 860c base on torque_sensor_ADC_offset and torque_sensor_ADC_max filled in 860C
			// ui8_adc_torque_calibration_offset (Rx_buf[53]); it is not used because it can't be modified by user and depends on calib is enabled or not
			// ui8_adc_torque_middle_offset_adj (Rx_buf[54]) ; it is not used ; replaced by a fixed value 20 as middel of range_adj

		// we have to take care of an offset in ADC torque value.
		// Experience shows that for TSDZ8 ADC value with no load depends on the position of the pedal.
		// So the value measured during the first 3 sec (with no load) is not valid for further handling
		// Best is to use a value provided by the user.
		// We have also to add a margin (for safety and to allow a kind of dead zone for small pressure).
		// In TSDZ2 there are 2 margins (one hardcoded and one to be filled by user).
		// Furthermore, the applied margin depends on range and on calibration enabled or not.
		// Furthermore, the margin has a range 0/34 in 860c and -20/+20 (but transmitted 0/40) in VLCD5 javaconfigurator 
		// this is complex.
		// to make it easier, for TSDZ8, margin has to be included in ui16_adc_pedal_torque_offset
		// So ui16_adc_pedal_torque_offset has to be provided by the user and should be
		//   the max ADC value of the torque sensor with no load (but over a 360° pedal rotation) + a margin
		
	if (toffset_cycle_counter < TOFFSET_CYCLES) {  // less than 3 sec
		// filter again the adc_torque_filtered value
		ui16_adc_pedal_torque_offset_init = filter(ui16_adc_torque_filtered, ui16_adc_pedal_torque_offset_init , 4) ; // get filtered torque captured in motor.c irq1
		toffset_cycle_counter++;
		// mstrens when offset in configuration is very low, we use the parameter in the config as the margin 
		// and we add it to the init value
		if (ui8_torque_sensor_calibrated) {
			if ( m_config.pedal_torque_adc_offset < 50 ){  
				ui16_adc_pedal_torque_offset_set = ui16_adc_pedal_torque_offset_init + m_config.pedal_torque_adc_offset;
			} else { 
				ui16_adc_pedal_torque_offset_set = m_config.pedal_torque_adc_offset;
			}
		} else {   // not calibrated 
			ui16_adc_pedal_torque_offset_set = ui16_adc_pedal_torque_offset_init + PEDAL_TORQUE_ADC_OFFSET_MARGIN_DEFAULT;
		}	
		// check the offset calibration at the end of the 3 sec delay with
		// the value min and max calculated based on the offset (no load) value sent by 860c and some tolerances (for min)
		if (toffset_cycle_counter == TOFFSET_CYCLES) {  
			if ((ui16_adc_pedal_torque_offset_init > ui16_adc_pedal_torque_offset_min)&& 
			(ui16_adc_pedal_torque_offset_init < ui16_adc_pedal_torque_offset_max)) {
				ui8_adc_pedal_torque_offset_error = 0;
			}
			else {
				ui8_adc_pedal_torque_offset_error = 1;
			}
		}
		ui16_adc_pedal_torque = ui16_adc_pedal_torque_offset_init;
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_set ;	
	} else { // after 3 sec
		// we recalculate the offset because changing some value on the display did not updated the offset without a reset
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_set ;
		// commented by mstrens 
		/*
		if (ui8_pedal_cadence_RPM) {
			ui16_adc_pedal_torque_offset -= ui8_adc_torque_middle_offset_adj; // take care of the ratio range/target (160)
			ui16_adc_pedal_torque_offset += ui8_adc_pedal_torque_offset_adj; // take care of the ratio range/target (160)
		}
		
		if ((ui8_coaster_brake_enabled)&&(ui16_adc_pedal_torque_offset > ui8_coaster_brake_torque_threshold)) {
			//ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset - ui8_coaster_brake_torque_threshold;
			ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset_cal - ui8_coaster_brake_torque_threshold;
		}
		else {
			ui16_adc_coaster_brake_threshold = 0;
		}
		*/
		
		// was used only when katana and spider was not used
		// When cadence decrease ratio exceed some value, we reset the current and previous Max rotation value
		// max cadence is 120; so max cadence decrease ratio is (120-1)*256/(120+1) = 256
		// (2-1)*256/1 = 256
		// (110-90)*256/100 = 25
 		//#define CADENCE_DECREASE_RATIO 25
		//if (i16_pedal_cadence_RPM_decrease_ratio > CADENCE_DECREASE_RATIO) {
		//	ui16_adc_torque_actual_rotation = 0;
		//	ui16_adc_torque_previous_rotation = 0 ;
		//	ui8_adc_torque_rotation_reset = 1 ; // will force also a reset in the motor.c irq to be safe and reset rpm counter
		//}

		/*
		// get adc pedal torque
		// by default we use ui16_adc_torque_filtered (calculated in motor.c irq)
		// when cadence is high enough, we use the max between actual value, actual rotation and previous rotation
		ui16_adc_pedal_torque = ui16_adc_torque_filtered; // copy the value from irq because it can change
		#if (USE_KATANA1234_LOGIC_FOR_TORQUE == (0)) // when we use the logic based on the max of current, max of current rotation, max of previous rotation
		#define PEDAL_CADENCE_MIN_FOR_USING_ROTATION 30
		if (ui8_pedal_cadence_RPM > PEDAL_CADENCE_MIN_FOR_USING_ROTATION) { 
			if ( ui16_adc_pedal_torque < ui16_adc_torque_actual_rotation) ui16_adc_pedal_torque = ui16_adc_torque_actual_rotation ;
			if ( ui16_adc_pedal_torque < ui16_adc_torque_previous_rotation) ui16_adc_pedal_torque = ui16_adc_torque_previous_rotation ;
		} else {
			ui8_adc_torque_rotation_reset = 1 ; // will force also a reset of torque rotation in the motor.c irq and so we use the actual value 
		}
		#endif
		*/
	}

	// here we know the ui16_adc_pedal_torque but we still have to take care of 
	// - offset
	// - remap in order to have a max range of 160 (value used by TSDZ2
	uint16_t ui16_adc_pedal_torque_delta_160 = 0; // first set delat = 0 (= delta when lower than offset)
	if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset ) {
		ui16_adc_pedal_torque_delta_160 = ((ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset) * ADC_TORQUE_SENSOR_RANGE_TARGET) / ui16_adc_pedal_torque_range ;
		if (ui16_adc_pedal_torque_delta_160 > ADC_TORQUE_SENSOR_RANGE_TARGET) ui16_adc_pedal_torque_delta_160 = ADC_TORQUE_SENSOR_RANGE_TARGET;
	}
	
	#if (USE_KATANA1234_LOGIC_FOR_TORQUE == (1))
		// no cadence -> immediately clear the buffer to have a quicker starting / stopping reaction
		// fill the value one or several times in the buffer and calculate average
		if (ui8_pedal_cadence_RPM == 0U) { 			
			katana_sum = 0;
			katana_count = 0;
			katana_index = 0;
		}
		uint16_t katana_dif = 0;
		uint8_t katana_factor = 16;
		if (ui16_adc_pedal_torque_delta_160 > ui16_adc_pedal_torque_noExpo) katana_dif = ui16_adc_pedal_torque_delta_160 - ui16_adc_pedal_torque_noExpo;
		else katana_dif = ui16_adc_pedal_torque_noExpo - ui16_adc_pedal_torque_delta_160;
		
		if (katana_dif < 60) katana_factor = 1;
		else if (katana_dif < 80) katana_factor = 2;
		else if (katana_dif < 100) katana_factor = 4;
		else if (katana_dif < 130) katana_factor = 8;
		while ( katana_factor){  // fill buffer one or several times
			katana_sum += ui16_adc_pedal_torque_delta_160;
			if (katana_count < KATANA_BUFFER_LEN) katana_count++;
			else katana_sum -= katana_buffer[katana_index];
			katana_buffer[katana_index] = ui16_adc_pedal_torque_delta_160 ;
			katana_index++;
			if (katana_index >= KATANA_BUFFER_LEN) katana_index = 0;
			katana_factor--;
		}
		if (katana_count > 0) {
			ui16_adc_pedal_torque_delta_160 = katana_sum / katana_count ;
		}			
	#endif
	#if (USE_KATANA1234_LOGIC_FOR_TORQUE == (2))
		// no cadence -> immediately clear the buffer to have a quicker starting / stopping reaction
		// fill the value one or several times in the buffer and calculate average
		if (ui8_pedal_cadence_RPM == 0U) { 			
			katana_sum = 0;
			katana_count = 0;
			katana_next_read = 0;
			katana_next_write = 0;
			katana_buffer_len = KATANA_BUFFER_LEN_MAX;
		} else {
			// increase or decrease logical buffer length based on the cadence if required
			// cadence is in RPM; this function is called 60sec* 40call/sec = 2400 per min.
			// we try that buffer len == number of call per rotation.
			// still we respect some min/max limit and we increase/decrease the length only only by 1 at each call if needed
			// reduce execution time and smooth changes.
			uint16_t katana_len_max_cadence = 2400/ ui8_pedal_cadence_RPM;
			if (( katana_len_max_cadence > katana_buffer_len ) && (katana_buffer_len < KATANA_BUFFER_LEN_MAX )) {
				katana_buffer_len++;
			} else if (( katana_len_max_cadence < katana_buffer_len ) && (katana_buffer_len > KATANA_BUFFER_LEN_MIN )) {
				katana_buffer_len--;
				if (katana_count > 0) {
					katana_sum -= katana_buffer[katana_next_read];
					katana_count--;
					katana_next_read++;
					katana_next_read &= KATANA_MODULO; // reset to 0 when reach 64
				}
			}		
		}
		// number of values added depends on the (absolute) difference between current value and new one	
		uint16_t katana_dif = 0;
		uint8_t katana_factor = 16;
		if (ui16_adc_pedal_torque_delta_160 > ui16_adc_pedal_torque_noExpo) katana_dif = ui16_adc_pedal_torque_delta_160 - ui16_adc_pedal_torque_noExpo;
		else katana_dif = ui16_adc_pedal_torque_noExpo - ui16_adc_pedal_torque_delta_160;
		if (katana_dif < 60) katana_factor = 1;
		else if (katana_dif < 80) katana_factor = 2;
		else if (katana_dif < 100) katana_factor = 4;
		else if (katana_dif < 130) katana_factor = 8;
		while ( katana_factor){  // fill buffer one or several times
			if ( katana_count == katana_buffer_len){ // when buffer is totally used, first remove oldiest value (but do not increase count)
				katana_sum -= katana_buffer[katana_next_read];
				katana_next_read++;
				katana_next_read &= KATANA_MODULO;
			} else {                                // when there is place in the buffer, increase count
				katana_count++;
			}
			// add new value to sum and in the buffer
			katana_sum += ui16_adc_pedal_torque_delta_160;
			katana_buffer[katana_next_write] = ui16_adc_pedal_torque_delta_160 ;
			katana_next_write++;
			katana_next_write &= KATANA_MODULO;
			katana_factor--;
		}
		if (katana_count > 0) {
			ui16_adc_pedal_torque_delta_160 = katana_sum / katana_count ; // calculate average as final result
		}
			// check maximum : should not happen but added for safety.
		if 	(ui16_adc_pedal_torque_delta_160 > ADC_TORQUE_SENSOR_RANGE_TARGET) ui16_adc_pedal_torque_delta_160 = ADC_TORQUE_SENSOR_RANGE_TARGET;
	#endif
	// save the value before applying expo in order to calculate variation in katana1234 logic on next loop (here above)
	ui16_adc_pedal_torque_noExpo = ui16_adc_pedal_torque_delta_160 ; 
	
	// apply expo : value is in range 0/256 and so does not need to be remaped 
	//              coeff expect a value in range-256/256 while range_adj is in range 0/40
	//                      so we have to substract 20 and multiply by 12.
	ui16_adc_pedal_torque_delta =  expo(
		(int) ui16_adc_pedal_torque_noExpo  ,
			((int) ui8_adc_pedal_torque_range_adj - 20) * 12 ); // apply expo ; *12 because expo expect a value in range -256/+256

	// here ui16_adc_pedal_torque_delta is known
	ui16_adc_pedal_torque_delta_temp = ui16_adc_pedal_torque_delta;
	
	// for cadence sensor check
	ui16_adc_pedal_torque_delta_no_boost = ui16_adc_pedal_torque_delta;
	
    // calculate torque on pedals
    uint16_t ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100;
	
	// calculate human power x10
	// mstrens : always use ui16_pedal_torque_x100 ; discard m_configuration_variables.ui8_torque_sensor_adv_enabled
	ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 96); // see note below
	
	//if ((ui8_torque_sensor_calibrated)&&(m_configuration_variables.ui8_torque_sensor_adv_enabled)) {
	//	ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_calc_x100 * ui8_pedal_cadence_RPM) / 96);
	//}
	//else {
	//	ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 96); // see note below
	//}
	
	/*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
	------------------------------------------------------------------------*/
}
#endif // get_pedal_torque

/* not used when eeprom is not used
struct_configuration_variables* get_configuration_variables(void)
{
    return &m_configuration_variables;
}
*/

static void check_system(void)
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E09 ERROR_MOTOR_CHECK (E08 blinking for XH18)
// E09 shared with ERROR_WRITE_EEPROM ; ERROR in config also send this code to the display
#define MOTOR_CHECK_TIME_GOES_ALONE_TRESHOLD         	60 // 60 * 100ms = 6.0 seconds (100msec because we perform the check only once on 4 ticks of 25ms)
// changed by mstrens for TSDZ8
#define MOTOR_CHECK_ERPS_THRESHOLD                  	10 // for TSDZ2 with 8 poles :20 ERPS 
                                                           // for TSDZ8 with 4 poles we have 2 less erps for the mecanical speed
static uint8_t ui8_riding_torque_mode = 0;
static uint8_t ui8_motor_check_goes_alone_timer = 0U;
	
	// riding modes that use the torque sensor
	if (((m_configuration_variables.ui8_riding_mode == POWER_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == TORQUE_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == HYBRID_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == eMTB_ASSIST_MODE))
		&& (ui8_adc_throttle_assist == 0U)) {
			ui8_riding_torque_mode = 1;
	}
	else {
		ui8_riding_torque_mode = 0;
	}
	// Check if the motor goes alone and with current or duty cycle target = 0 (safety)
	if ((ui16_motor_speed_erps > MOTOR_CHECK_ERPS_THRESHOLD)
		&&((ui8_riding_torque_mode) || (m_configuration_variables.ui8_riding_mode == CADENCE_ASSIST_MODE))
		&& (ui8_adc_battery_current_target == 0U || ui8_duty_cycle_target == 0U)) {
			ui8_motor_check_goes_alone_timer++;
	}
	else {
		ui8_motor_check_goes_alone_timer = 0;
	}
	if (ui8_motor_check_goes_alone_timer > MOTOR_CHECK_TIME_GOES_ALONE_TRESHOLD) {
		ui8_system_state = ERROR_MOTOR_CHECK;
	}
	// added by mstrens
	#if (USE_CONFIG_FROM_COMPILATION != 1)
	// check that version in flash is compatible with this version defined when compiling 
	uint16_t * pConfig = (uint16_t *) ADDRESS_OF_M_CONFIG_FLASH;  // point to the begin of user preference parameters in flash 
	if (( *pConfig != m_config.main_version)){ // keep motor blocked error if the config main version is not the same as the version at compilation
		ui8_system_state = ERROR_MOTOR_CHECK;	
	}
	#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E02 ERROR_TORQUE_SENSOR
    // check torque sensor
	// mstrens : adapted for TSDZ8
    if (ui8_riding_torque_mode) {
		if ((ui16_adc_pedal_torque_offset > 300)
		  ||(ui16_adc_pedal_torque_offset < 10)
		  ||(ui16_adc_pedal_torque > 700)   // mstrens TSDZ8 has a higher value than TSDZ2
		  ||(ui8_adc_pedal_torque_offset_error)) {
			// set torque sensor error code
			ui8_system_state = ERROR_TORQUE_SENSOR;
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E03 ERROR_CADENCE_SENSOR
#define CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD          250 // 250 * 100ms = 25 seconds
#define ADC_TORQUE_SENSOR_DELTA_THRESHOLD				(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET >> 1) + 20)
	static uint8_t ui8_check_cadence_sensor_counter;
	
	// check cadence sensor
	if ((ui16_adc_pedal_torque_delta_no_boost > ADC_TORQUE_SENSOR_DELTA_THRESHOLD)
	  &&(!ui8_startup_assist_flag)&&(ui8_riding_torque_mode)
	  &&((ui8_pedal_cadence_RPM > 130)||(ui8_pedal_cadence_RPM == 0U))) {
		ui8_check_cadence_sensor_counter++;
	}
	else {
		ui8_check_cadence_sensor_counter = 0;
		// auto-clear cadence error when condition goes away (no restart needed)
		if (ui8_system_state == ERROR_CADENCE_SENSOR) {
			ui8_system_state = 0;
		}
	}
	
	if (ui8_check_cadence_sensor_counter > CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD) {
		// set cadence sensor error code
		ui8_system_state = ERROR_CADENCE_SENSOR;
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E08 ERROR_SPEED_SENSOR
#define CHECK_SPEED_SENSOR_COUNTER_THRESHOLD          125 // 125 * 100ms = 12.5 seconds
// changed by mstrens for TSDZ8
#define MOTOR_ERPS_SPEED_THRESHOLD	                  90 // 180 for TSDZ2; should be 2 X less for TSDZ8 because 4 poles instead of 8
// "if" added by mstrens to avoid error while testing
	if (ui8_test_mode_flag == NORMAL_RUNNING_MODE) {  // check only in normal running mode ; not when testing
		static uint16_t ui16_check_speed_sensor_counter;
		
		// check speed sensor
		if ((ui16_motor_speed_erps > MOTOR_ERPS_SPEED_THRESHOLD)
		&&(m_configuration_variables.ui8_riding_mode != WALK_ASSIST_MODE)
		&&(m_configuration_variables.ui8_riding_mode != CRUISE_MODE)) {
			ui16_check_speed_sensor_counter++;
		}
		else {
			ui16_check_speed_sensor_counter = 0;
		}
		
		if (ui16_wheel_speed_x10) {
			ui16_check_speed_sensor_counter = 0;
		}
		
		if (ui16_check_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) {
			// set speed sensor error code
			ui8_system_state = ERROR_SPEED_SENSOR;
		}
	} // error speed sensor

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E04 ERROR_MOTOR_BLOCKED
// #define MOTOR_BLOCKED_COUNTER_THRESHOLD in CONFIG.H
// are not used, left for ini file compatibility
#define MOTOR_BLOCKED_COUNTER_THRESHOLD_NEW				10  // 10 * 100ms = 1.0 seconds
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10_NEW	30 // 30 = 3.0 amps
#define MOTOR_BLOCKED_ERPS_THRESHOLD_NEW				10 // 20 ERPS for TSDZ2; so 10 for TSDZ8 (4 poles instead of 8 poles)

    static uint8_t ui8_motor_blocked_counter;

    // if battery current is over the current threshold and the motor ERPS is below threshold start setting motor blocked error code
    if ((ui8_battery_current_filtered_x10 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10_NEW)
      && (ui16_motor_speed_erps < MOTOR_BLOCKED_ERPS_THRESHOLD_NEW)) {
             // increment motor blocked counter with 100 milliseconds
            ++ui8_motor_blocked_counter;

            // check if motor is blocked for more than some safe threshold
        if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD_NEW) {
			// set error code
			ui8_system_state = ERROR_MOTOR_BLOCKED;

			// reset motor blocked counter as the error code is set
			ui8_motor_blocked_counter = 0;
		}
	}
	else {
		// current is below the threshold and/or motor ERPS is above the threshold so reset the counter
		ui8_motor_blocked_counter = 0;
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E05 ERROR_THROTTLE
#define THROTTLE_CHECK_COUNTER_THRESHOLD		 20 // 20 * 100ms = 2.0 seconds
#define ADC_THROTTLE_MIN_VALUE_THRESHOLD		(uint8_t)(m_config.adc_throttle_min_value + 5)

    static uint8_t ui8_throttle_check_counter;
	// mstrens next line has been copied from motor.c to save time in irq 1
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5    	
	if (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) {
		if (ui8_throttle_check_counter < THROTTLE_CHECK_COUNTER_THRESHOLD) { // 20 // so we have to wait 25 msec * 20 = 500msec
			ui8_throttle_check_counter++;
		
			if ((ui16_adc_throttle >> 2) > ADC_THROTTLE_MIN_VALUE_THRESHOLD) {
				ui8_system_state = ERROR_THROTTLE;
			}
    	}
	}
}


uint8_t ui8_default_flash_state;

void ebike_control_lights(void)
{
#define DEFAULT_FLASH_ON_COUNTER_MAX      3
#define DEFAULT_FLASH_OFF_COUNTER_MAX     2
#define BRAKING_FLASH_ON_COUNTER_MAX      1
#define BRAKING_FLASH_OFF_COUNTER_MAX     1

    //static uint8_t ui8_default_flash_state;
    static uint8_t ui8_default_flash_state_counter; // increments every function call -> 100 ms
    static uint8_t ui8_braking_flash_state;
    static uint8_t ui8_braking_flash_state_counter; // increments every function call -> 100 ms

    /****************************************************************************/

    // increment flash counters
    ++ui8_default_flash_state_counter;
    ++ui8_braking_flash_state_counter;

    /****************************************************************************/

    // set default flash state
    if ((ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 0;
    }
	else if ((!ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 1;
    }

    /****************************************************************************/

    // set braking flash state
    if ((ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 0;
    }
	else if ((!ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 1;
    }

    /****************************************************************************/

    // select lights configuration
    switch (m_configuration_variables.ui8_lights_configuration) {
      case 0:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
      case 1:
        // check lights state
        if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 2:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 3:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 4:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 5:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 6:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 7:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 8:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
	  default:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
    }

    /*------------------------------------------------------------------------------------------------------------------

     NOTE: regarding the various light modes

     (0) lights ON when enabled
     (1) lights FLASHING when enabled

     (2) lights ON when enabled and BRAKE-FLASHING when braking
     (3) lights FLASHING when enabled and ON when braking
     (4) lights FLASHING when enabled and BRAKE-FLASHING when braking

     (5) lights ON when enabled, but ON when braking regardless if lights are enabled
     (6) lights ON when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     (7) lights FLASHING when enabled, but ON when braking regardless if lights are enabled
     (8) lights FLASHING when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     ------------------------------------------------------------------------------------------------------------------*/
}

void fillRxBuffer() {
	if (XMC_USIC_CH_RXFIFO_IsEmpty(	CYBSP_DEBUG_UART_HW	))	return; // Skip when there is no data in rxfifo 
	
	// when there are data:

	// Read data : Reading data clear UART2_FLAG_RXNE flag
	uint8_t	ui8_byte_received = (uint8_t) XMC_USIC_CH_RXFIFO_GetData(CYBSP_DEBUG_UART_HW);

    switch (ui8_state_machine) {
        case 0:
            // see if we get start package byte
            if (ui8_byte_received == RX_STX)  {
                ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
                ui8_rx_counter++;
                ui8_state_machine = 1;
            }
            else {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
            }
        break;

        case 1:
            ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
            ui8_rx_counter++;

            // see if is the last byte of the package
            if (ui8_rx_counter >= UART_RX_BUFFER_LEN) {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
                ui8_received_package_flag = 1; // signal that we have a full package to be processed
            }
        break;

        default:
        break;

    }
}


// once every 4*25 msec = 100 msec
void uart_receive_package() {  
	uint8_t ui8_i;
	uint8_t ui8_rx_check_code;
	uint8_t ui8_assist_level_mask;
	static uint8_t no_rx_counter = 0;
	static uint8_t ui8_lights_counter = 0;
	static uint8_t ui8_walk_assist_button_pressed = 0;
	static uint8_t ui8_walk_assist_button_released = 0;

	// mstrens next 3 fields are always defined
	static uint8_t ui8_walk_assist_debounce_flag = 0;
	static uint8_t ui8_walk_assist_debounce_counter = 0;
	
	// increment walk assist counter
	ui8_walk_assist_debounce_counter++;
	// increment the comms safety counter
    no_rx_counter++;
		// increment lights_counter
	ui8_lights_counter++;
	// increment display menu counter
	ui8_menu_counter++;

	if (ui8_received_package_flag) {
		// verify check code of the package
		ui8_rx_check_code = 0x00;
							 
		for(ui8_i = 0; ui8_i < RX_CHECK_CODE; ui8_i++)
		{
			ui8_rx_check_code += ui8_rx_buffer[ui8_i];
		}

		// see if check code is ok...
		if (ui8_rx_check_code == ui8_rx_buffer[RX_CHECK_CODE]) {
			// Reset the safety counter when a valid message from the LCD is received
            no_rx_counter = 0;
			
			// mask lights button from display
			ui8_lights_button_flag = ui8_rx_buffer[1] & 0x01;
			
			// mask walk assist button from display
			ui8_walk_assist_button_pressed = ui8_rx_buffer[1] & 0x20;
			
			// mask assist level from display
			ui8_assist_level_mask = ui8_rx_buffer[1] & 0xDE; // mask: 11011110
			ui8_assist_level_5_flag = 0;
			
			// set assist level
			switch (ui8_assist_level_mask) {
				case ASSIST_PEDAL_LEVEL0: ui8_assist_level = OFF; break;
				case ASSIST_PEDAL_LEVEL1: ui8_assist_level = ECO; break;
				case ASSIST_PEDAL_LEVEL2: ui8_assist_level = TOUR; break;
				case ASSIST_PEDAL_LEVEL3: ui8_assist_level = SPORT; break;
				case ASSIST_PEDAL_LEVEL4: ui8_assist_level = TURBO; break;
				// mstrens : we activate always the case for level 5
				case ASSIST_PEDAL_LEVEL5:
					if( m_config.assist_level_5_mode == BEFORE_ECO) {
						ui8_assist_level = ECO;
						ui8_assist_level_5_flag = 1;
					} else if ( m_config.assist_level_5_mode == AFTER_TURBO) {
						ui8_assist_level = TURBO;
						ui8_assist_level_5_flag = 1;
					}
					break;
			}
			
			if (!ui8_display_ready_flag) {
				// assist level temp at power on
				ui8_assist_level_temp = ui8_assist_level;
			}
			// display ready
			ui8_display_ready_flag = 1;
			
			// display lights button pressed:
			if (ui8_lights_button_flag) {
				// lights off:
				if (((!ui8_lights_flag)
				 &&((m_configuration_variables.ui8_set_parameter_enabled)
				  ||(ui8_assist_level == OFF)))
				   &&(!ui8_torque_sensor_calibration_flag)) {
					// set startup flag
					if (!ui8_startup_flag) {
						ui8_startup_flag = 1;
					}
					
					// lights 5s on
					ui8_lights_on_5s = 1;
					
					// menu flag
					if (!ui8_menu_flag) 
					{
						// set menu flag
						ui8_menu_flag = 1;
							
						// set menu index
						if (++ui8_menu_index > 3) { 
							ui8_menu_index = 1;
						}
						
						// display status alternative lights configuration
						ui8_display_alternative_lights_configuration = 0;
						
						// restore previous parameter
						switch (ui8_assist_level) {	
							case OFF:
								switch (ui8_menu_index) {
									case 2:
										// restore previous set parameter
										m_configuration_variables.ui8_set_parameter_enabled = ui8_set_parameter_enabled_temp;
										ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
										break;
									case 3:
										// restore previous auto display data
										m_configuration_variables.ui8_auto_display_data_enabled = ui8_auto_display_data_enabled_temp;
										ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
										break;
								}
								break;
								
							case ECO:
								switch (ui8_menu_index) {
									case 2:
										// restore previous street mode
										m_configuration_variables.ui8_street_mode_enabled = ui8_street_mode_enabled_temp;
										ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
										break;
									case 3:
										// restore previous startup boost
										m_configuration_variables.ui8_startup_boost_enabled = ui8_startup_boost_enabled_temp;
										ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
											// restore torque sensor advanced
											m_configuration_variables.ui8_torque_sensor_adv_enabled = ui8_torque_sensor_adv_enabled_temp;
											ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
										break;
								}
								break;
							
							case TURBO:	
								switch (ui8_menu_index) {
									case 2:
										if (ui8_lights_configuration_2 == 9) {
											// restore previous lights configuration
											m_configuration_variables.ui8_lights_configuration = ui8_lights_configuration_temp;
											ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
											// display status
											ui8_display_alternative_lights_configuration = 1;
										}
										break;
									case 3:
										if (ui8_lights_configuration_2 == 9) {
											// restore previous assist without pedal rotation
											m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = ui8_assist_without_pedal_rotation_temp;
											ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										}
										else {
											// restore previous lights configuration
											m_configuration_variables.ui8_lights_configuration = ui8_lights_configuration_temp;
											ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
										}
										
										if (ui8_lights_configuration_3 == 10) {
											// display status
											ui8_display_alternative_lights_configuration = 1;
										}
										break;
								}							
								break;
						} // end of switch
						
						// display function code enabled (E02, E03,E04)
						ui8_display_function_code = ui8_menu_index + 1;
						// display function code temp (for display function status VLCD5/6 and EKD01)
						ui8_display_function_code_temp = ui8_display_function_code;
						// display data function enabled
						ui8_display_data_enabled = 1;
						
						// restart lights counter
						ui8_lights_counter = 0;	
						// restart menu counter
						ui8_menu_counter = 0;					
					}
					
					// after some seconds: switch on lights (if enabled) and abort function
					if ((ui8_lights_counter >= ui8_delay_display_function)
					  ||((ui8_assist_level != ui8_assist_level_temp)&&(!ui8_auto_display_data_flag))) {
						// set lights flag
						ui8_lights_flag = 1;
						// lights 5s off		
						ui8_lights_on_5s = 0;
						// clear menu flag
						ui8_menu_flag = 0;
						// clear menu index
						ui8_menu_index = 0;
						// clear menu counter
						ui8_menu_counter = ui8_delay_display_function;
						// clear lights counter
						ui8_lights_counter = DELAY_MENU_ON;
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
					}
				}
				else {
					// set lights flag
					ui8_lights_flag = 1;
				}
			}
			else {
				// lights off:
				if (!ui8_lights_flag)
				{
					// menu flag active:
					if (ui8_menu_flag) {
						// clear menu flag
						ui8_menu_flag = 0;
							
						// lights 5s off		
						ui8_lights_on_5s = 0;	
							
						// restart menu counter
						ui8_menu_counter = 0;
								
						// menu function enabled
						ui8_menu_function_enabled = 1;
					}
				}
				else {
					// clear lights flag
					ui8_lights_flag = 0;
				}
			}

			// restart menu display function
			if ((ui8_menu_counter >= ui8_delay_display_function)
			  ||((ui8_assist_level != ui8_assist_level_temp)
				&&(!ui8_torque_sensor_calibration_flag)
				&&(!ui8_auto_display_data_flag))) {					
				// clear menu flag
				ui8_menu_flag = 0;
				// clear menu index
				ui8_menu_index = 0;
				// clear menu counter
				ui8_menu_counter = ui8_delay_display_function;
				// menu function disabled
				ui8_menu_function_enabled = 0;
				// display data function disabled
				ui8_display_data_enabled = 0;
				// display function code disabled
				ui8_display_function_code = NO_FUNCTION;
			}

			// display menu function
			if (ui8_menu_function_enabled) {
				// display status lights configuration
				ui8_display_alternative_lights_configuration = 0;
				// set display parameter
				if (((m_configuration_variables.ui8_set_parameter_enabled)
				  &&(!ui8_assist_level_5_flag))
					||(ui8_assist_level == OFF)) {
					
					switch (ui8_assist_level) {	
						case OFF:
							// set parameter
							switch (ui8_menu_index) {
								case 1:
									// for restore set parameter
									ui8_set_parameter_enabled_temp = m_configuration_variables.ui8_set_parameter_enabled;
									
									// set parameter enabled  // invert the parameter
									m_configuration_variables.ui8_set_parameter_enabled = !m_configuration_variables.ui8_set_parameter_enabled;
									ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
									break;
								case 2:		   
									// for restore auto display data
									ui8_auto_display_data_enabled_temp = m_configuration_variables.ui8_auto_display_data_enabled;
									
									// set auto display data
									m_configuration_variables.ui8_auto_display_data_enabled = !m_configuration_variables.ui8_auto_display_data_enabled;
									ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
									break;
								case 3:
									// save current configuration
									EEPROM_write(); // changed by mstrens to  use XMC13 function
									break;
							}
							break;
						
						case ECO:
							// set street/offroad mode
							switch (ui8_menu_index) {
								case 1:
									// for restore street mode
									ui8_street_mode_enabled_temp = m_configuration_variables.ui8_street_mode_enabled;
									
									// change street mode
									m_configuration_variables.ui8_street_mode_enabled = !m_configuration_variables.ui8_street_mode_enabled;
									ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
									break;
								case 2:																		 
									// for restore startup boost
									ui8_startup_boost_enabled_temp = m_configuration_variables.ui8_startup_boost_enabled;	
									
									// change startup boost
									m_configuration_variables.ui8_startup_boost_enabled = !m_configuration_variables.ui8_startup_boost_enabled;
									ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
									break;
								case 3:
										// for restore torque sensor advanced
										ui8_torque_sensor_adv_enabled_temp = m_configuration_variables.ui8_torque_sensor_adv_enabled;
									
										// change torque sensor advanced mode
										m_configuration_variables.ui8_torque_sensor_adv_enabled = !m_configuration_variables.ui8_torque_sensor_adv_enabled;
										ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
									break;
							}
							break;

						case TOUR:
							// set riding mode 1
							switch (ui8_menu_index) {
								case 1: m_configuration_variables.ui8_riding_mode = POWER_ASSIST_MODE; break;
								case 2: m_configuration_variables.ui8_riding_mode = TORQUE_ASSIST_MODE; break;
								case 3: m_configuration_variables.ui8_riding_mode = CADENCE_ASSIST_MODE; break;
							}
							ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
							break;
						
						case SPORT:
							// set riding mode 2
							switch (ui8_menu_index) {
								case 1:	m_configuration_variables.ui8_riding_mode = eMTB_ASSIST_MODE; break;
								case 2: m_configuration_variables.ui8_riding_mode = HYBRID_ASSIST_MODE; break;        
								case 3: m_configuration_variables.ui8_riding_mode = CRUISE_MODE; break;
							}
							ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
							break;
						
						case TURBO:
							// set lights mode
							switch (ui8_menu_index) {
								case 1:  
									// for restore lights configuration
										ui8_lights_configuration_temp = m_configuration_variables.ui8_lights_configuration;
									
										if (m_configuration_variables.ui8_lights_configuration != LIGHTS_CONFIGURATION_ON_STARTUP) {
										m_configuration_variables.ui8_lights_configuration = LIGHTS_CONFIGURATION_ON_STARTUP;
										}
										else {
											m_configuration_variables.ui8_lights_configuration = LIGHTS_CONFIGURATION_1;
										}
									break;
								case 2: // - LIGHTS ON & BRAKE FLASHING or ASSIST WITHOUT PEDALIG ROTATION (0 / 1)
									if (ui8_lights_configuration_2 == 9) {
										// for restore assist without pedal rotation
										ui8_assist_without_pedal_rotation_temp = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
									
										// change assist without pedal rotation
										m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = !m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										// display status
										ui8_display_alternative_lights_configuration = 1;
									}
									else {
										m_configuration_variables.ui8_lights_configuration = LIGHTS_CONFIGURATION_2;
										// for restore lights configuration
										ui8_lights_configuration_temp = m_configuration_variables.ui8_lights_configuration;
									
									}
									break;
								case 3: // LIGHTS FLASHING & BRAKE ON or ASSIST WITH SENSORS ERROR (0 / 1)
									if (ui8_lights_configuration_3 == 10) {
										// change system error enabled
										m_configuration_variables.ui8_assist_with_error_enabled = !m_configuration_variables.ui8_assist_with_error_enabled;
										ui8_display_function_status[2][TURBO] = m_configuration_variables.ui8_assist_with_error_enabled;
										// display status
										ui8_display_alternative_lights_configuration = 1;
									}
									else {
										m_configuration_variables.ui8_lights_configuration = LIGHTS_CONFIGURATION_3;
									}
									break;
							}
							// display lights configuration
							ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
							break;
					}
					
					// display data value enabled
					ui8_display_data_enabled = 1;
				}
			}
			
			// display function status VLCD5/6
			//#if ENABLE_VLCD5 || ENABLE_VLCD6 || ENABLE_EKD01
			if ((m_config.enable_vlcd5) || (m_config.enable_vlcd6) || (m_config.enable_ekd01)){
				if (ui8_menu_flag) {
				if (ui8_menu_counter >= DELAY_FUNCTION_STATUS)
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
				}
				else {
					if ((ui8_menu_counter >= (DELAY_MENU_ON - DELAY_FUNCTION_STATUS))&&
						(ui8_menu_counter < DELAY_MENU_ON)&&(ui8_menu_index > 0U)) {
						// restore display function code
						ui8_display_function_code = ui8_display_function_code_temp;
					}
					else {
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
					}
				}
			} // endif
			
			// menu function disabled
			ui8_menu_function_enabled = 0;

			// special riding modes with walk assist button
			switch (m_configuration_variables.ui8_riding_mode) {	
				case TORQUE_SENSOR_CALIBRATION_MODE:
					if (ui8_assist_level != OFF) {	
						// riding mode recovery at level change
						m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
						// clear torque sensor calibration flag
						ui8_torque_sensor_calibration_flag = 0;
						// display torque flag 1 disabled
						ui8_torque_sensor_calibration_flag_1 = 0;
						// display torque flag 2 disabled
						ui8_torque_sensor_calibration_flag_2 = 0;
						// display torque flag 2 disabled
						ui8_torque_sensor_calibration_flag_3 = 0;
						// display torque value disabled
						ui8_display_torque_sensor_calibration_value_flag = 0;
						// display torque step disabled
						ui8_display_torque_sensor_calibration_step_flag = 0;
						// display data function disabled
						ui8_display_data_enabled = 0;
						// clear menu counter
						ui8_menu_counter = ui8_delay_display_function;
					}
					else {
							// restart menu counter
							ui8_menu_counter = 0;
						// display data function enabled
						ui8_display_data_enabled = 1;
						
						// torque sensor calibration procedure 3 (with weight)
						if (ui8_torque_sensor_calibration_flag_3) {
							// set display torque sensor step for calibration with weight
							ui8_display_torque_sensor_calibration_step_flag = 1;
							
							// continue to torque sensor calibration with weight
							if (!ui8_walk_assist_button_pressed) {
								ui8_walk_assist_button_released = 1;
							}
							if ((ui8_walk_assist_button_pressed)&&(ui8_walk_assist_button_released)) {
								ui8_torque_sensor_calibration_with_weight_flag = 1;
							}
							else {
								if (ui8_torque_sensor_calibration_with_weight_flag) {
									// torque sensor, adc step advanced
									ui8_pedal_torque_per_10_bit_ADC_step_advanced_x100 = (uint8_t)((uint16_t)(PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100
										* ui8_pedal_torque_per_10_bit_ADC_step_detected_x100)
										/ m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_est_x100);
									// torque sensor, adc step estimated = detected
									m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_est_x100 = ui8_pedal_torque_per_10_bit_ADC_step_detected_x100;
									
									ui8_walk_assist_button_released = 0;
								}
								ui8_torque_sensor_calibration_with_weight_flag = 0;
							}
							
							// torque sensor value display counter
							ui8_torque_sensor_step_display_counter++;
							
							// torque sensor value display time
							if (ui8_torque_sensor_step_display_counter >= (DELAY_MENU_ON >> 1)) {
								// reset counter
								ui8_torque_sensor_step_display_counter = 0;
								// toggle data
								ui8_torque_sensor_step_display_flag = !ui8_torque_sensor_step_display_flag;
							}
							
							if (ui8_torque_sensor_step_display_flag) {
								// torque sensor advanced adc step to display
								ui8_torque_sensor_step_to_display = ui8_pedal_torque_per_10_bit_ADC_step_advanced_x100;
						}
						else {
								// torque sensor estimated adc step to display
								ui8_torque_sensor_step_to_display = m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_est_x100;
							}
						}
						// torque sensor calibration procedure 2 (estimated)
						else if (ui8_torque_sensor_calibration_flag_2) {
							// pedal torque adc step estimated
							uint16_t ui16_adc_pedal_torque_on_weight = ui16_adc_pedal_torque_offset_init + ((ui16_adc_pedal_torque_max_temp - ui16_adc_pedal_torque_offset_init) * 75) / 100;
							m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_est_x100 = (WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION * 167) / (ui16_adc_pedal_torque_on_weight - ui16_adc_pedal_torque_offset_init);
							ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_DEFAULT] = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100;
							// enable torque sensor estimated
							m_configuration_variables.ui8_torque_sensor_estimated = 1;
							// disable torque sensor advanced
							m_configuration_variables.ui8_torque_sensor_adv_enabled = 0;
							
							// torque sensor value display counter
							ui8_torque_sensor_value_display_counter++;
							
							// torque sensor value display time
							if (ui8_torque_sensor_value_display_counter >= (DELAY_MENU_ON >> 1)) {
								// reset counter
								ui8_torque_sensor_value_display_counter = 0;
								// toggle data
								ui8_torque_sensor_value_display_flag = !ui8_torque_sensor_value_display_flag;
							}
							
							if (ui8_torque_sensor_value_display_flag) {
								// torque sensor offset value to display
								ui16_torque_sensor_value_to_display = ui16_adc_pedal_torque_offset_init;
							}
							else {
								// torque sensor max value to display
								ui16_torque_sensor_value_to_display = ui16_adc_pedal_torque_max_temp;
							}
							
							// continue to torque sensor calibration procedure 3
							if (!ui8_walk_assist_button_pressed) {
								ui8_walk_assist_button_released = 1;
							}
							if ((ui8_walk_assist_button_pressed)&&(ui8_walk_assist_button_released)) {
								ui8_walk_assist_button_released = 0;
								ui8_torque_sensor_calibration_flag_3 = 1;
							}
						}
						// torque sensor calibration procedure 1 (manual)
						else if (ui8_torque_sensor_calibration_flag_1) {
							// torque sensor value to display
							ui8_display_torque_sensor_calibration_value_flag = 1;
							ui16_torque_sensor_value_to_display = ui16_adc_pedal_torque;
							// set torque sensor max value
							if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_max_temp) {
								ui16_adc_pedal_torque_max_temp = ui16_adc_pedal_torque;
							}
							
							// continue to torque sensor calibration procedure 2
							if (!ui8_walk_assist_button_pressed) {
								ui8_walk_assist_button_released = 1;
							}
							if ((ui8_walk_assist_button_pressed)&&(ui8_walk_assist_button_released)) {
								ui8_walk_assist_button_released = 0;
								ui8_torque_sensor_calibration_flag_2 = 1;
							}
						}
					}
					break;
					
				default:
					// manual setting battery SOC percentage x10 (actual charge)
					// walk assist button pressed within 5 seconds of power on
					if ((ui8_walk_assist_button_pressed)&&(!ui8_startup_flag)) {
						ui16_battery_SOC_percentage_x10 = read_battery_soc();
						// calculate watt-hours x10
						ui32_wh_offset_x10 = ((uint32_t)(1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
						// for display soc %
						ui8_display_data_on_startup = 1; // SOC%
						ui8_display_data_enabled = 1;
					}
					// torque sensor calibration *********************************
					else if ((ui8_walk_assist_button_pressed)&&(ui8_startup_flag)
					  &&(m_configuration_variables.ui8_set_parameter_enabled)
					  &&(ui8_assist_level == OFF)) {
						ui8_torque_sensor_calibration_flag = 1;
						// starting torque sensor calibration procedure 1
						ui8_torque_sensor_calibration_flag_1 = 1;
						
						// for recovery actual riding mode
						if (m_configuration_variables.ui8_riding_mode != TORQUE_SENSOR_CALIBRATION_MODE) {
							ui8_riding_mode_temp = m_configuration_variables.ui8_riding_mode;
						}
						// special riding mode (torque sensor calibration)
						m_configuration_variables.ui8_riding_mode = TORQUE_SENSOR_CALIBRATION_MODE;
					}
					// cruise mode ***********************************************
					else if (m_configuration_variables.ui8_riding_mode == CRUISE_MODE) {
						if ((ui8_walk_assist_button_pressed)&&(ui8_startup_flag)) {
						ui8_cruise_button_flag = 1;
						}
						else {
							ui8_cruise_button_flag = 0;
						}
					}
					// startup assist mode and walk assist mode ******************
					else if (ui8_assist_level != OFF) {
//#if STARTUP_ASSIST_ENABLED
						if (m_config.startup_assist_enabled) {
							// // startup assist mode
							if ((ui8_walk_assist_button_pressed)&&(ui8_startup_flag)
								&&(!ui8_walk_assist_flag)&&(ui8_lights_flag)) {
								ui8_startup_assist_flag = 1;
							}
							else {
								ui8_startup_assist_flag = 0;
							}
						} // endif
//#if ENABLE_WALK_ASSIST						
						if (m_config.enable_walk_assist){
							// walk assist mode
							if ((ui8_walk_assist_button_pressed)&&(ui8_startup_flag)&&(!ui8_startup_assist_flag)
								&&(ui8_walk_assist_enabled_array[m_configuration_variables.ui8_street_mode_enabled])) {
								if (!ui8_walk_assist_flag) {
									// set walk assist flag
									ui8_walk_assist_flag = 1;
									// for restore riding mode
									ui8_riding_mode_temp = m_configuration_variables.ui8_riding_mode;
									// set walk assist mode
									m_configuration_variables.ui8_riding_mode = WALK_ASSIST_MODE;
								}
							}
							else {
	//#if WALK_ASSIST_DEBOUNCE_ENABLED && ENABLE_BRAKE_SENSOR
								if ((m_config.walk_assist_debounce_enabled) && (m_config.enable_brake_sensor)){
									if (ui8_walk_assist_flag) {
										if (!ui8_walk_assist_debounce_flag) {
											// set walk assist debounce flag
											ui8_walk_assist_debounce_flag = 1;
											// restart walk assist counter
											ui8_walk_assist_debounce_counter = 0;
											// walk assist level during debounce time
											ui8_walk_assist_level = ui8_assist_level;
										}					
									
										if (ui8_walk_assist_debounce_counter < m_config.walk_assist_debounce_time) {
											// stop walk assist during debounce time
											if ((ui8_assist_level != ui8_walk_assist_level)||(ui8_brake_state)
											||(m_configuration_variables.ui8_street_mode_enabled)) {
												// restore previous riding mode
												m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
												// reset walk assist flag
												ui8_walk_assist_flag = 0;
												// reset walk assist debounce flag
												ui8_walk_assist_debounce_flag = 0;
												// reset walk assist speed flag
												ui8_walk_assist_speed_flag = 0;
											}
										}	
										else {
											// restore previous riding mode
											if (ui8_walk_assist_flag) {
												m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
											}
											// reset walk assist flag
											ui8_walk_assist_flag = 0;
											// reset walk assist debounce flag
											ui8_walk_assist_debounce_flag = 0;
											// reset walk assist speed flag
											ui8_walk_assist_speed_flag = 0;
										}
									}
								} else { //	#else
									// restore previous riding mode
									if (ui8_walk_assist_flag) {
										m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
									}
									// reset walk assist flag
									ui8_walk_assist_flag = 0;
									// reset walk assist speed flag
									ui8_walk_assist_speed_flag = 0;
								} //	#endif
							}
						}	//#endif
					}
					else {
						// reset startup assist flag
						ui8_startup_assist_flag = 0;
						// reset walk assist flag
						ui8_walk_assist_flag = 0;
					}
					break;
			}
			
			// set assist parameter
			ui8_riding_mode_parameter = ui8_riding_mode_parameter_array[m_configuration_variables.ui8_riding_mode - 1][ui8_assist_level];
//#if ASSIST_LEVEL_5_MODE
			if (m_config.assist_level_5_mode) {		
				if (ui8_assist_level_5_flag) {
				ui8_riding_mode_parameter = (uint8_t)(((uint16_t)(ui8_riding_mode_parameter * (uint8_t)ASSIST_LEVEL_5_PERCENT)) / 100U);
				}
			} // endif
			// automatic data display at lights on
			if (m_configuration_variables.ui8_auto_display_data_enabled) {	
				if ((ui8_lights_flag)&&(ui8_menu_index == 0U)&&(ui8_startup_flag)
				  &&(!ui8_torque_sensor_calibration_flag)) {
					if (!ui8_auto_display_data_flag) {	
						// set auto display data flag
						ui8_auto_display_data_flag = 1;
						ui8_auto_display_data_status = 1;
						// display data function enabled
						ui8_display_data_enabled = 1;
						// restart menu counter
						ui8_menu_counter = 0;
						// set data index
						ui8_data_index = 0;
						// assist level temp, ignore first change
						ui8_assist_level_temp = ui8_assist_level;
						// delay data function
						if (ui8_delay_display_array[ui8_data_index]) {
							ui8_delay_display_function  = ui8_delay_display_array[ui8_data_index];
						}
						else {
							ui8_delay_display_function  = DELAY_MENU_ON;
						}
					}
					
					// restart menu counter if data delay is zero
					if (!ui8_delay_display_array[ui8_data_index]) {
						ui8_menu_counter = 0;
					}
					if ((ui8_data_index + 1) < ui8_auto_data_number_display) {
						if ((ui8_menu_counter >= (ui8_delay_display_function - 4))||(ui8_assist_level != ui8_assist_level_temp)) {
							// restart menu counter
							ui8_menu_counter = 0;
							// increment data index
							ui8_data_index++;
							// delay data function
							if (ui8_delay_display_array[ui8_data_index]) {
								ui8_delay_display_function  = ui8_delay_display_array[ui8_data_index];
							}
							else {
								ui8_delay_display_function  = DELAY_MENU_ON;
							}
						}
					}
					else if (ui8_menu_counter >= ui8_delay_display_function) {
						ui8_auto_display_data_status = 0;
					}
				}
				else {
					if (ui8_auto_display_data_flag) {
						// reset auto display data flag
						ui8_auto_display_data_flag = 0;
						ui8_auto_display_data_status = 0;
						// display data function disabled
						ui8_display_data_enabled = 0;
					}
				}
			}
			
			// assist level temp, to change or stop operation at change of level
			ui8_assist_level_temp = ui8_assist_level;
			
			// set lights
			//#if ENABLE_LIGHTS
			if (m_config.enable_lights) {
				// switch on/switch off lights
				if ((ui8_lights_flag)||(ui8_lights_on_5s)) {
					ui8_lights_state = 1;
				}
				else {
					ui8_lights_state = 0;
				}
			}//#endif
			
			// ui8_rx_buffer[2] current max?
			
			// get wheel diameter from display
			ui8_oem_wheel_diameter = ui8_rx_buffer[3];
			
			// factor to calculate the value of the data to be displayed
			// change by mstrens due to removing the #if
			if (m_config.units_type) {// 1 mph and miles
				ui16_display_data_factor = 900 * ui8_oem_wheel_diameter; // OEM_WHEEL_FACTOR = 1435
			} else {                  // 0 = km/h and km
				ui16_display_data_factor = 1435 * ui8_oem_wheel_diameter; // OEM_WHEEL_FACTOR = 1435	
			}

			// ui8_rx_buffer[4] test?
			
			//#if ENABLE_WHEEL_MAX_SPEED_FROM_DISPLAY
			if (m_config.enable_wheel_max_speed_from_display){
				// set wheel max speed from display
				ui8_wheel_speed_max_array[OFFROAD_MODE] = ui8_rx_buffer[5];
				if (ui8_wheel_speed_max_array[STREET_MODE] > ui8_wheel_speed_max_array[OFFROAD_MODE]) {
					ui8_wheel_speed_max_array[STREET_MODE] = ui8_wheel_speed_max_array[OFFROAD_MODE];
				}
			}//#endif
			
			// set speed limit in street, offroad, walk assist, startup assist, throttle 6km/h mode
			if ((m_configuration_variables.ui8_riding_mode == WALK_ASSIST_MODE)
				||(ui8_startup_assist_flag)) {
				m_configuration_variables.ui8_wheel_speed_max = WALK_ASSIST_THRESHOLD_SPEED;
			}
			else if ((ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled] == W_O_P_6KM_H_ONLY)
				&& (ui8_throttle_adc_in)) {
					m_configuration_variables.ui8_wheel_speed_max = WALK_ASSIST_THRESHOLD_SPEED;
			}
			else {
				m_configuration_variables.ui8_wheel_speed_max = ui8_wheel_speed_max_array[m_configuration_variables.ui8_street_mode_enabled];
			}
			
			// current limit with power limit
			ui8_adc_battery_current_max_temp_2 = (uint8_t)((uint32_t)(ui32_adc_battery_power_max_x1000_array[m_configuration_variables.ui8_street_mode_enabled]
				/ ui16_battery_voltage_filtered_x1000));
			
			// set max battery current
			ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);

			// set pedal torque per 10_bit ADC step x100 estimated or advanced
			ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_pedal_torque_per_10_bit_ADC_step_x100_array[m_configuration_variables.ui8_torque_sensor_adv_enabled];
		}
		
		// signal that we processed the full package
		ui8_received_package_flag = 0;
		
		// assist level = OFF if connection with the LCD is lost for more than 0,3 sec (safety)
		if (no_rx_counter > 3) {
			ui8_assist_level = OFF;
		}
		// mstrens : uart does not have to be enabled for TSDZ8
	}
}

void uart_send_package(){
	uint8_t ui8_i;
	uint8_t ui8_tx_check_code;
	
	// display ready
	if (ui8_display_ready_flag) {
		// send the data to the LCD
		// start up byte
		ui8_tx_buffer[0] = TX_STX;

		// clear fault code
		ui8_display_fault_code = NO_FAULT;

		// initialize working status
		ui8_working_status &= 0xFE; // bit0 = 0 (battery normal)

		//#if ENABLE_VLCD6 || ENABLE_XH18
		if ((m_config.enable_vlcd6) || (m_config.enable_xh18)){
			switch (ui8_battery_state_of_charge) {
				case 0:
					ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
					ui8_tx_buffer[1] = 0x00;
					break;
				case 1:
					ui8_tx_buffer[1] = 0x00; // Battery 0/4 (empty and blinking)
					break;
				case 2:
					ui8_tx_buffer[1] = 0x02; // Battery 1/4
					break;
				case 3:
					ui8_tx_buffer[1] = 0x06; // Battery 2/4
					break;
				case 4:
					ui8_tx_buffer[1] = 0x09; // Battery 3/4
					break;
				case 5:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
					break;
				case 6:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (soc reset)
					break;
				case 7:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
					// E01 (E06 blinking for XH18) ERROR_OVERVOLTAGE
					ui8_display_fault_code = ERROR_OVERVOLTAGE; // Fault overvoltage
					break;
			}
		} else {//}	#else // ENABLE_VLCD5 or ENABLE_850C or ENABLE_EKD01
			switch (ui8_battery_state_of_charge) {
				case 0:
					ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
					ui8_tx_buffer[1] = 0x00;
					break;
				case 1:
					ui8_tx_buffer[1] = 0x00; // Battery 0/6 (empty and blinking)
					break;
				case 2:
					ui8_tx_buffer[1] = 0x02; // Battery 1/6
					break;
				case 3:
					ui8_tx_buffer[1] = 0x04; // Battery 2/6
					break;
				case 4:
					ui8_tx_buffer[1] = 0x06; // Battery 3/6
					break;
				case 5:
					ui8_tx_buffer[1] = 0x08; // Battery 4/6
					break;
				case 6:
					ui8_tx_buffer[1] = 0x0A; // Battery 5/6
					break;
				case 7:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
					break;
				case 8:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (soc reset)
					break;
				case 9:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
					// E01 ERROR_OVERVOLTAGE
					ui8_display_fault_code = ERROR_OVERVOLTAGE; // Fault overvoltage
					break;
			}
		} //#endif
		
		// reserved for VLCD5, torque sensor value TE and TE1
		//#if ENABLE_VLCD5
		if ( m_config.enable_vlcd5) {
			ui8_tx_buffer[3] = (uint8_t)ui16_adc_pedal_torque_offset_init;
			if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset_init) {
				ui8_tx_buffer[4] = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset_init;
			}
			else {
				ui8_tx_buffer[4] = 0;
			}
		} else if (m_config.enable_850c){ //} #elif ENABLE_850C
			ui8_tx_buffer[3] = ui8_battery_current_filtered_x10;
			// battery power filtered x 10 for display data
			ui16_battery_power_filtered_x10 = filter(ui16_battery_power_x10, ui16_battery_power_filtered_x10, 13);
			ui8_tx_buffer[4] = (uint8_t) (ui16_battery_power_filtered_x10 / 100);
		} else if (m_config.enable_ekd01){ //} #elif ENABLE_EKD01
			ui8_tx_buffer[3] = 0; // don't care
			// battery power filtered x 10 for display data
			ui16_battery_power_filtered_x10 =
					filter(ui16_battery_power_x10, ui16_battery_power_filtered_x10, 8);
			ui8_tx_buffer[4] = (uint8_t)(ui16_battery_power_filtered_x10 / 100);	
		} else { //#else
			ui8_tx_buffer[3] = 0x46;
			ui8_tx_buffer[4] = 0x46;
		} //#endif
		
		// fault temperature limit
		// E06 ERROR_OVERTEMPERATURE
		#if (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL) && ENABLE_TEMPERATURE_ERROR_MIN_LIMIT
		// temperature error at min limit value
		if (((uint8_t) (ui16_motor_temperature_filtered_x10 / 10)) >= ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE])
		{
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#elif (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL) && !ENABLE_TEMPERATURE_ERROR_MIN_LIMIT
		// temperature error at max limit value
		if (((uint8_t) (ui16_motor_temperature_filtered_x10 / 10)) >= ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE])
		{
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#elif BRAKE_TEMPERATURE_SWITCH
		if (ui8_brake_state) {
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#endif

// here new code added by mstrens
    #define MAX_TEMPERATURE 80
	/* Calculate temperature of the chip in Kelvin */
        uint32_t temp_k = XMC_SCU_CalcTemperature();
		/* Convert temperature to Celsius */
		if (temp_k > 273){
			ui8_temp_celcius = (uint8_t) (XMC_SCU_CalcTemperature() - 273u) ; 
		} else {
			ui8_temp_celcius = 0;
		}
        if (ui8_temp_celcius > MAX_TEMPERATURE){
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}


		// blocked motor error has priority
		if (ui8_system_state == ERROR_MOTOR_BLOCKED) {	
			ui8_display_fault_code = ERROR_MOTOR_BLOCKED;
		}
		else {
			if ((ui8_system_state != NO_ERROR)&&(ui8_display_fault_code == NO_FAULT)) {
				ui8_display_fault_code = ui8_system_state;
			}
		}

		// send to display function code or fault code
		if ((ui8_display_fault_code != NO_FAULT)&&(ui8_display_function_code == NO_FUNCTION)) {
			//#if ENABLE_XH18
			if (m_config.enable_xh18){
				if (ui8_display_fault_code == ERROR_WRITE_EEPROM) {
					// shared with ERROR_MOTOR_CHECK
					// instead of E09, display blinking E08
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 8;
					}
				}
				else if (ui8_display_fault_code == ERROR_OVERVOLTAGE) {	
					// instead of E01, display blinking E06
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 6;
					}
				}
				else if (ui8_display_fault_code == ERROR_THROTTLE) {	
					// instead of E05, display blinking E03
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 3;
					}
				}
				else if (ui8_display_fault_code == ERROR_BATTERY_OVERCURRENT) {	
					// instead of E07, display blinking E04
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 4;
					}
				}
				else {
					// fault code
					ui8_tx_buffer[5] = ui8_display_fault_code;
				}
			}
			else if ((m_config.enable_vlcd5) || (m_config.enable_vlcd6) || (m_config.enable_ekd01)) { //#elif ENABLE_VLCD5 || ENABLE_VLCD6 ||  ENABLE_EKD01
				if ((ui8_auto_display_data_status)
				|| (m_configuration_variables.ui8_assist_with_error_enabled)) {
					// display data
					ui8_tx_buffer[5] = CLEAR_DISPLAY;
				}
				else {
					// fault code
					ui8_tx_buffer[5] = ui8_display_fault_code;
				}
			}
			else { //#else // 850C
				// fault code
				ui8_tx_buffer[5] = ui8_display_fault_code;
			} // #endif
		}
		else if (ui8_display_function_code != NO_FUNCTION) {
            // ruedbi: this is the handling of the menu function codes displayed as error codes
			// on parameter change accept
			// function code
			if (m_config.enable_ekd01){ //#if ENABLE_EKD01
			// on this display there is no error code 3 to be found
			// there is also no other consecutive sequence of 3 errors to be found
			// so the sequence chosen is 1,2,4 instead
				switch(ui8_menu_index) {
					case 1:
						ui8_display_function_code = 1;
						break;
					case 2:
						ui8_display_function_code = 2;
						break;
					case 3:
						ui8_display_function_code = 4;
						break;
				}
			} // #endif
			// function code
			if ((!ui8_menu_flag)&&(ui8_menu_index > 0U)
			  &&((m_configuration_variables.ui8_set_parameter_enabled)
				||(ui8_assist_level == OFF))) {
				// display blinking function code
				if (ui8_default_flash_state) {
					ui8_tx_buffer[5] = ui8_display_function_code;
				}
				else {
					// clear code
					ui8_tx_buffer[5] = CLEAR_DISPLAY;
				}
			}
			else {
				// display data function code
				ui8_tx_buffer[5] = ui8_display_function_code;
			}
		}
		else {
			// clear code
			ui8_tx_buffer[5] = CLEAR_DISPLAY;
		}
		
		// send to display data value or wheel speed
		if (ui8_display_data_enabled) {
			// display data
			// The maximum value displayable on the display is 99.9, and is always sent in km/h.
			// By setting mph, it is the display that converts it, so the maximum displayable value becomes 62.4 (99.9/1.6),
			// Data that can exceed this value is best always divided by 10.
			if (ui8_torque_sensor_calibration_flag) {
				if (ui8_torque_sensor_calibration_with_weight_flag) {
					ui16_display_data = ui16_display_data_factor / (ui16_pedal_weight_x100 / 10U);
				}
				else if (ui8_display_torque_sensor_calibration_step_flag) {
					ui16_display_data = (ui16_display_data_factor / (uint16_t)ui8_torque_sensor_step_to_display) * 10U;
				}
				else if (ui8_display_torque_sensor_calibration_value_flag) {
					ui16_display_data = ui16_display_data_factor / ui16_torque_sensor_value_to_display;
				}
			}
			else if ((ui8_display_data_on_startup)&&(!ui8_startup_flag)) {
			  switch (ui8_display_data_on_startup) {
				case 1:
					if (m_config.units_type == MILES) {
					ui16_display_data = (ui16_display_data_factor / ui16_battery_SOC_percentage_x10) * 10U;
					} else {
					ui16_display_data = ui16_display_data_factor / ui16_battery_SOC_percentage_x10;
					}
				  break;
				case 2:
					// battery voltage calibrated x10 for display data
						ui16_battery_voltage_calibrated_x10 = (ui16_battery_voltage_filtered_x10 * ACTUAL_BATTERY_VOLTAGE_PERCENT) / 100U;
					ui16_display_data = ui16_display_data_factor / ui16_battery_voltage_calibrated_x10;
				  break;
				default:
					ui16_display_data = 0;
				  break;
			  }
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function)&&(ui8_menu_index > 0U)&&((ui8_assist_level < 2)||(ui8_display_alternative_lights_configuration))) { // OFF & ECO & alternative lights configuration
			  uint8_t index_temp = (ui8_display_function_status[ui8_menu_index - 1][ui8_assist_level]);
			  switch (index_temp) {
				case 0:
					ui16_display_data = ui16_display_data_factor / FUNCTION_STATUS_OFF;
				  break;
				case 1:
					ui16_display_data = ui16_display_data_factor / FUNCTION_STATUS_ON;
				  break;
				default:
				  break;
			  }
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function)&&(ui8_menu_index > 0U)&&(ui8_assist_level == TURBO)) {
				ui16_display_data = ui16_display_data_factor / (ui8_display_lights_configuration * (uint8_t)100 + DISPLAY_STATUS_OFFSET);
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function)&&(ui8_menu_index > 0U)) {
				ui16_display_data = ui16_display_data_factor / (ui8_display_riding_mode * (uint8_t)100 + DISPLAY_STATUS_OFFSET);
			}
			else {
			  switch (ui8_data_index_array[ui8_data_index]) {
				case 0:
					if (m_config.units_type == MILES) {
					ui16_display_data = (ui16_display_data_factor / ui16_motor_temperature_filtered_x10) * 10U;
					} else {
					ui16_display_data = ui16_display_data_factor / ui16_motor_temperature_filtered_x10;
					}
				  break;
				case 1:
					if (m_config.units_type == MILES) {
					ui16_display_data = (ui16_display_data_factor / ui16_battery_SOC_percentage_x10) * 10U;
					} else {
					ui16_display_data = ui16_display_data_factor / ui16_battery_SOC_percentage_x10;
					}
				  break;
				case 2:
					// battery voltage calibrated x10 for display data
					ui16_battery_voltage_calibrated_x10 = (ui16_battery_voltage_filtered_x10 * ACTUAL_BATTERY_VOLTAGE_PERCENT) / 100;
					ui16_display_data = ui16_display_data_factor / ui16_battery_voltage_calibrated_x10;
				  break;
				case 3:
					ui16_display_data = ui16_display_data_factor / ui8_battery_current_filtered_x10;
				  break;
				case 4:
					// battery power filtered x 10 for display data
					ui16_battery_power_filtered_x10 = filter(ui16_battery_power_x10, ui16_battery_power_filtered_x10, 13);
					if (m_config.units_type == MILES) {
					ui16_display_data = ui16_display_data_factor / (ui16_battery_power_filtered_x10);
					} else {
					ui16_display_data = ui16_display_data_factor / (ui16_battery_power_filtered_x10 / 10U);
					}
				  break;
				case 5:
					ui16_display_data = ui16_display_data_factor / (ui16_adc_throttle >> 2);
				  break;
				case 6:
					ui16_display_data = ui16_display_data_factor / ui16_adc_pedal_torque;
				  break;
				case 7:
					if (m_config.units_type == MILES) {
						ui16_display_data = (ui16_display_data_factor / ui8_pedal_cadence_RPM) * 10U;
					} else {
						if (ui8_pedal_cadence_RPM > 99) {
							ui16_display_data = ui16_display_data_factor / ui8_pedal_cadence_RPM;
						}
						else {
							ui16_display_data = (ui16_display_data_factor / ui8_pedal_cadence_RPM) * 10U;
						}
					}
				  break;
				case 8:
					// human power filtered x 10 for display data
					ui16_human_power_filtered_x10 = filter(ui16_human_power_x10, ui16_human_power_filtered_x10, 13);
					ui16_display_data = ui16_display_data_factor / (ui16_human_power_filtered_x10 / 10U);
				  break;
				case 9:
					ui16_display_data = ui16_display_data_factor / ui16_adc_pedal_torque_delta;
				  break;
				case 10:
					if (m_config.units_type == MILES) {
						ui16_display_data = ui16_display_data_factor / (uint16_t) ui32_wh_x10;
					} else {
						ui16_display_data = ui16_display_data_factor / (uint16_t) (ui32_wh_x10 / 10U);
					}
				  break;
				case 11:
					ui16_display_data = ui16_display_data_factor / ui16_motor_speed_erps;
				  break;
				case 12:
					// added by mstrens because g_duty_cycle is now in 16 bits
					uint16_t temp_duty_cycle = (ui16_g_duty_cycle + 0x80) >> 8; // rounding
					ui16_duty_cycle_percent = (uint16_t) ((temp_duty_cycle * (uint8_t)100) / PWM_DUTY_CYCLE_MAX) - 1;
					ui16_display_data = (ui16_display_data_factor / ui16_duty_cycle_percent) * 10U;
				  break;
				default:
				  break;
			  }
			}
			
			// todo, filter for 500C display
			
			// valid value
			if (ui16_display_data > 0U) {
				ui8_tx_buffer[6] = (uint8_t) (ui16_display_data & 0xFF);
				ui8_tx_buffer[7] = (uint8_t) (ui16_display_data >> 8);
			}
			else {
				ui8_tx_buffer[6] = 0x07;
				ui8_tx_buffer[7] = 0x07;
			}
		}
		else {
			// wheel speed
			if (ui16_oem_wheel_speed_time > 0U) {
				//#if ALTERNATIVE_MILES
				if (m_config.alternative_miles){
					// in VLCD6 display the km/miles conversion is not present.
					// alternative mph for VLCD6 converts the sent speed time
					// applicable to other displays type, setting km/h on diplay
					// odometer history would remain in km, only those added would be in miles.
					ui16_data_value = (ui16_oem_wheel_speed_time * 16) / 10;
					ui8_tx_buffer[6] = (uint8_t) (ui16_data_value & 0xFF);
					ui8_tx_buffer[7] = (uint8_t) (ui16_data_value >> 8);
				} else { //	#else
					// km/h or mph
					ui8_tx_buffer[6] = (uint8_t) (ui16_oem_wheel_speed_time & 0xFF);
					ui8_tx_buffer[7] = (uint8_t) (ui16_oem_wheel_speed_time >> 8);
				} //#endif
			}
			else {
				ui8_tx_buffer[6] = 0x07;
				ui8_tx_buffer[7] = 0x07;
			}
		}
				
		// set working flag
		//#if ENABLE_DISPLAY_ALWAYS_ON
		if (m_config.enable_display_always_on){
			// set working flag used to hold display always on
			ui8_working_status |= 0x04;
		} //#endif
		
		//#if ENABLE_DISPLAY_WORKING_FLAG
		if (m_config.enable_display_working_flag) {
			// wheel turning
			if (ui16_oem_wheel_speed_time > 0U) {
				// bit7 = 1 (wheel turning)
				ui8_working_status |= 0x80;
			}
			else {
				// bit7 = 0 (wheel not turning)
					ui8_working_status &= 0x7F;
			}
			// motor working
			// Mstrens : modified because g_duty_cycle is now in 16 bits
			if (ui16_g_duty_cycle > (10 << 8)) {
				// bit6 = 1 (motor working)
				ui8_working_status |= 0x40;
			}
			else {
				// bit6 = 0 (motor not working)
				ui8_working_status &= 0xBF;
			}
			// motor working or wheel turning?
			if (ui8_working_status & 0xC0)
			{
				// set working flag used by display
				ui8_working_status |= 0x04;
			}
			else {
				// clear working flag used by display
				ui8_working_status &= 0xFB;
			}
		} //#endif

		// working status
		ui8_tx_buffer[2] = (ui8_working_status & 0x1F);
		
		// clear motor working, wheel turning and working flags
		ui8_working_status &= 0x3B;	
		
		// prepare check code of the package
		ui8_tx_check_code = 0x00;
		for(ui8_i = 0; ui8_i < TX_CHECK_CODE; ui8_i++)
		{
			ui8_tx_check_code += ui8_tx_buffer[ui8_i];
		}
		ui8_tx_buffer[TX_CHECK_CODE] = ui8_tx_check_code;

		// send the buffer on uart
		if ((30 - XMC_USIC_CH_TXFIFO_GetLevel(CYBSP_DEBUG_UART_HW) ) >= 10){ // check if there is enough free space in Txfifo
			for(uint8_t i = 0; i < 9; i++)  {
				XMC_USIC_CH_TXFIFO_PutData(CYBSP_DEBUG_UART_HW, (uint16_t) ui8_tx_buffer[i]);
				//XMC_UART_CH_Transmit(CYBSP_DEBUG_UART_HW , ui8_tx_buffer[i]);
			}
		}	 
	}
}




static void calc_oem_wheel_speed(void)
{ 
	if (ui8_display_ready_flag) {
		// oem wheel speed (wheel turning time) ms/2 - speed conversion for different perimeter
		// ui8_oem_wheel_diameter is in inches.
		// Conversion inche to mm perimeter = 25.4 * 3.1415 = 79.8 = 80
		ui16_oem_wheel_speed_time = (uint16_t)((uint32_t)(uint16_t)(ui8_oem_wheel_diameter * 80U * 10U) * ui16_wheel_speed_sensor_ticks 
			/ ((uint32_t)m_configuration_variables.ui16_wheel_perimeter * OEM_WHEEL_SPEED_DIVISOR)); // OEM_WHEEL_SPEED_DIVISOR is x10
	}
	
	//#if ENABLE_ODOMETER_COMPENSATION
	if (m_config.enable_odometer_compensation){
		uint16_t ui16_wheel_speed;
		uint16_t ui16_data_speed;
		uint16_t ui16_speed_difference;

		// calc wheel speed  mm/0.1 sec
		if (ui16_oem_wheel_speed_time > 0U) {
			ui16_wheel_speed = (ui16_display_data_factor / ui16_oem_wheel_speed_time) * (100U / 36U);
		}
		else {
			ui16_wheel_speed = 0;
		}
		// calc data speed  mm/0.1 sec
		if (ui16_display_data > 0U) {
			ui16_data_speed = (ui16_display_data_factor / ui16_display_data) * (100U / 36U);
		}
		else {
			ui16_data_speed = 0;
		}
		// calc odometer difference
		if (ui8_display_data_enabled) {	
			if (ui16_data_speed > ui16_wheel_speed) {	
				// calc + speed difference mm/0.1 sec
				ui16_speed_difference = ui16_data_speed - ui16_wheel_speed;
				// add difference to odometer
				ui32_odometer_compensation_mm += (uint32_t) ui16_speed_difference;
			}
			else {
				// calc - speed difference mm/0.1 sec
				ui16_speed_difference = ui16_wheel_speed - ui16_data_speed;
					// subtracts difference from odometer
				ui32_odometer_compensation_mm -= (uint32_t) ui16_speed_difference;
			}
		}
		else {
			// odometer compensation
			if ((ui16_wheel_speed)&&(ui32_odometer_compensation_mm > ZERO_ODOMETER_COMPENSATION)) {
				ui32_odometer_compensation_mm -= (uint32_t) ui16_wheel_speed;
				ui16_oem_wheel_speed_time = 0;
			}
		}
	} // endif
} 


static void calc_watt_hours_used(void)
{
	// battery power x 10
	ui16_battery_power_x10 = (uint16_t)(((uint32_t) ui16_battery_voltage_filtered_x10 * ui8_battery_current_filtered_x10) / 10);
	
	// consumed watt-hours
	ui32_wh_sum_x10 += ui16_battery_power_x10;
	// calculate watt-hours X10 since power on
	ui32_wh_since_power_on_x10 = ui32_wh_sum_x10 / 32400; // 36000 -10% for calibration
	// calculate watt-hours X10 since last full charge
	ui32_wh_x10 = ui32_wh_offset_x10 + ui32_wh_since_power_on_x10;
}


static void check_battery_soc(void)
{
	#define BATTERY_SOC_COUNTER_THRESHOLD			100 // 10 seconds
	static uint8_t ui8_no_load_counter = 20;
	static uint8_t ui8_battery_state_of_charge_temp = 0;
	static uint8_t ui8_battery_state_of_charge_counter = 0;
	uint16_t ui16_battery_voltage_x10;
	uint16_t ui16_battery_SOC_used_x10;
	uint16_t ui16_actual_battery_SOC_x10;
	
	// battery voltage x10
	ui16_battery_voltage_x10 = (ui16_battery_voltage_filtered_x1000) / 100;
	
	// filter battery voltage x10
	ui16_battery_voltage_filtered_x10 = filter(ui16_battery_voltage_x10, ui16_battery_voltage_filtered_x10, 4);
	
	// save no load voltage x10 if current is < adc current min for 2 seconds
	if (ui8_adc_battery_current_filtered < 2) {
		if (++ui8_no_load_counter > 20) {
			ui16_battery_no_load_voltage_filtered_x10 = ui16_battery_voltage_x10;
			ui8_no_load_counter--;
		}
	}
	else {
		ui8_no_load_counter = 0;
	}

	// filter battery voltage soc x10
	ui16_battery_voltage_soc_filtered_x10 = filter(ui16_battery_no_load_voltage_filtered_x10, ui16_battery_voltage_soc_filtered_x10, 3);

	//#if ENABLE_VLCD6 || ENABLE_XH18
	if ((m_config.enable_vlcd6) || (m_config.enable_xh18)) {
		if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_6_X10) { ui8_battery_state_of_charge = 7; }		// overvoltage
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_5_X10) { ui8_battery_state_of_charge = 6; }	// 4 bars -> SOC reset
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_4_X10) { ui8_battery_state_of_charge = 5; }	// 4 bars -> full
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_3_X10) { ui8_battery_state_of_charge = 4; }	// 3 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_2_X10) { ui8_battery_state_of_charge = 3; }	// 2 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_1_X10) { ui8_battery_state_of_charge = 2; }	// 1 bar
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_0_X10) { ui8_battery_state_of_charge = 1; }	// blink -> empty
		else { ui8_battery_state_of_charge = 0; } // undervoltage
	//#else // ENABLE_VLCD5 or 850C || ENABLE_EKD01
	} else {
		if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_8_X10) { ui8_battery_state_of_charge = 9; }   		// overvoltage
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_7_X10) { ui8_battery_state_of_charge = 8; }	// 6 bars -> SOC reset
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_6_X10) { ui8_battery_state_of_charge = 7; }	// 6 bars -> full
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_5_X10) { ui8_battery_state_of_charge = 6; }	// 5 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_4_X10) { ui8_battery_state_of_charge = 5; }	// 4 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_3_X10) { ui8_battery_state_of_charge = 4; }	// 3 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_2_X10) { ui8_battery_state_of_charge = 3; }	// 2 bars
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_1_X10) { ui8_battery_state_of_charge = 2; }	// 1 bar
		else if (ui16_battery_voltage_soc_filtered_x10 > c_BATTERY_SOC_VOLTS_0_X10) { ui8_battery_state_of_charge = 1; }	// blink -> empty
		else { ui8_battery_state_of_charge = 0; } // undervoltage
	}
	// To avoid SOC fluctuations when voltage value is equal to BATTERY_SOC_VOLTS_x
	if (ui8_display_ready_flag) {
		if (ui8_battery_state_of_charge_temp != ui8_battery_state_of_charge) {
			ui8_battery_state_of_charge_counter++;
		}
		else {
			ui8_battery_state_of_charge_counter = 0;
		}
		if (ui8_battery_state_of_charge_counter >= BATTERY_SOC_COUNTER_THRESHOLD) {
			ui8_battery_state_of_charge_temp = ui8_battery_state_of_charge;
		}
		else {
			ui8_battery_state_of_charge = ui8_battery_state_of_charge_temp;
		}
	}
	else {
		ui8_battery_state_of_charge_temp = ui8_battery_state_of_charge;
	}
	
	// calculate and set remaining percentage x10
	if (m_configuration_variables.ui8_soc_percent_calculation == SOC_CALC_VOLTS) {
			ui16_battery_SOC_percentage_x10 = read_battery_soc();
	}
	else { // Auto or Wh
		// calculate percentage battery capacity used x10
		ui16_battery_SOC_used_x10 = (uint16_t)(((uint32_t) ui32_wh_x10 * 100) / ui16_actual_battery_capacity);
		
		// limit used percentage to 100 x10
		if (ui16_battery_SOC_used_x10 > 1000) {
			ui16_battery_SOC_used_x10 = 1000;
		}
		ui16_battery_SOC_percentage_x10 = 1000 - ui16_battery_SOC_used_x10;
	}
	
	// automatic reset battery SOC percentage x10 at power on
	if ((ui8_display_ready_flag)&&(!ui8_startup_flag)) {
		if (ui8_startup_counter < DELAY_MENU_ON) {
			ui8_startup_counter++;
			
			// if the battery is fully charged
			if ((!ui8_battery_SOC_reset_flag)
				&&(ui16_battery_voltage_filtered_x10 > BATTERY_VOLTAGE_RESET_SOC_PERCENT_X10))	{
					ui16_battery_SOC_percentage_x10 = 1000;
					ui32_wh_offset_x10 = 0;
					ui8_battery_SOC_reset_flag = 1;
		}
		
		// if SOC calculation is set to auto
		if ((!ui8_battery_SOC_reset_flag)
			  &&(ui8_startup_counter >= (DELAY_MENU_ON >> 1)))	{
			// check soc percentage
			if (m_configuration_variables.ui8_soc_percent_calculation == SOC_CALC_AUTO) {
				ui16_actual_battery_SOC_x10 = read_battery_soc();
					
				if (((ui16_actual_battery_SOC_x10 + BATTERY_SOC_PERCENT_THRESHOLD_X10) < ui16_battery_SOC_percentage_x10)
					|| (ui16_actual_battery_SOC_x10 > (ui16_battery_SOC_percentage_x10 + BATTERY_SOC_PERCENT_THRESHOLD_X10))) {
						// reset soc percentage
					ui16_battery_SOC_percentage_x10 = ui16_actual_battery_SOC_x10;
					// calculate watt-hours x10
						ui32_wh_offset_x10 = ((uint32_t)(1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
					}
					//ui8_battery_SOC_reset_flag = 1;
			}
			else {
				ui8_battery_SOC_reset_flag = 1;
			}
		}
	}
		else {
			ui8_startup_flag = 1;
			ui8_battery_SOC_reset_flag = 1;
		}
	}
	
	// convert remaining percentage x10 to 8 bit
	m_configuration_variables.ui8_battery_SOC_percentage_8b = (uint8_t)(ui16_battery_SOC_percentage_x10 >> 2);
}


// read battery percentage x10 (actual charge)
uint16_t read_battery_soc(void)
{
	uint16_t ui16_battery_SOC_calc_x10 = 0;
	
	//#if ENABLE_VLCD6 || ENABLE_XH18
	if ((m_config.enable_vlcd6) || (m_config.enable_xh18)) {
		switch (ui8_battery_state_of_charge) {
			case 0:	ui16_battery_SOC_calc_x10 = 10; break;  // undervoltage
			case 1:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(1, 250, c_LI_ION_CELL_VOLTS_1_X100, c_LI_ION_CELL_VOLTS_0_X100); break; // blink - empty
			case 2:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(250, 250, c_LI_ION_CELL_VOLTS_2_X100, c_LI_ION_CELL_VOLTS_1_X100); break; // 1 bars
			case 3:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(500, 250, c_LI_ION_CELL_VOLTS_3_X100, c_LI_ION_CELL_VOLTS_2_X100); break; // 2 bars
			case 4:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(750, 250, c_LI_ION_CELL_VOLTS_4_X100, c_LI_ION_CELL_VOLTS_3_X100); break; // 3 bars
			case 5:	ui16_battery_SOC_calc_x10 = 1000; break; // 4 bars - full
			case 6:	ui16_battery_SOC_calc_x10 = 1000; break; // 4 bars - SOC reset
			case 7:	ui16_battery_SOC_calc_x10 = 1000; break; // overvoltage
		}
	//#else // ENABLE_VLCD5 or 850C  or ENABLE_EKD01
	} else {
		switch (ui8_battery_state_of_charge) {
			case 0:	ui16_battery_SOC_calc_x10 = 10; break;  // undervoltage
			case 1:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(1, 167, c_LI_ION_CELL_VOLTS_1_X100, c_LI_ION_CELL_VOLTS_0_X100); break; // blink - empty
			case 2:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(167, 167, c_LI_ION_CELL_VOLTS_2_X100, c_LI_ION_CELL_VOLTS_1_X100); break; // 1 bars
			case 3:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(334, 167, c_LI_ION_CELL_VOLTS_3_X100, c_LI_ION_CELL_VOLTS_2_X100); break; // 2 bars
			case 4:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(500, 167, c_LI_ION_CELL_VOLTS_4_X100, c_LI_ION_CELL_VOLTS_3_X100); break; // 3 bars
			case 5:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(667, 167, c_LI_ION_CELL_VOLTS_5_X100, c_LI_ION_CELL_VOLTS_4_X100); break; // 4 bars
			case 6:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(834, 167, c_LI_ION_CELL_VOLTS_6_X100, c_LI_ION_CELL_VOLTS_5_X100); break; // 5 bars
			case 7:	ui16_battery_SOC_calc_x10 = 1000; break; // 6 bars - full
			case 8:	ui16_battery_SOC_calc_x10 = 1000; break; // 6 bars - SOC reset
			case 9:	ui16_battery_SOC_calc_x10 = 1000; break; // overvoltage
		}	
	} // endif
	
	return ui16_battery_SOC_calc_x10;
}


// calculate battery soc percentage x10
uint16_t calc_battery_soc_x10(uint16_t ui16_battery_soc_offset_x10, uint16_t ui16_battery_soc_step_x10, uint16_t ui16_cell_volts_max_x100, uint16_t ui16_cell_volts_min_x100)
{
#define CELL_VOLTS_CALIBRATION				8
	
	uint16_t ui16_cell_voltage_soc_x100 = ui16_battery_voltage_soc_filtered_x10 * 10 / BATTERY_CELLS_NUMBER;
	
	uint16_t ui16_battery_soc_calc_temp_x10 = (ui16_battery_soc_offset_x10 + (ui16_battery_soc_step_x10 * (ui16_cell_voltage_soc_x100 - ui16_cell_volts_min_x100) / (ui16_cell_volts_max_x100 - ui16_cell_volts_min_x100 + CELL_VOLTS_CALIBRATION)));
	
	return ui16_battery_soc_calc_temp_x10;
}
