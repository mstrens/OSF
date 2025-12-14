#include "main.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "motor.h"
#include "ebike_app.h"
#include "common.h"
#include "adc.h"
#include <math.h>
#include "systick.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"
#if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
#include "ProbeScope/probe_scope.h"
#endif

volatile uint32_t ui32_ms_counter = 0;

// cadence sensor
//#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;

// wheel speed sensor
volatile uint16_t ui16_wheel_speed_sensor_ticks = 0;
volatile uint32_t ui32_wheel_speed_sensor_ticks_total = 0;


// new wheel and cadence variables
// =============== VARIABLES PARTAGÉES =============== 
volatile uint32_t ui32_pwm_ticks = 0;          // compteur soft 19kHz
volatile uint32_t ui32_cadence_last_ticks[6] = {0};   // timestamps pédalage (codes 0..5)
volatile uint32_t ui32_wheel_last_pwm_ticks = 0; // dernier front roue (ui32_pwm_ticks)

uint8_t lead_angle_multiplicator = 64;

// battery current variables
volatile uint16_t ui16_adc_motor_phase_current = 0; // mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage = 0;

//Torque added by mstrens
volatile uint16_t ui16_adc_torque_filtered = 0 ; // filtered adc torque

// brakes
volatile uint8_t ui8_brake_state = 0;


// battery soc
volatile uint8_t ui8_battery_SOC_saved_flag = 0;
volatile uint8_t ui8_battery_SOC_reset_flag = 0;

// to manage torque sensor using the logic of mspider in https://github.com/TSDZ2-ESP32/TSDZ2-Smart-EBike
// 1 = one of 1/20 of a rotation occured (= 4 state transitions )
// 0x80  = reverse rotation  or timeout detected (stop)-> reset
volatile uint8_t ui8_pas_new_transition = 0; // use also in ebike_app.c and main.c

volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73

// voltage check
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2

volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint16_t ui16_g_duty_cycle = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;

// Field Weakening Hall offset (added during interpolation)
volatile uint8_t ui8_fw_hall_counter_offset = 0;
volatile uint8_t ui8_fw_hall_counter_offset_max = 0;
volatile uint8_t ui8_field_weakening_enabled = 0;


//uint16_t ui16_debug_fw_cnt= 0;
//int8_t i8_debug_idx_ref = -2;
//uint32_t ui32_debug_delta_ticks = 0;

volatile uint32_t debug_rpm = 0;
volatile uint32_t debug_erps = 0;


// this function is called in systick ISR (at 1kHz) 
// it calculates wheel and cadence ticks using the data collected at 19 kHz; so ticks are at PWM frequency
// conversion to rpm is done is ebike.app
void SysTick_Handler(void) {
    
    // --- Wheel --- 
    static uint32_t ui32_prev_wheel_pwm_tick = 0;
    static uint32_t ui32_last_wheel_ms = 0;
    // --- cadence --- 
    static int8_t i8_prev_cadence_index = -1;      // -1 = pas encore de référence
    static uint32_t ui32_prev_cadence_tick = 0;
    static uint32_t ui32_last_cadence_ms = 0;
    static uint32_t ui32_prev_cadence_tick_max = 0;          // pour détecter un vrai nouveau front
    static uint8_t ui8_pas_counter = 0; // counter to detect a full pedal rotation (after 20 valid transitions)

//  ========= only for documentation if we have to use pwm ticks
//static inline uint32_t read_ui32_pwm_ticks_atomic(void) {
//    uint32_t a,b;
//    do { a = ui32_pwm_ticks; b = ui32_pwm_ticks; } while (a != b);
//    return a;
//}
    ui32_ms_counter++;  // used to detect timeout
    // --------- 1) cadence --------- 
    // Cherche l’index (0..4) ayant le timestamp le plus grand
    uint8_t ui8_cadence_idx_max = 0;
    uint32_t ui32_cadence_tick_max = ui32_cadence_last_ticks[0];
    for (uint8_t i = 1; i <= 4; ++i) {
        uint32_t t = ui32_cadence_last_ticks[i];
        if(t > ui32_cadence_tick_max) { ui32_cadence_tick_max = t; ui8_cadence_idx_max = i; }
    }

    // Check if a new cadence event occured
    if (ui32_cadence_tick_max != ui32_prev_cadence_tick_max) {
        ui32_prev_cadence_tick_max = ui32_cadence_tick_max;
        if (ui8_cadence_idx_max == 4) { // --- reverse cadence rotation ---
            ui16_cadence_sensor_ticks = 0; // reset value used in ebike_app.c
            i8_prev_cadence_index = -1;
             ui8_pas_new_transition = 0x80; // used in mspider logic for torque sensor // to do
        } else { // --- forward cadence (codes 0..3) ---
            //ui16_debug_fw_cnt++;
            if (i8_prev_cadence_index < 0) {   // Premier front après arrêt → initialise seulement
                i8_prev_cadence_index = (int8_t)ui8_cadence_idx_max;
                //i8_debug_idx_ref = i8_prev_cadence_index;
                ui32_prev_cadence_tick = ui32_cadence_tick_max;
                ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation used to detect a full rotation for torque (spider)
            } else { // On a déjà une référence
                uint32_t ui32_curr_cadence_tick = ui32_cadence_last_ticks[i8_prev_cadence_index]; 
                // if tick for same index is different, then calculate elapsed ticks
                if (ui32_curr_cadence_tick != ui32_prev_cadence_tick) {
                    uint32_t ui32_cadence_delta_ticks = ui32_curr_cadence_tick  - ui32_prev_cadence_tick;
                    //ui32_debug_delta_ticks = ui32_cadence_delta_ticks ; 
                    ui16_cadence_sensor_ticks = (uint16_t) ui32_cadence_delta_ticks;
                    ui32_prev_cadence_tick =  ui32_curr_cadence_tick;
                    
                    ui8_pas_new_transition = 1; // mspider logic for torque sensor;mark for one of the 20 transitions per rotation
                    ui8_pas_counter++; // mstrens : increment the counter when the transition is valid           
                } else {
                    // when max timestamp changed (but not yet the timestamp of reference transition)
                    //  set the cadence to 7 RPM for immediate start if it was 0
                    if (ui16_cadence_sensor_ticks == 0) ui16_cadence_sensor_ticks = CADENCE_TICKS_STARTUP; // 7619
                }
            }
        }
        ui32_last_cadence_ms = ui32_ms_counter;
    }

    // cadence TIMEOUTS --------- 
    if ((ui32_ms_counter - ui32_last_cadence_ms) > (ui16_cadence_ticks_count_min_speed_adj)) { // adj =4270 at 4km/h ... 341 at 40 km/h
        ui16_cadence_sensor_ticks = 0; // reset cadence
        i8_prev_cadence_index = -1;
        ui32_prev_cadence_tick = 0;
        ui8_pas_new_transition = 0x80; // for mspider logic for torque sensor
        ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation
    }
     
    // --------- 2) Wheel --------- 
    uint32_t ui32_wheel_pwm_tick = ui32_wheel_last_pwm_ticks; // ui32_wheel_last_pwm_ticks = pwm ticks of last rising edge
    if (ui32_wheel_pwm_tick != ui32_prev_wheel_pwm_tick) {
        uint32_t ui32_wheel_delta_ticks;
        if (ui32_prev_wheel_pwm_tick == 0) {
            ui32_wheel_delta_ticks = 0; // first ticks after a stop
        } else {
            ui32_wheel_delta_ticks = (ui32_wheel_pwm_tick - ui32_prev_wheel_pwm_tick);
        }
        ui32_prev_wheel_pwm_tick = ui32_wheel_pwm_tick; // save for next comparison
        ui32_last_wheel_ms = ui32_ms_counter;           // used to detect when wheel stopped (time out)
        if (ui32_wheel_delta_ticks > 0) {
            if (ui32_wheel_delta_ticks > 600) { // 600 at 19Khz => 2000mm/1000000(km) * 19000kHz/600 * 3600sec = 228 km/h
                // set the value used in ebike_app.c to wheel speed when speed is not to high
                ui16_wheel_speed_sensor_ticks = ui32_wheel_delta_ticks ; // ticks are based on PWM frequency
                ++ui32_wheel_speed_sensor_ticks_total; // used only in 860C version to calculate the distance in 860c
            } else {
                // nothing :  discard the value and keep previous speed
            }    
        }
    }

    // wheel TIMEOUTS --------- 
    if ((ui32_ms_counter - ui32_last_wheel_ms) > (WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN/19)) {
        ui16_wheel_speed_sensor_ticks = 0; // reset wheel speed
        ui32_prev_wheel_pwm_tick = 0;
    }

    //      3)  get raw adc torque sensor (in 10 bits) and filter
    uint16_t ui16_adc_torque_raw   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_TORQUE_RESULT_REG ) & 0xFFF) >> 2; // torque gr0 ch7 result 7 in bg p2.2
    //filter it (3 X previous + 1 X new)
    uint16_t ui16_adc_torque_new_filtered = ( ui16_adc_torque_raw + (ui16_adc_torque_filtered<<1) + ui16_adc_torque_filtered) >> 2;
    if (ui16_adc_torque_new_filtered == ui16_adc_torque_filtered){ // code to ensure it reaches the limits
        if ( ui16_adc_torque_new_filtered < ui16_adc_torque_raw) 
            ui16_adc_torque_new_filtered++; 
        else if (ui16_adc_torque_new_filtered > ui16_adc_torque_raw) 
            ui16_adc_torque_new_filtered--;
    }
    ui16_adc_torque_filtered = ui16_adc_torque_new_filtered;
    
    //      4) get the voltage 
     //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
    // changed to take care of infineon VADC init (result in reg 6)
    ui16_adc_voltage = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_VDC_RESULT_REG ) & 0x0FFF) >> 2; // battery gr1 ch6 result 6

    //      5) get ui16_g_motor_phase_current (used to reduced duty cycle in systick and to get an error in ebike_app.c)
    if (ui16_g_duty_cycle > 0) {
        // calculate phase current.
        if (ui16_g_duty_cycle > (2<<8)) {
            ui16_adc_motor_phase_current = (uint16_t)((uint16_t)(((uint16_t)ui8_adc_battery_current_filtered) << 8)) / (ui16_g_duty_cycle >> 8);
        } else {
            ui16_adc_motor_phase_current = (uint16_t)ui8_adc_battery_current_filtered;
        }
    } else {
        ui16_adc_motor_phase_current = 0;
    }

    //     6) get brake state-
    ui8_brake_state = XMC_GPIO_GetInput(IN_BRAKE_PORT, IN_BRAKE_PIN) == 0; // Low level means that brake is on
        
    /*  Has been commented for testing PLL; to be activated later on !!!!!!!!!!!!!!!!!!!!!!
    //     5) get lead angle 
    static uint32_t ui32_foc_angle_accum = 0; // use more bits for better accuracy in IIR
    // update foc_angle and adc_motor_phase_current
        // foc_angle is added to the position given by hall sensor + interpolation )
        
//            if (ui8_foc_flag) { // is set on 1 when rotor is at 150° so once per electric rotation
				//uint16_t ui16_adc_foc_angle_current = ((uint16_t)(ui8_adc_battery_current_filtered ) + (ui16_adc_motor_phase_current )) >> 1;
                // mstrens : added 128 for better rounding
                //ui8_foc_flag = ((ui16_adc_foc_angle_current * ui8_foc_angle_multiplicator) + 128) >> 8 ; // multiplier = 39 for 48V tsdz2, 
                // foc based on current
                //uint16_t ui8_foc = ((uint16_t) ui8_adc_battery_current_filtered * (uint16_t) ui8_foc_angle_multiplicator)  ; // multiplier = 39 for 48V tsdz2, 
                // foc based on velocity
                // ratio RPM to velocity is 4,474; so for 1000 rpm => velocity = 4474
                // with multiplicator = 64  ,  foc = 4474 * 64 / 256 => foc = about 1120 = about 4 ° in Q8_8
            uint32_t ui32_foc = ((ui32_hall_velocity_q8_8X1024 * (uint32_t) lead_angle_multiplicator )>>8) ; 
            // * 64 >> 8 = 64/256 
            
            // max = 23 *100 / 16 * 40 = 22
            if (ui32_foc > (25 * 256)) // limit in Q8_8
                ui32_foc = (25 * 256);
            // filtre iir convergent
            ui32_foc_angle_accum = ui32_foc_angle_accum - (ui32_foc_angle_accum >> 4) + (ui32_foc);
            ui16_g_foc_angle_q8_8 = (uint16_t)(ui32_foc_angle_accum >> 4);
        } else { // duty cycle = 0
            ui16_adc_motor_phase_current = 0;
            ui32_foc_angle_accum = 0; // reset accumulator (used for accuracy)
            ui16_g_foc_angle_q8_8 = 0;             
        }
    */
    update_lead_angle();
    systick_security_checks(); // this must be before update_duty_cycle() because it can change the way duty cycle is calculated
    update_duty_cycle();    // apply ramp up/down on duty cycle   
    debug_rpm = pll_get_rpm();
    debug_erps = pll_get_erps();
         
} // end systick_handler


// ---------------------------------------------------
// constant and formats for lead angle 
// ---------------------------------------------------
#define Q30_SCALE           (1UL << 30)
#define DEG_TO_Q8_8(x)      ((uint16_t)((x) * (65536.0f / 360.0f) + 0.5f))
#define HALL_VELOCITY_RATIO          (4.474f)      // ratio between RPM and hall velocity_q8_8x1024

#define LEAD_STEP_MIN_DEGREE  (0.02)       // ≈ 0.022° // lead angle correction is updated per small steps; varies between min and max
#define LEAD_STEP_MAX_DEGREE  (0.35)       // ≈ 0.35°
#define MAX_LEAD_CORR_DEGREE  (5)        // max for correction (in plus and min)

#define LOW_SPEED_RPM        (200)       // below this speed, lead angle is set on 0
#define SPEED_FILTER_A_Q15   (30000)  // coeff IIR vitesse (α≈0.9)
#define SPEED_FILTER_B_Q15   (32768 - SPEED_FILTER_A_Q15)

#define IDABS_DEFAULT        (100)       // seuil absolu min en ADC units ;  Dead band adaptatif; this is the min ; it applies on Id
#define K_REL_Q15            (1638)      // 0.05 * 32768 (5%)
#define HYST_FACTOR_Q15      (29491)     // 0.9 en Q15

#define LEAD_ANGLE_Q8_8_PER_ADC_STEP (17) // = 750 / 45 = 17 : Test showed that for a speed of about 2500 RPM,
                 // lead angle should varies by about 3° = 750 q8_8 units for a delta of 45 ADC steps
                 //                           45 = between 10 and 55 ADC current 10 bits 

#define MAX_LEAD_CORR_Q8_8  ((uint16_t)((MAX_LEAD_CORR_DEGREE << 16)/360))  // apply on corection
#define LEAD_STEP_MIN_Q8_8  ((uint16_t)(LEAD_STEP_MIN_DEGREE * (65536.0f / 360.0f) + 0.5f)) // apply on total   
#define LEAD_STEP_MAX_Q8_8  ((uint16_t)(LEAD_STEP_MAX_DEGREE * (65536.0f / 360.0f) + 0.5f)) // apply on total

// ---------------------------------------------------
// Tables de base (utilisateur)
// ---------------------------------------------------
const uint16_t speed_tab[] = {0,     500,  1000, 2000, 3000, 4000};
#define SPEED_TAB_SIZE (sizeof(speed_tab) / sizeof(speed_tab[0]))

const float lead_base_deg[] = {0.0f, 2.0f, 4.0f, 7.5f, 11.0f, 14.0f};

// ---------------------------------------------------
// Tables internes générées au premier passage
// ---------------------------------------------------
static uint16_t velocity_tab[SPEED_TAB_SIZE];
static uint16_t lead_base_q8_8[SPEED_TAB_SIZE];
static uint32_t inv_delta_velocity_q30[SPEED_TAB_SIZE - 1];
static uint32_t hall_low_speed_threshold = 0;
static uint8_t tables_initialized = 0;

// ---------------------------------------------------
// Variables dynamiques
// ---------------------------------------------------
static uint16_t tick_5ms = 0;
static int32_t  i32_pll_velocity_filt_q8_8X1024 = 0;
static int32_t  lead_corr_q8_8 = 0;
static uint16_t lead_base_q8_8_val = 0;
uint16_t ui16_lead_total_q8_8 = 0;
static int16_t i16_adc_battery_current_for_lead_base = 0;
// Deadband adaptatif
static int32_t last_deadband = 0;

// ---------------------------------------------------
// Fonctions utilitaires
// ---------------------------------------------------
uint8_t debug_tab_index = 0; 
uint16_t debug_velocity_tab = 0;
uint16_t debug_lead_base_q8_8 = 0;

static void init_lead_tables(void)
{
    for (uint8_t i = 0; i < SPEED_TAB_SIZE; i++) {
        velocity_tab[i] = (uint16_t)(speed_tab[i] * HALL_VELOCITY_RATIO + 0.5f);
        lead_base_q8_8[i] = DEG_TO_Q8_8(lead_base_deg[i]);
    }

    for (uint8_t i = 0; i < SPEED_TAB_SIZE - 1; i++) {
        uint32_t delta = (uint32_t)(velocity_tab[i + 1] - velocity_tab[i]);
        if (delta == 0) delta = 1;
        inv_delta_velocity_q30[i] = Q30_SCALE / delta;
    }
    hall_low_speed_threshold = (uint32_t)(LOW_SPEED_RPM * HALL_VELOCITY_RATIO + 0.5f);

    tables_initialized = 1;
}

static uint16_t interpolate_lead_base_from_hall_velocity(uint16_t hall_vel)
{
    if (hall_vel <= velocity_tab[0])
        return lead_base_q8_8[0];
    if (hall_vel >= velocity_tab[SPEED_TAB_SIZE - 1])
        return lead_base_q8_8[SPEED_TAB_SIZE - 1];

    uint8_t idx = 0;
    while (hall_vel > velocity_tab[idx + 1])
        idx++;

    uint32_t delta_hall_vel = hall_vel - velocity_tab[idx];
    uint32_t t_q15 = (delta_hall_vel * inv_delta_velocity_q30[idx])>>15; // >>15 because number can be to big
    uint32_t delta_angle = (uint32_t)(lead_base_q8_8[idx + 1] - lead_base_q8_8[idx]);
    uint32_t interp = (uint32_t)lead_base_q8_8[idx] + ((t_q15 * delta_angle) >> 15); // >>15 is the remaining part of inv_...q30

    return (uint16_t)interp;
}


static inline int32_t clamp32(int32_t val, int32_t min, int32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}


// ---------------------------------------------------
// Boucle systick 1kHz
// ---------------------------------------------------
void update_lead_angle(void)
{
    if (!tables_initialized)
        init_lead_tables();

    tick_5ms++;
    if (tick_5ms < 5)
        return; // update à 200 Hz
    tick_5ms = 0;

    // to debug filling the tab
    //debug_velocity_tab = velocity_tab[debug_tab_index];
    //debug_lead_base_q8_8 = lead_base_q8_8[debug_tab_index];
    
    // get PLL velocity
    uint32_t ui32_pll_velocity_q8_8X1024 = pll_get_velocity();

    // Filtrage Hall velocity (évite le jitter)
    i32_pll_velocity_filt_q8_8X1024 = filter_i32((int32_t) ui32_pll_velocity_q8_8X1024, i32_pll_velocity_filt_q8_8X1024 , 4);
    int32_t pll_velocity_used = i32_pll_velocity_filt_q8_8X1024;

    // ----------------------
    // Lead base interpolation based on pll velocity
    // ----------------------
    uint16_t ui16_lead_base_q8_8_val = interpolate_lead_base_from_hall_velocity((uint16_t)pll_velocity_used);
    // add more lead angle when current incease
    i16_adc_battery_current_for_lead_base = filter_i16((int16_t)ui8_adc_battery_current_filtered,i16_adc_battery_current_for_lead_base, 6);
    lead_base_q8_8_val = ui16_lead_base_q8_8_val + (i16_adc_battery_current_for_lead_base * LEAD_ANGLE_Q8_8_PER_ADC_STEP); 
    if (lead_base_q8_8_val > (25<<8))lead_base_q8_8_val = (25<<8); // do not exceed 25*360 /256 = 35°

    // lead correction based on Id (taking care of Iq and speed)
    int32_t Id_filt = 0;
    int32_t Iq_filt = 0;
    
    // Here we calculate Id and Iq filtered (based on process in ISR0 or ISR 1) that are used for optimisation of lead angle based on Id    
    if ( ui8_id_iq_counter == 0 ){    
        Id_filt = i32_id_sum >> 6; 
        Iq_filt = i32_iq_sum >> 6;
        // only for debug
        debug_id = Id_filt;
        debug_iq = Iq_filt;
        i32_id_sum = 0;
        i32_iq_sum = 0;
        ui8_id_iq_counter = ID_IQ_COUNTER; // 64 Reset counter
    }

    // ----------------------
    // Deadband adaptatif
    // ----------------------
    int32_t abs_Id = (Id_filt < 0) ? -Id_filt : Id_filt;
    int32_t abs_Iq = (Iq_filt < 0) ? -Iq_filt : Iq_filt;
    if (abs_Iq < 1) abs_Iq = 1; // protection

    int32_t Trel = (abs_Iq * K_REL_Q15) >> 15; // = 5% of Iq
    int32_t T = (Trel > IDABS_DEFAULT) ? Trel : IDABS_DEFAULT; // Use 5% of Iq when this is bigger than default (100) 

    // apply hysteresis
    int32_t T_low = (last_deadband == 0) ? T : ((last_deadband * HYST_FACTOR_Q15) >> 15);
    if (T > last_deadband)
        last_deadband = T;
    else if (abs_Id < T_low)
        last_deadband = T;

    int32_t Id_effective = (abs_Id < last_deadband) ? 0 : Id_filt; // use Id_filter (or 0 when within deadband and hysteresis)

    // Reset du correctif si vitesse trop basse
    if (pll_velocity_used < hall_low_speed_threshold) {
        lead_corr_q8_8 = 0;
    } else if (Id_effective != 0) {
        // Step adaptatif proportionnel à |Id/Iq|
        int32_t step_q15 = ( (abs(Id_effective) << 15) / abs_Iq ); // Q15 ratio
        int32_t step = (step_q15 * LEAD_STEP_MAX_Q8_8) >> 15;
        step = clamp32(step, LEAD_STEP_MIN_Q8_8, LEAD_STEP_MAX_Q8_8); // limit step per iteration between 0,02° and 0,35°
        if (Id_effective > 0)
            lead_corr_q8_8 += step;
        else
            lead_corr_q8_8 -= step;
        // clamp correction 
        lead_corr_q8_8 = clamp32(lead_corr_q8_8, -MAX_LEAD_CORR_Q8_8, MAX_LEAD_CORR_Q8_8); // max = -10° + 10°
    }
 
    // Calcul total
    ui16_lead_total_q8_8 = (uint16_t)lead_base_q8_8_val + (uint16_t)lead_corr_q8_8;

}



// security checks in systick
// Timer / persistance 
static uint16_t t_phase_rms_persist = 0;
static uint16_t t_motor_rms_persist = 0;
static uint16_t t_idc_slow_persist = 0;
// Timer anti ramp up (avoid ramp up when soft error occured for some ms)
static uint16_t t_ramp_up_delay = 0; // compteur en ticks de SysTick (1 tick = 1 ms par exemple)
static bool duty_limit_active = false;
static bool fault_phase_rms = false;
static bool fault_motor_rms = false;
static bool fault_Idc_slow = false;

void systick_security_checks(void){
    duty_limit_active = false; // reset the general flag
 
    // --- Phase RMS protection (sans sqrt) ---
    // find max of filtered values
    uint32_t ui32_Iphase_max2 = ui32_Iu_rms_2_filt;
    if(ui32_Iv_rms_2_filt > ui32_Iphase_max2) ui32_Iphase_max2 = ui32_Iv_rms_2_filt;
    if(ui32_Iw_rms_2_filt > ui32_Iphase_max2) ui32_Iphase_max2 = ui32_Iw_rms_2_filt;

    // Check on max of each phase rms
    if(ui32_Iphase_max2 > PHASE_RMS_WARN2) {
        t_phase_rms_persist++;
        if(t_phase_rms_persist > 20){
            t_phase_rms_persist--;
            fault_phase_rms = true;
            duty_limit_active = true;
            t_ramp_up_delay = RAMP_UP_DELAY_TICKS; // bloque le ramp-up
        } 
    } else t_phase_rms_persist = 0;

    // --- Motor RMS protection --- motor rms = sum of the 3 phase rms
    if(ui32_Imotor_rms_2_filt > IMOTOR_RMS_WARN2) {
        t_motor_rms_persist++;
        if(t_motor_rms_persist > 100) {
            t_motor_rms_persist--;
            fault_motor_rms = true;
            duty_limit_active = true;
            t_ramp_up_delay = RAMP_UP_DELAY_TICKS; // bloque le ramp-up
        }
    } else t_motor_rms_persist = 0;

    // Idc slow
    if( ((uint16_t) ui8_adc_battery_current_filtered) > IDC_SLOW_WARN) {
        t_idc_slow_persist++;
        if(t_idc_slow_persist > 100){
            t_idc_slow_persist--;
            fault_Idc_slow = true;
            duty_limit_active = true;
            t_ramp_up_delay = RAMP_UP_DELAY_TICKS; // bloque le ramp-up
        }
    } else t_idc_slow_persist = 0;

    // --- décrémente le timer anti-ramp-up si actif ---
    if(t_ramp_up_delay > 0) {
        t_ramp_up_delay--;
        if (t_ramp_up_delay == 0){ //reset the reasons
            fault_phase_rms = false;
            fault_motor_rms = false;
            fault_Idc_slow = false;
            duty_limit_active = false;         
        }
    }    
}


        /****************************************************************************/
// PWM duty_cycle controller:
// - limit battery undervolt
// - limit battery max current
// - limit motor max phase current
// - limit motor max ERPS
// - ramp up/down PWM duty_cycle and/or field weakening angle value

// check if to decrease, increase or maintain duty cycle
//note:
// ui8_adc_battery_current_filtered is calculated just here above
// ui16_adc_motor_phase_current_max = 135 per default for TSDZ2 (13A *100/16) *187/112 = battery_current convert to ADC10bits *and ratio between adc max for phase and for battery
//        is initiaded in void ebike_app_init(void) in ebyke_app.c


// every 25ms ebike_app_controller fills
//  - ui8_controller_adc_battery_current_target
//  - ui8_controller_duty_cycle_target // is usually filled with 255 (= 100%)
//  - ui8_controller_duty_cycle_ramp_up_inverse_step
//  - ui8_controller_duty_cycle_ramp_down_inverse_step
// Furthermore,  when ebyke_app_controller start pwm, g_duty_cycle is first set on 30 *256 (= 12%)

uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step_prev= 0;
uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step_prev= 0;
uint16_t ui16_controller_duty_cycle_ramp_up_step= 0;
uint16_t ui16_controller_duty_cycle_ramp_down_step= 0;
uint16_t ui16_fw_hall_counter_offset = 0;

uint32_t debug_duty_limit_active_cnt = 0;

void update_duty_cycle(void){
    // update ramp steps when they change
    if (ui8_controller_duty_cycle_ramp_up_inverse_step_prev != ui8_controller_duty_cycle_ramp_up_inverse_step) {
        if (ui8_controller_duty_cycle_ramp_up_inverse_step == 0) ui8_controller_duty_cycle_ramp_up_inverse_step = 10;  // éviter division par zéro
        ui16_controller_duty_cycle_ramp_up_step = ((((uint32_t)PWM_CYCLES_SECOND)/1000) << 8) / ui8_controller_duty_cycle_ramp_up_inverse_step;
        ui8_controller_duty_cycle_ramp_up_inverse_step_prev = ui8_controller_duty_cycle_ramp_up_inverse_step;
    }
    
    if (ui8_controller_duty_cycle_ramp_down_inverse_step_prev != ui8_controller_duty_cycle_ramp_down_inverse_step) {
        if (ui8_controller_duty_cycle_ramp_down_inverse_step == 0) ui8_controller_duty_cycle_ramp_down_inverse_step = 10;  // éviter division par zéro
        ui16_controller_duty_cycle_ramp_down_step = ((((uint32_t)PWM_CYCLES_SECOND)/1000) << 8) / ui8_controller_duty_cycle_ramp_down_inverse_step;
        ui8_controller_duty_cycle_ramp_down_inverse_step_prev = ui8_controller_duty_cycle_ramp_down_inverse_step;
    }

    // --- Application duty en fonction protection ---
    if(fault_phase_current_peak || fault_idc_fast) {
        ui16_g_duty_cycle = 0;
        return;
    }
    if(duty_limit_active) {
        ui16_g_duty_cycle -= ui16_g_duty_cycle >> 3 ; // reduce duty_cycle by 1/8
        debug_duty_limit_active_cnt++;
        return;
    }     
    
    if ((ui8_controller_duty_cycle_target < (ui16_g_duty_cycle >> 8))                     // requested duty cycle is lower than actual
            || (ui8_controller_adc_battery_current_target < ui8_adc_battery_current_filtered)  // requested current is lower than actual
            || (ui16_adc_motor_phase_current >  ui16_adc_motor_phase_current_max)               // motor phase is to high
    //      || (ui16_hall_counter_total < (HALL_COUNTER_FREQ / MOTOR_OVER_SPEED_ERPS))        // Erps is to high
            || (ui16_adc_voltage < ui16_adc_voltage_cut_off)                                  // voltage is to low
            || (ui8_brake_state)
            ) {                                                           // brake is ON
        //  first decrement field weakening angle if set or duty cycle if not
        if (ui16_fw_hall_counter_offset > 0) {
            if(ui16_fw_hall_counter_offset > ui16_controller_duty_cycle_ramp_down_step){
                ui16_fw_hall_counter_offset -= ui16_controller_duty_cycle_ramp_down_step;
            } else {
                ui16_fw_hall_counter_offset = 0;
            }        
        }   else {
            if (ui16_g_duty_cycle > ui16_controller_duty_cycle_ramp_down_step) {
                    ui16_g_duty_cycle  -= ui16_controller_duty_cycle_ramp_down_step;
            } else {
                ui16_g_duty_cycle = 0;
            }
        }
    } else if(t_ramp_up_delay == 0) { // ramp up but only if not delayed due to a security check
        if ((ui8_controller_duty_cycle_target > (ui16_g_duty_cycle >> 8))                     // requested duty cycle is higher than actual
                && (ui8_controller_adc_battery_current_target > ui8_adc_battery_current_filtered)) { //Requested current is higher than actual
            uint32_t temp_duty = ui16_g_duty_cycle + ui16_controller_duty_cycle_ramp_up_step;
            // increment duty cycle
            if (temp_duty < (PWM_DUTY_CYCLE_STARTUP << 8)) {
                temp_duty = PWM_DUTY_CYCLE_STARTUP << 8;
            }	
            else if (temp_duty > ((uint32_t)ui8_pwm_duty_cycle_max << 8)) {
                temp_duty = ((uint32_t)ui8_pwm_duty_cycle_max << 8);
            }
            ui16_g_duty_cycle = temp_duty;
        }
        else if ((ui8_field_weakening_enabled) && (ui16_g_duty_cycle == (ui8_pwm_duty_cycle_max << 8))) {
            // increment field weakening angle
            uint32_t temp_fw = ui16_fw_hall_counter_offset + ui16_controller_duty_cycle_ramp_up_step;        
            // clamp
            if (temp_fw > (ui8_fw_hall_counter_offset_max << 8)) {
                temp_fw = (ui8_fw_hall_counter_offset_max << 8);
            }
            ui16_fw_hall_counter_offset = temp_fw;
        }
    }    
}

