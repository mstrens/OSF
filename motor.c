/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
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

#define RPM_FOR_MOTOR_STOP        100UL    // par exemple 100 tr/min


// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters

// ---------------- DEFINE for PLL ---------------- 
//#define DEBUG_PLL

#define NB_SECTORS      6
#define MOTOR_POLE_PAIRS          4UL      // nombre de paires de pôles moteur

#define PWM_HZ          19000u
#define PLL_HZ          1000u
#define HALL_TIMEOUT_MS (50U)  // millis sec

/* timeout exprimé en nombre de ticks PWM (arrondi vers le haut) */
#define HALL_TIMEOUT_TICKS   ( (uint32_t)(((uint32_t)HALL_TIMEOUT_MS * (uint32_t)PWM_HZ + 999UL) / 1000UL) )


#define HALLS_PER_ELEC_REV 6
#define HALLS_PER_MECH_REV (HALLS_PER_ELEC_REV * MOTOR_POLE_PAIRS)

#define KP_Q11   (500)     // 160 >> 11 => environ 0,1 ; 1000= environ 50%;  1<<11 =  2048 ; 500 = environ 25%
#define KI_Q11   (50)      // 16 >> 11 =>  environ 0,01

#define DEG_TO_Q8_8(d) ((int32_t)((d) * 65536 / 360))
#define HALL_OFFSET_DEG (30)
#define HALL_OFFSET_Q8_8 ( (uint16_t)DEG_TO_Q8_8(HALL_OFFSET_DEG) )

#define SNAP_THRESHOLD_Q8_8   DEG_TO_Q8_8(40)
#define BLEND_THRESHOLD_DEG   (5)
#define SPEED_FRAC_BITS       (16)
#define MAX_ANGLE_BETWEEN_HALL_Q8_24 (uint32_t)((DEG_TO_Q8_8(70)) << SPEED_FRAC_BITS)
#define INTEGRATOR_MAX_Q8_19  (1500 << 11) // we shift by 11 to multiply by 2048 because KI is in Q11
#define INTEGRATOR_MIN_Q8_19  (-1500 << 11)
#define MAX_SPEED_Q8_24  (400000 * 256)

#define MIN_SPEED_Q8_16  (0)
#define PWM_TICK_US     (1000000u / PWM_HZ)

// Nouveau defines pour smoothing
#define SMOOTHING_LIMIT_DEG 20
#define SMOOTHING_LIMIT_Q8_8 DEG_TO_Q8_8(SMOOTHING_LIMIT_DEG)
#define SMOOTHING_BITS 3  // split part exceeding the limit ; 
                          // use a shift to avoid division; here shift = 3 because at 6000rpm, there is about 8 pwm cycleper transition

#define RPM_LOW_SPEED   (300)   // below this speed, at each hall front, rotor position = hall centered (+ no interpolation)
                                // just above, at each hall front, rotor position = hall (+ interpolation based only on previous sector speed
#define RPM_START_PLL_SPEED   (500)   // above this speed, at each front position, rotor position is partly corrected (bend) when hall is in advance more than a threshold
                                    // interpolation is done with speed pll correction 
#define RPM_MAX_SPEED   (6000)  // above high, rotor position is not corrected at each front

#define RPM_MIN_SPEED_TO_CALIBRATE_HALL (500) // calibrate hall sensor only when this speed has been reached

/* convert RPM -> dt_us threshold between successive Hall transitions (microseconds) */
#define RPM_TO_DT_US(rpm)   ( (uint32_t)(60000000UL / ((uint32_t)(rpm) * (uint32_t)HALLS_PER_MECH_REV)) )

#define DT_US_LOW_SPEED        RPM_TO_DT_US(RPM_LOW_SPEED)
#define DT_US_START_PLL_SPEED  RPM_TO_DT_US(RPM_START_PLL_SPEED)
#define DT_US_TO_CALIBRATE     RPM_TO_DT_US(RPM_MIN_SPEED_TO_CALIBRATE_HALL)
#define DT_US_MAX_SPEED        RPM_TO_DT_US(RPM_MAX_SPEED)
#define US_PER_PWM_Q0_7       ((uint32_t)((1000000UL * 128UL + (PWM_HZ/2)) / PWM_HZ)) // 128 = (1<<7) to avoid overflow

#define FACTOR_INV_DT_US_Q0_16       ((uint32_t)((1000000ULL * 65536ULL) / 19000ULL)) // 3449263 (fit in 32 bits)


// ++++++++++++ DEFINE for clarck and park (optimising lead angle based on Id) // some are used only if cordic is used
#define SQRT3                                       (1.732050807569F)       /* √3 */
#define DIV_SQRT3                                   (591)                  /* ((int16_t)((1/SQRT3) * (1<<SCALE_SQRT3))) */
#define DIV_SQRT3_Q14                               (9459U)
#define SCALE_DIV_3                                 (14U)                   /* For 1/3 scaling. */
#define DIV_3                                       (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */

#define DEGREE_90                                   (4194304U << 8U)        /* 90° angle (0 ~ 2^23 represent electrical angle 0° ~ 180° in CORDIC) */
#define DEGREE_X                                    (DEGREE_90 * 1U)        /* X = 0°, 90°, 180°, or 270° */
#define DEGREE_SHIFT                                (652448U << 8U)         /* 14° angle shift */

#define CORDIC_VECTORING_MODE                       (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_ROTATION_MODE                        (0x6A)                  /*  CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/
#define CORDIC_SHIFT                                (14U)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/

// pattern sequence for hall sensor is 1,3,2,6,4, 5
// with full use of posif possibilities, this table should be read with expected pattern and so upload in shadow register for the next expected
// when current pattern is 1 and expected = 3 , the sadow register should be prepare for the next transition with current = 3 and exp=6 
// in current version, it is used only to detect if a transition is valid in irq0
const uint8_t expected_pattern_table[8] = {
    3, // 0 should not happen
    3, // after 1 => 3 
    6, // after 2 => 6
    2, // after 3 => 2
    5, // after 4 => 5
    1, // after 5 => 1
    4, // after 6 => 4
    1 // 7 should not happen 
};
// === Mapping des états Hall vers secteurs === Sequence is 1, 3, 2, 6, 4, 5 
//                                                 for sect 0, 1, 2, 3, 4, 5
const int8_t hall_to_sector[8] = {
    0, 0, 2, 1, 4, 5, 3, 0 // when hall pattern is invalid (0 or 7) we use sector 0 to avoid further checs
};

// table has to be updated if PWM frequency change !!!!!!!!!!!!!!
// table generated with sin(x) + 1/6*sin(3*x) scaled to -800/+800 to avoid being to close of the limits (-840/+840 for 19 kHz)
// first value in the table is for x = 90° (to be similar to TSDZ2)
static const int16_t i16_LUT_SINUS[256] = {
    770,770,770,771,772,773,775,776,778,780,782,784,786,789,791,793,
    795,796,798,799,800,800,800,799,798,796,794,791,787,782,776,770,
    762,754,744,733,722,709,695,680,664,647,629,609,589,567,544,521,
    496,470,443,416,387,358,328,297,266,234,201,168,135,102,68,34,
    0,-34,-68,-102,-135,-168,-201,-234,-266,-297,-328,-358,-387,-416,-443,-470,
    -496,-521,-544,-567,-589,-609,-629,-647,-664,-680,-695,-709,-722,-733,-744,-754,
    -762,-770,-776,-782,-787,-791,-794,-796,-798,-799,-800,-800,-800,-799,-798,-796,
    -795,-793,-791,-789,-786,-784,-782,-780,-778,-776,-775,-773,-772,-771,-770,-770,
    -770,-770,-770,-771,-772,-773,-775,-776,-778,-780,-782,-784,-786,-789,-791,-793,
    -795,-796,-798,-799,-800,-800,-800,-799,-798,-796,-794,-791,-787,-782,-776,-770,
    -762,-754,-744,-733,-722,-709,-695,-680,-664,-647,-629,-609,-589,-567,-544,-521,
    -496,-470,-443,-416,-387,-358,-328,-297,-266,-234,-201,-168,-135,-102,-68,-34,
    0,34,68,102,135,168,201,234,266,297,328,358,387,416,443,470,
    496,521,544,567,589,609,629,647,664,680,695,709,722,733,744,754,
    762,770,776,782,787,791,794,796,798,799,800,800,800,799,798,796,
    795,793,791,789,786,784,782,780,778,776,775,773,772,771,770,770
};

/*
// table generated with sin(x) scaled to -800/+800 to avoid being to close of the limits (-840/+840 for 19 kHz)
// first value in the table is for x = 90° (to be similar to TSDZ2)
static const int16_t i16_LUT_SINUS[256] = {
    800,800,799,798,796,794,791,788,785,781,776,771,766,760,753,746,
    739,731,723,715,706,696,686,676,665,654,643,631,618,606,593,579,
    566,552,537,523,508,492,477,461,444,428,411,394,377,360,342,324,
    306,288,270,251,232,213,194,175,156,137,117,98,78,59,39,20,
    0,-20,-39,-59,-78,-98,-117,-137,-156,-175,-194,-213,-232,-251,-270,-288,
    -306,-324,-342,-360,-377,-394,-411,-428,-444,-461,-477,-492,-508,-523,-537,-552,
    -566,-579,-593,-606,-618,-631,-643,-654,-665,-676,-686,-696,-706,-715,-723,-731,
    -739,-746,-753,-760,-766,-771,-776,-781,-785,-788,-791,-794,-796,-798,-799,-800,
    -800,-800,-799,-798,-796,-794,-791,-788,-785,-781,-776,-771,-766,-760,-753,-746,
    -739,-731,-723,-715,-706,-696,-686,-676,-665,-654,-643,-631,-618,-606,-593,-579,
    -566,-552,-537,-523,-508,-492,-477,-461,-444,-428,-411,-394,-377,-360,-342,-324,
    -306,-288,-270,-251,-232,-213,-194,-175,-156,-137,-117,-98,-78,-59,-39,-20,
    0,20,39,59,78,98,117,137,156,175,194,213,232,251,270,288,
    306,324,342,360,377,394,411,428,444,461,477,492,508,523,537,552,
    566,579,593,606,618,631,643,654,665,676,686,696,706,715,723,731,
    739,746,753,760,766,771,776,781,785,788,791,794,796,798,799,800
};
*/

// this table says which phases are best read by ADC to have a larger window at mid point
// 1 = phase U and V ; 2 = phase U and W ; 3 = phase V and W
// this table is specific for lut sinus with sin(x) + 1/6 sin(3x)
static const uint8_t ui8_LUT_SECTOR_CASE[256] = {
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
};

uint16_t ui16_base_sector_q8_8[NB_SECTORS]   = { // values are ovewritten at the end of hall calibration
    24<<8,  // 1 -> 24 * 360 / 256 degré
    66<<8,  // 3 -> 66 * 360 / 256 degré
    107<<8, // 2 -> 107 * 360 / 256 degré
    152<<8, // 6 -> 152 * 360 / 256 degré    
    195<<8, // 4 -> 195 * 360 / 256 degré
    235<<8, // 5 -> 235 * 360 / 256 degré
};
uint16_t ui16_sector_angle_q8_8[NB_SECTORS] = {  // values are ovewritten at the end of hall calibration
    (66-24)<<8,  // 1 -> 24 * 360 / 256 degré
    (107-66)<<8, // 2 -> 107 * 360 / 256 degré
    (152-107)<<8,  // 3 -> 66 * 360 / 256 degré
    (195-152)<<8, // 4 -> 195 * 360 / 256 degré
    (235-195)<<8, // 5 -> 235 * 360 / 256 degré
    (24+256-235)<<8, // 6 -> 152 * 360 / 256 degré
};



// use in hall irq to capture pattern and timestamp
typedef union __attribute__((aligned(4))) {
    struct {
        uint16_t ticks;     // timestamp (16 bits)
        uint8_t  pattern;   // hall pattern (3 bits utiles)
        uint8_t  flags;     // réservé (ex: sens, erreur, etc.)
    };
    uint32_t raw;           // accès 32 bits atomique
} hall_sample_t;
volatile hall_sample_t hall_irq_sample;   // mis à jour dans ISR HALL
volatile bool hall_event_pending = false; // flag lu dans ISR PWM

// Hall positions in Q8.8
// Position rotorique in Q8.8 et vitesse en Q16.16
typedef int32_t q16_16_t; // (signed) (16 bits for decimal, 16bits for unit, 1 unit = 360/256 = 1.4°)
/* Q8.8 typedefs */
typedef int16_t q8_8_t;     // valeur signée Q8.8 (−128..+127.996) si on veut signed
typedef uint16_t uq8_8_t;   // valeur non signée Q8.8 (0..255.996) pour index / LUT

// for hall position (pattern , sector, angle 
uint8_t ui8_curr_hall_pattern = 0;
uq8_8_t ui16_curr_base_angle_q8_8 = 0;  // position of hall at the begin of the current sector
uint16_t ui16_prev_base_angle_q8_8 = 0;          // position of hall at the begin of previous sector (valid only after a  second transition and if seq is OK)
uint8_t ui8_curr_sector = 0; //
uint8_t ui8_prev_sector = 0; 
volatile uint8_t ui8_hall_sensors_state = 0; // name used by ebike_app.c to identify ui8_curr_hall_pattern; added here for compatibility
uint16_t hall_pattern_error_counter = 0; // to debug only
bool motor_just_starting = true;    // is also set on true each time motor is enabled
bool valid_prev_hall_ticks = false; // is also set on false each time motor is enabled


    

uint8_t hall_reference_angle = 0 ; // !! Is not in Q8_8 but only in uint8 ;This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 
// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks = usec ; we need  about 60 usec : 
//     55usec = delay between measuring at begin of ISR0 and applying PWM change at end of PWM cycle;
//     There is also some delay in hall sensor but it is probably included in hall calibration process
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
//uint8_t ui8_hall_counter_offset = 60;  !! currently not used in PLL interpol !!!!!!!!!!!!!

// to calculate Id and optimise lead angle
uint16_t ui16_angle_for_id_prev_q8_8; // position; saved at begin of ISR 0 to match with current iu,Iv,iw measured at begin of ISR 1
volatile uint16_t ADC_Bias_Iu = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iv = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iw = 1 << 11; // ADC is 12 bits, 0 = mid point 

// used in systick to optimise lead angle based on average Id, Iq 
int32_t i32_id_sum = 0;     // accumulate Id to use an average in systick based on 64 values
int32_t i32_iq_sum = 0;     // idem for Iq
uint8_t ui8_id_iq_counter = ID_IQ_COUNTER; // 64 ; used to filter id & iq ; pwm at 19kHz and systick at 200Hz => 19000/200 = 95 measurements

// to debug or used with cordic
int16_t I_u; // to check current in each phase
int16_t I_v;
int16_t I_w;
int16_t I_t;



// for current calculation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc
volatile uint8_t ui8_adc_battery_current_filtered = 0; // current in adc10 bits units (= moving average on 64 PWM cycles)

//uint32_t ui32_adc_battery_current_15b_moving_average = 0;
int battery_current_moving_avg_index = 0;
int battery_current_moving_avg_sum = 0;
int battery_current_moving_avg_buffer[64] = {0};

// for security checks ; shared with systick.c
volatile uint32_t ui32_Iu_rms_2_filt = 0;
volatile uint32_t ui32_Iv_rms_2_filt = 0;
volatile uint32_t ui32_Iw_rms_2_filt = 0;
volatile uint32_t ui32_Imotor_rms_2_filt = 0;

// For security checks : Flags fault shared with ebike.c
volatile bool fault_phase_current_peak = false;
volatile bool fault_idc_fast = false;

// check which permutation are valid
uint8_t debug_permutation = 0; // this field was used to test (with ucProbe) different permutation of I1,I2,I3 with IU, Iv,IW
                            // using this requires to uncomment some lines in ISR1

// to debug time spent in irq0 and irq1
volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;

uint16_t irq0_min = 0xFFFF;
uint16_t irq0_max = 0;
uint16_t irq1_min = 0xFFFF;
uint16_t irq1_max = 0;

volatile int32_t debug_id = 0;
volatile int32_t debug_iq = 0;
volatile int32_t debug_I1 = 0;
volatile int32_t debug_I2 = 0;
volatile int32_t debug_I3 = 0;
int32_t volatile debug_Iu;
int32_t volatile debug_Iv;
int32_t volatile debug_Iw;
int32_t volatile debug_Iuvw;
int32_t volatile debug_Ialpha;
int32_t volatile debug_Ibeta;
int32_t volatile debug_angle;
int32_t volatile debug_va ; // to debug
int32_t volatile debug_vb ; // to debug
int32_t volatile debug_vc ;  // to debug
uint8_t cordic_offset =128;
int32_t debug_id_accum = 0;
int32_t debug_iq_accum = 0;
uint8_t debug_id_filter = 6;
int32_t volatile debug_foc = 0; 
int32_t debug_cordic_angle = 0;
int32_t debug_cordic_offset = 0;   
int32_t debug_raw_id = 0;
int32_t debug_raw_iq = 0;
int32_t debug_i32_Iu1 = 0;
int32_t debug_i32_Iv1 = 0;
int32_t debug_i32_Iw1 = 0;
int32_t debug_i_avg = 0;


// to debug isr 
volatile uint32_t debug_isr1_timer_start = 0;
volatile uint32_t debug_isr1_timer_end = 0;
volatile uint32_t debug_isr0_timer_start = 0;
volatile uint32_t debug_isr0_timer_end = 0;

//to check that adc conversion has been done when reading
volatile uint32_t ui32_adc_conversion_gr0 = 0;  
volatile uint32_t ui32_adc_conversion_gr1 = 0;


//+++++++++++++++  for cadence (and torque when spider logic is used)
// new wheel and cadence variables : moved to systick.c
// =============== VARIABLES PARTAGÉES =============== 
//volatile uint32_t ui32_pwm_ticks = 0;          // compteur soft 19kHz
//volatile uint32_t ui32_cadence_last_ticks[6] = {0};   // timestamps pédalage (codes 0..5)
//volatile uint32_t ui32_wheel_last_pwm_ticks = 0; // dernier front roue (ui32_pwm_ticks)

/****************************************************************************/
/*
    * - New pedal start/stop detection Algorithm (by MSpider65) -
    *
    * Pedal start/stop detection uses both transitions of both PAS sensors
    * ui8_temp stores the PAS1 and PAS2 state: bit0=PAS1,  bit1=PAS2
    * Pedal forward ui8_temp sequence is: 0x01 -> 0x00 -> 0x02 -> 0x03 -> 0x01
    * After a stop, the first forward transition is taken as reference transition
    * Following forward transition sets the cadence to 7RPM for immediate startup
    * Then, starting from the second reference transition, the cadence is calculated based on counter value
    * All transitions are a reference for the stop detection counter (4 time faster stop detection):
    */


//  -------- TABLE DE TRANSITION QUADRATURE 16→CODE -------------
//   index = (prev<<2) | curr
//   prev,curr ∈ [0..3] → 16 combinaisons possibles
//   mapping :
//     reverse : 00->01 (1), 01->11 (7), 11->10 (14), 10->00 (8)  ; table filled with 4
//     forward : 00->10 (2), 10->11 (11), 11->01 (13), 01->00 (4) ; table filled with 0...3
//     no-change / invalid : autres cas                           ; table filled with 5
const uint8_t ui8_cadence_transpose[16] = {
    /*0*/ 5,  /*1*/ 4,  /*2*/ 0,  /*3*/ 5,
    /*4*/ 3,  /*5*/ 5,  /*6*/ 5,  /*7*/ 4,
    /*8*/ 4,  /*9*/ 5,  /*10*/5,  /*11*/1,
    /*12*/5,  /*13*/2,  /*14*/4,  /*15*/5
};


        
// this function has to be called in ISR0 or ISR1 (at 19kHz) to collect the data that are processed in a systick irq at 1kHz
static inline __attribute__((always_inline))  void collect_wheel_cadence_data(){
    static uint8_t ui8_prev_cadence_state = 0;   // 2 bits combinés prev A/B
    static uint8_t ui8_prev_wheel_state = 0;
        
    ui32_pwm_ticks++; // incrément soft timer 32 bits
    // --- wheel sensor ---
    uint8_t ui8_wheel_state = (uint8_t) XMC_GPIO_GetInput(IN_SPEED_PORT, IN_SPEED_PIN);
    if (!ui8_prev_wheel_state && ui8_wheel_state) {
        ui32_wheel_last_pwm_ticks = ui32_pwm_ticks; // rising edge
    }
    ui8_prev_wheel_state = ui8_wheel_state;

    // --- cadence sensor  (2 bits) ---
    uint8_t ui8_cadence_state = (uint8_t) (XMC_GPIO_GetInput(IN_PAS1_PORT, IN_PAS1_PIN ) | 
                                    ( XMC_GPIO_GetInput(IN_PAS2_PORT, IN_PAS2_PIN ) <<1 ));
    if ( ui8_cadence_state != ui8_prev_cadence_state) {
        uint8_t ui8_cadence_idx = ((ui8_prev_cadence_state << 2) | ui8_cadence_state) & 0x0F;
        uint8_t ui8_cadence_code = ui8_cadence_transpose[ui8_cadence_idx];
        ui32_cadence_last_ticks[ui8_cadence_code] = ui32_pwm_ticks; // enregistre l’instant du code
        ui8_prev_cadence_state = ui8_cadence_state;
    }    
}

// used to calculate hall angles based of linear regression of all ticks intervals
// are filled in irq0 and transmitted in ebike_app.c using segger_rtt_print 
#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
#endif



inline __attribute__((always_inline)) uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}
/*
inline __attribute__((always_inline)) uint32_t filtering_function(uint32_t ui32_temp_15b , uint32_t ui32_filtered_15b , uint32_t alpha){
    uint32_t ui32_temp_new = ui32_temp_15b * (16U - alpha);
    uint32_t ui32_temp_old =  ui32_filtered_15b * alpha;
    uint32_t ui32_filtered_value = ((ui32_temp_new + ui32_temp_old + (8)) >> 4);                    
    if (ui32_filtered_value == ui32_filtered_15b) {
        if (ui32_filtered_value < ui32_temp_15b)
            ui32_filtered_value++;
        else if (ui32_filtered_value > ui32_temp_15b)
            ui32_filtered_value--;
    }
    return ui32_filtered_value ;                  
}
*/


void VADC0_G0_0_IRQHandler() {  // VADC is configured to compare the total current (12bits) with "1000" and generate an irq
    ui8_system_state = ERROR_BATTERY_OVERCURRENT; // set the error to avoid that motor starts again
    // disable the motor
    ui8_motor_enabled = 0;
    motor_disable_pwm();
}

// Lecture atomique et rapide des Halls
__STATIC_INLINE uint8_t read_hall_pattern(void)
{
    // Supposons que HALL0, HALL1, HALL2 sont sur le même port
    uint32_t portval = IN_HALL0_PORT->IN;  // lecture unique du port
    // Extraction et positionnement exact des bits
    uint8_t pattern = 0;
    pattern |= (uint8_t)((portval >> IN_HALL0_PIN) & 1);      // Hall0 -> bit 0
    pattern |= (uint8_t)((portval >> IN_HALL1_PIN) & 1) << 1; // Hall1 -> bit 1
    pattern |= (uint8_t)((portval >> IN_HALL2_PIN) & 1) << 2; // Hall2 -> bit 2
    return pattern;
}

// this irq callback occurs when posif detects a new pattern 
__RAM_FUNC void POSIF0_0_IRQHandler(){
    hall_sample_t s;

    s.ticks   =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW); // Capture time stamp 
    s.pattern = read_hall_pattern() ;// capture hall pattern
    // Écriture atomique unique sur 32 bits
    hall_irq_sample.raw = s.raw;
    hall_event_pending = true; // juste un flag
}

// +++++++++++++    to calibrate ++++++++++++++++
uint8_t hall_calib_state = HALL_TO_CALIBRATE;
uint32_t hall_cal_sum[6];
uint16_t hall_cal_count[6];
uint16_t hall_cal_total_count;

// to debug
uint16_t base_angle_0;
uint16_t base_angle_1;
uint16_t base_angle_2;
uint16_t base_angle_3;
uint16_t base_angle_4;
uint16_t base_angle_5;
uint16_t sector_angle_0;
uint16_t sector_angle_1;
uint16_t sector_angle_2;
uint16_t sector_angle_3;
uint16_t sector_angle_4;
uint16_t sector_angle_5;

// function to call in main loop or every 25msec
void hall_calibrate(){
    if (hall_calib_state == HALL_MEASURED){
        uint32_t avg_duration[6];
        uint32_t total_duration = 0;
        uint16_t sector_angle[6] ; // angle in q8.8 (65356=360°) in this sector (begin with this index)
        uint16_t base_angle[6];    // angle at the begin of the sector

        // --- Moyenne par secteur ---
        for (int i = 0; i < 6; i++) {
            if (hall_cal_count[i] == 0) avg_duration[i] = 1; // éviter div0
            else avg_duration[i] = hall_cal_sum[i] / hall_cal_count[i];
            total_duration += avg_duration[i];
        }
    
        if (total_duration == 0) {
            hall_calib_state = HALL_CALIBRATION_ERROR; // sécurité
            return;
        }    
        // --- Calcul des angles Q16 (0..65536 = 360°) ---
        int32_t offset =  24<<8;
        int16_t accum_angle = offset; // 24 to match current tabel 
        for (int i = 0; i < 6; i++) {
            // sector_angle[i] proportionnel à avg_duration[i] / total_duration
            sector_angle[i] = (uint16_t)((avg_duration[i] * 65536u) / total_duration);
            base_angle[i] = (uint16_t)accum_angle;
            accum_angle += sector_angle[i];
        }
        // --- Corriger le dernier secteur pour compenser arrondi ---
        if (accum_angle != (65536u + offset))  {
            int32_t diff = 65536u + offset - accum_angle;
            sector_angle[5] = (uint16_t)((int32_t) sector_angle[5] + diff);
        }
        // fast copy in table used by iSR
        for (int i = 0; i < 6; i++) {
            ui16_sector_angle_q8_8[i] = sector_angle[i];
            ui16_base_sector_q8_8[i] = base_angle[i];
        }
        hall_calib_state = HALL_CALIBRATED;
        
        // to debug
        base_angle_0 = (base_angle[0] + 128) >> 8;
        base_angle_1 = (base_angle[1] + 128) >> 8;
        base_angle_2 = (base_angle[2] + 128) >> 8;
        base_angle_3 = (base_angle[3] + 128) >> 8;
        base_angle_4 = (base_angle[4] + 128) >> 8;
        base_angle_5 = (base_angle[5] + 128) >> 8;
        sector_angle_0 = sector_angle[0];
        sector_angle_1 = sector_angle[1];
        sector_angle_2 = sector_angle[2];
        sector_angle_3 = sector_angle[3];
        sector_angle_4 = sector_angle[4];
        sector_angle_5 = sector_angle[5];
    }
}
// ++++++++++++++ end for calibrate ++++++++++++++


// =========  filtrage sans reliquat du au calcul en entier =========
//int32_t diff = omega_mech - i32_omega_est X256;
//int32_t delta = diff >> OMEGA_ALPHA_SHIFT;
//if (delta == 0 && diff != 0)  delta = (diff > 0) ? 1 : -1;
//i32_omega_est X256 += delta;


void capture_3_phase_current_offset(){  // called by main
    // when motor is blocked since some time, we update first the ADC bias for Iu, iv, iW
    // when motor is not running (based on ui8_motor_enabled) we reset foc and foc PID
    // when motor is running we use a PI based on ID (calculated and filtered in ISR) to update FOC angle
    // in a second step we can calculate a value for foc angle based on rpm and current and apply pid as a correction.
    #define ADC_BIAS_SHIFT (9)
    #define SHIFT_BIAS_CURRENT_LPF 7

    static uint32_t ui32_ADC_Bias_Iu = 1<<(11+ADC_BIAS_SHIFT); // variable are shifted to increase accuracy
    static uint32_t ui32_ADC_Bias_Iv = 1<<(11+ADC_BIAS_SHIFT);
    static uint32_t ui32_ADC_Bias_Iw = 1<<(11+ADC_BIAS_SHIFT);
    // first when motor is not running, update adc bias
    if (ui8_motor_enabled == 0) {
            //ADC sequences - Iw -> Iv -> Iu  = default sequence set in ISR0
            //    VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //    VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL); 
            // tests on Id Iq shows that Iu=I1, Iv= I2, Iw=I3 for this setting of alias !!!!!
        uint32_t I1 = VADC_I1_GROUP->RESD[VADC_I1_RESULT_REG]&0x0FFF; // IW is first measured current based on set up in ISR0
        uint32_t I2 = VADC_I2_GROUP->RESD[VADC_I2_RESULT_REG]&0x0FFF; // IV is second one
        uint32_t I3 = VADC_I3_GROUP->RESD[VADC_I3_RESULT_REG]&0x0FFF; // IU is third one 
        debug_I1 = I1; debug_I2 = I2; debug_I3 = I3;   
        // Read Iu ADC bias and apply filter
        //uint32_t Iu = ((uint32_t)(XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF)) << 10 ; // << 10 to increase accuracy
        uint32_t Iu = I1 << ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iu =  (uint32_t) ((ui32_ADC_Bias_Iu * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iu) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iu = (uint16_t) (ui32_ADC_Bias_Iu >> ADC_BIAS_SHIFT) ;
        /* Read Iv ADC bias */
        //uint32_t Iv = ((uint32_t)XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF) << 10;
        uint32_t Iv = I2 <<ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iv = (uint32_t) ((ui32_ADC_Bias_Iv * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iv) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iv = (uint16_t) (ui32_ADC_Bias_Iv >> ADC_BIAS_SHIFT) ;
        /* Read Iw ADC bias */
        //uint32_t Iw = ((uint32_t) XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF) << 10;
        uint32_t Iw = I3 << ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iw = (uint32_t) ((ui32_ADC_Bias_Iw * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iw) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iw = (uint16_t) (ui32_ADC_Bias_Iw >> ADC_BIAS_SHIFT) ;
    }
}

// ++++++++++++   for park transfom when sinus table is used +++++++++++++++++
#define SIN_TABLE_SIZE      256
#define ANGLE_TO_INDEX_SHIFT 8  // 16 bits Q8.8 → 8 bits d'index (256)

__attribute__((aligned(4), section(".rodata"))) const int16_t sin_table[SIN_TABLE_SIZE] = {
    0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,
12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,
23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,
30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,
32767,32758,32729,32679,32610,32522,32413,32286,32138,31972,31786,31581,31357,31114,30853,30572,
30274,29957,29622,29269,28899,28511,28106,27684,27246,26791,26320,25833,25330,24812,24279,23732,
23170,22595,22006,21403,20788,20160,19520,18868,18205,17531,16846,16151,15447,14733,14010,13279,
12540,11793,11039,10279,9512,8740,7962,7180,6393,5602,4808,4011,3212,2411,1608,804,
0,-804,-1608,-2411,-3212,-4011,-4808,-5602,-6393,-7180,-7962,-8740,-9512,-10279,-11039,-11793,
-12540,-13279,-14010,-14733,-15447,-16151,-16846,-17531,-18205,-18868,-19520,-20160,-20788,-21403,-22006,-22595,
-23170,-23732,-24279,-24812,-25330,-25833,-26320,-26791,-27246,-27684,-28106,-28511,-28899,-29269,-29622,-29957,
-30274,-30572,-30853,-31114,-31357,-31581,-31786,-31972,-32138,-32286,-32413,-32522,-32610,-32679,-32729,-32758,
-32768,-32758,-32729,-32679,-32610,-32522,-32413,-32286,-32138,-31972,-31786,-31581,-31357,-31114,-30853,-30572,
-30274,-29957,-29622,-29269,-28899,-28511,-28106,-27684,-27246,-26791,-26320,-25833,-25330,-24812,-24279,-23732,
-23170,-22595,-22006,-21403,-20788,-20160,-19520,-18868,-18205,-17531,-16846,-16151,-15447,-14733,-14010,-13279,
-12540,-11793,-11039,-10279,-9512,-8740,-7962,-7180,-6393,-5602,-4808,-4011,-3212,-2411,-1608,-804
};

// Multiplication Q15
__RAM_FUNC static __attribute__((always_inline)) inline int16_t mult_q15(int16_t a, int16_t b)
{
    int32_t temp = (int32_t)a * (int32_t)b;
    temp += 0x4000;  // arrondi
    return (int16_t)(temp >> 15);
}

__RAM_FUNC static __attribute__((always_inline)) inline void park_transform_q15(int16_t Ialpha, int16_t Ibeta, uint16_t angle_q8_8,
                        int16_t *Id, int16_t *Iq)
{
    // Index dans la table
    uint16_t index = (angle_q8_8 >> ANGLE_TO_INDEX_SHIFT) & (SIN_TABLE_SIZE - 1);
    uint16_t cos_index = (index + (SIN_TABLE_SIZE / 4)) & (SIN_TABLE_SIZE - 1);

    int16_t sin_t = sin_table[index];
    int16_t cos_t = sin_table[cos_index];

    // Park transform
    int16_t Id_tmp = mult_q15(Ialpha, cos_t) + mult_q15(Ibeta, sin_t);
    int16_t Iq_tmp = mult_q15(Ibeta, cos_t) - mult_q15(Ialpha, sin_t);

    *Id = Id_tmp;
    *Iq = Iq_tmp;
}

volatile uint32_t debug_us_2_fronts = 0;
volatile uint32_t debug_angle_2_fronts = 0;
// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8   
//   it takes between 5 and 18 usec (so less than 26 usec)
__RAM_FUNC void CCU80_0_IRQHandler(void)
{
    // to debug


    static uint16_t ui16_prev_hall_ticks = 0;
    static uint8_t ui8_prev_hall_pattern = 0;
    
    // to set flux position
    uint16_t ui16_angle_no_ref_no_lead_q8_8;
    uint16_t ui16_angle_no_lead_q8_8;  // angle based on Hall or hybrid with ref angle but no lead angle
    //uint16_t ui16_hall_angle_no_ref_no_lead_q8_8;
    
    uint8_t ui8_curr_hall_pattern_local = ui8_curr_hall_pattern;   // local copy just for faster processing
    
    // read irq data before reading current time stamp (to be sure that time now follow the ISR timestamp)
    hall_sample_t hall_isr_sample_local;
    bool hall_event_pending_local = hall_event_pending;
    hall_isr_sample_local.raw = hall_irq_sample.raw;// Lecture atomique (32 bits)
        
    // get now timestamp
    uint16_t ui16_curr_ISR0_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    
    debug_isr0_timer_start = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);

    //uint16_t curr_ticks = (uint16_t)(XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW));
    //uint8_t curr_pattern = ui8_curr_hall_pattern;
    //hall_sample_t hs_local = hall_irq_sample;     // lecture atomique
    //bool hall_pending = hall_event_pending;

    if (motor_just_starting) {  // is true at first run and at each motor_enable
        motor_just_starting = false;
        valid_prev_hall_ticks = false;
        ui16_g_duty_cycle = 0;
        ui8_fw_hall_counter_offset = 0; // field weakening        
        ui8_curr_hall_pattern_local = read_hall_pattern();
        ui8_curr_sector = hall_to_sector[ui8_curr_hall_pattern_local];               // get current sector
        ui16_curr_base_angle_q8_8 =  ui16_base_sector_q8_8[ui8_curr_sector] ;  // get current base angle
        ui8_prev_sector = (ui8_curr_sector>0) ? ui8_curr_sector-1 : 5;        // get previous sector
        pll_init(); // on initialise all PLL structure sur base de ui16_curr_base_angle_q8_8
    }

    if (hall_event_pending_local) { //set on true in hall ISR when a new hall pattern occured
        hall_event_pending = false; // reset flag localement
        // split hall ISR data (pattern & timestamp)
        uint16_t ui16_curr_hall_ticks = hall_isr_sample_local.ticks; // Extraction from ISR
        ui8_curr_hall_pattern_local = hall_isr_sample_local.pattern;  

        ui8_curr_sector = hall_to_sector[ui8_curr_hall_pattern_local];               // get current sector
        ui16_curr_base_angle_q8_8 =  ui16_base_sector_q8_8[ui8_curr_sector] ;  // get current base angle
        // get previous sector
        ui8_prev_sector = (ui8_curr_sector>0) ? ui8_curr_sector-1 : 5;
        //ui16_prev_base_angle_q8_8 = ui16_base_sector_q8_8[ui8_prev_sector];  // get base of previous sector
        
        // Vérification de séquence
        bool seq_ok = (ui8_curr_hall_pattern_local == expected_pattern_table[ui8_prev_hall_pattern]);

        // Calculate time and angle interval only when previous timestamp is valid and sequence is valid
        uint16_t ui16_us_between_2_hall_fronts = 0;
        uint16_t ui16_angle_between_2_hall_fronts = 0;
        if (valid_prev_hall_ticks && seq_ok) {
            ui16_us_between_2_hall_fronts = ui16_curr_hall_ticks - ui16_prev_hall_ticks;
            if (ui16_us_between_2_hall_fronts == 0) ui16_us_between_2_hall_fronts = 1; // avoid division by 0
            ui16_angle_between_2_hall_fronts = ui16_curr_base_angle_q8_8 - ui16_prev_base_angle_q8_8;
        }
        // call even when sequence is wrong or when no valid_ui16_prev_hall_ticks
        pll_on_hall_event(ui16_us_between_2_hall_fronts, (uint16_t) ui16_curr_base_angle_q8_8, ui16_angle_between_2_hall_fronts, seq_ok);       // gère phase, speed, timeout
        ui16_prev_hall_ticks = ui16_curr_hall_ticks;
        ui8_prev_hall_pattern = ui8_curr_hall_pattern_local;  // used to check the sequence
        ui16_prev_base_angle_q8_8 = ui16_curr_base_angle_q8_8;
        valid_prev_hall_ticks = true; // flag that says that we have a valid ticks
        
        // for debug
        debug_us_2_fronts = ui16_us_between_2_hall_fronts;
        debug_angle_2_fronts = (((uint32_t)ui16_angle_between_2_hall_fronts) * 360 ) >> 16;
        // check if hall calibration is required
        if (!seq_ok) {
            if (hall_calib_state == HALL_CALIBRATING) hall_calib_state = HALL_TO_CALIBRATE;
        } else if ((hall_calib_state == HALL_TO_CALIBRATE) && (ui16_us_between_2_hall_fronts < DT_US_TO_CALIBRATE)) { // vitesse > 500 rpm
            hall_calib_state = HALL_CALIBRATING;
            hall_cal_total_count = 6*200;
            for(uint8_t i=0;i<6;i++) { hall_cal_sum[i]=0; hall_cal_count[i]=0; }
        } else if ((hall_calib_state == HALL_CALIBRATING) && valid_prev_hall_ticks) {
            hall_cal_sum[ui8_prev_sector] += ui16_us_between_2_hall_fronts;
            hall_cal_count[ui8_prev_sector]++;
            hall_cal_total_count--;
            if (hall_cal_total_count == 0) hall_calib_state = HALL_MEASURED;
    } 

    } // end of new hall front registerd in hall ISR

    ui8_curr_hall_pattern = ui8_curr_hall_pattern_local;
    
    // ---- incrémentation phase par step même si erreur séquence ----
    pll_on_pwm_tick();

    // ---- publication angle FOC/SVM ----
    ui16_angle_no_ref_no_lead_q8_8 = pll_get_angle_q8_8();
    ui16_angle_no_lead_q8_8 = ui16_angle_no_ref_no_lead_q8_8 + ((uint16_t)hall_reference_angle<<8);
    // add lead angle
//    uint16_t ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + (uint16_t)(ui8_g_foc_angle<<8);
    // here ui16_g_foc_angle_q8_8 is just based on hall velocity and a multiplicator (see systick).
    //ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + (ui16_g_foc_angle_q8_8);
    // here we use the lead angle based on a table on velocity and a correction to set Id around 0 
    uint16_t ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + ui16_lead_total_q8_8;
    uint8_t ui8_lut_index = (uint8_t)(ui16_SVM_table_index_q8_8 >> 8);

    /*
    if (ui8_motor_enabled) {
        ui8_measured_phases = ui8_LUT_SECTOR_CASE[ui8_lut_index];
    } else {
        // take care that this must be the same sequence as used to calibrate the ADC offset (done when motor is not enabled)
        ui8_measured_phases = 3; // use default config for bias when motor is off
    }
    */
    
    /*
    // in case 1, use phase u and v, 2 = phase u and w ,  3 = phase v and w
    switch (ui8_measured_phases){
        case 1:
            //ADC sequences - Iu -> Iv -> Iw 
            VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
        break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G0_CHANNEL);
        break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL); 
        break;
    }
    */
    
    // fill the PWM parameters; 
    uint16_t temp_duty_cycle = (ui16_g_duty_cycle + 0x80) >> 8; // rounding
    uint8_t ui8_lut_index_A = (ui8_lut_index + 171) & 0xFF; // -120° = 256*2/3 ≈ 171
    int16_t svm_A = i16_LUT_SINUS[ui8_lut_index_A];
    PHASE_U_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_A * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256;
    
    uint8_t ui8_lut_index_B = ui8_lut_index ;
    int16_t svm_B = i16_LUT_SINUS[ui8_lut_index_B];    
    PHASE_V_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_B * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    
    uint8_t ui8_lut_index_C = (ui8_lut_index + 85) & 0xFF; // + 120°
    int16_t svm_C = i16_LUT_SINUS[ui8_lut_index_C];
    PHASE_W_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_C * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256  

    // updload of the shadow registers of PWM timers is done in ISR1 after the mid point
    

    #define DEBUG_IRQO_TIME (1) // 1 = calculate the time spent in irq0
    #if (DEBUG_IRQO_TIME == (1))
    debug_isr0_timer_end = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);
    if (hall_calib_state == HALL_CALIBRATED) {  // we measure only when hall are calibrated to get more realistic values
        uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ;
            temp = temp - ui16_curr_ISR0_ticks;
        if (irq0_min > temp) irq0_min = temp; // store the in enlapsed time in the irq
        if (irq0_max < temp) irq0_max = temp; // store the max enlapsed time in the irq
    }
    #endif
          
    
    #if (uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
    //I_u = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
    //I_w = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
    //I_v = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    
    // ProbeScope_Sampling must be called to update data displayed on PC in graph
    // if we do not require a high refresh rate, this could be set in another loop 
    ProbeScope_Sampling(); // this is here in a interrupt that run fast
    #endif
    
} // end of CCU80_0_IRQHandler

// ************* irq handler ******************************
#define DEBUG_IRQ1_TIME (1) // 1 = calculate time spent in irq1

__RAM_FUNC void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 1300 counting DOWN (= about 5 usec after mid point = adc conversion)    
// this function takes between 12 and 16 usec; so it ends before end of pwm cycle  (5+16 < 26)   
    #if (DEBUG_IRQ1_TIME == (1))
    // to debug max time in this iSR
    uint16_t start_ticks  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    #endif

//    debug_isr1_timer_start = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW); // to know when ISR start running (at about timer 1200 when compare is set on 1300)
//    debug_isr1_timer_start++;

// collect wheel and cadence ticks to further process in 1 msec irq +++++++++++
    // this could be in isr0 or ISR1 (probably best in ISR0)
    collect_wheel_cadence_data();

// for debugging ; check when ADC conversion is done
//    ui32_adc_conversion_gr1 = ((VADC_G1->VFR) & 0xFF) ;
//    ui32_adc_conversion_gr0 = ((VADC_G0->VFR) & 0xFF) ;

// Enable shadow transfer for slice 0,1,2 for CCU80 Kernel
    ccu8_0_HW->GCSS = ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 );
// get phases currents (when ADC conversion is done)
        // it measures actual currents but angle must be one one that was apply for PWM and so it is the angle from isr 0 before update.
    // read the 3 ADC and substact the ADC bias and avg
    // calculate i_alpha and i_beta (clark transform)
    // fill cordic to get IQ ID (park transform)

    // Read current ADC (ADC synchronous conversion) 
    uint16_t I1 = VADC_I1_GROUP->RES[VADC_I1_RESULT_REG]&0X0FFF; // first conversion = G1 ch 0
    uint16_t I2 = VADC_I2_GROUP->RES[VADC_I2_RESULT_REG]&0X0FFF; // second conversion = G0 ch 0
    uint16_t I3 = VADC_I3_GROUP->RES[VADC_I3_RESULT_REG]&0X0FFF; // third conversion = G1 ch 1
    
    // take care that result registers from ADC could contains different phase currents depending on rotor position (alias being used)
    // furthermore in some cases, only 2 currents from the 3 should be used (not in this version)
    // in this version, we just use a fix setting for alias
    // this is the alias being used and this code is normally not required because it is already in ADC init and here we use only 1 sequence for this debug
    
    //VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
    //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
    
    int32_t i32_raw_Iu; int32_t i32_raw_Iv; int32_t i32_raw_Iw;
    // Here we use permutation 0 (see below) so Iu = I1, Iv=I2 , Iw = I3(with cordic offset = 128)
    i32_raw_Iu = (int32_t)I1; i32_raw_Iv = (int32_t)I2; i32_raw_Iw = (int32_t)I3; // here ADC in 12 bits
    
    // Whe have also to take care that there are 6 permutations of I1, I2, I3 with Iu, Iv, IW
    // It seems that 3 could be used but each of them requires a different cordic offset to get valid Id, Iq (at park transform)
    /*
    // This code allows to test all 6 permutations changing debug_permutation and cordic_offset within ucProbe
    // take care that a function updates bias when motor is disabled and must use a fix setting for alias!!
    switch (debug_permutation) {
        case 0:
            i32_raw_Iu = (int32_t)I1; i32_raw_Iv = (int32_t)I2; i32_raw_Iw = (int32_t)I3; break; //cordic offset 128
        case 1:
            i32_raw_Iu = I1; i32_raw_Iv = I3; i32_raw_Iw = I2; break;
        case 2:
            i32_raw_Iu = I2; i32_raw_Iv = I1; i32_raw_Iw = I3; break;
        case 3:
            i32_raw_Iu = I2; i32_raw_Iv = I3; i32_raw_Iw = I1; break; // cordic offset 42
        case 4:
            i32_raw_Iu = I3; i32_raw_Iv = I1; i32_raw_Iw = I2; break; // cordic offset 213
        default:
            i32_raw_Iu = I3; i32_raw_Iv = I2; i32_raw_Iw = I1; break;
    }
    */
       
    /* 
    // here some code that was prepared to use different settings for alias in order to sample the 2 most signicant currents depending on rotor position
    // take care that this code has not been updated to use the good permutation between I12,3 and Iu,v,w.
    // the code from infineon did not use a good permutation and so can't be used as model
    switch (ui8_measured_phases){ // in case 1, use phase u and v, 2 = phase u and w ,  3 = phase v and w
        case 1:
            //ADC sequences - Iu -> Iv -> Iw 
            //VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
            i16_raw_Iu = I1;
            i16_raw_Iv = I2;
            i16_raw_Iw = I3;
        break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            //VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G0_CHANNEL);
            i16_raw_Iu = I2;
            i16_raw_Iv = I3;
            i16_raw_Iw = I1;
        break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            //VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
            i16_raw_Iu = I3;
            i16_raw_Iv = I2;
            i16_raw_Iw = I1;
        break;
    }
    */
    
    int32_t i32_Iu = (i32_raw_Iu -(int32_t)ADC_Bias_Iu ) << 3; // change from 12 bits to 15 bits to use Q15 in cordic
    int32_t i32_Iv = (i32_raw_Iv - (int32_t)ADC_Bias_Iv ) << 3;
    int32_t i32_Iw = (i32_raw_Iw - (int32_t)ADC_Bias_Iw ) << 3;

    int32_t i_avg = (((i32_Iu + i32_Iv + i32_Iw) * (int32_t) DIV_3)) >>  SCALE_DIV_3 ; 
    i32_Iu -= i_avg;
    i32_Iv -= i_avg;
    i32_Iw -= i_avg;
    // here we have the 3 phase currents without offset in 15 bits

    debug_i32_Iu1 = i32_Iu;
    debug_i32_Iv1 = i32_Iv;
    debug_i32_Iw1 = i32_Iw;
    //debug_i_avg = i_avg;

// measure IDC
        //the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
        // in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
        // In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
        //      SO 1 adc step 12bits = 1/24,576 = 0,040A
        // For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
        // For 10 A, TSDZ8 shoud give 10*24,576 steps = 246 steps
        // to convert TSDZ8 steps 12bits  in the same units as TSDZ2, we shoud take ADC12bits *62/245,76 = 0,25 and divide by 4 (or >>2)
        // current is available in gr0 result 15 in queue 0 p2.8 and/or in gr1 result 152 (p2.8)
        // both results use IIR filters and so results are in 14 bits instead of 12 bits
        // use measurement from the 2 groups
    // changed when using infineon init for vadc (result in 12bits and in ch 1)
    uint32_t ui32_temp_adc_battery_current_15b = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_I4_RESULT_REG ) & 0xFFFF) <<3; // change from 12 to 15 digits 
    ui32_adc_battery_current_15b = ui32_temp_adc_battery_current_15b;

    uint32_t ui32_adc_battery_current_15b_moving_average = update_moving_average(ui32_temp_adc_battery_current_15b);
    if (ui32_adc_battery_current_15b_moving_average > (255 << 5)) { // clamp for safety ; << 5 because in TSDZ2 current is in ADC10 bits and max is 255
        ui32_adc_battery_current_15b_moving_average = 255 << 5;
    }  
    ui8_adc_battery_current_filtered = ui32_adc_battery_current_15b_moving_average  >> 5;

// perform security checks
    //security checks: could be moved after calculating Id, IQ if cordic is used
    // RMS IIR phase (32 bits suffisent)

    // carrés des phases
    uint32_t i32_Iu2 = (i32_Iu >> 3) * (i32_Iu >> 3); // reduce to 12 * 12 bits to avoid overflow in i32 in systicks
    uint32_t i32_Iv2 = (i32_Iv >> 3) * (i32_Iv >> 3);
    uint32_t i32_Iw2 = (i32_Iw >> 3) * (i32_Iw >> 3);

    // Phase current Peak protection
    if(i32_Iu2 > PHASE_PEAK_TRIP2 || i32_Iv2 > PHASE_PEAK_TRIP2 || i32_Iw2 > PHASE_PEAK_TRIP2) {
        fault_phase_current_peak = true;
        motor_disable_pwm();
        ui8_motor_enabled = 0;
    }

    // RMS IIR phase (assembleur-like)
    int32_t diff;
    diff = (int32_t)i32_Iu2 - (int32_t)ui32_Iu_rms_2_filt;
    ui32_Iu_rms_2_filt += diff >> PHASE_RMS_ALPHA;
    diff = (int32_t)i32_Iv2 - (int32_t)ui32_Iv_rms_2_filt;
    ui32_Iv_rms_2_filt += diff >> PHASE_RMS_ALPHA;
    diff = (int32_t)i32_Iw2 - (int32_t)ui32_Iw_rms_2_filt;
    ui32_Iw_rms_2_filt += diff >> PHASE_RMS_ALPHA;

    // RMS moteur (somme carrés)
    ui32_Imotor_rms_2_filt = ui32_Iu_rms_2_filt + ui32_Iv_rms_2_filt + ui32_Iw_rms_2_filt; 

    // Idc fast
    if(ui32_adc_battery_current_15b > IDC_FAST_TRIP) {
        fault_idc_fast = true;
        motor_disable_pwm();
        ui8_motor_enabled = 0;
            }

// calculate clack transform 
    int32_t I_Alpha_1Q31;
    int32_t I_Beta_1Q31;
    // when we use the 3 phase currents; I32_Ix is in 15 bits
    I_Alpha_1Q31 = (((i32_Iu << 1) - (i32_Iv + i32_Iw)) * (int32_t) DIV_3) >> SCALE_DIV_3 ; // !! here in 15 bits even if less accurate
    I_Beta_1Q31 = ((i32_Iv - i32_Iw) * (int32_t) DIV_SQRT3_Q14) >> SCALE_DIV_3;
    //debug_Ialpha = I_Alpha_1Q31  ; // to get same units as Iu,Iv,Iw
    //debug_Ibeta = I_Beta_1Q31 ;

    /* // if we want to use only 2 phases depending on rotor position; this was with cordic and so with 14 more bits!!!!!
    switch (ui8_measured_phases){
        case 1:
            //ADC sequences - Iu -> Iv -> Iw
            I_Alpha_1Q31 = i32_Iu << CORDIC_SHIFT;
            I_Beta_1Q31 = (i32_Iu + (i32_Iv << 1)) * (DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14));
            break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            I_Alpha_1Q31 =  i32_Iu << CORDIC_SHIFT;
            I_Beta_1Q31 =  (i32_Iu + (i32_Iw << 1)) * (-(DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14)));
            break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            I_Alpha_1Q31 = (-(i32_Iv + i32_Iw)) << CORDIC_SHIFT;
            I_Beta_1Q31 = (i32_Iv - i32_Iw) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
            break;
    }
    */                    
// apply park transform
    // 1) using table sinus
    // here we keep phases currents in 15 bits (it could be different for Cordic)
    // take care to use cordic_offset according to the used permutation (here 128 to be *256 for Q8_8) 
    uint16_t ui16_angle = ui16_angle_for_id_prev_q8_8 + (((uint16_t)cordic_offset) <<8);
    int16_t i16_id;
    int16_t i16_iq;
    // get Id, Iq with table
    park_transform_q15((int16_t) I_Alpha_1Q31, (int16_t) I_Beta_1Q31, ui16_angle, &i16_id, &i16_iq);
    if (ui8_id_iq_counter){ //used to filter id & iq ; pwm at 19kHz and systick at 200Hz => 19000/200 = 95 measurements; here we take 64 measurements
        i32_id_sum += i16_id;
        i32_iq_sum += i16_iq;
        ui8_id_iq_counter--;
    } 

    //debug_I1 = I1; debug_I2 = I2; debug_I3 = I3; // 12 bits  
    //debug_Iu = i32_Iu; debug_Iv = i32_Iv; debug_Iw = i32_Iw;// 15 bits
    //debug_Iuvw =  debug_Iu + debug_Iv +debug_Iw;  // so in 15 bits
    //debug_va = ui16_a; debug_vb = ui16_b; debug_vc = ui16_c; // to debug
    //debug_cordic_offset = (int32_t)(((uint16_t)cordic_offset) <<8);
    //debug_Ialpha = (I_Alpha_1Q31 );  debug_Ibeta = (I_Beta_1Q31 ) ;
    //debug_id += ((i16_id - debug_id) + ((i16_id - debug_id > 0) ? 1 : (i16_id - debug_id < 0 ? -1 : 0))) >> 4;
    //debug_iq += ((i16_iq - debug_iq) + ((i16_iq - debug_iq > 0) ? 1 : (i16_iq - debug_iq < 0 ? -1 : 0))) >> 4;
    debug_angle = (int32_t) ui16_angle_for_id_prev_q8_8 ;
    debug_raw_id = i16_id; debug_raw_iq = i16_iq; // in 15 bits

    /*
    // With cordic ; !!!!!!!!!! code has to be checked ; there are some schanges required
    // calculate I alpha and I beta
    // I_Alpha = (2 * I_U - (I_V + I_W))/3   // ou Ialpha = (2/3) * (Ia - 0.5*Ib - 0.5*Ic)
    //HandlePtr->I_Alpha_1Q31 = ((CurrentPhaseU << 1) - (CurrentPhaseV + CurrentPhaseW)) * (DIV_3 << (CORDIC_SHIFT-14));
    // DIV3 = 5461 ; 1/3 = 5461 / (1<<14).; we avoid dividing by 1<<14 in order to get the value from 12 bit to 12+14 = 28 bit
    //    I_Alpha_1Q31 = ((i32_Iu << 1) - (i32_Iv + i32_Iw)) * DIV_3 ;

    //  I_Beta = (I_V - I_W)/√3 in 1Q31
    //HandlePtr->I_Beta_1Q31 = (CurrentPhaseV - CurrentPhaseW) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
    // here also we avoid >>14 in order to get the adc 12 bits +14 bits for better accuracy
    //    I_Beta_1Q31 = (i32_Iv - i32_Iw) * DIV_SQRT3_Q14 ;
    // prepare parktransform with cordic
    // General control of CORDIC Control Register 
    MATH->CON = CORDIC_ROTATION_MODE;
    uint16_t ui16_angle = ui16_angle_for_id_prev_q8_8 + (((uint16_t)cordic_offset) <<8);
    MATH->CORDZ = (( int32_t) ui16_angle) << 16; // we convert angle in 0/65536 to Q31 
    // Y = I_Alpha 
    MATH->CORDY = I_Alpha_1Q31;
    // X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) 
    MATH->CORDX = I_Beta_1Q31;
    // Wait if CORDIC is still running calculation 
    while (MATH->STATC & 0x01)
    {
        continue;
    }
    // Read CORDIC results Iq and Id - 32-bit. CORDIC Result Register [7:0] are 0x00 
    int32_t i32_iq = MATH->CORRX;
    i32_iq >>= CORDIC_SHIFT; // shift 14
    i32_iq = (i32_iq * 311) >> 8;   // x MPS/K.;
    //Idem for Id
    int32_t i32_id = MATH->CORRY;
    i32_id >>= CORDIC_SHIFT;
    i32_id = (i32_id * 311) >> 8;   // x MPS/K.;    
    // here id and iq are equivalent to 15 bits
    debug_id = i32_id;
    debug_iq = i32_iq;
    */
		  
    // to debug
    //uint16_t temp1e  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    //temp1e = temp1e - start_ticks;
    //if (temp1e > debug_time_ccu8_irq1e) debug_time_ccu8_irq1e = temp1e; // store the max enlapsed time in the irq


    #if (DEBUG_IRQ1_TIME == (1))
    uint16_t temp1  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    temp1 = temp1 - start_ticks;
    if (irq1_min > temp1) irq1_min = temp1; // store the min enlapsed time in the irq
    if (irq1_max < temp1) irq1_max = temp1; // store the min enlapsed time in the irq
    #endif
    debug_isr1_timer_end = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);
}  // end of CCU8_1_IRQ


/*  !!!!!!!!!! perhaps better to do it like infineon
// still to disable, perhaps better to set all pins used by PWM to LOW level instead of tristate (same level as when PWM is disabled)
// to check what happens in debug mode when cpu is halted (what happens with PWM pins???)

void pmsm_foc_disable_inverter(void){
    // added by MStrens to discard INVERTER_EN_PIN for TSDZ8
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL); // Disable gate driver. 
    #endif
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    Motor.Inverter_status = 0;
  
  }
  void pmsm_foc_enable_inverter(void){
    // added by MStrens to discard INVERTER_EN_PIN for TSDZ8
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, ENABLE_LEVEL); // Enable gate driver.
    #endif
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);
    Motor.Inverter_status = 1;
  
  }
*/  

void motor_enable_pwm(void) { //set posif with current position & restart the timers
    motor_just_starting = true;   
    // one solution to activate is to generate an event that starts all timers in a synchronized way
    // Enable Global Start Control CCU80  in a synchronized way
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    uint32_t retry_start_counter = 10;
    while ((!XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW)) && (retry_start_counter > 0)){ // to be sure it is running
        XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
        XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    }
    // Note if we want to use one slice U, V or W to trigger VADC, we should activate the GPIO; see note for disable
}


void motor_disable_pwm(void) {
    // we stop and clear the 3 timers that control motor PWM
    XMC_CCU8_SLICE_StopClearTimer(PHASE_U_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_V_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_W_TIMER_HW);
    // slice CCU8_3 is not stopped becauses it is required to manage some tasks (speed, torque,...) 
    // Note: if we want to use slice 1 to manage a VADC trigger based on the channel 2 compare value, we should not stop the timer.
    //       we should then set all PWM gpio on Thri-state; this is still perhaps less secure
    // currently, when PWM timers are stopped, levels are set to passive LOW  
}

void get_curr_hall_pattern(){  // use to initialise at power on and in motor_enable()
    ui8_curr_hall_pattern = read_hall_pattern();
}


// ----------- here code used to manage pll ----------------
// ---------------- PLL states ----------------
typedef enum { PLL_STATE_FREE=0, PLL_STATE_SMOOTHING, PLL_STATE_FREEZE } pll_state_e;


typedef struct {
    uint32_t ui32_phase_acc_q8_24;
    uint32_t ui32_phase_acc_max_q8_24;
    uint32_t ui32_pll_step_q8_24; // step to add at each PWM (include pll correction or not depending on the case); there is a test to avoid negative value
    uint32_t ui32_hall_step_q8_24; // step to add at each PWM based only on the speed on previous sector
    int32_t  i32_integrator_q8_19;
    uint32_t pwm_counter;          // compteur global PWM only for pll purpose (timeout of hall)
    uint32_t ui32_last_hall_pwm_count;  // compteur capturé au dernier front hall

    uint16_t ui16_last_hall_phase_q8_8;
    
    // smoothing variables
    pll_state_e pll_state;
    uint32_t ui32_smoothing_step_q8_24;
    uint32_t ui32_smoothing_remaining_q8_24;
    uint32_t ui32_catchup_remaining_q8_24;

    uint8_t  hall_transition_count; // to detect we got at lest 2 transitions to calculate the speed
} pll_state_t;

// used variable (only in the 3 pll functions)
pll_state_t pll;
/* ---------------- Volatile / ISR shared ---------------- */
uint16_t g_pll_phase_q8_8 = 0; // is best position



// ++++++++++ for debug ++++++++++++++++++++
uint32_t debug_phase_hall_acc_q8_24;
volatile int32_t debug_pll_angle = 0; // value of PLL (even when speed is to low for pll)
volatile int32_t debug_hall_angle= 0; // value on hall (interpolate only when there is enough transitions)
volatile uint32_t debug_dt_us_is_0_cnt = 0;
volatile uint32_t debug_pll_step = 0;
volatile uint32_t debug_hall_step = 0;  
volatile uint8_t debug_pll_case = 0;
volatile uint8_t debug_pll_phase_acc = 0;
volatile int32_t debug_pll_phase_error = 0;
volatile int32_t debug_p_term = 0;
volatile int32_t debug_i_term = 0;
volatile int32_t debug_angle_correction = 0;

volatile uint8_t debug_pll_phase_acc_max = 0;                
volatile uint32_t debug_pll_timeout_cnt = 0;
volatile uint32_t debug_case_smoothing = 0;
volatile uint32_t debug_case_freezing = 0;


/* ---------------- Helpers ---------------- */
//static inline int32_t i32_abs(int32_t v) { return (v < 0) ? -v : v; }

__RAM_FUNC static __attribute__((always_inline)) inline int32_t phase_diff_q8_8(uint16_t target, uint16_t current)
{
    int32_t d = (int32_t)target - (int32_t)current;
    if (d >  32767) d -= 65536;
    if (d < -32768) d += 65536;
    return d;
}

/* ---------------- Init ---------------- */
void pll_init(void)
{
    // se baser sur le secteur courant et la base angle déjà préparés
    uint16_t init_phase_q8_8 = ui16_curr_base_angle_q8_8;
    pll.ui32_phase_acc_q8_24 = ((uint32_t)init_phase_q8_8) << SPEED_FRAC_BITS;
    pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24 ;
    pll.ui32_pll_step_q8_24 = 0;
    pll.ui32_hall_step_q8_24 = 0;
    pll.i32_integrator_q8_19 = 0;
    pll.ui16_last_hall_phase_q8_8 = init_phase_q8_8;
    pll.hall_transition_count = 0;
    pll.pwm_counter = 0;
    pll.ui32_last_hall_pwm_count = 0;
    
    pll.pll_state = PLL_STATE_FREE;
    pll.ui32_smoothing_remaining_q8_24 = 0;
    pll.ui32_smoothing_step_q8_24 = 0;
    pll.ui32_catchup_remaining_q8_24 = 0;
}	


/* ---------------- PWM tick integration (always called every PWM) ---------------- */
// called at each PWM cycle
// check for timeout (more than 50 msec since previous front -  check done in PWM ticks)
// When case = smoothing, apply smoothing step as long as needed, apply also pll step; when done switch to FREE
// When case =  freeze, do not apply pll step up to having catched all the catchup. ; when done, swtich to FREE
// When case = free, apply pll step (can be 0 or measured step depending on the way it has been filled at hall front)
// So, in most case position is interpolated but it can also be frozzen.
// clamp to a max value (usualy nex hall position + some margin e.g. 70° because hall interval is normally 60°)
// avoid moving position backward (pll step must be positive and in case on big error we freeze)
// Store position in Q8.8 in g_pll_phase_q8_8 used for updating PWM timers
__RAM_FUNC __attribute__((always_inline)) inline void pll_on_pwm_tick(void)
{
    pll.pwm_counter++;

    // Timeout handling
    if ((pll.pwm_counter - pll.ui32_last_hall_pwm_count) > HALL_TIMEOUT_TICKS) { // when time out occured
        uint16_t hall_center_q8_8 = pll.ui16_last_hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        pll.ui32_phase_acc_q8_24 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24;
        pll.ui32_pll_step_q8_24 = 0;
        pll.i32_integrator_q8_19 = 0; // Q19 because ki is in Q11 (so it is q8 /q11)
        pll.hall_transition_count = 0;  // reset count
        // IMPORTANT: remettre la référence pour éviter rebouclage immédiat
        pll.ui32_last_hall_pwm_count = pll.pwm_counter;
        pll.pll_state = PLL_STATE_FREE;
        pll.ui32_smoothing_remaining_q8_24 = 0;
        pll.ui32_smoothing_step_q8_24 = 0;
        pll.ui32_catchup_remaining_q8_24 = 0;
        #ifdef DEBUG_PLL
        debug_phase_hall_acc_q8_24 = pll.ui32_phase_acc_q8_24 ;
        debug_pll_timeout_cnt++;
        #endif
        return;
    }
    
    
    uint32_t step_q8_24 = pll.ui32_pll_step_q8_24 ; 

    switch(pll.pll_state) {
        case PLL_STATE_FREE:
            
            break;
        case PLL_STATE_SMOOTHING:
            step_q8_24 += pll.ui32_smoothing_step_q8_24; // add smoothing to pll step
            if (pll.ui32_smoothing_remaining_q8_24 > pll.ui32_smoothing_step_q8_24) {
                pll.ui32_smoothing_remaining_q8_24 -= pll.ui32_smoothing_step_q8_24;
        } else {
                pll.pll_state = PLL_STATE_FREE;
            }
            break;
        case  PLL_STATE_FREEZE : 
            step_q8_24 = 0;              // avoid any increase
            // continue when catchup is not done
            if (pll.ui32_catchup_remaining_q8_24 > pll.ui32_pll_step_q8_24) {
                pll.ui32_catchup_remaining_q8_24 -= pll.ui32_pll_step_q8_24;
					} else {
                pll.pll_state = PLL_STATE_FREE;
            }
        break;    
    }

    // Protection contre overflow avant clamp
    uint32_t space_q8_24 = pll.ui32_phase_acc_max_q8_24 - pll.ui32_phase_acc_q8_24;

    uint32_t new_angle;
    if (step_q8_24 >= space_q8_24) {
        // Empêcher un overflow : saturer directement
        new_angle = pll.ui32_phase_acc_max_q8_24;
    } else {
        // Pas de risque d'overflow : addition normale
        new_angle = pll.ui32_phase_acc_q8_24 + step_q8_24;
		}

    // Clamp final (garde la logique originale : cast signed avec wrap-safe)
    if ((int32_t)(new_angle - pll.ui32_phase_acc_max_q8_24) > 0) {
        pll.ui32_phase_acc_q8_24 = pll.ui32_phase_acc_max_q8_24;
            } else {
        pll.ui32_phase_acc_q8_24 = new_angle;
    }
    
    g_pll_phase_q8_8 = pll.ui32_phase_acc_q8_24 >> SPEED_FRAC_BITS;

    #ifdef DEBUG_PLL
   // apply step on hall for debug (comparison haal/pll)
    debug_phase_hall_acc_q8_24 += pll.ui32_hall_step_q8_24; // automatic wrap
    debug_hall_angle = (uint16_t)(debug_phase_hall_acc_q8_24 >> SPEED_FRAC_BITS);

    debug_pll_angle = (uint16_t)(pll.ui32_phase_acc_q8_24 >> SPEED_FRAC_BITS);
    #endif
}

        
// ---------------- Hall event from ISR PWM (dt_us already computed) ---------------- 
// called when a hall front occurs, update phase_acc, speed to be used by ISR0; (keep also integrator for pll)
// apply different rules:
// when sequence is wrong, discard (keep phase and speed)
// When low number of transitions (at start) , position = hall centerd and speed = 0, case = FREE
// when speed is low, use hall position centered + speed = 0, case = FREE
// when speed is lower than start pwm, use hall position + speed from previous sector, case = FREE
// above this speed use PLL logic
//       if phase error exceed a limit (hall in advance on estimated and so we must increase furthermore the phase):
//             split the error in 2 parts : part exceeding the limit is handle by smooting, the limit part is handled by PLL)
//       if phase error is lower than limit, new pll step is calculated on the whole error but
//             exceeding part will be managed by freezing (no change of phase as long as the exceeding part has not been consumed by pll_step)
//       When error is within the limits, PLL is used based on the whole error

__RAM_FUNC __attribute__((always_inline)) inline void pll_on_hall_event(uint16_t dt_us, uint16_t hall_phase_q8_8, uint16_t ui16_angle_between_2_hall_fronts_q8_8,bool seq_ok)
{
    // calculate measured speed based on previous sector using math.div
    if (dt_us < DT_US_MAX_SPEED) { 
        dt_us = DT_US_MAX_SPEED; 
        #ifdef DEBUG_PLL
        debug_dt_us_is_0_cnt++;
        #endif
    } // avoid overflow in some multiplication
    // start division; result will be in Q24
    /* Unsigned division is selected */
    #define MATH_DIVCON_USIGN_Pos                 (2UL)                     /*!< MATH DIVCON: USIGN (Bit 2)                                  */
    #define XMC_MATH_UNSIGNED_DIVISION                    ((uint32_t) 1 << MATH_DIVCON_USIGN_Pos)
    MATH->DIVCON = XMC_MATH_UNSIGNED_DIVISION;
    MATH->DVD    = (uint32_t)FACTOR_INV_DT_US_Q0_16;
    MATH->DVS    = (uint32_t)dt_us;
    
    //pll.prev_hall_phase_q8_8 = pll.ui16_last_hall_phase_q8_8;
    pll.ui16_last_hall_phase_q8_8 = hall_phase_q8_8;

    // --- update pwm counter to avoid timeout
    pll.ui32_last_hall_pwm_count = pll.pwm_counter;

    if (!seq_ok) return; // keep phase & speed, next PWM increments by speed

    // when speed is unknow, phase is aligned on hall + 30° and speed is set on 0 (no interpolation)
    if (pll.hall_transition_count < 10) {
        uint16_t hall_center_q8_8 = hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        pll.ui32_phase_acc_q8_24 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24;
        pll.ui32_pll_step_q8_24 = 0;
        pll.i32_integrator_q8_19 = 0;
        pll.hall_transition_count++;
        pll.ui32_hall_step_q8_24 = 0;
 
        pll.pll_state = PLL_STATE_FREE;
        #ifdef DEBUG_PLL
        debug_phase_hall_acc_q8_24 = pll.ui32_phase_acc_q8_24; // to debug
        debug_pll_phase_acc_max = pll.ui32_phase_acc_max_q8_24;
        debug_pll_case = 0;
        #endif
        return;
    }
 
    // here we perform some calculation waiting for the result of the math.div
    // at low speed, we do not have to wait for division result
    if (dt_us > DT_US_LOW_SPEED) { // very low speed  = use only hall position (centered)
        uint16_t hall_center_q8_8 = hall_phase_q8_8 + HALL_OFFSET_Q8_8; 
        pll.ui32_phase_acc_q8_24 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24;
        pll.ui32_pll_step_q8_24 = 0;
        pll.i32_integrator_q8_19 = 0;
        pll.ui32_hall_step_q8_24 = 0; // discard iterpolation
        pll.pll_state = PLL_STATE_FREE;
        #ifdef DEBUG_PLL
        debug_pll_case = 1;
        debug_phase_hall_acc_q8_24 = pll.ui32_phase_acc_q8_24; // to debug
        debug_pll_phase_acc_max = pll.ui32_phase_acc_max_q8_24;
        #endif
        return;
        }

    // calculate phase error before having the result of division
    int32_t phase_est_q8_8 = (int32_t)(pll.ui32_phase_acc_q8_24 >> SPEED_FRAC_BITS);
    int32_t e_q8_8 = phase_diff_q8_8(hall_phase_q8_8, (uint16_t)phase_est_q8_8);
    #ifdef DEBUG_PLL
    debug_pll_phase_error = e_q8_8;
    #endif
    
    int32_t max_error_q8_8 = SMOOTHING_LIMIT_Q8_8;

    // Determine smoothing / freeze / free
    if (e_q8_8 > max_error_q8_8) {
        // hall ahead → apply smoothing
        pll.ui32_smoothing_remaining_q8_24 = (e_q8_8 - max_error_q8_8) << SPEED_FRAC_BITS;
        e_q8_8 = max_error_q8_8; // limit the part being used for pll because remaining is taken by smoothing
        pll.ui32_smoothing_step_q8_24 = pll.ui32_smoothing_remaining_q8_24 >> SMOOTHING_BITS; // split on 8 pwm cycle
        pll.pll_state = PLL_STATE_SMOOTHING;
        #ifdef DEBUG_PLL
        debug_pll_case = 3;
        debug_case_smoothing++;
        #endif
    } else if (e_q8_8 < -max_error_q8_8) {
        // hall behind → freeze PLL

        pll.ui32_catchup_remaining_q8_24 = (-e_q8_8 - max_error_q8_8) << SPEED_FRAC_BITS;
        pll.pll_state = PLL_STATE_FREEZE;
        #ifdef DEBUG_PLL
        debug_pll_case = 4;
        debug_case_freezing++;
        #endif
    } else {
        pll.pll_state = PLL_STATE_FREE;
        #ifdef DEBUG_PLL
        debug_pll_case = 5;
        #endif
    }

    //  next line (commented) gives normal code if we should not take care to increase precision of cumulative I term
    //int32_t p_term_q8_8 = (e_q8_8 * (int32_t) KP_Q11) >> 11; // Si KP = 160 => 160/2048 = environ 0,1:
    int32_t p_term_q8_19 = (e_q8_8 * (int32_t) KP_Q11) ; // here we keep the value *2048 for better accuracy in the sum

    //int32_t i_delta_q8_8 = (e_q8_8 * (int32_t) KI_Q11) >> 11;  // si Ki = 16 = > 16/2048 = environ 0,01
    int32_t i_delta_q8_19 = (e_q8_8 * (int32_t) KI_Q11) ;  // here we keep the value *2048 for better accuracy in the sum

    pll.i32_integrator_q8_19 += i_delta_q8_19;  // here we still have extended precision
    // clamp integrator 
    if (pll.i32_integrator_q8_19 > INTEGRATOR_MAX_Q8_19) pll.i32_integrator_q8_19 = INTEGRATOR_MAX_Q8_19;
    else if (pll.i32_integrator_q8_19 < INTEGRATOR_MIN_Q8_19) pll.i32_integrator_q8_19 = INTEGRATOR_MIN_Q8_19;

    // calculate angle correction in the same units as angle between 2 fronts (so q8.8)
    int32_t i32_angle_correction_q8_8 = (p_term_q8_19 + pll.i32_integrator_q8_19) >> 11; // angle correction is now in q8.8 = same units as angle interval


    // get result of division
    uint32_t inverse_dt_us_q16 = ((uint32_t) MATH->QUOT);
    uint32_t ui32_hall_step_q8_24 = ((uint32_t) ui16_angle_between_2_hall_fronts_q8_8) * inverse_dt_us_q16 ; // q24 to keep precision during cumul at low speed
    
    // clamp step ; !!! this is not 100% required
    if (ui32_hall_step_q8_24 > MAX_SPEED_Q8_24) ui32_hall_step_q8_24 = MAX_SPEED_Q8_24; // step when speed is more than 6000 rpm
    
    pll.ui32_hall_step_q8_24 = ui32_hall_step_q8_24; // used to provide RPM, ERPS and Velocity; could be avoided if using pll_step
    #ifdef DEBUG_PLL
    debug_hall_step = ui32_hall_step_q8_24 >> SPEED_FRAC_BITS;
    #endif

    if (dt_us > DT_US_START_PLL_SPEED) { // speed is less than required for PLL: use hall position with interpolation based on last sector speed
        pll.ui32_phase_acc_q8_24 = ((uint32_t)hall_phase_q8_8) << SPEED_FRAC_BITS;
        pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24;
        pll.ui32_pll_step_q8_24 = ui32_hall_step_q8_24; 
        pll.i32_integrator_q8_19 = 0;
        pll.pll_state = PLL_STATE_FREE;
        #ifdef DEBUG_PLL
        debug_pll_case = 2;
        debug_phase_hall_acc_q8_24 = pll.ui32_phase_acc_q8_24;
        debug_pll_phase_acc_max = pll.ui32_phase_acc_max_q8_24;
        #endif
        return;
    } 

    // from here we can apply PLL (but perhaps with smooting or freezing)
    // total angle = ui16_angle_between_2_hall_fronts_q8_8 + angle_correction_q8_8; only if > 0
    int32_t i32_total_angle_q8_8 = (int32_t)ui16_angle_between_2_hall_fronts_q8_8 + i32_angle_correction_q8_8;
    if (i32_total_angle_q8_8 < 0) i32_total_angle_q8_8 = 0; // avoid backward rotating
    // calculate total step (including pll correction)
    uint32_t ui32_pll_step_q8_24 = ((uint32_t) i32_total_angle_q8_8)  * inverse_dt_us_q16 ; // in Q24
    // clamp to max 
    if (ui32_pll_step_q8_24 > MAX_SPEED_Q8_24) {
        ui32_pll_step_q8_24 = MAX_SPEED_Q8_24; // step when speed is more than 6000 rpm
    }
    pll.ui32_pll_step_q8_24 = ui32_pll_step_q8_24;
    // we do not updateppl.ui32_phase_acc_q8_24 because we use PLL
    pll.ui32_phase_acc_max_q8_24 = pll.ui32_phase_acc_q8_24 + MAX_ANGLE_BETWEEN_HALL_Q8_24;
    
    #ifdef DEBUG_PLL
    // for debug
    debug_phase_hall_acc_q8_24 = ((uint32_t)hall_phase_q8_8) << SPEED_FRAC_BITS; // to debug
    debug_p_term = p_term_q8_19 >> 11;
    debug_i_term = pll.i32_integrator_q8_19 >> 11;
    debug_pll_step = ui32_pll_step_q8_24 >> SPEED_FRAC_BITS; // in q8.8
    debug_angle_correction = i32_angle_correction_q8_8;
    debug_pll_phase_acc_max = pll.ui32_phase_acc_max_q8_24;
    #endif
}



/* ---------------- Accessors ---------------- */
inline uint16_t pll_get_angle_q8_8(void) { return g_pll_phase_q8_8; }

//  rpm 	erps	us/rotation	us/transition	step_q8_24  velocity_q8_8X1024  PWM/transition
//  100	   7	     150000	        25000	   1507006           447               475,0
//  500	  33	      30000	         5000	   7535030          2237                95,0
// 3000	 200	       5000	          833	  45210182         13422                15,8
// 4700	 313	       3191	          532	  70829285         21027                10,1
// 6000	 400	       2500	          417	  90420364         26844                 7,9

// Ratios:  to convert, multiply by 
// step => rpm       0,00006635673344    = 4453  >> 26 (first >> 10 then * then >> 16)
// step => erps      0,000004423782229   = 1187  >> 28 (first >> 12 then * then >> 16)
// step => velocityQ8_8X104 0,000296875  = 4981 >> 24 (first >> 8 then * then >> 16)
// rpm => erps       0,06666667
// rpm  => velocity  4,473924267        = 
// rpm => step       15070
// erps => rpm       15
// erps => velocity  67
// erps => step      226051
// velocity => rpm   0,2235
// velocity => eprs  0,000004423782
// velocity => step  3368

// rpm
uint32_t pll_get_rpm(void){
    #define PLL_SPTEP_TO_RPM ((uint32_t)4453)                 
    return (((pll.ui32_hall_step_q8_24 >> 10) * PLL_SPTEP_TO_RPM) >> 16);
}

// erps = pll.speed_q8_24 >> 8  * 6 * PWM_HZ / (1<<24); Num exceed 1<<31, so we split 24 in 2 part 6 + 18
inline uint16_t pll_get_erps(void){
    #define PLL_STEP_TO_ERPS ((uint32_t)1187) 
    return (uint16_t)(( (pll.ui32_hall_step_q8_24 >> 12) * PLL_STEP_TO_ERPS) >> 16);
}

// velocity is in Q8.8 X1024
uint32_t pll_get_velocity(void){
    #define PLL_STEP_TO_VELOCITY ((uint32_t)4981) 
    return (uint32_t) (((pll.ui32_hall_step_q8_24 >> 8) * PLL_STEP_TO_VELOCITY) >> 16) ; // *4891 / 65536 = 0,076 to get velocity units
}


