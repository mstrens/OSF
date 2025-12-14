
/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "adc.h"
#include "xmc_vadc.h"


// we use 2 groups
// master is group 1, slave is group 0
// when an external trigger (based on CCU8 slice 3) occurs, group 1 is triggered
// Slave (group 0) is synchronize and so one channel of G0 is converted after each conversion of G1
// Queues are used :  one for G1 and one for G0
// Sequence is queue g1 is channel 0, then channel 1 , channel 7 (Throttle) and channel 6 (Vdc);
//                            results go respectively in result reg 0 , 1 , 7 and 6 (of group 1)
//                   g0 is channel 0 and then channel 1 and 7;
//                            results go respectively in result reg 0 , 1 , 7 (of group 0)
// even if conversions occur in sequence, goup 1 and 0 perform a sample at the same time.
// sampling can be as short as 31 nsec (1 clock at 32Mhz) but we used a set up of 3 so 4*31 nsec per sample
// Alias feature is used; it means that e.g. g1 channel 0 can be connected to différent pins (to measure IU, IV or IW)
// The values named I1, I2, I3, I4 correspond to the sequence
// even if alias allow to connect different pin to Channel 0 and 1, the results go in the specified result register so always in reg 0 and 1
// As there is only one result register for e.g. I1, we have to take care of the channel set in G1 alias 0 to know which current it is.
// Pin 2.8 = Idc = Gr0 Ch1 or  Gr1 Ch0
// Pin 2.9 = Iu???  = Gr0 Ch2 or  Gr1 Ch4
// Pin 2.10= Iv???  = Gr0 Ch3 or  Gr1 Ch2
// pin 2.11= Iw???  = Gr0 Ch4 or  Gr1 Ch3
// Throttle in group 1: XMC_VADC_CHANNEL_CONFIG_t G1_CH7_THROTTLE_P2_5_config , result in Group 1 reg 7 Synchronised with Torque
// Torque in group 0 :   XMC_VADC_CHANNEL_CONFIG_t G0_CH7_TORQUE_P2_2_config; result in Group 0 reg 7
// Voltage in group1 : XMC_VADC_CHANNEL_CONFIG_t G1_CH6_BATTERY_P2_4_config; result in Group 1 reg 6 Not synchronised


/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
/**
 *  Data Structure initialization - VADC global Configuration.
 */
XMC_VADC_GLOBAL_CONFIG_t VADC_global_config =
{
  .module_disable = 0U,   /* 1 disable the module*/
  .disable_sleep_mode_control = 1U,
  .clock_config =
  {
	  .analog_clock_divider = 0U,       /*Divider Factor for the Analog Internal Clock*/
    .arbiter_clock_divider = 0U,
    .msb_conversion_clock = 0U,
  },
  .data_reduction_control         = 0U, /* Data Reduction disabled*/
  .wait_for_read_mode             = 0U, /* 1 = GLOBRES Register will not be overwriten untill the previous value is read*/
  .event_gen_enable               = 0U, /* Result Event from GLOBRES is disabled*/

  .boundary0    = 0U, /* Lower boundary value for Normal comparison mode*/
  .boundary1    = 0U/* Upper boundary value for Normal comparison mode*/
};

/**
 *  Data Structure initialization - VADC group 0 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_grp0_init =
{
  .emux_config =             /*External multiplexer config*/
  {
    .stce_usage                = 0U,           /*Use STCE when the setting changes*/
    .emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
    .emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
    .starting_external_channel = (uint32_t) 0U,                   /* Channel starts at 0 for EMUX*/
    .connected_channel         = (uint32_t) 0U                    /* Channel connected to EMUX*/
  },
  .class0 =
  {
    .sample_time_std_conv            = 3U,                /*The Sample time is (4*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 4U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  },  /* !<ICLASS-0 */
  .class1 =
  {
    .sample_time_std_conv             = 3U,                /*The Sample time is (4*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 4U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  }, /* !< ICLASS-1 */
  .boundary0  = 0U,  /* Lower boundary value for Normal comparison mode*/
  .boundary1  = 0U,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0U,  // It was 0;  4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run. Here always*/
};

/**
 *  Data Structure initialization - VADC group 1 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_grp1_init =
{
  .emux_config = // Not used
  {
    .stce_usage                = 0U,           /*Use STCE when the setting changes*/
    .emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
    .emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
    .starting_external_channel = (uint32_t) 0U,                   /* Channel starts at 0 for EMUX*/
    .connected_channel         = (uint32_t) 0U                    /* Channel connected to EMUX*/
  },
  .class0 =
  {
    .sample_time_std_conv            = 3U,                /*The Sample time is (4*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 4U,                // Not used
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      // Not used
  },  /* !<ICLASS-0 */
  .class1 =
  {
    .sample_time_std_conv            = 3U,                /*The Sample time is (4*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 4U,                // Not used
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      // Not used
  }, /* !< ICLASS-1 */
  .boundary0  = 0U,  /* Lower boundary value for Normal comparison mode*/
  .boundary1  = 0U,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0U,  // It was 0 ; 4 arbitration slots per round selected (tarb = 4*tadcd) 
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run.*/
};

/**
 *  Data Structure initialization - VADC group scan request source.
 */

/* Not used anymore
// Mstrens : background is used for Throttle and total IDC current
// run in low priority and continuously (autoscan) and is triggered by software
XMC_VADC_BACKGROUND_CONFIG_t VADC_grp_scan_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,  // here = wait for start mode =  complete current conversion
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0, // lowest priority
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      //If trigger needed the signal input*//*?? trigeger event P generated by???
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_NONE,   //Trigger edge needed if trigger enabled
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            //If gating needed the signal input
  .timer_mode       = (uint32_t) 0,                            // Disabled equidistant sampling
  .external_trigger = (uint32_t) 0,
  .req_src_interrupt = (uint32_t) 0,                              //Background Request source interrupt Disabled
  .enable_auto_scan  = (uint32_t) 1,                              // continue conversion
  .load_mode       = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE,   //Response from SCAN when a Load event occours.
};
*/


// Mstrens : we use only 3 shunt sync conv because it is the best for XMC13: #if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_grp_queue_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_CIR,     /* Conversion start mode WFS/CIR/CNR , Here = Cancel-inject-repeat mode*/
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority, here  = high priority*/
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_CCU80_SR3, //XMC_VADC_REQ_TR_P, //XMC_VADC_REQ_TR_P,      /*Use gate signal as trigger*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled , here trigger on rising edge*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_A,            /*CCU80.ST3A signal */
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1            /*External trigger Enabled/Disabled , = Enabled*/
};


//  Data Structure initialization - VADC group queue entries.
// we are using 4 entries for group 1
XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_alias_ch0 =  // entry 0 = first channel to convert (it is the only one getting a trigger)
{
  .channel_num = VADC_I1_CHANNEL,   // 0 
  .external_trigger = true,    // require a trigger to start conversion
  .generate_interrupt = false, /*no interrupt when this channel is requeste*/
  .refill_needed = true /* quue entry is refill in the queue when conversion is done*/
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_alias_ch1 =  // enty 1
{
  .channel_num = VADC_I3_CHANNEL,  // 1
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_pot =
{
  .channel_num = VADC_POT_CHANNEL, //7
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_volt_DC =
{
  .channel_num = VADC_VDC_CHANNEL, //6
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};


//  Data Structure initialization - VADC group channels. Note :  there are 2*16+1 result register (16 per group + 1 global)
// We define the 4 channels to be converted in group 1
XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_ch0_init =
{
  .alias_channel = (int8_t) 3, // = chan 3 = pin 2.11 ; could be changed using alias
  .result_reg_number = 0,
#if(ADC_ALTERNATE_REF_PHASEUVW == DISABLED)
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
#elif(ADC_ALTERNATE_REF_PHASEUVW == ENABLED)
  .alternate_reference =  XMC_VADC_CHANNEL_REF_ALT_CH0,
#endif
  .channel_priority = 3,
  .sync_conversion = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_ch1_init =
{
  .alias_channel = (int8_t) 4, // this is the channel 4 = pin 2.9; could be changed using alias
  .result_reg_number = 1,
#if(ADC_ALTERNATE_REF_PHASEUVW == DISABLED)
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
#elif(ADC_ALTERNATE_REF_PHASEUVW == ENABLED)
  .alternate_reference =  XMC_VADC_CHANNEL_REF_ALT_CH0,
#endif
  .channel_priority = 3,
  .sync_conversion = true
};

/* Throttle ADC channel data configuration */ // will be assign to background (line 479)
XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_channel_pot_init =
//XMC_VADC_CHANNEL_CONFIG_t G1_CH7_THROTTLE_P2_5_config , result in reg 7=
{
  .alias_channel = -1,  /* -1 = no alias*/
  .result_reg_number = VADC_POT_RESULT_REG,   /* result of conv is stored in this register*/
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF, /*use internal V ref for conversion*/
  .channel_priority = 0,          /* lowest priority*/
  .sync_conversion = true , // it was false when channel was assigned to background   false,       /*no syncronization*/
};


/* DC voltage ADC channel data configuration */ // will be assigned to background (line 483) 
XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_channel_vdc_init =
//const XMC_VADC_CHANNEL_CONFIG_t G1_CH6_BATTERY_P2_4_config; result in group 1, reg 6
{
  .alias_channel = -1,
  .result_reg_number = VADC_VDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false,
#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
  .lower_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
  .event_gen_criteria = XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
  .invert_boundary_flag_ch0   = (uint32_t) false,
  .invert_boundary_flag_ch1   = (uint32_t) false,
  .invert_boundary_flag_ch2   = (uint32_t) false,
  .invert_boundary_flag_ch3   = (uint32_t) false,
  .flag_output_condition_ch0  = (uint32_t) false,
  .flag_output_condition_ch1  = (uint32_t) false,
  .flag_output_condition_ch2  = (uint32_t) false,
  .flag_output_condition_ch3  = (uint32_t) false,
  .boundary_flag_mode_ch0     = (uint32_t) 1,
  .boundary_flag_mode_ch1     = (uint32_t) 0,
  .boundary_flag_mode_ch2     = (uint32_t) 0,
  .boundary_flag_mode_ch3     = (uint32_t) 0,
#endif
};



XMC_VADC_CHANNEL_CONFIG_t VADC_grp0_ch0_init =
{
  .alias_channel = (int8_t) 3, //chan 3 = pin 2.10 ; could be changed using ALIAS
  .result_reg_number = 0,
#if(ADC_ALTERNATE_REF_PHASEUVW == DISABLED)
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
#elif(ADC_ALTERNATE_REF_PHASEUVW == ENABLED)
  .alternate_reference =  XMC_VADC_CHANNEL_REF_ALT_CH0,
#endif
  .channel_priority = 3,
  .sync_conversion = true 
};

XMC_VADC_CHANNEL_CONFIG_t VADC_grp0_ch1_init =
{
  .alias_channel = (int8_t) 1, //chan 1 = P2.8 = Idc ; no reason to change using ALIAS
  .result_reg_number = 1,
#if(ADC_ALTERNATE_REF_PHASEUVW == DISABLED)
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
#elif(ADC_ALTERNATE_REF_PHASEUVW == ENABLED)
  .alternate_reference =  XMC_VADC_CHANNEL_REF_ALT_CH0,
#endif
  .channel_priority = 3,
  .sync_conversion = true // it was false
};

/* Torque ADC channel data configuration */ // will be assign to background (line 479)
XMC_VADC_CHANNEL_CONFIG_t VADC_grp0_channel_torque_init =
//XMC_VADC_CHANNEL_CONFIG_t = G0_CH7_TORQUE_P2_2_config; result in Group 0 reg 7
{
  .alias_channel = -1,  /* -1 = no alias*/
  .result_reg_number = VADC_TORQUE_RESULT_REG,   /* result of conv is stored in this register*/
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF, /*use internal V ref for conversion*/
  .channel_priority = 0,          /* lowest priority*/
  .sync_conversion = true , // it was false when channel was assigned to background  false,       /*no syncronization*/
};



// ***********************************************
// * API IMPLEMENTATION using only 2 groups and 2 queues (no background nor scan)
// ***************************************
void pmsm_adc_module_init(void)
{
    // --- 1. Initialisation globale ---
    XMC_VADC_GLOBAL_Init(VADC, &VADC_global_config);

    // --- 2. Initialisation des groupes ---
    XMC_VADC_GROUP_Init(VADC_G0, &VADC_grp0_init);
    XMC_VADC_GROUP_Init(VADC_G1, &VADC_grp1_init);

    // --- 3. Mettre tous les groupes en mode normal pour config ---
    XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

#if(ADC_STARTUP_CALIBRATION == ENABLED)
    pmsm_foc_adc_startupcalibration();
#else
    XMC_VADC_GLOBAL_StartupCalibration(VADC);
    XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 0U);
    XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 1U);
#endif

    // --- 6. Configuration de la queue du group 1 (master)---
    XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_grp_queue_config);
    XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);
    XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch0);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch1);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_pot);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_volt_DC);

    // --- 7. Initialisation des channels Master G1 ---
    XMC_VADC_GROUP_ChannelInit(VADC_G1, VADC_I1_CHANNEL, &VADC_grp1_ch0_init);
    XMC_VADC_GROUP_ChannelInit(VADC_G1, VADC_I3_CHANNEL, &VADC_grp1_ch1_init);
    XMC_VADC_GROUP_ChannelInit(VADC_G1 , 7, &VADC_grp1_channel_pot_init);  // Throttle use ch 7
    XMC_VADC_GROUP_ChannelInit(VADC_G1 , 6, &VADC_grp1_channel_vdc_init);  // Voltahe use ch 6

    // ---  Configuration de la queue du group 1 (master)---
    XMC_VADC_QUEUE_CONFIG_t VADC_grp0_queue_config =
    {
        .conv_start_mode  = XMC_VADC_STARTMODE_CIR,
        .req_src_priority = XMC_VADC_GROUP_RS_PRIORITY_3,
        .trigger_signal   = 0,
        .trigger_edge     = XMC_VADC_TRIGGER_EDGE_NONE,
        .gate_signal      = 0,
        .timer_mode       = 0,
        .external_trigger = 0
    };
    XMC_VADC_GROUP_QueueInit(VADC_G0, &VADC_grp0_queue_config);
    XMC_VADC_GROUP_QueueFlushEntries(VADC_G0);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G0, (XMC_VADC_QUEUE_ENTRY_t){ .channel_num = 0, .external_trigger=false, .refill_needed=true });
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G0, (XMC_VADC_QUEUE_ENTRY_t){ .channel_num = 1, .external_trigger=false, .refill_needed=true });
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G0, (XMC_VADC_QUEUE_ENTRY_t){ .channel_num = 7, .external_trigger=false, .refill_needed=true });
    
    // --- 8. Initialisation des channels Slave G0 ---
    XMC_VADC_GROUP_ChannelInit(VADC_G0, VADC_I2_CHANNEL, &VADC_grp0_ch0_init);
    XMC_VADC_GROUP_ChannelInit(VADC_G0, VADC_I4_CHANNEL, &VADC_grp0_ch1_init);
    XMC_VADC_GROUP_ChannelInit(VADC_G0, 7, &VADC_grp0_channel_torque_init);
    
    // --- 9. Config synchronisation master/slave ---
    XMC_VADC_GROUP_SetSyncSlave(VADC_G0, 1U, 0U);
    XMC_VADC_GROUP_CheckSlaveReadiness(VADC_G0, 0U);
    XMC_VADC_GROUP_SetSyncMaster(VADC_G1);

    // to be sure that arbitration is enabled
    XMC_VADC_GROUP_QueueEnableArbitrationSlot(VADC_G0);
    XMC_VADC_GROUP_QueueEnableArbitrationSlot(VADC_G1);
    
}




/* Original code not working - register VFR is not filled when isr1 runs. Solution: avoid background conversion and add more queue conversion
void pmsm_adc_module_init(void)
{

  XMC_VADC_GLOBAL_Init(VADC, &VADC_global_config);
  XMC_VADC_GROUP_Init(VADC_G0, &VADC_grp0_init);
  XMC_VADC_GROUP_Init(VADC_G1, &VADC_grp1_init);

  // Configuration of VADC_G1/0,  Turn on ADC modules 
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

#if(ADC_STARTUP_CALIBRATION == ENABLED)
  pmsm_foc_adc_startupcalibration();
#else
  // Trigger Start-up Calibration
  XMC_VADC_GLOBAL_StartupCalibration(VADC);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 0U);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 1U);
#endif


  XMC_VADC_GLOBAL_BackgroundInit(VADC, &VADC_grp_scan_config); // autoscan is enable, so once started, it will continue conversion
  // Configure the gating mode for scan
//  XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0,XMC_VADC_GATEMODE_IGNORE);
//  XMC_VADC_GLOBAL_BackgroundSetGatingMode(VADC,XMC_VADC_GATEMODE_IGNORE);

    //pmsm_phasecurrent_init();  // prepare queues for gr 0 and gr 1
  // use G1 ch0, g1 ch1, G0 ch0, G0 ch1, g0 IDC
  // create queue for group 1, clear it, accept always triger, add alias_ch0 and ch1
  // init channels for group 1 and 0 , I1, I3, .. = number of the channel in the group (0,..7)
  // init group1 channel I1 as grp1_ch0
  //           1         I3       1   1
  //           0         I2       0   0
  //           0         I4       0   1
  //           0         Idc      0   channel idc init 

  // Configuration of VADC_G1 - Q source 
  // External trigger 1,  refill 1 
  XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_grp_queue_config); // highest priority, trigger by CCU8 SR3 (= mid point)

  XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);

   // Configure the gating mode for queue
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE); //IGNORE = External triggers are unconditionally passed
  // added by mstrens to be sure
  //XMC_VADC_GROUP_QueueSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE); //IGNORE = External triggers are unconditionally passed

  // Request the LLD to insert the channel  // Add 2 alias channel to group 1
  XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch0); // VADC_I1_CHANNEL (0), require a trigger to start conv
  XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch1); // VADC_I3_CHANNEL (1), do not require a trigger to start conv

  // Master group - G1 for Synchronous ADC 
  // I1 = first conversion, Result Register group 1 RES0 (hard coded)
  XMC_VADC_GROUP_ChannelInit(VADC_I1_GROUP, VADC_I1_CHANNEL, &VADC_grp1_ch0_init); // result in reg 0 of gr 1, alias channel 3
  // I3 = third conversion, Result Register group 1 RES1 (hard coded)
  XMC_VADC_GROUP_ChannelInit(VADC_I3_GROUP, VADC_I3_CHANNEL, &VADC_grp1_ch1_init); // result in reg 1 of gr 1, alias channel 4

  // slave group GO
  // I2 =  second conversion, Result Register group 0 RES0 (hard coded)
  XMC_VADC_GROUP_ChannelInit(VADC_I2_GROUP, VADC_I2_CHANNEL, &VADC_grp0_ch0_init); // result in reg 0 of gr 0,  alias channel 3
  // I4 = Idc = fourth conversion , Result Register group 0 RES1 (hard coded)
  XMC_VADC_GROUP_ChannelInit(VADC_I4_GROUP, VADC_I4_CHANNEL, &VADC_grp0_ch1_init);  // result in reg 1 of gr 0, alias of channel 1
  // commented by mstrens to keep the conversion via the alias that is synchronised
  //XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init); // result in reg 1 of group 0 , no alias

  // disable power saving
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_OFF);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_OFF);

  // G0: synchronization slave 
  XMC_VADC_GROUP_SetSyncSlave(VADC_G0, 1U, 0U); // VADC_G0 = pointer to slave group, 1= master group, 0 = slave group

  // Ready input R1 is considered 
  XMC_VADC_GROUP_CheckSlaveReadiness(VADC_G0, 0U);

  // G1: synchronization master 
  XMC_VADC_GROUP_SetSyncMaster(VADC_G1); // define the master

  // ANONS = 11B: Normal Operation 
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL); // group is powered on here

  // added by mstrens for safety (perhaps not required because it is a slave group)
    XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL); // group is powered on here
    
  // commented by mstrens - not sure it is required
  //XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);
  // commented by mstrens - IDC is measured via alias
  //XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL); // add idc channel to the sequence

// Added by mstrens to configure the alias but this is changed dynamically in motor.c
// here consider that sequence will be IW, IV, IU and IDC but tests shows that in fact Iw is I3 and Iu is I1 and not the opposite
//VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
//VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
        
        
//pmsm_adc_dclink_init();  // add conversion of DC voltage to background in group 1 channel 6 result in reg G1 6
  // Initializes the DC Link VADC channel for conversion 
  XMC_VADC_GROUP_ChannelInit(VADC_VDC_GROUP, VADC_VDC_CHANNEL, &VADC_channel_vdc_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_VDC_GROUP_NO,VADC_VDC_CHANNEL);

// commented by mstrens  - could be reused as example for overcurrent protection
//#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
//  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,VDC_MIN_LIMIT);
//  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,VDC_OVER_LIMIT);
//    NVIC_SetPriority (VADC0_G1_0_IRQn, 1U);
//    NVIC_EnableIRQ(VADC0_G1_0_IRQn);
//#endif


// Initializes the POT VADC channel for conversion // is part of group 1; channel is 7  result in reg G1 7; conversion is background
  XMC_VADC_GROUP_ChannelInit(VADC_POT_GROUP, VADC_POT_CHANNEL, &VADC_channel_pot_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_POT_GROUP_NO,VADC_POT_CHANNEL);

//added by mstrens to convert torque
// Initializes the Torque VADC channel for conversion  // is part of group 0; channel is 7 ; result in reg G0 7conversion is background
  XMC_VADC_GROUP_ChannelInit(VADC_TORQUE_GROUP, VADC_TORQUE_CHANNEL, &VADC_channel_torque_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_TORQUE_GROUP_NO,VADC_TORQUE_CHANNEL);

  XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC); // start a conversion in background (and so it will continue due to autoscan)
}
*/ // End of original code 


/*
// API to initialize VADC channel for potentiameter voltage sensing that is used to set user speed
// pins of VADC are hardcode in XMC13 ; there are 12 pins that are assinged to 2*8 channels (8 per group)
void pmsm_adc_pot_init(void)
{
  // Initializes the POT VADC channel for conversion 
  XMC_VADC_GROUP_ChannelInit(VADC_POT_GROUP, VADC_POT_CHANNEL, &VADC_channel_pot_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_POT_GROUP_NO,VADC_POT_CHANNEL);
}

// API to initialize VADC channel for DC link voltage sensing 
void pmsm_adc_dclink_init(void)
{
  // Initializes the DC Link VADC channel for conversion
  XMC_VADC_GROUP_ChannelInit(VADC_VDC_GROUP, VADC_VDC_CHANNEL, &VADC_channel_vdc_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_VDC_GROUP_NO,VADC_VDC_CHANNEL);

#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,VDC_MIN_LIMIT);
  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,VDC_OVER_LIMIT);

    NVIC_SetPriority (VADC0_G1_0_IRQn, 1U);
    NVIC_EnableIRQ(VADC0_G1_0_IRQn);
  
#endif
}
*/



/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Turn on Group 0 converter to calibrates VADC by triggering dummy conversions.
 * This startup calibration is required only for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void pmsm_foc_adc_startupcalibration(void)
{
   volatile uint32_t dealy_counter; /* Delay counter for the startup calibration to complete. */
   *SHS0_CALOC0 = REG_RESET;
   *SHS0_CALOC1 = REG_RESET;

   #if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
   /* Enable Group 0 for Calibration */
   XMC_VADC_GROUP_SetPowerMode(VADC_G0,XMC_VADC_GROUP_POWERMODE_NORMAL);
   #endif

   /* Enable the StartUp calibration in the VADC */
   VADC->GLOBCFG |= (((uint32_t)1 << (uint32_t)VADC_GLOBCFG_SUCAL_Pos) & (uint32_t)VADC_GLOBCFG_SUCAL_Msk) |
                    (((uint32_t)1 << (uint32_t)VADC_GLOBCFG_DPCAL0_Pos) & (uint32_t)VADC_GLOBCFG_DPCAL0_Msk);

   /* Wait for 1920cycles or 60us for the startup calibration to complete. */
   dealy_counter = VADC_CALIBRATION_CYCLE_DELAY;

   while (dealy_counter > 0U)
   {
     dealy_counter--;
     /* Clear offset calibration values */
     CLEAR_OFFSET_CALIB_VALUES;
   }
   pmsm_foc_adc_gaincalib();

   /* Switch off Group 0 converter if it is not used for any conversions */
   #if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
   XMC_VADC_GROUP_SetPowerMode(VADC_G0,XMC_VADC_GROUP_POWERMODE_OFF);
   #endif
}



/* This API is used for VADC gain calibration. */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Calibrates VADC by triggering dummy conversion for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void pmsm_foc_adc_gaincalib(void)
{
  volatile uint16_t dummy_conv_counter = VADC_DUMMY_CONVERSION_COUNTER;  /* Used for keeping track for dummy conversions */
  volatile uint32_t adc_result_aux;                                      /* Used for reading ADC values after dummy conversions */

  /* ADC_AI.004 errata*/
  *SHS0_CALCTR = 0X3F100400U;

  /* add a channel in group-0 for dummy conversion*/
  VADC->BRSSEL[0] = VADC_BRSSEL_CHSELG0_Msk;

  /*Clear the DPCAL0, DPCAL1 and SUCAL bits*/
  VADC->GLOBCFG &= ~( (uint32_t)VADC_GLOBCFG_DPCAL0_Msk | (uint32_t)VADC_GLOBCFG_DPCAL1_Msk | (uint32_t)VADC_GLOBCFG_SUCAL_Msk);

  /* Clear offset calibration values*/
  CLEAR_OFFSET_CALIB_VALUES;

  VADC->BRSMR = ((uint32_t)1 << (uint32_t)VADC_BRSMR_ENGT_Pos);
  #if UC_SERIES != XMC11
  VADC_G0->ARBPR = (VADC_G_ARBPR_ASEN2_Msk);
  #endif
  /*Trigger dummy conversion for 9* 2000 times*/

  while (dummy_conv_counter > 0U)
  {
    /*load event */
    VADC->BRSMR |= (uint32_t)VADC_BRSMR_LDEV_Msk;
    #if (UC_SERIES != XMC11)
    /*Wait until a new result is available*/
    while (VADC_G0->VFR == 0U)
    {

    }

    /*dummy read of result */
    adc_result_aux = VADC_G0->RES[0];
    #else
    /*Wait untill a new result is available*/
    while ((VADC->GLOBRES & VADC_GLOBRES_VF_Msk) == 0)
    {

    }

    /*dummy read of result */
    adc_result_aux = VADC->GLOBRES;
    #endif

    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES;
    dummy_conv_counter--;
  }

  /* To avoid a warning*/
  adc_result_aux &= 0U;

  /* Wait until last gain calibration step is finished */
  while ( (SHS0->SHSCFG & (uint32_t)SHS_SHSCFG_STATE_Msk) != 0U )
  {
    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES;
  }
  /* Re enable SUCAL DPCAL */
  VADC->GLOBCFG |= ( (uint32_t)VADC_GLOBCFG_DPCAL0_Msk | (uint32_t)VADC_GLOBCFG_DPCAL1_Msk);
  VADC->BRSMR     = (uint32_t)0x00;
  VADC->BRSSEL[0] = (uint32_t)0x00;
  #if (UC_SERIES != XMC11)
  VADC_G0->REFCLR = 1U;
  VADC_G0->ARBPR &= ~((uint32_t)VADC_G_ARBPR_ASEN2_Msk);
  #endif
}




