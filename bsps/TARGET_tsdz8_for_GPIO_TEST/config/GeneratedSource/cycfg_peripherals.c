/*******************************************************************************
 * File Name: cycfg_peripherals.c
 *
 * Description:
 * Peripheral Hardware Block configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.30.0
 * device-db 4.20.0.7450
 * mtb-xmclib-cat3 4.4.0.4715
 *
 *******************************************************************************
 * Copyright 2025 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "cycfg_peripherals.h"
#include "../../adc.h"

#define HALL_POSIF_HALPS(EP, CP) (((uint32_t)EP <<  3) | (uint32_t)CP)


const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_DELAY_TIMER_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_SINGLE,
    .shadow_xfer_clear = true,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = false,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event0_config =
{
    .mapped_input = CCU40_IN0_EV0IS_VALUE,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};

const XMC_CCU4_SLICE_CAPTURE_CONFIG_t HALL_SPEED_TIMER_capture_config =
{
    .fifo_enable = false,
    .timer_clear_mode = XMC_CCU4_SLICE_TIMER_CLEAR_MODE_NEVER,
    .same_event = false,
    .ignore_full_flag = true,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_64, // for 250000 Hz, it was XMC_CCU4_SLICE_PRESCALER_256
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .timer_concatenation = false,
};

const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event0_config =
{
    .mapped_input = CCU40_IN1_EV0IS_VALUE,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_COMPARE_CONFIG_t PWM_TORQUE_TIMER_compare_config =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = false,
    .dither_timer_period = false,
    .dither_duty_cycle = false,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = false,
    .prescaler_initval = XMC_CCU4_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .timer_concatenation = false,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event0_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event1_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event2_config =
{
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_U_TIMER_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 1,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 0,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_U_TIMER_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = false,
    .channel1_st_path = true,
    .channel2_st_path = false,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = false,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 120,
    .channel2_st_rising_edge_counter = 0,
    .channel1_st_falling_edge_counter = 120,
    .channel2_st_falling_edge_counter = 0,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event0_config =
{
    .mapped_input = CCU80_IN0_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event1_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_V_TIMER_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 1,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 0,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_V_TIMER_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = false,
    .channel1_st_path = true,
    .channel2_st_path = false,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = false,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 120,
    .channel2_st_rising_edge_counter = 0,
    .channel1_st_falling_edge_counter = 120,
    .channel2_st_falling_edge_counter = 0,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event0_config =
{
    .mapped_input = CCU80_IN1_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event1_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_W_TIMER_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 1,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 0,
    .invert_out0 = true,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_W_TIMER_dead_time_config =
{
    .enable_dead_time_channel1 = true,
    .enable_dead_time_channel2 = false,
    .channel1_st_path = true,
    .channel2_st_path = false,
    .channel1_inv_st_path = true,
    .channel2_inv_st_path = false,
    .div = XMC_CCU8_SLICE_DTC_DIV_1,
    .channel1_st_rising_edge_counter = 120,
    .channel2_st_rising_edge_counter = 0,
    .channel1_st_falling_edge_counter = 120,
    .channel2_st_falling_edge_counter = 0,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event0_config =
{
    .mapped_input = CCU80_IN2_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event1_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_IRQ_TIMER_compare_config =
{
    .timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0,
    .dither_timer_period = 0,
    .dither_duty_cycle = 0,
    .prescaler_mode = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable = 0,
    .mcm_ch2_enable = 0,
    .slice_status = XMC_CCU8_SLICE_STATUS_CHANNEL_1_AND_2,
    .passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out1 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out2 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .passive_level_out3 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .asymmetric_pwm = 0,
    .invert_out0 = false,
    .invert_out1 = false,
    .invert_out2 = false,
    .invert_out3 = false,
    .prescaler_initval = XMC_CCU8_SLICE_PRESCALER_1,
    .float_limit = XMC_CCU8_SLICE_PRESCALER_32768,
    .dither_limit = 0U,
    .timer_concatenation = 0,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event0_config =
{
    .mapped_input = CCU80_IN3_SCU_GSC80,
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event1_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event2_config =
{
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
    .level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
    .duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
};
const uint32_t HALL_POSIF_Hall_Pattern[] =
{
    [1] = HALL_POSIF_HALPS(3, 1),
    [3] = HALL_POSIF_HALPS(2, 3),
    [2] = HALL_POSIF_HALPS(6, 2),
    [6] = HALL_POSIF_HALPS(4, 6),
    [4] = HALL_POSIF_HALPS(5, 4),
    [5] = HALL_POSIF_HALPS(1, 5),
    [9] = HALL_POSIF_HALPS(5, 1),
    [11] = HALL_POSIF_HALPS(1, 3),
    [10] = HALL_POSIF_HALPS(3, 2),
    [14] = HALL_POSIF_HALPS(2, 6),
    [12] = HALL_POSIF_HALPS(6, 4),
    [13] = HALL_POSIF_HALPS(4, 5),
};
const XMC_POSIF_CONFIG_t HALL_POSIF_config =
{
    .mode = XMC_POSIF_MODE_HALL_SENSOR,
    .input0 = POSIF0_PCONF_INSEL0,
    .input1 = POSIF0_PCONF_INSEL1,
    .input2 = POSIF0_PCONF_INSEL2,
    .filter = XMC_POSIF_FILTER_64_CLOCK_CYCLE,
};
const XMC_POSIF_HSC_CONFIG_t HALL_POSIF_HSC_InitHandle =
{
    .disable_idle_signal = 1U,
    .sampling_trigger = POSIF0_PCONF_DSEL,
    .sampling_trigger_edge = XMC_POSIF_HSC_TRIGGER_EDGE_RISING,
    .external_error_enable = 0U,
};
const XMC_UART_CH_CONFIG_t CYBSP_DEBUG_UART_config =
{
    .baudrate = 9600UL,
    .normal_divider_mode = false,
    .data_bits = 8U,
    .frame_length = 8U,
    .stop_bits = 1U,
    .oversampling = 16U,
    .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE,
};
XMC_VADC_GROUP_CONFIG_t vadc_0_group0_init_config =
{
    .emux_config.starting_external_channel = (uint32_t) 0,
    .emux_config.connected_channel = (uint32_t) 0,
    .emux_config.emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL,
    .emux_config.emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY,
    .emux_config.stce_usage = (uint32_t) 0,
    .boundary0 = (uint32_t) 0,
    .boundary1 = (uint32_t) 0,
    .arbitration_round_length = (uint32_t) 0,
    .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,
};
XMC_VADC_GROUP_CONFIG_t vadc_0_group1_init_config =
{
    .emux_config.starting_external_channel = (uint32_t) 0,
    .emux_config.connected_channel = (uint32_t) 0,
    .emux_config.emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL,
    .emux_config.emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY,
    .emux_config.stce_usage = (uint32_t) 0,
    .boundary0 = (uint32_t) 0,
    .boundary1 = (uint32_t) 0,
    .arbitration_round_length = (uint32_t) 0,
    .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,
};
const XMC_VADC_GLOBAL_CONFIG_t vadc_0_config =
{
    .boundary0 = 0U,
    .boundary1 = 1000U,
    .clock_config.analog_clock_divider = 1U,
    .clock_config.msb_conversion_clock = 0U,
    .clock_config.arbiter_clock_divider = 0U,
    .class0.sample_time_std_conv = (uint32_t) 31,
    .class0.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .class0.sampling_phase_emux_channel = (uint32_t) 0,
    .class0.conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
    .class1.sample_time_std_conv = (uint32_t) 31,
    .class1.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .class1.sampling_phase_emux_channel = (uint32_t) 0,
    .class1.conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
    .data_reduction_control = 0U,
    .wait_for_read_mode = 0U,
    .event_gen_enable = 0U,
    .disable_sleep_mode_control = 0U,
};
const XMC_VADC_BACKGROUND_CONFIG_t vadc_0_background_scan_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_0,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_signal = (uint32_t) XMC_VADC_REQ_TR_A,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,
    .gate_signal = (uint32_t) XMC_VADC_REQ_GT_A,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) true,
    .req_src_interrupt = (uint32_t) false,
    .enable_auto_scan = (uint32_t) true,
    .load_mode = (uint32_t) XMC_VADC_SCAN_LOAD_OVERWRITE,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_0 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_1 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_2 =
{
    .channel_num = (uint8_t)7,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_3 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_4 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_5 =
{
    .channel_num = (uint8_t)2,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_6 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_7 =
{
    .channel_num = (uint8_t)1,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_15_config =
{
    .data_reduction_control = (uint32_t) 14,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_FILTERING_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_9_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_2_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_0_queue_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_ANY,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) false,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_0 =
{
    .sample_time_std_conv = (uint32_t) 31,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_1 =
{
    .sample_time_std_conv = (uint32_t) 31,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_CHANNEL_CONFIG_t G0_CH1_CURRENT_P2_8_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 15,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t G0_CH2_CURRENT_U_P2_9_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 9,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t G0_CH7_TORQUE_P2_2_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 2,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_0 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_1 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_2 =
{
    .channel_num = (uint8_t)7,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_3 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_4 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_5 =
{
    .channel_num = (uint8_t)6,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_6 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_7 =
{
    .channel_num = (uint8_t)0,
    .refill_needed = (uint32_t)true,
    .generate_interrupt = (uint32_t)false,
    .external_trigger = (uint32_t)false,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_15_config =
{
    .data_reduction_control = (uint32_t) 14,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_FILTERING_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = true,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_4_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_5_config =
{
    .data_reduction_control = (uint32_t) 0,
    .post_processing_mode = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
    .wait_for_read_mode = (uint32_t) false,
    .part_of_fifo = (uint32_t) false,
    .event_gen_enable = false,
};
const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_1_queue_config =
{
    .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3,
    .src_specific_result_reg = (uint32_t) 0,
    .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_ANY,
    .timer_mode = (uint32_t) false,
    .external_trigger = (uint32_t) false,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_0 =
{
    .sample_time_std_conv = (uint32_t) 31,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_1 =
{
    .sample_time_std_conv = (uint32_t) 31,
    .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT,
    .sampling_phase_emux_channel = (uint32_t) 0,
    .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT,
};
const XMC_VADC_CHANNEL_CONFIG_t G1_CH0_CURRENT_P2_8_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GLOBAL_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GLOBAL_BOUND1,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 15,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t G1_CH6_BATTERY_P2_4_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 4,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_VADC_CHANNEL_CONFIG_t G1_CH7_THROTTLE_P2_5_config =
{
    .input_class = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
    .lower_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .event_gen_criteria = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
    .sync_conversion = (uint32_t) false,
    .alternate_reference = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,
    .result_reg_number = (uint32_t) 5,
    .use_global_result = (uint32_t) 0,
    .result_alignment = (uint32_t) XMC_VADC_RESULT_ALIGN_LEFT,
    .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND,
    .broken_wire_detect = (uint32_t) false,
    .channel_priority = (bool) false,
    .alias_channel = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,
};
const XMC_WDT_CONFIG_t wdt_0_config =
{
    .window_lower_bound = 0.00,
    .window_upper_bound = 500.00,
    .prewarn_mode = false,
    .service_pulse_width = 1U,
    .run_in_debug_mode = 0U,
};

void init_cycfg_peripherals(void)
{
    XMC_CCU4_Init(ccu4_0_HW, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_StartPrescaler(ccu4_0_HW);
    XMC_CCU4_SLICE_CompareInit(HALL_DELAY_TIMER_HW, &HALL_DELAY_TIMER_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(HALL_DELAY_TIMER_HW, 5U); // it was 2
    XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_DELAY_TIMER_HW, 10U);  // it was 4
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_0_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
    XMC_CCU4_EnableShadowTransfer(ccu4_0_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_0 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
    XMC_CCU4_SLICE_ConfigureEvent(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_EVENT_0, &HALL_DELAY_TIMER_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_EVENT_1, &HALL_DELAY_TIMER_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_EVENT_2, &HALL_DELAY_TIMER_event2_config);
    XMC_CCU4_SLICE_StartConfig(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU4_SLICE_SetInterruptNode(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP, XMC_CCU4_SLICE_SR_ID_1);
    XMC_CCU4_SLICE_EnableEvent(HALL_DELAY_TIMER_HW, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
    XMC_CCU4_EnableClock(ccu4_0_HW, HALL_DELAY_TIMER_NUM);
    XMC_CCU4_SLICE_SetTimerValue(HALL_DELAY_TIMER_HW, 0U);
    XMC_CCU4_SLICE_StartTimer(HALL_DELAY_TIMER_HW);

    XMC_CCU4_SLICE_CaptureInit(HALL_SPEED_TIMER_HW, &HALL_SPEED_TIMER_capture_config);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_SPEED_TIMER_HW, 0XFFFFU);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_0_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1);
    XMC_CCU4_EnableShadowTransfer(ccu4_0_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_1 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1 );
    XMC_CCU4_SLICE_ConfigureEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_0, &HALL_SPEED_TIMER_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_1, &HALL_SPEED_TIMER_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_2, &HALL_SPEED_TIMER_event2_config);
    XMC_CCU4_SLICE_Capture0Config(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_0);
    //XMC_CCU4_SLICE_SetInterruptNode(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_SR_ID_0); // added by mstrens to get an irq (even not activated in nivq)
    //XMC_CCU4_SLICE_EnableEvent(HALL_SPEED_TIMER_HW, XMC_CCU4_SLICE_EVENT_0); // added by mstrens to get an irq
    XMC_CCU4_EnableClock(ccu4_0_HW, HALL_SPEED_TIMER_NUM);
    XMC_CCU4_SLICE_SetTimerValue(HALL_SPEED_TIMER_HW, 0U);
    XMC_CCU4_SLICE_StartTimer(HALL_SPEED_TIMER_HW);

    XMC_CCU4_SLICE_CompareInit(PWM_TORQUE_TIMER_HW, &PWM_TORQUE_TIMER_compare_config);
    XMC_CCU4_SLICE_SetTimerCompareMatch(PWM_TORQUE_TIMER_HW, 160U);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(PWM_TORQUE_TIMER_HW, 1279U);
    XMC_CCU4_SetMultiChannelShadowTransferMode(ccu4_0_HW, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3);
    XMC_CCU4_EnableShadowTransfer(ccu4_0_HW,
        XMC_CCU4_SHADOW_TRANSFER_SLICE_3 |
        XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_3 |
        XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_3 );
    XMC_CCU4_SLICE_ConfigureEvent(PWM_TORQUE_TIMER_HW, XMC_CCU4_SLICE_EVENT_0, &PWM_TORQUE_TIMER_event0_config);
    XMC_CCU4_SLICE_ConfigureEvent(PWM_TORQUE_TIMER_HW, XMC_CCU4_SLICE_EVENT_1, &PWM_TORQUE_TIMER_event1_config);
    XMC_CCU4_SLICE_ConfigureEvent(PWM_TORQUE_TIMER_HW, XMC_CCU4_SLICE_EVENT_2, &PWM_TORQUE_TIMER_event2_config);
    XMC_CCU4_EnableClock(ccu4_0_HW, PWM_TORQUE_TIMER_NUM);
    XMC_CCU4_SLICE_SetTimerValue(PWM_TORQUE_TIMER_HW, 0U);
    XMC_CCU4_SLICE_StartTimer(PWM_TORQUE_TIMER_HW);

    XMC_CCU8_Init(ccu8_0_HW, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU8_StartPrescaler(ccu8_0_HW);
    
    XMC_CCU8_SLICE_CompareInit(PHASE_U_TIMER_HW, &PHASE_U_TIMER_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PHASE_U_TIMER_HW, 840U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PHASE_U_TIMER_HW, 0U); // original code
// modified by mstrens to get a trigger for VADC based on compare value of channel 2
//XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PHASE_U_TIMER_HW, 1678U); // trigger just after mid point (to test)
// Router cet événement vers le service request 0 (SR0)
//XMC_CCU8_SLICE_SetInterruptNode(PHASE_U_TIMER_HW,
//                                XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_DOWN_CH_2,
//                                XMC_CCU8_SLICE_SR_ID_3);
// Activer l’événement de compare-match sur CH2 (sur comptage montant)
//XMC_CCU8_SLICE_EnableEvent(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_DOWN_CH_2);


    XMC_CCU8_SLICE_SetTimerPeriodMatch(PHASE_U_TIMER_HW, 1680U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(ccu8_0_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
    XMC_CCU8_EnableShadowTransfer(ccu8_0_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_0 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, &PHASE_U_TIMER_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_EVENT_1, &PHASE_U_TIMER_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_EVENT_2, &PHASE_U_TIMER_event2_config);
    XMC_CCU8_SLICE_StartConfig(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    //XMC_CCU8_EnableClock(ccu8_0_HW, PHASE_U_TIMER_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PHASE_U_TIMER_HW, &PHASE_U_TIMER_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PHASE_U_TIMER_HW, 0U);
    
    XMC_CCU8_SLICE_CompareInit(PHASE_V_TIMER_HW, &PHASE_V_TIMER_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PHASE_V_TIMER_HW, 840U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PHASE_V_TIMER_HW, 0U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PHASE_V_TIMER_HW, 1680U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(ccu8_0_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1);
    XMC_CCU8_EnableShadowTransfer(ccu8_0_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_1 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_1 );
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, &PHASE_V_TIMER_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_EVENT_1, &PHASE_V_TIMER_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_EVENT_2, &PHASE_V_TIMER_event2_config);
    XMC_CCU8_SLICE_StartConfig(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    //XMC_CCU8_EnableClock(ccu8_0_HW, PHASE_V_TIMER_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PHASE_V_TIMER_HW, &PHASE_V_TIMER_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PHASE_V_TIMER_HW, 0U);
    
    XMC_CCU8_SLICE_CompareInit(PHASE_W_TIMER_HW, &PHASE_W_TIMER_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PHASE_W_TIMER_HW, 840U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PHASE_W_TIMER_HW, 0U);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PHASE_W_TIMER_HW, 1680U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(ccu8_0_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE2);
    XMC_CCU8_EnableShadowTransfer(ccu8_0_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_2 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_2 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_2 );
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, &PHASE_W_TIMER_event0_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_EVENT_1, &PHASE_W_TIMER_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_EVENT_2, &PHASE_W_TIMER_event2_config);
    XMC_CCU8_SLICE_StartConfig(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    //XMC_CCU8_EnableClock(ccu8_0_HW, PHASE_W_TIMER_NUM);
    XMC_CCU8_SLICE_DeadTimeInit(PHASE_W_TIMER_HW, &PHASE_W_TIMER_dead_time_config);
    XMC_CCU8_SLICE_SetTimerValue(PHASE_W_TIMER_HW, 0U);
    
    XMC_CCU8_SLICE_CompareInit(PWM_IRQ_TIMER_HW, &PWM_IRQ_TIMER_compare_config);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_IRQ_TIMER_HW, 1679); // trigger ADC just before mid point // should be 1679
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_IRQ_TIMER_HW, 1300U); //  call ISR1 about 5 usec after mid point (when ADC conversions are done)
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_IRQ_TIMER_HW, 1680U);
    XMC_CCU8_SetMultiChannelShadowTransferMode(ccu8_0_HW, XMC_CCU8_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3);
    XMC_CCU8_EnableShadowTransfer(ccu8_0_HW,XMC_CCU8_SHADOW_TRANSFER_SLICE_3 |XMC_CCU8_SHADOW_TRANSFER_DITHER_SLICE_3 |XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_3 );
    XMC_CCU8_SLICE_ConfigureEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, &PWM_IRQ_TIMER_event0_config); // synchronised start
    XMC_CCU8_SLICE_ConfigureEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_EVENT_1, &PWM_IRQ_TIMER_event1_config);
    XMC_CCU8_SLICE_ConfigureEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_EVENT_2, &PWM_IRQ_TIMER_event2_config);
    XMC_CCU8_SLICE_StartConfig(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    
    // here we trigger SR3 nearly at mid point of PWM cycle; to choose another time, Perhaps could be changed based on duty cycle and sector
    
    // do not activate period match and one match simultanously !!!!
    //XMC_CCU8_SLICE_SetInterruptNode(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_3);
    XMC_CCU8_SLICE_SetInterruptNode(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_ONE_MATCH, XMC_CCU8_SLICE_SR_ID_0); // call ISR0 at begin of cycle
    XMC_CCU8_SLICE_SetInterruptNode(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_1, XMC_CCU8_SLICE_SR_ID_3); // trigger ADC
    XMC_CCU8_SLICE_SetInterruptNode(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_DOWN_CH_2, XMC_CCU8_SLICE_SR_ID_1); // call ISR1 after ADC conv
    
    //XMC_CCU8_SLICE_EnableEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
    XMC_CCU8_SLICE_EnableEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_ONE_MATCH);
    XMC_CCU8_SLICE_EnableEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_1);
    XMC_CCU8_SLICE_EnableEvent(PWM_IRQ_TIMER_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_DOWN_CH_2);
    //XMC_CCU8_EnableClock(ccu8_0_HW, PWM_IRQ_TIMER_NUM);
    XMC_CCU8_SLICE_SetTimerValue(PWM_IRQ_TIMER_HW, 0U);

    XMC_POSIF_Enable(HALL_POSIF_HW);
    XMC_POSIF_SetMode(HALL_POSIF_HW, XMC_POSIF_MODE_HALL_SENSOR);
    XMC_POSIF_Init(HALL_POSIF_HW, &HALL_POSIF_config);
    XMC_POSIF_HSC_Init(HALL_POSIF_HW, &HALL_POSIF_HSC_InitHandle);
// mstrens - commented when using a capture instead of an irq to read the ccu4 running timer
    XMC_POSIF_SetInterruptNode(HALL_POSIF_HW, XMC_POSIF_IRQ_EVENT_HALL_INPUT, XMC_POSIF_SR_ID_0);
    XMC_POSIF_EnableEvent(HALL_POSIF_HW, XMC_POSIF_IRQ_EVENT_HALL_INPUT);

    XMC_UART_CH_InitEx(CYBSP_DEBUG_UART_HW, &CYBSP_DEBUG_UART_config, false);
    XMC_UART_CH_SetInputSource(CYBSP_DEBUG_UART_HW, (XMC_UART_CH_INPUT_t)XMC_USIC_CH_INPUT_DX0, CYBSP_DEBUG_UART_DX0_INPUT);
    XMC_UART_CH_SetSamplePoint(CYBSP_DEBUG_UART_HW, 8U);
    XMC_USIC_CH_SetFractionalDivider(CYBSP_DEBUG_UART_HW, XMC_USIC_CH_BRG_CLOCK_DIVIDER_MODE_FRACTIONAL, 289U);
    XMC_USIC_CH_SetBaudrateDivider(CYBSP_DEBUG_UART_HW, XMC_USIC_CH_BRG_CLOCK_SOURCE_DIVIDER, false, 58U, XMC_USIC_CH_BRG_CTQSEL_PDIV, 0U, 15U);
    XMC_USIC_CH_RXFIFO_Configure(CYBSP_DEBUG_UART_HW, CYBSP_DEBUG_UART_RXFIFO_DPTR, CYBSP_DEBUG_UART_RXFIFO_SIZE, CYBSP_DEBUG_UART_RXFIFO_LIMIT);
    XMC_USIC_CH_TXFIFO_Configure(CYBSP_DEBUG_UART_HW, CYBSP_DEBUG_UART_TXFIFO_DPTR, CYBSP_DEBUG_UART_TXFIFO_SIZE, CYBSP_DEBUG_UART_TXFIFO_LIMIT);
    XMC_UART_CH_Start(CYBSP_DEBUG_UART_HW);
        // here is normally the code to init VADC; in this version, it is done by code in file adc.c to use the same setup as infineon
    // VADC_init();
    XMC_WDT_Init(&wdt_0_config);
    //XMC_WDT_Start();
}


// removed by mstrens to test another vadc init
void VADC_initxxx(){
    /* Update group input classes configuration. */
    vadc_0_group0_init_config.class0 = vadc_0_0_iclass_0;
    vadc_0_group1_init_config.class0 = vadc_0_1_iclass_0;
    vadc_0_group0_init_config.class1 = vadc_0_0_iclass_1;
    vadc_0_group1_init_config.class1 = vadc_0_1_iclass_1;
    /* Global configuration. */
    XMC_VADC_GLOBAL_Init(vadc_0_HW, &vadc_0_config);
    XMC_VADC_GROUP_Init(vadc_0_group_0_HW, &vadc_0_group0_init_config);
    XMC_VADC_GROUP_Init(vadc_0_group_1_HW, &vadc_0_group1_init_config);
    XMC_VADC_GROUP_SetPowerMode(vadc_0_group_0_HW, (XMC_VADC_GROUP_POWERMODE_t) XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GROUP_SetPowerMode(vadc_0_group_1_HW, (XMC_VADC_GROUP_POWERMODE_t) XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GLOBAL_EnablePostCalibration(vadc_0_HW, 0U);
    XMC_VADC_GLOBAL_EnablePostCalibration(vadc_0_HW, 1U);
    XMC_VADC_GLOBAL_StartupCalibration(vadc_0_HW);
    XMC_VADC_GLOBAL_BackgroundSetGatingMode(vadc_0_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GLOBAL_BackgroundInit(vadc_0_HW, &vadc_0_background_scan_config);
    /* Request source initializations. */
    XMC_VADC_GROUP_QueueSetGatingMode(vadc_0_group_0_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR0);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR1);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR2);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_0_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR3);
    XMC_VADC_GROUP_QueueInit(vadc_0_group_0_HW, &vadc_0_group_0_queue_config);
    
    /* Initialize input classes. */
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_0_HW,
                               vadc_0_0_iclass_0,
                               XMC_VADC_GROUP_CONV_STD,
                               vadc_0_group_0_ICLASS_0);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_0_HW,
                               vadc_0_0_iclass_0,
                               XMC_VADC_GROUP_CONV_EMUX,
                               vadc_0_group_0_ICLASS_0);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_0_HW,
                               vadc_0_0_iclass_1,
                               XMC_VADC_GROUP_CONV_STD,
                               vadc_0_group_0_ICLASS_1);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_0_HW,
                               vadc_0_0_iclass_1,
                               XMC_VADC_GROUP_CONV_EMUX,
                               vadc_0_group_0_ICLASS_1);
    
    XMC_VADC_GROUP_SetBoundaries(vadc_0_group_0_HW,
                               vadc_0_group_0_LOWER_BOUND_VALUE,
                               vadc_0_group_0_UPPER_BOUND_VALUE);
    
    /* RESULT init. */
    XMC_VADC_GROUP_ResultInit(vadc_0_group_0_HW, (uint32_t)15, &vadc_0_group_0_result_15_config);
    XMC_VADC_GROUP_ResultInit(vadc_0_group_0_HW, (uint32_t)9, &vadc_0_group_0_result_9_config);
    XMC_VADC_GROUP_ResultInit(vadc_0_group_0_HW, (uint32_t)2, &vadc_0_group_0_result_2_config);
    
    /* Insert channels into the background request sources. */
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_0);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_1);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_2);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_3);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_4);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_5);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_6);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_0_HW, vadc_0_group_0_queue_entries_7);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_0_HW, (uint32_t)1, &G0_CH1_CURRENT_P2_8_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_0_HW, (uint32_t)2, &G0_CH2_CURRENT_U_P2_9_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_0_HW, (uint32_t)7, &G0_CH7_TORQUE_P2_2_config);
    /* Request source initializations. */
    XMC_VADC_GROUP_QueueSetGatingMode(vadc_0_group_1_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR0);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR1);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR2);
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(vadc_0_group_1_HW, (XMC_VADC_SR_t) XMC_VADC_SR_GROUP_SR3);
    XMC_VADC_GROUP_QueueInit(vadc_0_group_1_HW, &vadc_0_group_1_queue_config);
    
    /* Initialize input classes. */
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_1_HW,
                               vadc_0_1_iclass_0,
                               XMC_VADC_GROUP_CONV_STD,
                               vadc_0_group_1_ICLASS_0);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_1_HW,
                               vadc_0_1_iclass_0,
                               XMC_VADC_GROUP_CONV_EMUX,
                               vadc_0_group_1_ICLASS_0);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_1_HW,
                               vadc_0_1_iclass_1,
                               XMC_VADC_GROUP_CONV_STD,
                               vadc_0_group_1_ICLASS_1);
    XMC_VADC_GROUP_InputClassInit(vadc_0_group_1_HW,
                               vadc_0_1_iclass_1,
                               XMC_VADC_GROUP_CONV_EMUX,
                               vadc_0_group_1_ICLASS_1);
    
    /* RESULT init. */
    XMC_VADC_GROUP_ResultInit(vadc_0_group_1_HW, (uint32_t)15, &vadc_0_group_1_result_15_config);
    XMC_VADC_GROUP_ResultInit(vadc_0_group_1_HW, (uint32_t)4, &vadc_0_group_1_result_4_config);
    XMC_VADC_GROUP_ResultInit(vadc_0_group_1_HW, (uint32_t)5, &vadc_0_group_1_result_5_config);
    
    /* Insert channels into the background request sources. */
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_0);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_1);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_2);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_3);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_4);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_5);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_6);
    XMC_VADC_GROUP_QueueInsertChannel(vadc_0_group_1_HW, vadc_0_group_1_queue_entries_7);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_1_HW, (uint32_t)0, &G1_CH0_CURRENT_P2_8_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_1_HW, (uint32_t)6, &G1_CH6_BATTERY_P2_4_config);
    /* Channel init. */
    XMC_VADC_GROUP_ChannelInit(vadc_0_group_1_HW, (uint32_t)7, &G1_CH7_THROTTLE_P2_5_config);
}