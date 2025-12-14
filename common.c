/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "common.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"

extern volatile uint32_t ui32_ms_counter;

// Function to map a value from one range to another based on given input and output ranges.
// Uses nearest integer rounding for precision.
// Note: Input min has to be smaller than input max.
// Parameters:
// - in: Value to be mapped.
// - in_min: Minimum value of the input range. 
// - in_max: Maximum value of the input range.
// - out_min: Minimum value of the output range.
// - out_max: Maximum value of the output range.
// Returns the mapped value within the specified output range.
uint16_t map_ui16(uint16_t in, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    // If input is out of bounds, clamp it to the nearest boundary value
    if (in < in_min) {return out_min;}
    if (in >= in_max) {return out_max;}

    // Calculate the input and output ranges
    uint16_t in_range = in_max - in_min;

    uint16_t out;
    if (out_max < out_min) {
        out = out_min - (uint16_t)(uint32_t)(((uint32_t)((uint32_t)(uint16_t)(in - in_min) * (uint32_t)(uint16_t)(out_min - out_max)) + (uint32_t)(uint16_t)(in_range/2U)) / in_range);
    } else {
        out = out_min + (uint16_t)(uint32_t)(((uint32_t)((uint32_t)(uint16_t)(in - in_min) * (uint32_t)(uint16_t)(out_max - out_min)) + (uint32_t)(uint16_t)(in_range/2U)) / in_range);
    }
    return out;
}

// Function to map 8bit a values from one range to another based on given input and output ranges.
// Uses floor integer rounding for maximum performance.
// Note: Input min has to be smaller than input max.
// Parameters:
// - in: Value to be mapped.
// - in_min: Minimum value of the input range.
// - in_max: Maximum value of the input range.
// - out_min: Minimum value of the output range.
// - out_max: Maximum value of the output range.
// Returns the mapped value within the specified output range.
uint8_t map_ui8(uint8_t in, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
    // If input is out of bounds, clamp it to the nearest boundary value
    if (in <= in_min) {return out_min;}
    if (in >= in_max) {return out_max;}

    if (out_max < out_min) {
        return out_min - (uint8_t)(uint16_t)((uint16_t)((uint8_t)(in - in_min) * (uint8_t)(out_min - out_max)) / (uint8_t)(in_max - in_min)); // cppcheck-suppress misra-c2012-10.8 ; direct cast to a wider essential to ensure mul in,a usage
    } else {
        return out_min + (uint8_t)(uint16_t)((uint16_t)((uint8_t)(in - in_min) * (uint8_t)(out_max - out_min)) / (uint8_t)(in_max - in_min)); // cppcheck-suppress misra-c2012-10.8 ; direct cast to a wider essential to ensure mul in,a usage
    }
}


uint8_t ui8_min(uint8_t value_a, uint8_t value_b) {
    if (value_a < value_b) {
        return value_a;
    } else {
        return value_b;
    }
}

uint8_t ui8_max(uint8_t value_a, uint8_t value_b) {
    if (value_a > value_b)
        return value_a;
    else
        return value_b;
}




uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha) {
    #define MAX_ALPHA_BITS 4
    const uint8_t ui8_max_alpha = 1<<MAX_ALPHA_BITS;  //16
    if (ui8_alpha < ui8_max_alpha) {
        uint32_t ui32_temp_new = (uint32_t) ui16_new_value * (uint32_t)(ui8_max_alpha - ui8_alpha);
		uint32_t ui32_temp_old = (uint32_t) ui16_old_value * (uint32_t) ui8_alpha;
        uint16_t ui16_filtered_value = (uint16_t)((ui32_temp_new + ui32_temp_old + (ui8_max_alpha>>1)) >> MAX_ALPHA_BITS);

        if (ui16_filtered_value == ui16_old_value) {
            if (ui16_filtered_value < ui16_new_value)
				ui16_filtered_value++;
			else if (ui16_filtered_value > ui16_new_value)
				ui16_filtered_value--;
        }

        return ui16_filtered_value;
    } else {
        return 0;
    }
}

// from here: https://github.com/FxDev/PetitModbus/blob/master/PetitModbus.c
/*
 * Function Name        : CRC16
 * @param[in]           : ui8_data  - Data to Calculate CRC
 * @param[in/out]       : ui16_crc   - Anlik CRC degeri
 * @How to use          : First initial data has to be 0xFFFF.
 */
void crc16(uint8_t ui8_data, uint16_t *ui16_crc) {
    unsigned int i;

    *ui16_crc = *ui16_crc ^ (uint16_t) ui8_data;

    for (i = 8; i > 0; i--) {
        if (*ui16_crc & 0x0001) {
            *ui16_crc = (*ui16_crc >> 1) ^ 0xA001;
        } else {
            *ui16_crc >>= 1;
        }
    }
}

// mstrens :  this function was moved from another file in TSDZ2 and adapted for new mcu
void lights_set_state(uint8_t ui8_state) {
    if (ui8_state) {
        XMC_GPIO_SetOutputHigh(OUT_LIGHT_PORT,OUT_LIGHT_PIN) ; // GPIO_WriteHigh(LIGHTS__PORT, LIGHTS__PIN);
    } else {
        XMC_GPIO_SetOutputLow(OUT_LIGHT_PORT,OUT_LIGHT_PIN) ;  // GPIO_WriteLow(LIGHTS__PORT, LIGHTS__PIN);
    }
}

uint32_t last_action_ms[10]= {0};

// retun true when enlapsed time expired
bool take_action(uint32_t index, uint32_t interval){
    if (index < 10) {
        if ((ui32_ms_counter - last_action_ms[index]) > interval){
            last_action_ms[index] = ui32_ms_counter;
            return true;
        }
    }
    return false;
}



/*
void wait_ms(uint32_t time){
    uint32_t start = ui32_ms_counter;
    uint32_t counter = 0;
    while ( (ui32_ms_counter - start) < time){
        counter++;
    }
    counter = 0;
}
*/
#include "SEGGER_RTT.h"
#include <stdarg.h>
#include <stdint.h>

static void _itoa_fast(int value, char *buf) {
  char tmp[12];
  int i = 0, j = 0;
  unsigned int v;

  if (value < 0) {
    buf[j++] = '-';
    v = (unsigned int)(-value);
  } else {
    v = (unsigned int)value;
  }

  if (v == 0) {
    buf[j++] = '0';
    buf[j] = 0;
    return;
  }

  while (v > 0 && i < (int)sizeof(tmp)) {
    tmp[i++] = '0' + (v % 10);
    v /= 10;
  }

  while (i > 0) buf[j++] = tmp[--i];
  buf[j] = 0;
}

/**
 * @brief Log multi-entiers avec suffixe texte optionnel (non bloquant, sans IRQ off)
 *
 * @param label  Préfixe (ex: "ADC")
 * @param count  Nombre d'entiers à afficher
 * @param tail   Texte final (ex: "\r\n", " END", ou NULL pour rien)
 * @param ...    Les valeurs (int)
 *
 * Exemple:
 *   RTT_LogN_Tail("ADC", 3, "\r\n", 10, 20, 30);
 *   → "ADC=10,20,30\r\n"
 *
 *   RTT_LogN_Tail("DATA", 2, " OK", 1, 2);
 *   → "DATA=1,2 OK"
 */
void RTT_LogN_Tail(const char *label, unsigned int count, const char *tail, ...) {
  char msg[128];
  unsigned int len = 0;
  va_list args;

  // préfixe
  while (*label && len < sizeof(msg) - 1)
    msg[len++] = *label++;

//  msg[len++] = '=';

  // valeurs
  va_start(args, tail);
  for (unsigned int i = 0; i < count && len < sizeof(msg) - 8; i++) {
    int val = va_arg(args, int);
    char numbuf[16];
    _itoa_fast(val, numbuf);

    for (unsigned int j = 0; numbuf[j] && len < sizeof(msg) - 1; j++)
      msg[len++] = numbuf[j];

    if (i < count - 1 && len < sizeof(msg) - 1)
      msg[len++] = ',';
  }
  va_end(args);

  // texte de fin optionnel
  if (tail) {
    while (*tail && len < sizeof(msg) - 1)
      msg[len++] = *tail++;
  }

  SEGGER_RTT_WriteNoLock(0, msg, len);
}

#include "SEGGER_RTT.h"
#include <stdarg.h>
#include <stdint.h>

/**
 * @brief Conversion rapide en chaîne hexadécimale (8 caractères)
 */
static void _itoa_hex(uint32_t value, char *buf) {
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 7; i >= 0; i--) {
    buf[7 - i] = hex[(value >> (i * 4)) & 0xF];
  }
  buf[8] = 0;
}

/**
 * @brief Log multi-entiers au format hexadécimal avec texte final optionnel
 *
 * @param label  Préfixe (ex: "REG")
 * @param count  Nombre d'entiers à afficher
 * @param tail   Texte final optionnel ("\r\n", " OK", NULL, etc.)
 * @param ...    Les valeurs (int)
 *
 * Exemple:
 *   RTT_LogN_TailHex("REG", 3, "\r\n", 0x1234, 0xDEAD, 0xBEEF);
 *   → "REG=00001234,0000DEAD,0000BEEF\r\n"
 */
void RTT_LogN_TailHex(const char *label, unsigned int count, const char *tail, ...) {
  char msg[128];
  unsigned int len = 0;
  va_list args;

  // Préfixe
  while (*label && len < sizeof(msg) - 1)
    msg[len++] = *label++;

  msg[len++] = '=';

  // Valeurs hexadécimales
  va_start(args, tail);
  for (unsigned int i = 0; i < count && len < sizeof(msg) - 10; i++) {
    uint32_t val = (uint32_t)va_arg(args, int); // valeurs signées converties
    char hexbuf[9];
    _itoa_hex(val, hexbuf);

    for (unsigned int j = 0; hexbuf[j] && len < sizeof(msg) - 1; j++)
      msg[len++] = hexbuf[j];

    if (i < count - 1 && len < sizeof(msg) - 1)
      msg[len++] = ',';  // séparateur
  }
  va_end(args);

  // Texte final optionnel
  if (tail) {
    while (*tail && len < sizeof(msg) - 1)
      msg[len++] = *tail++;
  }

  SEGGER_RTT_WriteNoLock(0, msg, len);
}

