/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "esp_idf_version.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static adc_oneshot_unit_handle_t adcUnitHandle = NULL;
static adc_cali_handle_t adcCaliHandle = NULL;
static bool adcCalibrationEnabled = false;
#ifdef CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_7; //GPIO35 if ADC1
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
static const adc_channel_t channel = ADC_CHANNEL_1;     // GPIO2 if ADC1
#endif

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
#define NO_OF_SAMPLES   30          //Multisampling

static void adcCalibrationInit(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t caliConfig = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&caliConfig, &adcCaliHandle) == ESP_OK) {
        adcCalibrationEnabled = true;
        DEBUG_PRINTI("ADC calibrated with curve fitting");
        return;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t caliConfig = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&caliConfig, &adcCaliHandle) == ESP_OK) {
        adcCalibrationEnabled = true;
        DEBUG_PRINTI("ADC calibrated with line fitting");
        return;
    }
#endif

    adcCalibrationEnabled = false;
    DEBUG_PRINTW("ADC calibration not available, using raw conversion");
}

float analogReadVoltage(uint32_t pin)
{
    (void)pin;
    uint32_t adc_reading = 0;
    int raw = 0;

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (adc_oneshot_read(adcUnitHandle, channel, &raw) == ESP_OK) {
            adc_reading += (uint32_t)raw;
        }
    }

    adc_reading /= NO_OF_SAMPLES;

    int voltageMv = 0;
    if (adcCalibrationEnabled) {
        if (adc_cali_raw_to_voltage(adcCaliHandle, adc_reading, &voltageMv) != ESP_OK) {
            voltageMv = ((int)adc_reading * 3300) / 4095;
        }
    } else {
        voltageMv = ((int)adc_reading * 3300) / 4095;
    }

    return voltageMv / 1000.0f;
}

void adcInit(void)
{
    if (isInit) {
        return;
    }

    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    if (adc_oneshot_new_unit(&initConfig, &adcUnitHandle) != ESP_OK) {
        DEBUG_PRINTW("adc_oneshot_new_unit failed");
        return;
    }

    adc_oneshot_chan_cfg_t channelConfig = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = atten,
    };

    if (adc_oneshot_config_channel(adcUnitHandle, channel, &channelConfig) != ESP_OK) {
        DEBUG_PRINTW("adc_oneshot_config_channel failed");
        return;
    }

    adcCalibrationInit();

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
