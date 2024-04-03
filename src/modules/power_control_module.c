#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "modules/power_control_module.h"

#if CONFIG_BATTERY_SENSE_PIN_35
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_7
#elif CONFIG_BATTERY_SENSE_PIN_VP
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_0
#elif CONFIG_BATTERY_SENSE_PIN_VN
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_3
#elif CONFIG_BATTERY_SENSE_PIN_34
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_6
#elif CONFIG_BATTERY_SENSE_PIN_32
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_4
#elif CONFIG_BATTERY_SENSE_PIN_33
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_5
#elif CONFIG_BATTERY_SENSE_PIN_25
#define BATTERY_SENSE_CHANNEL ADC_CHANNEL_8
#endif

#define BATTERY_SENSE_ADC_ATTEN ADC_ATTEN_DB_0

// TODO: Define a pin to control the battery-sense transistor
// TODO: Initiliaze the GPIO pin control

// Line fitting is the only calibration scheme supported on the ESP32.

const static char* TAG = "BatteryControl-Module";

static bool is_sense_chan_calib = false;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_chan_handle = NULL;

/*
 * Multiplication by a coefficient due to the voltage divider.
 */
#define BATTERY_LEVEL_HIGH CONFIG_BATTERY_LEVEL_HIGH * 0.223744292F
#define BATTERY_LEVEL_MEDIUM CONFIG_BATTERY_LEVEL_MEDIUM * 0.223744292F
#define BATTERY_LEVEL_LOW CONFIG_BATTERY_LEVEL_LOW * 0.223744292F
#define BATTERY_LEVEL_CRITICAL CONFIG_BATTERY_LEVEL_CRITICAL * 0.223744292F
#define BATTERY_LEVEL_SHUTDOWN CONFIG_BATTERY_LEVEL_SHUTDOWN * 0.223744292F

static bool battery_control_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t* out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
  }

  *out_handle = handle;
  if (ret == ESP_OK) {
    calibrated = true;
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else if (ret == ESP_ERR_NO_MEM) {
    ESP_LOGE(TAG, "No memory");
  } else {
    ESP_LOGE(TAG, "Invalid arg");
  }

  return calibrated;
}

static void battery_control_adc_calibration_deinit(adc_cali_handle_t handle) {
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
}

battery_control_error battery_control_sense_init() {
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = BATTERY_SENSE_ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_SENSE_CHANNEL, &config));

  is_sense_chan_calib = battery_control_adc_calibration_init(ADC_UNIT_1, BATTERY_SENSE_CHANNEL, BATTERY_SENSE_ADC_ATTEN, &adc1_cali_chan_handle);
  if (is_sense_chan_calib) {
    return BAT_CTRL_OK;
  }
  return BAT_CTRL_ERROR_SENSE_INIT_FAIL;
}

battery_control_error battery_control_sense_deinit() {
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
  if (is_sense_chan_calib) {
    battery_control_adc_calibration_deinit(adc1_cali_chan_handle);
    return BAT_CTRL_OK;
  }
  return BAT_CTRL_ERROR_SENSE_DEINIT_FAIL;
}

battery_status battery_control_sense_voltage() {
  if (is_sense_chan_calib) {
    int voltage_to_eval = 0;
#if CONFIG_BATTERY_SENSE_AVERAGE_MEASURE
    int temp_raw_value = 0;
    int temp_voltage_value = 0;
    for (uint8_t i = 0; i < CONFIG_BATTERY_SENSE_MAX_MEASUREMENTS; i++) {
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_SENSE_CHANNEL, &temp_raw_value));
      ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", 1, BATTERY_SENSE_CHANNEL, temp_raw_value);

      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan_handle, temp_raw_value, &temp_voltage_value));
      ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", 1, BATTERY_SENSE_CHANNEL, temp_voltage_value);

      voltage_to_eval += temp_voltage_value;
    }

    voltage_to_eval = (int)(voltage_to_eval / (CONFIG_BATTERY_SENSE_MAX_MEASUREMENTS));
    ESP_LOGI(TAG, "Average voltage over %d samples: %d mV", CONFIG_BATTERY_SENSE_MAX_MEASUREMENTS, temp_voltage_value);
#else
    int temp_raw_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_SENSE_CHANNEL, &temp_raw_value));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", 1, BATTERY_SENSE_CHANNEL, temp_raw_value);

    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan_handle, temp_raw_value, &voltage_to_eval));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", 1, BATTERY_SENSE_CHANNEL, voltage_to_eval);
#endif

    if (BATTERY_LEVEL_HIGH > voltage_to_eval && voltage_to_eval > BATTERY_LEVEL_MEDIUM) {
      return BAT_STATUS_CHARGE_HIGH;
    } else if (BATTERY_LEVEL_LOW < voltage_to_eval && voltage_to_eval < BATTERY_LEVEL_MEDIUM) {
      return BAT_STATUS_CHARGE_MEDIUM;
    } else if (BATTERY_LEVEL_CRITICAL < voltage_to_eval && voltage_to_eval < BATTERY_LEVEL_LOW) {
      return BAT_STATUS_CHARGE_LOW;
    } else if (BATTERY_LEVEL_SHUTDOWN < voltage_to_eval && voltage_to_eval < BATTERY_LEVEL_CRITICAL) {
      return BAT_STATUS_CHARGE_CRITICAL;
    } else {
      return BAT_STATUS_CHARGE_SHUTDOWN;
    }
  } else {
    return BAT_STATUS_SENSE_NOT_CALIBRATED;
  }
  // Should not be reachable
  return BAT_STATUS_FAIL;
}