#include <stdint.h>

#ifndef POWER_CONTROL_MODULE_H
#define POWER_CONTROL_MODULE_H

typedef enum {
  BAT_STATUS_CHARGE_HIGH,
  BAT_STATUS_CHARGE_MEDIUM,
  BAT_STATUS_CHARGE_LOW,
  BAT_STATUS_CHARGE_CRITICAL,
  BAT_STATUS_CHARGE_SHUTDOWN,
  BAT_STATUS_SENSE_NOT_CALIBRATED,
  BAT_STATUS_FAIL
} battery_status;

typedef enum {
  BAT_CTRL_OK,
  BAT_CTRL_ERROR_SENSE_INIT_FAIL,
  BAT_CTRL_ERROR_SENSE_DEINIT_FAIL,
  BAT_CTRL_ERROR_UNKNOWN
} battery_control_error;

battery_control_error battery_control_sense_init();
battery_control_error battery_control_sense_deinit();

/**
 * There is a certain correlation of battery's charge with respect to its voltage.
 */
battery_status battery_control_sense_voltage();

#endif