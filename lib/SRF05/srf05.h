#include <stdint.h>

#include "freertos/FreeRTOS.h"

#ifndef SRF05_H
#define SRF05_H

#define MAX_SENSOR_DISTANCE_CM ((uint16_t)450)
#define US_ROUNDTRIP_CM (uint16_t)58

#define _MAX_SENSOR_DELAY (uint16_t)5800  // Maximum uS it takes for sensor to start the ping. Default=5800
#define _TRIGGER_WIDTH (uint32_t)10

typedef enum {
  SRF05_OK,
  SRF05_ERROR_TRIGGER_PIN_FAIL,
  SRF05_ERROR_ECHO_PIN_FAIL,
  SRF05_ERROR_UNKNOWN
} SRF05_error;

typedef struct {
  uint64_t triggerPin;
  uint64_t echoPin;
  uint64_t maxTimeout_uS;
  uint16_t maxSensorDistance_cm;
  spinlock_t delay_spinlock;
} srf05_object_t;

SRF05_error srf05_init(srf05_object_t* sensorObject);
SRF05_error srf05_deinit(srf05_object_t* sensorObject);

int srf05_distance_cm(srf05_object_t* sensorObject);

#endif