#include <stdint.h>

#include "esp_attr.h"
#include "hal/gpio_types.h"

#ifndef UTILS_H
#define UTILS_H

uint64_t IRAM_ATTR micros();
void IRAM_ATTR delay_microseconds(uint32_t us);

gpio_num_t uint_to_gpio_num(uint64_t pin);

#endif
