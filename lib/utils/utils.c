#include "utils.h"
#include "esp_timer.h"

#define NOP() asm volatile("nop")

uint64_t micros() {
  return (uint64_t)(esp_timer_get_time());
}

void delay_microseconds(uint32_t us) {
  uint32_t m = micros();
  if (us) {
    uint32_t e = (m + us);
    if (m > e) {  // overflow
      while (micros() > e) {
        NOP();
      }
    }
    while (micros() < e) {
      NOP();
    }
  }
}

gpio_num_t uint_to_gpio_num(uint64_t pin) {
  for (uint8_t i = 0; i < GPIO_NUM_MAX; i++) {
    if (pin & (1ULL << i)) {
      return (gpio_num_t)i;
    }
  }
  return GPIO_NUM_NC;
}
