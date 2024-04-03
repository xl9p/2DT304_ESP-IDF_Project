#include "driver/gpio.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "srf05.h"
#include "utils.h"

SRF05_error srf05_init(srf05_object_t* sensorObject) {
  sensorObject->maxSensorDistance_cm = (sensorObject->maxSensorDistance_cm <= MAX_SENSOR_DISTANCE_CM) ? sensorObject->maxSensorDistance_cm : MAX_SENSOR_DISTANCE_CM;
  sensorObject->maxTimeout_uS = (sensorObject->maxSensorDistance_cm * US_ROUNDTRIP_CM) + _MAX_SENSOR_DELAY;

  gpio_num_t triggerPin = uint_to_gpio_num(sensorObject->triggerPin);
  gpio_num_t echoPin = uint_to_gpio_num(sensorObject->echoPin);

  if (GPIO_IS_VALID_GPIO(triggerPin) && (GPIO_IS_VALID_OUTPUT_GPIO(triggerPin) || GPIO_IS_VALID_DIGITAL_IO_PAD(triggerPin))) {
    sensorObject->triggerPin = (1ULL << triggerPin);
  } else {
    return SRF05_ERROR_TRIGGER_PIN_FAIL;
  }

  if (GPIO_IS_VALID_GPIO(echoPin) && GPIO_IS_VALID_DIGITAL_IO_PAD(echoPin)) {
    sensorObject->echoPin = (1ULL << echoPin);
  } else {
    return SRF05_ERROR_ECHO_PIN_FAIL;
  }

  gpio_config_t echoPinConfig = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = sensorObject->echoPin,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE};

  gpio_config_t triggerPinConfig = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = sensorObject->triggerPin,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE};

  ESP_ERROR_CHECK(gpio_config(&echoPinConfig));
  ESP_ERROR_CHECK(gpio_config(&triggerPinConfig));

  return SRF05_OK;
}

SRF05_error srf05_deinit(srf05_object_t* sensorObject) {
  sensorObject->maxSensorDistance_cm = MAX_SENSOR_DISTANCE_CM;

  gpio_num_t triggerPin = uint_to_gpio_num(sensorObject->triggerPin);
  gpio_num_t echoPin = uint_to_gpio_num(sensorObject->echoPin);

  gpio_reset_pin(triggerPin);
  gpio_reset_pin(echoPin);

  sensorObject->triggerPin = 0ULL;
  sensorObject->echoPin = 0ULL;

  return SRF05_OK;
}

/**
 * @brief Ping and measure the distance by analyzing how long it took for the echo to return.
 *
 * @note This method is blocking and in theory may be preempted by another task/interrupt.
 * In order to guarantee a proper functionality of this function it is advised to:
 * - Use the API only on one core simultaneously.
 * - If execution on only one core is ensured, protect the function call with portDISABLE_INTERRUPTS()/portENABLE_INTERRUPTS().
 * - Or alternatively - use semaphores to protect the function call.
 *
 * @returns
 *        - -1 if unsuccessful
 */
int srf05_distance_cm(srf05_object_t* sensorObject) {
  gpio_num_t triggerPin = uint_to_gpio_num(sensorObject->triggerPin);
  gpio_num_t echoPin = uint_to_gpio_num(sensorObject->echoPin);

  // Reset output pin
  gpio_set_level(triggerPin, 0);

  if (gpio_get_level(echoPin)) {
    return -1;  // Echo ping from previous trigger signal, a while loop better?
  }
  // If issues - Implement the library using tasks and the scheduler?
  taskENTER_CRITICAL(&sensorObject->delay_spinlock);
  // Set output pin high
  gpio_set_level(triggerPin, 1);
  // Delay for _TRIGGER_WIDTH
  delay_microseconds(_TRIGGER_WIDTH);
  // Set output pin low
  gpio_set_level(triggerPin, 0);

  uint64_t start = micros();

  while (gpio_get_level(echoPin) == 0) {
    if ((micros() - start) >= sensorObject->maxTimeout_uS) {
      taskEXIT_CRITICAL(&sensorObject->delay_spinlock);
      // Check if the echo pin is low for too long
      return -2;
    }
  }

  start = micros();

  while (gpio_get_level(echoPin) == 1) {
    if ((micros() - start) >= sensorObject->maxTimeout_uS) {
      taskEXIT_CRITICAL(&sensorObject->delay_spinlock);
      // Check if the echo pin is high for too long with respect to the maximum distance set
      return -3;
    }
  }
  taskEXIT_CRITICAL(&sensorObject->delay_spinlock);

  return (int)((micros() - start) / US_ROUNDTRIP_CM);
}