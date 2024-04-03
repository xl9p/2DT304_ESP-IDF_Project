#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "miniz.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "modules/mqtt_module.h"
#include "modules/power_control_module.h"
#include "modules/time_sync_module.h"
#include "modules/wifi_module.h"

#include "srf05.h"
#include "utils.h"

#define MAX_IDENTIFICATION_LENGTH 16
#define DEVICE_IDENTIFICATION CONFIG_DEVICE_IDENTIFICATION "\0"
static_assert(sizeof(DEVICE_IDENTIFICATION) <= MAX_IDENTIFICATION_LENGTH,
              "DEVICE_IDENTIFICATION variable is too large");

#define SEND_DATA_PERIODICITY 25000

#define DETECTION_THRESHOLD_DISTANCE_CM 450
#define DETECTION_HOLD_TIMEOUT_MS 1000
#define DETECTION_THRESHOLD_CONFIG_NUM_SAMPLES 15
#define DETECTION_MAX_THRESHOLD_DIFFERENCE 10
#define DETECTION_THRESHOLD_MARGIN 10
#define DETECTION_CONFIG_BUTTON_PIN 4

#define WIFI_CONNECTED_BIT (1 << 0)
#define WIFI_FAIL_BIT (1 << 1)

#define MQTT_DATA_SEND_OK (1 << 0)
#define MQTT_DATA_SEND_FAIL (1 << 1)
#define MQTT_MODULE_FINISHED (1 << 2)
#define MQTT_CONNECTION_OK (1 << 3)
#define MQTT_CONNECTION_FAIL (1 << 4)

#define SNTP_TIME_SYNC_OK (1 << 5)
#define SNTP_TIME_SYNC_FAIL (1 << 6)
#define SNTP_MODULE_FINISHED (1 << 7)

#define ENTRY_SENSOR_INITIALIZED (1 << 0)
#define EXIT_SENSOR_INITIALIZED (1 << 1)

static int detection_threshold_distance = DETECTION_THRESHOLD_DISTANCE_CM;

static SemaphoreHandle_t xWifiAccessMutex, xWifiMustStartSemaphore, xWifiMustStopSemaphore;
static SemaphoreHandle_t xMQTTMustStartSemaphore, xSNTPMustStartSemaphore;
static SemaphoreHandle_t xDetectorConfigTaskSemaphore, xDataShouldSendMutex, xDataReadyForMQTT;
static SemaphoreHandle_t xProcessingQueueBusyMutex, xMQTTQueueBusyMutex, xBacklogQueueBusyMutex;
static SemaphoreHandle_t xDetectorCalibFinished;

static EventGroupHandle_t sWifiStateGroup, sMQTTSNTPStateGroup;
static EventGroupHandle_t sSensorsConfigGroup;

static srf05_object_t srf05_entry, srf05_exit;

static QueueHandle_t to_process_queue, to_mqtt_queue, backlog_queue, data_queue, notifications_queue;

static spinlock_t lock_deinit_detector_interrupt;

static bool are_sensors_configured = false;

typedef struct {
  srf05_object_t* sensor_obj_pointer;
  bool is_triggered;
  uint64_t time_triggered;
} detector_object_t;

typedef struct {
  mqtt_payload* payload_pointer;
  bool time_synced;
} payload_wrapper;

static const char* TAG = "Main-App";

// #region - Protocol related
void send_data_periodic_task(void* pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SEND_DATA_PERIODICITY));
    ESP_LOGI(TAG, "Send Data Periodic task running%s", "");

    xSemaphoreTake(xDataShouldSendMutex, portMAX_DELAY);

    mqtt_payload* data_from_detector;
    if (xQueueReceive(data_queue, &data_from_detector, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(xDataShouldSendMutex);

      xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);
      if (xQueueSend(to_process_queue, &data_from_detector, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "Sent data to processing queue%s", "");
      }
      xSemaphoreGive(xProcessingQueueBusyMutex);
    }
  }
}

void send_notifications_task(void* pvParameters) {
  while (1) {
    mqtt_payload* notification = NULL;
    if (xQueueReceive(notifications_queue, &notification, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);
      if (xQueueSend(to_process_queue, &notification, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "Sent data to processing queue%s", "");
      }
      xSemaphoreGive(xProcessingQueueBusyMutex);
    }
  }
}

void process_data_task(void* pvParameters) {
  while (1) {
    mqtt_payload* test_buf;

    // We have to block on empty queue without extracting any elements
    if (xQueuePeek(to_process_queue, &test_buf, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(xWifiMustStartSemaphore);

      EventBits_t wifi_connection_state = xEventGroupWaitBits(sWifiStateGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(CONFIG_WIFI_MAX_INIT_TIME_MS));
      if (wifi_connection_state & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Unblocking SNTP task %s", "");
        xSemaphoreGive(xSNTPMustStartSemaphore);

        EventBits_t sntp_bits = xEventGroupWaitBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_OK | SNTP_TIME_SYNC_FAIL, pdTRUE, pdFALSE, portMAX_DELAY);
        if (sntp_bits & SNTP_TIME_SYNC_OK) {
          // Updating the time as soon as possible
          time_t time_now;
          time(&time_now);

          ESP_LOGI(TAG, "Unblocking MQTT task %s", "");
          xSemaphoreGive(xMQTTMustStartSemaphore);

          EventBits_t mqtt_bits = xEventGroupWaitBits(sMQTTSNTPStateGroup, MQTT_CONNECTION_OK | MQTT_CONNECTION_FAIL, pdTRUE, pdFALSE, pdMS_TO_TICKS(CONFIG_MQTT_MAX_INIT_TIME_MS));
          if (mqtt_bits & MQTT_CONNECTION_OK) {
            ESP_LOGI(TAG, "MQTT Connected, preparing data%s", "");

            // #region - Transfering data for MQTT Transmission
            int queue_size_back = uxQueueMessagesWaiting(backlog_queue);
            if (queue_size_back > 0) {
              ESP_LOGI(TAG, "Preparing %i items from backlog queue", queue_size_back);

              xSemaphoreTake(xBacklogQueueBusyMutex, portMAX_DELAY);

              payload_wrapper* data_holder = NULL;
              while (xQueueReceive(backlog_queue, &data_holder, 0) == pdTRUE) {
                // If time_synced is false, timestamp is in uS since device boot
                if (!data_holder->time_synced) {
                  ESP_LOGI(TAG, "Entry with un-synced time%s", "");
                  ESP_LOGI(TAG, "Payload Time: %llu", data_holder->payload_pointer->timestamp);

                  uint64_t time_difference = micros() - data_holder->payload_pointer->timestamp;  // Time difference in uS
                  uint64_t adjusted_time = ((uint64_t)time_now) - ((uint64_t)(time_difference / 1000000U));
                  data_holder->payload_pointer->timestamp = adjusted_time;

                  ESP_LOGI(TAG, "Adjusted Time: %llu", adjusted_time);
                }

                ESP_LOGI(TAG, "Adding the entry onto the queue%s", "");

                xSemaphoreTake(xMQTTQueueBusyMutex, portMAX_DELAY);
                if (xQueueSend(to_mqtt_queue, &data_holder->payload_pointer, 0) != pdTRUE) {
                  ESP_LOGE(TAG, "Failed to send data to queue%s", "");
                  return;
                }
                xSemaphoreGive(xMQTTQueueBusyMutex);

                ESP_LOGI(TAG, "Freeing memory, size before: %u", xPortGetFreeHeapSize());
                ESP_LOGI(TAG, "data_holder%s", "");
                vPortFree(data_holder);
                ESP_LOGI(TAG, "Freeing memory, size after: %u", xPortGetFreeHeapSize());

                heap_caps_check_integrity_all(true);
                ESP_LOGI(TAG, "Done%s", "");
              }

              xSemaphoreGive(xBacklogQueueBusyMutex);
            }

            xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);

            mqtt_payload* data_holder = NULL;
            while (xQueueReceive(to_process_queue, &data_holder, 0) == pdTRUE) {
              ESP_LOGI(TAG, "Setting time%s", "");
              data_holder->timestamp = time_now;
              ESP_LOGI(TAG, "Adding the entry onto the queue%s", "");

              xSemaphoreTake(xMQTTQueueBusyMutex, portMAX_DELAY);
              if (xQueueSend(to_mqtt_queue, &data_holder, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send data to queue%s", "");
                return;
              }
              xSemaphoreGive(xMQTTQueueBusyMutex);

              ESP_LOGI(TAG, "Done%s", "");
            }

            xSemaphoreGive(xProcessingQueueBusyMutex);
            // #endregion

            ESP_LOGI(TAG, "Data ready for sending%s", "");
            xSemaphoreGive(xDataReadyForMQTT);
          } else {
            // MQTT Fail
            xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);

            mqtt_payload* data_holder = NULL;
            while (xQueueReceive(to_process_queue, &data_holder, 0) == pdTRUE) {
              // Allocating memory for the wrapper
              ESP_LOGI(TAG, "Allocating memory, size before: %u", xPortGetFreeHeapSize());
              payload_wrapper* new_payload_holder = pvPortMalloc(sizeof(payload_wrapper));
              if (new_payload_holder == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for payload wrapper%s", "");
                continue;
              }
              ESP_LOGI(TAG, "Allocating memory, size after: %u", xPortGetFreeHeapSize());

              new_payload_holder->payload_pointer = data_holder;
              new_payload_holder->payload_pointer->timestamp = time_now;
              new_payload_holder->time_synced = true;

              xSemaphoreTake(xBacklogQueueBusyMutex, portMAX_DELAY);

              if (xQueueSend(backlog_queue, &new_payload_holder, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send data to queue%s", "");
                switch (new_payload_holder->payload_pointer->payload_type) {
                  case MQTT_PAYLOAD_TYPE_DATA: {
                    vPortFree(new_payload_holder->payload_pointer->payload.data_payload);
                    break;
                  }
                  case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
                    vPortFree(new_payload_holder->payload_pointer->payload.notification_payload);
                    break;
                  }
                  default: {
                    ESP_LOGE(TAG, "Unsupported payload type%s", "");
                  }
                }
                vPortFree(new_payload_holder->payload_pointer);
                vPortFree(new_payload_holder);

                vPortFree(data_holder);
                heap_caps_check_integrity_all(true);
                continue;
              }
              heap_caps_check_integrity_all(true);

              xSemaphoreGive(xBacklogQueueBusyMutex);
            }

            xSemaphoreGive(xProcessingQueueBusyMutex);
          }
          // Wait for two bits, one from mqtt, another one from sntp
          EventBits_t mqtt_sntp_bits = xEventGroupWaitBits(sMQTTSNTPStateGroup, MQTT_MODULE_FINISHED | SNTP_MODULE_FINISHED, pdTRUE, pdTRUE, portMAX_DELAY);
          if (mqtt_sntp_bits & (MQTT_MODULE_FINISHED | SNTP_MODULE_FINISHED)) {
            ESP_LOGI(TAG, "SNTP, MQTT finished, stopping wifi");
            xSemaphoreGive(xWifiMustStopSemaphore);
            xEventGroupClearBits(sMQTTSNTPStateGroup, MQTT_MODULE_FINISHED | SNTP_MODULE_FINISHED);
          }
        } else {
          // SNTP Fail
          xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);

          mqtt_payload* data_holder = NULL;
          while (xQueueReceive(to_process_queue, &data_holder, 0) == pdTRUE) {
            // Allocating memory for the wrapper
            ESP_LOGI(TAG, "Allocating memory, size before: %u", xPortGetFreeHeapSize());
            payload_wrapper* new_payload_holder = pvPortMalloc(sizeof(payload_wrapper));
            if (new_payload_holder == NULL) {
              ESP_LOGE(TAG, "Failed to allocate memory for payload wrapper%s", "");
              continue;
            }
            ESP_LOGI(TAG, "Allocating memory, size after: %u", xPortGetFreeHeapSize());

            new_payload_holder->payload_pointer = data_holder;
            new_payload_holder->payload_pointer->timestamp = micros();
            new_payload_holder->time_synced = false;

            xSemaphoreTake(xBacklogQueueBusyMutex, portMAX_DELAY);

            if (xQueueSend(backlog_queue, &new_payload_holder, 0) != pdTRUE) {
              ESP_LOGE(TAG, "Failed to send data to queue%s", "");
              switch (new_payload_holder->payload_pointer->payload_type) {
                case MQTT_PAYLOAD_TYPE_DATA: {
                  vPortFree(new_payload_holder->payload_pointer->payload.data_payload);
                  break;
                }
                case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
                  vPortFree(new_payload_holder->payload_pointer->payload.notification_payload);
                  break;
                }
                default: {
                  ESP_LOGE(TAG, "Unsupported payload type%s", "");
                }
              }
              vPortFree(new_payload_holder->payload_pointer);
              vPortFree(new_payload_holder);

              vPortFree(data_holder);
              heap_caps_check_integrity_all(true);
              continue;
            }
            heap_caps_check_integrity_all(true);

            xSemaphoreGive(xBacklogQueueBusyMutex);
          }

          xSemaphoreGive(xProcessingQueueBusyMutex);

          sntp_bits = xEventGroupWaitBits(sMQTTSNTPStateGroup, SNTP_MODULE_FINISHED, pdFALSE, pdTRUE, portMAX_DELAY);
          if (sntp_bits & SNTP_MODULE_FINISHED) {
            ESP_LOGI(TAG, "SNTP finished, stopping wifi");
            xSemaphoreGive(xWifiMustStopSemaphore);
            xEventGroupClearBits(sMQTTSNTPStateGroup, SNTP_MODULE_FINISHED);
          }
        }
      } else {
        xSemaphoreTake(xProcessingQueueBusyMutex, portMAX_DELAY);

        mqtt_payload* data_holder = NULL;
        while (xQueueReceive(to_process_queue, &data_holder, 0) == pdTRUE) {
          // Allocating memory for the wrapper
          ESP_LOGI(TAG, "Allocating memory, size before: %u", xPortGetFreeHeapSize());
          payload_wrapper* new_payload_holder = pvPortMalloc(sizeof(payload_wrapper));
          if (new_payload_holder == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for payload wrapper%s", "");
            continue;
          }
          ESP_LOGI(TAG, "Allocating memory, size after: %u", xPortGetFreeHeapSize());

          new_payload_holder->payload_pointer = data_holder;
          new_payload_holder->payload_pointer->timestamp = micros();
          new_payload_holder->time_synced = false;

          xSemaphoreTake(xBacklogQueueBusyMutex, portMAX_DELAY);

          if (xQueueSend(backlog_queue, &new_payload_holder, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send data to queue%s", "");
            switch (new_payload_holder->payload_pointer->payload_type) {
              case MQTT_PAYLOAD_TYPE_DATA: {
                vPortFree(new_payload_holder->payload_pointer->payload.data_payload);
                break;
              }
              case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
                vPortFree(new_payload_holder->payload_pointer->payload.notification_payload->description);
                vPortFree(new_payload_holder->payload_pointer->payload.notification_payload);
                break;
              }
              default: {
                ESP_LOGE(TAG, "Unsupported payload type%s", "");
              }
            }
            ESP_LOGI(TAG, "Freeing new_payload_holder->payload_pointer%s", "");
            vPortFree(new_payload_holder->payload_pointer);
            ESP_LOGI(TAG, "Freeing new_payload_holder%s", "");
            vPortFree(new_payload_holder);
            ESP_LOGI(TAG, "Freeing data_holder%s", "");
            vPortFree(data_holder);
            heap_caps_check_integrity_all(true);
            continue;
          }
          heap_caps_check_integrity_all(true);

          xSemaphoreGive(xBacklogQueueBusyMutex);
        }

        xSemaphoreGive(xProcessingQueueBusyMutex);

        ESP_LOGI(TAG, "No WiFi Connection, stopping the WiFi module%s", "");
        xSemaphoreGive(xWifiMustStopSemaphore);
      }
    } else {
      // Queue empty, shouldn't be reachable
      ESP_LOGI(TAG, "Data queue empty%s", "");
    }
  }
}

void mqtt_send_task(void* pvParameters) {
  while (1) {
    xSemaphoreTake(xMQTTMustStartSemaphore, portMAX_DELAY);

    if (xSemaphoreTake(xWifiAccessMutex, portMAX_DELAY) == pdTRUE) {
      mqtt_module_start();
      ESP_ERROR_CHECK(esp_register_shutdown_handler(&mqtt_module_stop));

      mqtt_state m_state = get_mqtt_module_state();

      switch (m_state) {
        case MQTT_STATE_CONNECTED: {
          ESP_LOGI(TAG, "MQTT Connected%s", "");
          xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_CONNECTION_OK);

          xSemaphoreTake(xDataReadyForMQTT, portMAX_DELAY);
          ESP_LOGI(TAG, "MQTT Ready to send%s", "");

          // #region - Send data
          mqtt_payload* data_holder;

          // Not using mutexes to protect the queue!
          while (xQueueReceive(to_mqtt_queue, &data_holder, 0) == pdTRUE) {
            char* topic = NULL;

            switch (data_holder->payload_type) {
              case MQTT_PAYLOAD_TYPE_DATA: {
                topic = CONFIG_MQTT_DATA_TOPIC;
                break;
              }
              case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
                topic = CONFIG_MQTT_NOTIFICATIONS_TOPIC;
                break;
              }
              default: {
                ESP_LOGE(TAG, "Data type not supported%s", "");
                return;
              }
            }

            mqtt_error send_err = mqtt_module_send(topic, data_holder, DEVICE_IDENTIFICATION);

            switch (send_err) {
              case MQTT_ERROR_OK: {
                ESP_LOGI(TAG, "MQTT: Sent data%s", "");

                ESP_LOGI(TAG, "Freeing memory, size before: %u", xPortGetFreeHeapSize());
                switch (data_holder->payload_type) {
                  case MQTT_PAYLOAD_TYPE_DATA: {
                    ESP_LOGI(TAG, "Freeing new_payload_holder->payload_pointer->payload.data_payload%s", "");
                    vPortFree(data_holder->payload.data_payload);
                    break;
                  }
                  case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
                    ESP_LOGI(TAG, "Freeing new_payload_holder->payload_pointer->payload.notification_payload->description%s", "");
                    vPortFree(data_holder->payload.notification_payload->description);
                    // ESP_LOGI(TAG, "Freeing new_payload_holder->payload_pointer->payload.notification_payload%s", "");
                    // vPortFree(data_holder->payload.notification_payload);
                    break;
                  }
                }
                ESP_LOGI(TAG, "Freeing data_holder%s", "");
                vPortFree(data_holder);
                ESP_LOGI(TAG, "Freeing memory, size after: %u", xPortGetFreeHeapSize());
                heap_caps_check_integrity_all(true);

                xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_DATA_SEND_OK);
                break;
              }
              default: {
                payload_wrapper* new_payload_holder = pvPortMalloc(sizeof(payload_wrapper));
                if (new_payload_holder == NULL) {
                  ESP_LOGE(TAG, "Failed to allocate memory for payload wrapper%s", "");
                  return;
                }

                new_payload_holder->payload_pointer = data_holder;

                if (xQueueSend(backlog_queue, &new_payload_holder, 0) != pdTRUE) {
                  ESP_LOGE(TAG, "Failed to send data to queue%s", "");
                  return;
                }
                heap_caps_check_integrity_all(true);
                xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_DATA_SEND_FAIL);
                break;
              }
            }
          }
          // #endregion
          ESP_LOGI(TAG, "MQTT Send finished%s", "");
          break;
        }
        case MQTT_STATE_DISCONNECTED: {
          ESP_LOGI(TAG, "MQTT Disconnected%s", "");
          xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_CONNECTION_FAIL);
          break;
        }
        case MQTT_STATE_INVALID: {
          ESP_LOGI(TAG, "MQTT Invalid State%s", "");
          xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_CONNECTION_FAIL);
          break;
        }
        default: {
          ESP_LOGI(TAG, "Unexpected%s", "");
          xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_CONNECTION_FAIL);
          break;
        }
      }

      mqtt_module_stop();
      ESP_ERROR_CHECK(esp_unregister_shutdown_handler(&mqtt_module_stop));

      xEventGroupSetBits(sMQTTSNTPStateGroup, MQTT_MODULE_FINISHED);

      xSemaphoreGive(xWifiAccessMutex);
    } else {
      // Timeout, should be unreachable
    }
  }
}

void sntp_sync_time_task(void* pvParameters) {
  while (1) {
    xSemaphoreTake(xSNTPMustStartSemaphore, portMAX_DELAY);

    if (xSemaphoreTake(xWifiAccessMutex, portMAX_DELAY) == pdTRUE) {
      time_t time_now;

      time_sync_error err = time_sync_sntp_init();

      if (!test_bool1) {
        err = TIME_SYNC_ERROR_INIT_FAIL;
        test_bool1 = true;
      }
      switch (err) {
        case TIME_SYNC_OK: {
          ESP_LOGI(TAG, "SNTP client initialized%s", "");

          err = time_sync_sntp_sync();
          switch (err) {
            case TIME_SYNC_OK: {
              time(&time_now);
              ESP_LOGI(TAG, "Internal clock synchronized, current time: %lld", time_now);
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_OK);
              break;
            }
            case TIME_SYNC_ERROR_SYNC_TIMEOUT: {
              ESP_LOGI(TAG, "Internal clock synchronization timed out%s", "");
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
              break;
            }
            case TIME_SYNC_ERROR_SYNC_NOT_FINISHED: {
              ESP_LOGI(TAG, "Internal clock synchronization wasn't finished due to another sync process active%s", "");
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
              break;
            }
            case TIME_SYNC_ERROR_SYNC_FAIL: {
              ESP_LOGI(TAG, "Internal clock synchronization failed%s", "");
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
              break;
            }
            case TIME_SYNC_ERROR_NOT_SUPPORTED: {
              ESP_LOGI(TAG, "Not supported%s", "");
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
              break;
            }
            default: {
              xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
              break;
            }
          }
          break;
        }
        default: {
          xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_TIME_SYNC_FAIL);
          ESP_LOGI(TAG, "An error ocurred%s", "");
          break;
        }
      }

      err = time_sync_sntp_deint();
      ESP_LOGI(TAG, "SNTP client deinitialized%s", "");

      xEventGroupSetBits(sMQTTSNTPStateGroup, SNTP_MODULE_FINISHED);
      xSemaphoreGive(xWifiAccessMutex);
      ESP_LOGI(TAG, "SNTP Task left%s", "");
    } else {
      // Timeout, should be unreachable
    }
  }
}

void wifi_connection_task(void* pvParameters) {
  while (1) {
    xSemaphoreTake(xWifiMustStartSemaphore, portMAX_DELAY);
    wifi_module_start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&wifi_module_stop));
    ESP_LOGI(TAG, "WiFi State: %s", get_wifi_module_state_s());

    wifi_state w_state = get_wifi_module_state();

    switch (w_state) {
      case WIFI_STATE_CONNECTED:
        xEventGroupSetBits(sWifiStateGroup, WIFI_CONNECTED_BIT);
        break;
      default:
        xEventGroupSetBits(sWifiStateGroup, WIFI_FAIL_BIT);
        break;
    }

    if (xSemaphoreTake(xWifiMustStopSemaphore, portMAX_DELAY) == pdTRUE) {
      // Should be executed even if the wifi didn't connect to any AP
      wifi_module_stop();
      ESP_ERROR_CHECK(esp_unregister_shutdown_handler(&wifi_module_stop));
      xEventGroupClearBits(sWifiStateGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }
  }
}
// #endregion

// #region - Sensor related
void IRAM_ATTR detector_config_task_isr_handler(void* arg) {
  // Avoiding interrupt nesting (may be caused by contact debounce for example)
  if (!are_sensors_configured) {
    taskENTER_CRITICAL(&lock_deinit_detector_interrupt);
    are_sensors_configured = true;
    xSemaphoreGiveFromISR(xDetectorConfigTaskSemaphore, NULL);
    taskEXIT_CRITICAL(&lock_deinit_detector_interrupt);
  }
}

void init_config_button_interrupt() {
  portMUX_INITIALIZE(&lock_deinit_detector_interrupt);

  ESP_LOGI(TAG, "Initializing configuration button interrupt on pin: %i", DETECTION_CONFIG_BUTTON_PIN);

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
  io_conf.pin_bit_mask = (1ULL << DETECTION_CONFIG_BUTTON_PIN);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_set_intr_type(DETECTION_CONFIG_BUTTON_PIN, GPIO_INTR_LOW_LEVEL));  // gpio_install_isr_service(ESP_INTR_FLAG_HIGH)
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(DETECTION_CONFIG_BUTTON_PIN, detector_config_task_isr_handler, NULL));
  ESP_LOGI(TAG, "Interrupt init finished%s", "");
}

void deinit_config_button_interrupt() {
  gpio_isr_handler_remove(DETECTION_CONFIG_BUTTON_PIN);
  gpio_intr_disable(DETECTION_CONFIG_BUTTON_PIN);
  gpio_reset_pin(DETECTION_CONFIG_BUTTON_PIN);
  are_sensors_configured = false;
}

void detector_config_task(void* pvParameters) {
  if (xSemaphoreTake(xDetectorConfigTaskSemaphore, portMAX_DELAY) == pdTRUE) {
    ESP_LOGI(TAG, "Deinitializing configuration button interrupt on pin: %i", DETECTION_CONFIG_BUTTON_PIN);
    deinit_config_button_interrupt();
    ESP_LOGI(TAG, "Starting sensors configuration sequence%s", "");

    // #region - Sensor initialization
    spinlock_t lock_entry;
    portMUX_INITIALIZE(&lock_entry);

    srf05_entry.echoPin = (1ULL << CONFIG_HC_SRF05_ENTRY_ECHO_PIN);
    srf05_entry.triggerPin = (1ULL << CONFIG_HC_SRF05_ENTRY_TRIGGER_PIN);
    srf05_entry.maxSensorDistance_cm = 450;
    srf05_entry.delay_spinlock = lock_entry;

    SRF05_error err = srf05_init(&srf05_entry);

    switch (err) {
      case SRF05_OK:
        ESP_LOGI(TAG, "Entry Sensor Init: %s", "OK");
        break;
      case SRF05_ERROR_ECHO_PIN_FAIL:
        ESP_LOGE(TAG, "Entry Sensor Init: %s", "Echo pin fail");
        break;
      case SRF05_ERROR_TRIGGER_PIN_FAIL:
        ESP_LOGE(TAG, "Entry Sensor Init: %s", "Trigger pin fail");
        break;
      case SRF05_ERROR_UNKNOWN:
        ESP_LOGE(TAG, "Entry Sensor Init: %s", "Unknown");
        break;
    }

    spinlock_t lock_exit;
    portMUX_INITIALIZE(&lock_exit);

    srf05_exit.echoPin = (1ULL << CONFIG_HC_SRF05_EXIT_ECHO_PIN);
    srf05_exit.triggerPin = (1ULL << CONFIG_HC_SRF05_EXIT_TRIGGER_PIN);
    srf05_exit.maxSensorDistance_cm = 450;
    srf05_exit.delay_spinlock = lock_exit;

    err = srf05_init(&srf05_exit);

    switch (err) {
      case SRF05_OK:
        ESP_LOGI(TAG, "Exit Sensor Init: %s", "OK");
        break;
      case SRF05_ERROR_ECHO_PIN_FAIL:
        ESP_LOGE(TAG, "Exit Sensor Init: %s", "Echo pin fail");
        break;
      case SRF05_ERROR_TRIGGER_PIN_FAIL:
        ESP_LOGE(TAG, "Exit Sensor Init: %s", "Trigger pin fail");
        break;
      case SRF05_ERROR_UNKNOWN:
        ESP_LOGE(TAG, "Exit Sensor Init: %s", "Unknown");
        break;
    }

    int samples_entry_accumulator = 0;
    int samples_exit_accumulator = 0;

    uint8_t counter = 0;

    ESP_LOGI(TAG, "Starting threshold distance calibration%s", "");

    while (counter < DETECTION_THRESHOLD_CONFIG_NUM_SAMPLES) {
      int entry_sample = srf05_distance_cm(&srf05_entry);
      samples_entry_accumulator += entry_sample;
      ESP_LOGI(TAG, "Entry: %icm", entry_sample);
      vTaskDelay(pdMS_TO_TICKS(350));

      int exit_sample = srf05_distance_cm(&srf05_exit);
      samples_exit_accumulator += exit_sample;
      ESP_LOGI(TAG, "Exit: %icm", exit_sample);
      vTaskDelay(pdTICKS_TO_MS(350));
      counter++;
      ESP_LOGW(TAG, "Progress: %u/%i", counter, DETECTION_THRESHOLD_CONFIG_NUM_SAMPLES);
    }

    int entry_threshold_distance = (int)(samples_entry_accumulator / DETECTION_THRESHOLD_CONFIG_NUM_SAMPLES);
    int exit_threshold_distance = (int)(samples_exit_accumulator / DETECTION_THRESHOLD_CONFIG_NUM_SAMPLES);

    ESP_LOGI(TAG, "Threshold distance calibration result: Entry: %icm; Exit: %icm", entry_threshold_distance, exit_threshold_distance);

    if (abs((entry_threshold_distance - exit_threshold_distance)) > DETECTION_MAX_THRESHOLD_DIFFERENCE) {
      // Send misconfiguration notification
      ESP_LOGE(TAG, "Difference between threshold distances is too big, cannot proceed, check your setup%s", "");
    } else {
      detection_threshold_distance = ((int)((entry_threshold_distance + exit_threshold_distance) / 2)) - DETECTION_THRESHOLD_MARGIN;
      ESP_LOGI(TAG, "Calibrated threshold distance: %icm", detection_threshold_distance);
      xEventGroupSetBits(sSensorsConfigGroup, ENTRY_SENSOR_INITIALIZED | EXIT_SENSOR_INITIALIZED);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
    xSemaphoreGive(xDetectorCalibFinished);
  }
  vTaskDelete(NULL);
  // #endregion
}

void motion_detection_task(void* pvParameters) {
  detector_object_t entry_detector = {
      .sensor_obj_pointer = &srf05_entry,
      .is_triggered = false,
      .time_triggered = 0U,
  };
  detector_object_t exit_detector = {
      .sensor_obj_pointer = &srf05_exit,
      .is_triggered = false,
      .time_triggered = 0U,
  };

  int entry_distance, exit_distance;

  mqtt_data_payload data_in_use = {
      .ins = 0U,
      .outs = 0U,
  };
  // #region - Scanning
  while (1) {
    if (xSemaphoreTake(xDataShouldSendMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!entry_detector.is_triggered) {
        entry_distance = srf05_distance_cm(entry_detector.sensor_obj_pointer);
        ESP_LOGI(TAG, "Distance Entry: %i", entry_distance);

        if (exit_distance > 0 && entry_distance < detection_threshold_distance) {
          entry_detector.is_triggered = true;
          entry_detector.time_triggered = micros();
          ESP_LOGI(TAG, "Entry time triggered: %llu", entry_detector.time_triggered);  // May be preempted before, micros() will be inaccurate
        }
        vTaskDelay(pdMS_TO_TICKS(35));
      } else {
        if (micros() > (entry_detector.time_triggered + DETECTION_HOLD_TIMEOUT_MS * 1000U)) {
          entry_detector.is_triggered = false;
          entry_detector.time_triggered = 0;
          ESP_LOGI(TAG, "Entry detector, hold reset%s", "");
        }
      }

      if (!exit_detector.is_triggered) {
        exit_distance = srf05_distance_cm(exit_detector.sensor_obj_pointer);
        ESP_LOGI(TAG, "Distance Exit: %i", exit_distance);

        if (exit_distance > 0 && exit_distance < detection_threshold_distance) {
          exit_detector.is_triggered = true;
          exit_detector.time_triggered = micros();
          ESP_LOGI(TAG, "Exit time triggered: %llu", exit_detector.time_triggered);
        }
        vTaskDelay(pdMS_TO_TICKS(35));
      } else {
        if (micros() > (exit_detector.time_triggered + DETECTION_HOLD_TIMEOUT_MS * 1000U)) {
          exit_detector.is_triggered = false;
          exit_detector.time_triggered = 0;
          ESP_LOGI(TAG, "Exit detector, hold reset%s", "");
        }
      }

      // #region - Detected
      if (entry_detector.is_triggered && exit_detector.is_triggered) {
        if (entry_detector.time_triggered > exit_detector.time_triggered) {
          data_in_use.outs++;
          ESP_LOGI(TAG, "Motion detected: Out%s", "");
        } else if (entry_detector.time_triggered < exit_detector.time_triggered) {
          data_in_use.ins++;
          ESP_LOGI(TAG, "Motion detected: In%s", "");
        }
        entry_detector.is_triggered = false;
        entry_detector.time_triggered = 0U;

        exit_detector.is_triggered = false;
        exit_detector.time_triggered = 0U;

        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      // #endregion
      xSemaphoreGive(xDataShouldSendMutex);
      vTaskDelay(pdMS_TO_TICKS(20));
    } else {
      ESP_LOGI(TAG, "Sending data to the periodic task%s", "");
      ESP_LOGI(TAG, "Allocating memory, size before: %u", xPortGetFreeHeapSize());
      // #region - Copying gathered data
      mqtt_data_payload* data_to_send = pvPortMalloc(sizeof(mqtt_data_payload));
      if (data_to_send == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory of data payload%s", "");
        return;
      }

      mqtt_payload* payload_to_send = pvPortMalloc(sizeof(mqtt_payload));
      if (payload_to_send == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
        vPortFree(data_to_send);
        return;
      }

      data_to_send->ins = data_in_use.ins;
      data_to_send->outs = data_in_use.outs;

      payload_to_send->payload_type = MQTT_PAYLOAD_TYPE_DATA;
      payload_to_send->payload.data_payload = data_to_send;
      payload_to_send->timestamp = 0U;

      // #endregion
      ESP_LOGI(TAG, "Allocating memory, size after: %u", xPortGetFreeHeapSize());
      // #region - Reseting old data
      data_in_use.ins = 0U;
      data_in_use.outs = 0U;
      // #endregion

      // Sending pointer of a pointer
      // The memory is dynamically allocated on heap
      // It has to be freed at some point.
      if (xQueueSend(data_queue, &payload_to_send, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send data to queue%s", "");
        vPortFree(data_to_send);
        vPortFree(payload_to_send);
        return;
      }

      heap_caps_check_integrity_all(true);
    }
  }
  // #endregion
}

void battery_sense_task(void* pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();

  battery_status status;
  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONFIG_BATTERY_SENSE_PERIODICITY));
    ESP_LOGI(TAG, "Starting battery sense task%s", "");

    battery_control_error err = battery_control_sense_init();

    switch (err) {
      case BAT_CTRL_OK:
        ESP_LOGI(TAG, "Battery sense initialized on pin: %d", CONFIG_BATTERY_SENSE_PIN_35);
        break;
      default: {
        ESP_LOGE(TAG, "Failed to initialize battery sense on pin: %d", CONFIG_BATTERY_SENSE_PIN_35);
        mqtt_notification_payload* notification_payload = pvPortMalloc(sizeof(mqtt_notification_payload));
        if (notification_payload == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          return;
        }

        mqtt_payload* notification_to_send = pvPortMalloc(sizeof(mqtt_payload));
        if (notification_to_send == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          vPortFree(notification_to_send);
          return;
        }
        strcpy(notification_payload->description, "BAT_SENSE_PIN_INIT_FAIL");
        notification_to_send->payload_type = MQTT_PAYLOAD_TYPE_NOTIFICATION;
        notification_to_send->timestamp = 0U;
        notification_to_send->payload.notification_payload = notification_payload;
        break;

        if (xQueueSend(notifications_queue, &notification_to_send, portMAX_DELAY) != pdPASS) {
          ESP_LOGE(TAG, "Failed to send data to queue%s", "");
          vPortFree(notification_payload);
          vPortFree(notification_to_send);
          return;
        }
      }
    }

    status = battery_control_sense_voltage();

    switch (status) {
      case BAT_STATUS_CHARGE_HIGH:
        ESP_LOGI(TAG, "Battery level is high%s", "");
        break;
      case BAT_STATUS_CHARGE_MEDIUM:
        ESP_LOGI(TAG, "Battery level is medium%s", "");
        break;
      case BAT_STATUS_CHARGE_LOW:
        ESP_LOGI(TAG, "Battery level is low%s", "");
        break;
      default: {
        mqtt_notification_payload* notification_payload = pvPortMalloc(sizeof(mqtt_notification_payload));
        if (notification_payload == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          return;
        }

        mqtt_payload* notification_to_send = pvPortMalloc(sizeof(mqtt_payload));
        if (notification_to_send == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          vPortFree(notification_to_send);
          return;
        }
        strcpy(notification_payload->description, "BAT_CHARGE_LOW");
        notification_to_send->payload_type = MQTT_PAYLOAD_TYPE_NOTIFICATION;
        notification_to_send->timestamp = 0U;
        notification_to_send->payload.notification_payload = notification_payload;
        break;

        if (xQueueSend(notifications_queue, &notification_to_send, portMAX_DELAY) != pdPASS) {
          ESP_LOGE(TAG, "Failed to send data to queue%s", "");
          vPortFree(notification_payload);
          vPortFree(notification_to_send);
          return;
        }
      }
    }
    err = battery_control_sense_deinit();

    switch (err) {
      case BAT_CTRL_OK:
        ESP_LOGI(TAG, "Battery sense deinitialized on pin: %d", CONFIG_BATTERY_SENSE_PIN_35);
        break;
      default: {
        mqtt_notification_payload* notification_payload = pvPortMalloc(sizeof(mqtt_notification_payload));
        if (notification_payload == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          return;
        }

        mqtt_payload* notification_to_send = pvPortMalloc(sizeof(mqtt_payload));
        if (notification_to_send == NULL) {
          ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
          vPortFree(notification_to_send);
          return;
        }
        ESP_LOGE(TAG, "Failed to deinitialize battery sense on pin: %d", CONFIG_BATTERY_SENSE_PIN_35);

        strcpy(notification_payload->description, "BAT_SENSE_PIN_DEINIT_FAIL");
        notification_to_send->payload_type = MQTT_PAYLOAD_TYPE_NOTIFICATION;
        notification_to_send->timestamp = 0U;
        notification_to_send->payload.notification_payload = notification_payload;

        if (xQueueSend(notifications_queue, &notification_to_send, portMAX_DELAY) != pdPASS) {
          ESP_LOGE(TAG, "Failed to send data to queue%s", "");
          vPortFree(notification_payload);
          vPortFree(notification_to_send);
          return;
        }
      }
    }
    heap_caps_check_integrity_all(true);
  }
}
// #endregion

void init_tasks(void* pvParameters) {
  // Hardware/Software not ready
  // xTaskCreatePinnedToCore(battery_sense_task, "Bat Sense Task", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(detector_config_task, "Det Conf Task", 2048, NULL, 2, NULL, 1);

  ESP_LOGI(TAG, "Waiting for calibration to finsih %s", "");

  xSemaphoreTake(xDetectorCalibFinished, pdMS_TO_TICKS(300000));

  xTaskCreatePinnedToCore(send_notifications_task, "Process Ntfs T", 2048, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(sntp_sync_time_task, "SNTP Sync T", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(mqtt_send_task, "MQTT Send T", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(wifi_connection_task, "Wifi Conn T", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(process_data_task, "Process Data T", 4096, NULL, 3, NULL, 0);

  EventBits_t sensors_bits = xEventGroupWaitBits(sSensorsConfigGroup, ENTRY_SENSOR_INITIALIZED | EXIT_SENSOR_INITIALIZED, pdFALSE, pdTRUE, pdMS_TO_TICKS(20000));

  if (sensors_bits & (ENTRY_SENSOR_INITIALIZED | EXIT_SENSOR_INITIALIZED)) {
    xTaskCreatePinnedToCore(motion_detection_task, "Detection Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(send_data_periodic_task, "Send Periodic T", 2048, NULL, 4, NULL, 0);

    // #region - Sensor init ok notification
    mqtt_notification_payload* notification_payload = pvPortMalloc(sizeof(mqtt_notification_payload));
    if (notification_payload == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
      return;
    }

    mqtt_payload* notification_to_send = pvPortMalloc(sizeof(mqtt_payload));
    if (notification_to_send == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
      vPortFree(notification_payload);
      return;
    }

    strcpy(notification_payload->description, "SENSORS_INIT_OK");
    notification_to_send->payload_type = MQTT_PAYLOAD_TYPE_NOTIFICATION;
    notification_to_send->timestamp = 0U;
    notification_to_send->payload.notification_payload = notification_payload;

    if (xQueueSend(notifications_queue, &notification_to_send, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Failed to send data to queue%s", "");
      vPortFree(notification_payload);
      vPortFree(notification_to_send);
      return;
    }
    // #endregion

  } else {
    // #region - Sensor init failed notification
    mqtt_notification_payload* notification_payload = pvPortMalloc(sizeof(mqtt_notification_payload));
    if (notification_payload == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
      return;
    }

    mqtt_payload* notification_to_send = pvPortMalloc(sizeof(mqtt_payload));
    if (notification_to_send == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload%s", "");
      vPortFree(notification_payload);
      return;
    }

    strcpy(notification_payload->description, "SENSORS_INIT_FAIL");
    notification_to_send->payload_type = MQTT_PAYLOAD_TYPE_NOTIFICATION;
    notification_to_send->timestamp = 0U;
    notification_to_send->payload.notification_payload = notification_payload;

    if (xQueueSend(notifications_queue, &notification_to_send, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Failed to send data to queue%s", "");
      vPortFree(notification_payload);
      vPortFree(notification_to_send);
      return;
    }
    // #endregion
  }

  vTaskDelete(NULL);
}

/**
 * Main application entry
 */
void app_main(void) {
  ESP_LOGI(TAG, "Startup..");
  ESP_LOGI(TAG, "Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

  esp_log_level_set("Main-App", ESP_LOG_VERBOSE);

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  init_config_button_interrupt();

  // #region - Program initialization

  // #region - Queue initialization
  backlog_queue = xQueueCreate(32, sizeof(struct payload_wrapper*));
  if (backlog_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create backlog queue%s", "");
    return;
  }
  to_mqtt_queue = xQueueCreate(64, sizeof(struct mqtt_payload*));
  if (to_mqtt_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create mqtt queue%s", "");
    return;
  }
  to_process_queue = xQueueCreate(20, sizeof(struct mqtt_payload*));
  if (to_process_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create processing queue%s", "");
    return;
  }
  notifications_queue = xQueueCreate(16, sizeof(struct mqtt_payload*));
  if (notifications_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create notification queue%s", "");
    return;
  }
  data_queue = xQueueCreate(4, sizeof(struct mqtt_payload*));
  if (data_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create data queue%s", "");
    return;
  }
  // #endregion

  // #region - Event groups initialization
  sWifiStateGroup = xEventGroupCreate();
  if (sWifiStateGroup == NULL) {
    ESP_LOGE(TAG, "Failed to create sWifiStateGroup%s", "");
    return;
  }
  sMQTTSNTPStateGroup = xEventGroupCreate();
  if (sMQTTSNTPStateGroup == NULL) {
    ESP_LOGE(TAG, "Failed to create sMQTTSNTPStateGroup%s", "");
    return;
  }
  sSensorsConfigGroup = xEventGroupCreate();
  if (sSensorsConfigGroup == NULL) {
    ESP_LOGI(TAG, "Failed to create sSensorConfigGroup%s", "");
    return;
  }
  // #endregion

  // #region - Semaphores initialization
  xWifiAccessMutex = xSemaphoreCreateMutex();
  if (xWifiAccessMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create xWifiAccessMutex%s", "");
    return;
  }
  xDataShouldSendMutex = xSemaphoreCreateMutex();
  if (xDataShouldSendMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create xDataShouldSendMutex%s", "");
    return;
  }
  xProcessingQueueBusyMutex = xSemaphoreCreateMutex();
  if (xProcessingQueueBusyMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create xProcessingQueueBusyMutex%s", "");
    return;
  }
  xMQTTQueueBusyMutex = xSemaphoreCreateMutex();
  if (xMQTTQueueBusyMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create xMQTTQueueBusyMutex%s", "");
    return;
  }
  xBacklogQueueBusyMutex = xSemaphoreCreateMutex();
  if (xBacklogQueueBusyMutex == NULL) {
    ESP_LOGE(TAG, "Failed to create xBacklogQueueBusyMutex%s", "");
    return;
  }
  xDetectorCalibFinished = xSemaphoreCreateBinary();
  if (xDetectorCalibFinished == NULL) {
    ESP_LOGE(TAG, "Failed to create xDetectorCalibFinished%s", "");
    return;
  }
  xMQTTMustStartSemaphore = xSemaphoreCreateBinary();
  if (xMQTTMustStartSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create xMQTTMustStartSemaphore%s", "");
    return;
  }
  xSNTPMustStartSemaphore = xSemaphoreCreateBinary();
  if (xSNTPMustStartSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create xSNTPMustStartSemaphore%s", "");
    return;
  }
  xDataReadyForMQTT = xSemaphoreCreateBinary();
  if (xDataReadyForMQTT == NULL) {
    ESP_LOGE(TAG, "Failed to create xDataReadyForMQTT%s", "");
    return;
  }
  xWifiMustStartSemaphore = xSemaphoreCreateBinary();
  if (xWifiMustStartSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create xWifiMustStartSemaphore%s", "");
    return;
  }
  xWifiMustStopSemaphore = xSemaphoreCreateBinary();
  if (xWifiMustStopSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create xWifiMustStopSemaphore%s", "");
    return;
  }
  xDetectorConfigTaskSemaphore = xSemaphoreCreateBinary();
  if (xDetectorConfigTaskSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create xDetectorConfigTaskSemaphore%s", "");
    return;
  }
  // #endregion

  xTaskCreate(init_tasks, "Init. T", 2048, NULL, 5, NULL);
  // #endregion
}
