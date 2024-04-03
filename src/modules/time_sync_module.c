#include <string.h>

#include "esp_err.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"

#include "modules/time_sync_module.h"

time_sync_error time_sync_sntp_init() {
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_TIME_SYNC_NTP_SERVER);
  config.start = false;
  config.smooth_sync = false;
  config.wait_for_sync = true;
  ESP_ERROR_CHECK(esp_netif_sntp_init(&config));

  return TIME_SYNC_OK;
}

time_sync_error time_sync_sntp_deint() {
  esp_netif_sntp_deinit();

  return TIME_SYNC_OK;
}

/**
 * Synchronize the internal clock.
 * @note Only waiting mode is supported.
 */
time_sync_error time_sync_sntp_sync() {
#if CONFIG_TIME_SYNC_WAIT_FOR_SYNC
  esp_netif_sntp_start();
  esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(CONFIG_TIME_SYNC_WAIT_TIMEOUT_MS));
  switch (err) {
    case ESP_OK:
      return TIME_SYNC_OK;
    case ESP_ERR_TIMEOUT:
      return TIME_SYNC_ERROR_SYNC_TIMEOUT;
    case ESP_ERR_NOT_FINISHED:
      return TIME_SYNC_ERROR_SYNC_NOT_FINISHED;
  }
#else
  return TIME_SYNC_ERROR_NOT_SUPPORTED;
#endif
  return TIME_SYNC_ERROR_SYNC_FAIL;
}