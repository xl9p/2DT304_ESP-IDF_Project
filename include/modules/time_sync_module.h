#include "esp_netif_sntp.h"

#ifndef TIME_SYNC_MODULE_H
#define TIME_SYNC_MODULE_H

typedef enum {
  TIME_SYNC_OK,
  TIME_SYNC_ERROR_INIT_FAIL,
  TIME_SYNC_ERROR_DEINIT_FAIL,
  TIME_SYNC_ERROR_SYNC_TIMEOUT,
  TIME_SYNC_ERROR_SYNC_NOT_FINISHED,
  TIME_SYNC_ERROR_SYNC_FAIL,
  TIME_SYNC_ERROR_NOT_SUPPORTED
} time_sync_error;

time_sync_error time_sync_sntp_init();
time_sync_error time_sync_sntp_deint();

time_sync_error time_sync_sntp_sync();

#endif