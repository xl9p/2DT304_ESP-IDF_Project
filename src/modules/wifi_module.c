/**
 * Based on the design from the ESP-IDF API github examples.
 */

#include "esp_eap_client.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "string.h"

#include "freertos/event_groups.h"

#include "modules/wifi_module.h"

#if CONFIG_WIFI_SCAN_FAST
#define WIFI_SCAN_METHOD WIFI_FAST_SCAN
#elif CONFIG_WIFI_SCAN_ALL_CHANNEL
#define WIFI_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#endif

#if CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
#define WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
#define WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#endif

#if CONFIG_WIFI_SCAN_AUTH_OPEN
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_WIFI_SCAN_AUTH_WEP
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_WIFI_SCAN_AUTH_WPA_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_WIFI_SCAN_AUTH_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_WIFI_SCAN_AUTH_WPA_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_WIFI_SCAN_AUTH_WPA2_ENTERPRISE
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_ENTERPRISE
#elif CONFIG_WIFI_SCAN_AUTH_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_WIFI_SCAN_AUTH_WPA2_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_WIFI_SCAN_AUTH_WAPI_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static const char* TAG = "WiFi-Module";
static esp_netif_t* sta_netif = NULL;

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static wifi_state state = WIFI_STATE_DISCONNECTED;

static int s_retry_num = 0;

static void handler_on_wifi_connect(void* esp_netif, esp_event_base_t event_base, int32_t event_id, void* event_data) {
}
static void handler_on_wifi_disconnect(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (s_retry_num < CONFIG_WIFI_MAX_RETRY) {
    s_retry_num++;
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
      return;
    }
    ESP_ERROR_CHECK(err);
  } else {
    ESP_LOGI(TAG, "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    return;
  }
}

static void handler_on_sta_got_ip(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  s_retry_num = 0;
  ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;

  ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));

  xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}

static void wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  sta_netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_wifi_set_default_wifi_sta_handlers();

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_destroy(void) {
  esp_err_t err = esp_wifi_stop();
  if (err == ESP_ERR_WIFI_NOT_INIT) {
    return;
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_wifi_deinit());
  ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(sta_netif));
  esp_netif_destroy(sta_netif);
  sta_netif = NULL;
}

static void wifi_sta_do_connect(wifi_config_t wifi_config) {
  s_wifi_event_group = xEventGroupCreate();
  s_retry_num = 0;

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &handler_on_sta_got_ip, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &handler_on_wifi_connect, sta_netif));

  ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

#if CONFIG_WIFI_WPA_EAP_NETWORK
  ESP_ERROR_CHECK(esp_eap_client_set_identity((uint8_t*)CONFIG_WPA_EAP_IDENTITY, strlen(CONFIG_WPA_EAP_IDENTITY)));  // Provide identity
  ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t*)CONFIG_WPA_EAP_USERNAME, strlen(CONFIG_WPA_EAP_USERNAME)));  // Provide username
  ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t*)CONFIG_WPA_EAP_PASSWORD, strlen(CONFIG_WPA_EAP_PASSWORD)));  // Provide password
  ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());                                                                 // Should not connect immediately?
#endif

  esp_err_t ret = esp_wifi_connect();

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
    state = WIFI_STATE_INVALID;
    return;
  }
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         CONFIG_WIFI_MAX_INIT_TIME_MS);

  if (bits & WIFI_CONNECTED_BIT) {
#if !CONFIG_WIFI_WPA_EAP_NETWORK
    ESP_LOGI(TAG, "Connected to an AP SSID:%s, Password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
#else
    ESP_LOGI(TAG, "Connected to an WPA-EAP AP SSID:%s, Identity:%s, Username:%s, Password:%s",
             CONFIG_WIFI_SSID, CONFIG_WPA_EAP_IDENTITY, CONFIG_WPA_EAP_USERNAME, CONFIG_WPA_EAP_PASSWORD);
#endif
    state = WIFI_STATE_CONNECTED;
  } else if (bits & WIFI_FAIL_BIT) {
#if !CONFIG_WIFI_WPA_EAP_NETWORK
    ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
#else
    ESP_LOGE(TAG, "Failed to connect to an WPA-EAP AP SSID:%s, Identity:%s, Username:%s, Password:%s",
             CONFIG_WIFI_SSID, CONFIG_WPA_EAP_IDENTITY, CONFIG_WPA_EAP_USERNAME, CONFIG_WPA_EAP_PASSWORD);
#endif
    state = WIFI_STATE_DISCONNECTED;
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
    state = WIFI_STATE_INVALID;
  }
}

static void wifi_sta_do_disconnect(void) {
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect));
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &handler_on_sta_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &handler_on_wifi_connect));

  ESP_ERROR_CHECK(esp_wifi_disconnect());
#if CONFIG_WIFI_WPA_EAP_NETWORK
  esp_eap_client_clear_identity();
  esp_eap_client_clear_username();
  esp_eap_client_clear_password();
  esp_wifi_sta_enterprise_disable();
#endif
  state = WIFI_STATE_DISCONNECTED;
}

wifi_state get_wifi_module_state(void) {
  return state;
}

const char* get_wifi_module_state_s(void) {
  switch (state) {
    case WIFI_STATE_CONNECTED:
      return "CONNECTED";
    case WIFI_STATE_DISCONNECTED:
      return "DISCONNECTED";
    case WIFI_STATE_INVALID:
      return "INVALID";
    default:
      return "UNKNOWN";
  }
}

void wifi_module_stop(void) {
  wifi_sta_do_disconnect();
  wifi_destroy();
}

void wifi_module_start(void) {
  ESP_LOGI(TAG, "Start wifi_module_start.");
  wifi_init();
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = CONFIG_WIFI_SSID,
#if !CONFIG_WIFI_WPA_EAP_NETWORK
          .password = CONFIG_WIFI_PASSWORD,
#endif
          .scan_method = WIFI_SCAN_METHOD,
          .sort_method = WIFI_CONNECT_AP_SORT_METHOD,
          .threshold.rssi = CONFIG_WIFI_SCAN_RSSI_THRESHOLD,
          .threshold.authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD,
      },
  };
  wifi_sta_do_connect(wifi_config);
}