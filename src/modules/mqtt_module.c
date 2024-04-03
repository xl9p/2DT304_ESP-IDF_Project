/**
 * Adopts a similar approach as the wifi_module.
 */

#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "inttypes.h"
#define MQTT_SKIP_PUBLISH_IF_DISCONNECTED 1
#include "mqtt_client.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"

#include "modules/mqtt_module.h"

static const char* TAG = "MQTT-Module";

static mqtt_state state = MQTT_STATE_DISCONNECTED;
static esp_mqtt_client_handle_t client = NULL;

extern const uint8_t mqtt_pem_start[] asm("_binary_mqtt_crt_start");
extern const uint8_t mqtt_pem_end[] asm("_binary_mqtt_crt_end");

static EventGroupHandle_t s_mqtt_event_group;

#define MQTT_CLIENT_CONNECTED_BIT BIT0
#define MQTT_CLIENT_FAIL_BIT BIT1

static void handler_on_mqtt_connected(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  esp_mqtt_connect_return_code_t connect_code = event->error_handle->connect_return_code;
  esp_mqtt_error_type_t error_type = event->error_handle->error_type;

  if ((connect_code == MQTT_CONNECTION_ACCEPTED) && (error_type == MQTT_ERROR_TYPE_NONE)) {
    client = event->client;
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    xEventGroupSetBits(s_mqtt_event_group, MQTT_CLIENT_CONNECTED_BIT);
    state = MQTT_STATE_CONNECTED;
  } else {
    state = MQTT_STATE_DISCONNECTED;
    xEventGroupSetBits(s_mqtt_event_group, MQTT_CLIENT_FAIL_BIT);
  }
}
static void handler_on_mqtt_disconnected(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
  xEventGroupClearBits(s_mqtt_event_group, MQTT_CLIENT_CONNECTED_BIT);
  state = MQTT_STATE_DISCONNECTED;
}
static void handler_on_mqtt_published(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
}
static void handler_on_mqtt_data(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  ESP_LOGI(TAG, "MQTT_EVENT_DATA");
  printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
  printf("DATA=%.*s\r\n", event->data_len, event->data);
}
static void handler_on_mqtt_error(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  esp_mqtt_event_handle_t event = event_data;

  esp_mqtt_connect_return_code_t connect_code = event->error_handle->connect_return_code;
  esp_mqtt_error_type_t error_type = event->error_handle->error_type;

  ESP_LOGW(TAG, "MQTT_EVENT_ERROR");

  state = MQTT_STATE_DISCONNECTED;
  xEventGroupSetBits(s_mqtt_event_group, MQTT_CLIENT_FAIL_BIT);
}

char* struct_to_json_string(const mqtt_payload* data, const char* device_identifier) {
  cJSON* root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "identification", device_identifier);
  cJSON_AddNumberToObject(root, "timestamp", data->timestamp);

  ESP_LOGI(TAG, "Got payload with type: %i", data->payload_type);

  switch (data->payload_type) {
    case MQTT_PAYLOAD_TYPE_DATA: {
      cJSON_AddNumberToObject(root, "ins", data->payload.data_payload->ins);
      cJSON_AddNumberToObject(root, "outs", data->payload.data_payload->outs);
      break;
    }
    case MQTT_PAYLOAD_TYPE_NOTIFICATION: {
      cJSON_AddStringToObject(root, "description", data->payload.notification_payload->description);
      break;
    }
    default: {
      ESP_LOGE(TAG, "Payload type not supported%s", "");
    }
  }

  char* json_str = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  return json_str;
}

mqtt_error mqtt_module_send(const char* topic, const mqtt_payload* data, const char* device_identifier) {
  int msg_id = esp_mqtt_client_publish(client, topic, struct_to_json_string(data, device_identifier), 0, 1, 0);

  if (msg_id < 0) {
    ESP_LOGE(TAG, "send failed");
    return MQTT_ERROR_PUBLISH_FAILED;
  }
  ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
  return MQTT_ERROR_OK;
}

mqtt_state get_mqtt_module_state(void) {
  return state;
}
const char* get_mqtt_module_state_s(void) {
  switch (state) {
    case MQTT_STATE_CONNECTED:
      return "CONNECTED";
    case MQTT_STATE_DISCONNECTED:
      return "DISCONNECTED";
    case MQTT_STATE_INVALID:
      return "INVALID";
    case MQTT_STATE_SUBSCRIBED:
      return "SUBSCRIBED";
    case MQTT_STATE_UNSUBSCRIBED:
      return "UNSUBSCRIBED";
    default:
      return "UNKNOWN";
  }
}

void mqtt_module_stop(void) {
  if (client) {
    ESP_LOGI(TAG, "Stopping MQTT Client%s", "");
    ESP_ERROR_CHECK(esp_mqtt_client_stop(client));

    ESP_LOGI(TAG, "Disconnecting MQTT Client%s", "");
    ESP_ERROR_CHECK(esp_mqtt_client_disconnect(client));

    ESP_LOGI(TAG, "Unregistering MQTT event handlers%s", "");
    ESP_ERROR_CHECK(esp_mqtt_client_unregister_event(client, MQTT_EVENT_CONNECTED, handler_on_mqtt_connected));
    ESP_ERROR_CHECK(esp_mqtt_client_unregister_event(client, MQTT_EVENT_DISCONNECTED, handler_on_mqtt_disconnected));
    ESP_ERROR_CHECK(esp_mqtt_client_unregister_event(client, MQTT_EVENT_PUBLISHED, handler_on_mqtt_published));
    ESP_ERROR_CHECK(esp_mqtt_client_unregister_event(client, MQTT_EVENT_DATA, handler_on_mqtt_data));
    ESP_ERROR_CHECK(esp_mqtt_client_unregister_event(client, MQTT_EVENT_ERROR, handler_on_mqtt_error));

    ESP_LOGI(TAG, "Destroying MQTT Client handler%s", "");
    ESP_ERROR_CHECK(esp_mqtt_client_destroy(client));
  } else {
    ESP_LOGE(TAG, "The MQTT client is null%s", "");
  }
}

void mqtt_module_start(void) {
  s_mqtt_event_group = xEventGroupCreate();

#if CONFIG_MQTT_USE_WSS
  struct verification_t ver_c = {
      .certificate = (const char*)mqtt_pem_start,
  };
  struct address_t address_c = {
      .path = "/mqtt",
      .port = 8084,
      .hostname = CONFIG_MQTT_BROKER_URL,
      .transport = MQTT_TRANSPORT_OVER_WSS,
  };
#else
  struct verification_t ver_c = {};
  struct address_t address_c = {
      .port = 1883,
      .hostname = CONFIG_MQTT_BROKER_URL,
      .transport = MQTT_TRANSPORT_OVER_TCP,
  };
#endif
  struct authentication_t auth_c = {
      .password = CONFIG_MQTT_PASSWORD,
  };

  struct credentials_t cred_c = {
      .authentication = auth_c,
      .username = CONFIG_MQTT_USERNAME,
  };

  struct last_will_t last_will_c = {
      .topic = CONFIG_MQTT_LWT_TOPIC,
      .msg = CONFIG_MQTT_LWT_MSG,
  };

  struct network_t network_c = {
#if CONFIG_MQTT_ENABLE_RECONNECT
    .disable_auto_reconnect = false,
#else
    .disable_auto_reconnect = true,
#endif
    .reconnect_timeout_ms = CONFIG_MQTT_RCNN_TIMEOUT_MS,
    .timeout_ms = CONFIG_MQTT_TIMEOUT_MS,
    .refresh_connection_after_ms = CONFIG_MQTT_REFRESH_CON_AFTER_MS,
  };

  struct session_t session_c = {
      .last_will = last_will_c,
  };

  struct task_t task_c = {
      .priority = CONFIG_MQTT_TASK_PRIORITY,
  };

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address = address_c,
      .broker.verification = ver_c,
      .credentials = cred_c,
      .session = session_c,
      .network = network_c,
      .task = task_c,
  };

  client = esp_mqtt_client_init(&mqtt_cfg);

  ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, handler_on_mqtt_connected, NULL));
  ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_DISCONNECTED, handler_on_mqtt_disconnected, NULL));
  ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_PUBLISHED, handler_on_mqtt_published, NULL));
  ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_DATA, handler_on_mqtt_data, NULL));
  ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_ERROR, handler_on_mqtt_error, NULL));

  esp_err_t err = esp_mqtt_client_start(client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MQTT connect failed! err:%x", err);
    state = MQTT_STATE_INVALID;
    return;
  }

  EventBits_t bits = xEventGroupWaitBits(s_mqtt_event_group,
                                         MQTT_CLIENT_CONNECTED_BIT | MQTT_CLIENT_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE, CONFIG_MQTT_MAX_INIT_TIME_MS);

  if (bits & MQTT_CLIENT_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to an MQTT broker, URL: %s", CONFIG_MQTT_BROKER_URL);
    state = MQTT_STATE_CONNECTED;
  } else if (bits & MQTT_CLIENT_FAIL_BIT) {
    ESP_LOGE(TAG, "Failed to connect to an MQTT broker, URL: %s", CONFIG_MQTT_BROKER_URL);
    state = MQTT_STATE_DISCONNECTED;
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
    state = MQTT_STATE_INVALID;
  }
}
