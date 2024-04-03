#include <stdint.h>

#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

typedef enum {
  MQTT_STATE_INVALID,
  MQTT_STATE_CONNECTED,
  MQTT_STATE_DISCONNECTED,
  MQTT_STATE_SUBSCRIBED,
  MQTT_STATE_UNSUBSCRIBED
} mqtt_state;

typedef enum {
  MQTT_ERROR_OK,
  MQTT_ERROR_PUBLISH_FAILED
} mqtt_error;

typedef enum {
  MQTT_PAYLOAD_TYPE_DATA,
  MQTT_PAYLOAD_TYPE_NOTIFICATION,
} mqtt_payload_type;

#pragma pack(1)
typedef struct {
  uint16_t ins;       // 2 bytes
  uint16_t outs;      // 2 bytes
} mqtt_data_payload;  // 4 bytes
#pragma pack()

typedef struct {
  char description[32];  // 16 bytes
} mqtt_notification_payload;

typedef struct {
  uint64_t timestamp;              // 8 bytes
  mqtt_payload_type payload_type;  // 4 bytes
  union {
    mqtt_data_payload* data_payload;                  // 2 bytes
    mqtt_notification_payload* notification_payload;  // 2 bytes
  } payload;
} mqtt_payload;

void mqtt_module_start(void);
void mqtt_module_stop(void);
mqtt_error mqtt_module_send(const char* topic, const mqtt_payload* data, const char* device_identifier);

mqtt_state get_mqtt_module_state(void);
const char* get_mqtt_module_state_s(void);
#endif /* MQTT_MODULE_H */