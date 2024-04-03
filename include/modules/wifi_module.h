#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

typedef enum {
  WIFI_STATE_INVALID,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_DISCONNECTED
} wifi_state;

// typedef enum {

// } wifi_error;

void wifi_module_start(void);
void wifi_module_stop(void);

wifi_state get_wifi_module_state(void);
const char* get_wifi_module_state_s(void);
#endif /* WIFI_MODULE_H */
