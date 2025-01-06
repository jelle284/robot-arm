#ifndef NET_CONNECTION_H
#define NET_CONNECTION_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

#define BIT_WIFI_CONNECTED BIT0

void network_init();

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

#endif // NET_CONNECTION_H