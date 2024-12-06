#include <stdio.h>
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define LOG_LEVEL ESP_LOG_ERROR
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

#include "mqtt_client.h"
#include "cJSON.h"

#include "parse_moves.h"
#include "motion.h"

#define BIT_WIFI_CONNECTED BIT0

#define NAXIS 6
#define STEPS_PR_REV 800

static const char* TAG = "main";

QueueHandle_t motion_queue;
EventGroupHandle_t event_group;

motion_axis_t* motion_axis[NAXIS];

void motion_task(void* arg) {
    for (;;) {
        RobotMove move;
        xQueueReceive(motion_queue, (char*)&move, portMAX_DELAY);
        print_move(move);
        for (int i = 0; i < NAXIS; ++i) {
            if (isnan(move.move[i])) continue;
            motion_instruction_t instr = {
                .angle = move.move[i],
                .duration = move.duration,
                .profile = MOTION_PROFILE_SCURVE
            };
            ESP_LOGI(TAG, "Executing move on axis %d...", i);
            motion_axis_move(motion_axis[i], instr);
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                xEventGroupSetBits(event_group, BIT_WIFI_CONNECTED);
            break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
                esp_wifi_connect();
                break;
        }
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    RobotMove robot_move;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, "/robot/position", 0);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            if (parse_robot_move(event->data, &robot_move))
            {
                xQueueSend(motion_queue, &robot_move, portMAX_DELAY);
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

void app_main() 
{
    ESP_LOGI(TAG, "init nvs");
    ESP_ERROR_CHECK(nvs_flash_init());
    
    ESP_LOGI(TAG, "init netif");
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "init event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    ESP_LOGI(TAG, "init Wi-Fi");
    event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_LOGI(TAG, "init provision");
    wifi_prov_mgr_config_t prov_cfg  = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
    };
    ESP_ERROR_CHECK( wifi_prov_mgr_init(prov_cfg) );
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));
    if (!provisioned){
        ESP_LOGI(TAG, "starting provision");
        const char *service_name = "my_device";
        const char *service_key  = "password";
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        const char *pop = "abcd1234";
        ESP_ERROR_CHECK( wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key) );
    } else {
        ESP_LOGI(TAG, "device already provisioned!");
    }
    wifi_prov_mgr_wait();
    wifi_prov_mgr_deinit();
    
    ESP_LOGI(TAG, "Connecting to Wi-Fi");
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    xEventGroupWaitBits(event_group, BIT_WIFI_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Succesfully connected");

    ESP_LOGI(TAG, "Configuring motion axis");
    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    for (int i = 0; i < NAXIS; ++i) {
        motion_axis[i] = motion_axis_create(pulse_pins[i], dir_pins[i], STEPS_PR_REV);
    }
    motion_axis[0]->gear_ratio = 56.0f * 50.0f/20.0f;
    motion_axis[1]->gear_ratio = 56.0f;
    motion_axis[2]->gear_ratio = 56.0f;
    motion_axis[3]->gear_ratio = 40.25f * 50.0f/16.0f;
    motion_axis[4]->gear_ratio = 80.0f/28.0f * 80.0f/12.0f;
    motion_axis[5]->gear_ratio = 80.0f/28.0f * 80.0f/12.0f;

    ESP_LOGI(TAG, "Starting MQTT");
    motion_queue = xQueueCreate(32, sizeof(RobotMove));
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.10",
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    ESP_LOGI(TAG, "Starting motion task");
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
}

/* TODO topics
DEAL WITH CURVE GOING OUT OF SCOPE!
--------------------------
architecture in motion lib
unit testing
axis gearing and position tracking
delta and absolute moves
event callbacks for rmt tx done (to check axis has finished in group moves)
json parse improvements, commands to specific axis instead of all in one message, get duration from json
control of max acceleration and velocity (and jerk)

*/