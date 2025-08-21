#include "net_connection.h"

static const char* TAG = "NET_CONNECTION";
EventGroupHandle_t wifi_event_group;

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                xEventGroupSetBits(wifi_event_group, BIT_WIFI_CONNECTED);
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

void network_init() {
    ESP_LOGI(TAG, "init nvs");
    ESP_ERROR_CHECK(nvs_flash_init());
    
    ESP_LOGI(TAG, "init netif");
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "init event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    ESP_LOGI(TAG, "init Wi-Fi");
    wifi_event_group = xEventGroupCreate();
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

    xEventGroupWaitBits(wifi_event_group, BIT_WIFI_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Succesfully connected");
}