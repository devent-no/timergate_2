#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "esp_spiffs.h"
#include "esp_timer.h"

#define MAX_WS_CLIENTS 5

static const char *TAG = "timergate-ap";

static httpd_handle_t server = NULL;
static int ws_clients[MAX_WS_CLIENTS];
static SemaphoreHandle_t ws_mutex;

char *generate_mock_data() {
    static char json_msg[128];
    snprintf(json_msg, sizeof(json_msg),
             "{\"type\":\"data\",\"payload\":{\"gate\":1,\"timestamp\":%lld,\"status\":\"active\"}}",
             esp_timer_get_time() / 1000);
    return json_msg;
}

void ws_broadcast_task(void *pvParameter) {
    while (1) {
        if (server == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char *msg = generate_mock_data();
        httpd_ws_frame_t ws_pkt = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)msg,
            .len = strlen(msg)
        };

        xSemaphoreTake(ws_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
            if (ws_clients[i] != 0) {
                esp_err_t ret = httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "WebSocket send failed to fd %d: %s", ws_clients[i], esp_err_to_name(ret));
                    ws_clients[i] = 0;
                }
            }
        }
        xSemaphoreGive(ws_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        int fd = httpd_req_to_sockfd(req);

        xSemaphoreTake(ws_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
            if (ws_clients[i] == 0) {
                ws_clients[i] = fd;
                break;
            }
        }
        xSemaphoreGive(ws_mutex);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT
    };

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame length: %s", esp_err_to_name(ret));
        return ret;
    }

    frame.payload = malloc(frame.len + 1);
    if (!frame.payload) {
        return ESP_ERR_NO_MEM;
    }

    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        free(frame.payload);
        return ret;
    }

    ((char *)frame.payload)[frame.len] = 0;
    ESP_LOGI(TAG, "Received message: %s", (char *)frame.payload);
    free(frame.payload);
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server_handle = NULL;

    if (httpd_start(&server_handle, &config) == ESP_OK) {
        httpd_uri_t ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server_handle, &ws);
    }

    return server_handle;
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Timergate",
            .ssid_len = strlen("Timergate"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };

    if (strlen((char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             "Timergate", "12345678", 1);

    ws_mutex = xSemaphoreCreateMutex();
    memset(ws_clients, 0, sizeof(ws_clients));

    server = start_webserver();
    xTaskCreate(ws_broadcast_task, "ws_broadcast_task", 4096, NULL, 5, NULL);
}