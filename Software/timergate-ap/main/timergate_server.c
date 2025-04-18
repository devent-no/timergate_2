#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
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
#define FILE_PATH_MAX 512
#define SPIFFS_READ_SIZE 512

static const char *TAG = "timergate-ap";

// Legg til denne funksjonen for å liste filer i SPIFFS
void list_spiffs_files(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s (errno: %d)", path, errno);
        return;
    }
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            ESP_LOGI(TAG, "  File: %s/%s", path, entry->d_name);
        } else if (entry->d_type == DT_DIR) {
            char next_path[FILE_PATH_MAX];
            snprintf(next_path, sizeof(next_path), "%s/%s", path, entry->d_name);
            ESP_LOGI(TAG, "  Dir: %s", next_path);
            list_spiffs_files(next_path); // Sjekk undermapper rekursivt
        }
    }
    closedir(dir);
}

esp_err_t init_fs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = "www",
        .max_files = 5,
        .format_if_mount_failed = false
    };
    
    // Loggmelding flyttet etter variabeldefinisjonen
    ESP_LOGI(TAG, "Mounting SPIFFS with base_path=%s, partition_label=%s", 
             conf.base_path, conf.partition_label);
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem (ESP_FAIL)");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition (ESP_ERR_NOT_FOUND)");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    
    // List alle filer i SPIFFS
    ESP_LOGI(TAG, "Listing all files in SPIFFS:");
    list_spiffs_files("/www");
    
    return ESP_OK;
}



esp_err_t spiffs_get_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;
    
    const char *base_path = (const char*) req->user_ctx;
    const char *uri_path = req->uri;

    ESP_LOGI(TAG, "Requested URI: %s", uri_path);
    
    // Hvis URI-en er /, bruk /index.html
    if (strcmp(uri_path, "/") == 0) {
        uri_path = "/index.html";
        ESP_LOGI(TAG, "Root URI requested, using %s", uri_path);
    }
    
    // Konstruer filsti
    size_t base_len = strlen(base_path);
    size_t uri_len = strlen(uri_path);
    size_t total_len = base_len + uri_len + 1;
    
    if (total_len > FILE_PATH_MAX) {
        ESP_LOGE(TAG, "File path too long: %d > %d", (int)total_len, FILE_PATH_MAX);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    strlcpy(filepath, base_path, FILE_PATH_MAX);
    strlcat(filepath, uri_path, FILE_PATH_MAX);

    ESP_LOGI(TAG, "Full filepath: %s", filepath);

    // Sjekk om filen eksisterer
    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to stat file: %s (errno: %d)", filepath, errno);
        

        if (strstr(uri_path, "/assets/") == uri_path) {
            char exact_path[FILE_PATH_MAX];
            strlcpy(exact_path, "/www", FILE_PATH_MAX);
            strlcat(exact_path, uri_path, FILE_PATH_MAX);
            
            ESP_LOGI(TAG, "Trying exact SPIFFS path: %s", exact_path);
            
            // Sjekk direkte på den eksakte banen
            if (stat(exact_path, &file_stat) != -1) {
                // Dette burde funke!
                strlcpy(filepath, exact_path, FILE_PATH_MAX);
                ESP_LOGI(TAG, "Found asset with exact path: %s", filepath);
            } else {
                ESP_LOGE(TAG, "Asset file not found with exact path either: %s (errno: %d)", exact_path, errno);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }


    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to open file: %s (errno: %d)", filepath, errno);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Sett riktig content-type basert på filendelse
    const char *dot = strrchr(filepath, '.');
    if (dot && !strcmp(dot, ".html")) {
        httpd_resp_set_type(req, "text/html");
    } else if (dot && !strcmp(dot, ".js")) {
        httpd_resp_set_type(req, "application/javascript");
    } else if (dot && !strcmp(dot, ".css")) {
        httpd_resp_set_type(req, "text/css");
    } else if (dot && !strcmp(dot, ".png")) {
        httpd_resp_set_type(req, "image/png");
    } else if (dot && !strcmp(dot, ".ico")) {
        httpd_resp_set_type(req, "image/x-icon");
    } else if (dot && !strcmp(dot, ".svg")) {
        httpd_resp_set_type(req, "image/svg+xml");
    } else {
        httpd_resp_set_type(req, "application/octet-stream");
    }

    // Send fil-innholdet
    char *buffer = malloc(SPIFFS_READ_SIZE);
    if (!buffer) {
        fclose(fd);
        ESP_LOGE(TAG, "Failed to allocate memory for file reading");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t read_bytes;
    ESP_LOGI(TAG, "Beginning file send: %s", filepath);
    do {
        read_bytes = fread(buffer, 1, SPIFFS_READ_SIZE, fd);
        if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK) {
                fclose(fd);
                free(buffer);
                ESP_LOGE(TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    
    free(buffer);
    fclose(fd);
    ESP_LOGI(TAG, "File sent successfully: %s", filepath);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}





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
        // WebSocket handler
        httpd_uri_t ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server_handle, &ws);
        
        // Root URI handler
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &root);
        
        // All other URIs
        httpd_uri_t common = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &common);

        ESP_LOGI(TAG, "Web server started");
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

    // Initialiser filsystemet
    ESP_ERROR_CHECK(init_fs());

    server = start_webserver();
    xTaskCreate(ws_broadcast_task, "ws_broadcast_task", 4096, NULL, 5, NULL);
}