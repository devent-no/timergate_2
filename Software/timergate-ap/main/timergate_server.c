#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
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
#include "esp_now.h"

// Definer MIN-makroen
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define MAX_WS_CLIENTS 5
#define FILE_PATH_MAX 512
#define SPIFFS_READ_SIZE 512

// Legg til disse nye definisjonene
#define MAX_POLES 5

// Datastruktur for å lagre informasjon fra målestolpene
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];  // MAC-adresse til målestolpen
    uint8_t k;                      // Type melding (0=ADC verdier, 1=Sensor breaks, 2=Settings)
    uint32_t t;                     // Timestamp (sekunder)
    uint32_t u;                     // Mikrosekunder
    int32_t v[7];                   // ADC verdier
    int32_t b[7];                   // Broken status
    int32_t o[7];                   // Offsets
    int32_t e[7];                   // Enabled status
} pole_data_t;

// Globale variabler for å lagre data fra målestolpene
static pole_data_t pole_data[MAX_POLES];
static int pole_count = 0;
static SemaphoreHandle_t pole_data_mutex;


static const char *TAG = "timergate-ap";

// Legg til en hjelpefunksjon for å liste filer i SPIFFS
void list_spiffs_files(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s (errno: %d)", path, errno);
        return;
    }
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // Bygg full filbane
        char full_path[FILE_PATH_MAX];
        if (snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name) >= sizeof(full_path)) {
            ESP_LOGE(TAG, "Path too long: %s/%s", path, entry->d_name);
            continue;
        }
        
        // Sjekk filtype
        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  Dir: %s", full_path);
                // Sjekk undermapper rekursivt
                list_spiffs_files(full_path);
            } else {
                ESP_LOGI(TAG, "  File: %s (Size: %ld bytes)", full_path, st.st_size);
            }
        } else {
            ESP_LOGE(TAG, "  Failed to stat: %s (errno: %d)", full_path, errno);
        }
    }
    closedir(dir);
}





// Funksjon for å håndtere tid-synkronisering API
esp_err_t time_sync_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "time_sync_handler called, content_len: %d", remaining);
    
    if (remaining > sizeof(buf)) {
        // For lang forespørsel
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Request too large");
        return ESP_FAIL;
    }
    
    // Les innhold fra forespørselen
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    // Parse JSON (forenklet - i en reell implementasjon ville du bruke et JSON-bibliotek)
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Mottok data: %s", buf);
    
    // Send tilbake bekreftelse
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Time synchronized\"}");
    
    return ESP_OK;
}

// Funksjon for å håndtere time/check API
esp_err_t time_check_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "time_check_handler called, content_len: %d", remaining);
    
    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Request too large");
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Mottok data: %s", buf);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"diff\":0}");
    
    return ESP_OK;
}

// Funksjon for å håndtere hsearch API
esp_err_t time_hsearch_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "time_hsearch_handler called, content_len: %d", remaining);
    
    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Request too large");
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Mottok data: %s", buf);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Highpoint search initiated\"}");
    
    return ESP_OK;
}

// Funksjon for å håndtere pole/break API
esp_err_t pole_break_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "pole_break_handler called, content_len: %d", remaining);
    
    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Request too large");
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Mottok data: %s", buf);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Break configuration updated\"}");
    
    return ESP_OK;
}

// Funksjon for å håndtere pole/enabled API
esp_err_t pole_enabled_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "pole_enabled_handler called, content_len: %d", remaining);
    
    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Request too large");
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Mottok data: %s", buf);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Sensor enabled status updated\"}");
    
    return ESP_OK;
}

// Funksjon for å håndtere OPTIONS forespørsler (CORS preflight)
esp_err_t options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Accept, X-Requested-With");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}










esp_err_t init_fs(void) {
    ESP_LOGI(TAG, "Mounting SPIFFS with base_path=/www, partition_label=www");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = "www",
        .max_files = 10,  // Økt fra 5 til 10 for å støtte flere filer
        .format_if_mount_failed = false
    };
    
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

// Forbedret debug_handler som sender data i chunks
esp_err_t debug_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "DEBUG handler called");
    
    httpd_resp_set_type(req, "text/plain");
    
    // Start med å sende en header
    httpd_resp_sendstr_chunk(req, "SPIFFS Filesystem Debug Info:\n\n");
    
    // Sjekk /www-mappen - send resultater i små deler
    DIR *dir = opendir("/www");
    if (dir) {
        httpd_resp_sendstr_chunk(req, "Files in /www directory:\n");
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            struct stat st;
            char path[FILE_PATH_MAX];
            snprintf(path, sizeof(path), "/www/%s", entry->d_name);
            
            if (stat(path, &st) == 0) {
                // Send hvert filnavn som en separat chunk
                char file_info[64]; // Mindre buffer for hver fil
                
                // Begrens filnavnlengden for sikkerhets skyld
                char safe_name[32];
                strlcpy(safe_name, entry->d_name, sizeof(safe_name));
                
                snprintf(file_info, sizeof(file_info), "  %s (%ld bytes)\n", safe_name, st.st_size);
                httpd_resp_sendstr_chunk(req, file_info);
            }
        }
        closedir(dir);
        httpd_resp_sendstr_chunk(req, "\n");
    } else {
        httpd_resp_sendstr_chunk(req, "Failed to open /www directory\n\n");
    }
    
    // Sjekk /www/assets-mappen - send resultater i små deler
    dir = opendir("/www/assets");
    if (dir) {
        httpd_resp_sendstr_chunk(req, "Files in /www/assets directory:\n");
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            struct stat st;
            char path[FILE_PATH_MAX];
            snprintf(path, sizeof(path), "/www/assets/%s", entry->d_name);
            
            if (stat(path, &st) == 0) {
                // Send hvert filnavn som en separat chunk
                char file_info[64];
                
                // Begrens filnavnlengden
                char safe_name[32];
                strlcpy(safe_name, entry->d_name, sizeof(safe_name));
                
                snprintf(file_info, sizeof(file_info), "  %s (%ld bytes)\n", safe_name, st.st_size);
                httpd_resp_sendstr_chunk(req, file_info);
            }
        }
        closedir(dir);
        httpd_resp_sendstr_chunk(req, "\n");
    } else {
        httpd_resp_sendstr_chunk(req, "Failed to open /www/assets directory\n");
    }
    
    // Sjekk spesifikke testfiler
    httpd_resp_sendstr_chunk(req, "\nTesting specific asset files:\n");
    
    const char* test_files[] = {
        "/www/index.html",
        "/www/assets/index.js",
        "/www/assets/index.css"
    };
    
    for (int i = 0; i < 3; i++) {
        struct stat st;
        if (stat(test_files[i], &st) == 0) {
            char file_info[64];
            
            // Bruk bare siste delen av filnavnet for sikkerhets skyld
            const char* basename = strrchr(test_files[i], '/');
            basename = basename ? basename + 1 : test_files[i];
            
            snprintf(file_info, sizeof(file_info), "  %s exists (%ld bytes)\n", test_files[i], st.st_size);
            httpd_resp_sendstr_chunk(req, file_info);
            
            // Prøv å åpne filen for å bekrefte lesbarhet
            FILE *f = fopen(test_files[i], "r");
            if (f) {
                httpd_resp_sendstr_chunk(req, "    File can be opened for reading: YES\n");
                fclose(f);
            } else {
                char err_info[64];
                snprintf(err_info, sizeof(err_info), "    File can be opened for reading: NO (errno: %d)\n", errno);
                httpd_resp_sendstr_chunk(req, err_info);
            }
        } else {
            char err_info[64];
            snprintf(err_info, sizeof(err_info), "  %s does NOT exist (errno: %d)\n", test_files[i], errno);
            httpd_resp_sendstr_chunk(req, err_info);
        }
    }
    
    // Avslutt responsen med en tom chunk
    httpd_resp_sendstr_chunk(req, NULL);
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
    
    // Spesialbehandling for assets
    bool is_asset = strstr(uri_path, "/assets/") == uri_path;
    if (is_asset) {
        ESP_LOGI(TAG, "Asset request detected: %s", uri_path);
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
        
        // Prøv flere alternative baner for assets
        if (is_asset) {
            // Prøv med direkte filnavn for kjente assets
            char test_path[FILE_PATH_MAX];
            bool found = false;
            
            if (strstr(uri_path, ".js")) {
                strlcpy(test_path, "/www/assets/index.js", FILE_PATH_MAX);
                ESP_LOGI(TAG, "Trying JS fallback path: %s", test_path);
                if (stat(test_path, &file_stat) != -1) {
                    strlcpy(filepath, test_path, FILE_PATH_MAX);
                    found = true;
                    ESP_LOGI(TAG, "Found JS file with fallback path: %s", filepath);
                }
            }
            else if (strstr(uri_path, ".css")) {
                strlcpy(test_path, "/www/assets/index.css", FILE_PATH_MAX);
                ESP_LOGI(TAG, "Trying CSS fallback path: %s", test_path);
                if (stat(test_path, &file_stat) != -1) {
                    strlcpy(filepath, test_path, FILE_PATH_MAX);
                    found = true;
                    ESP_LOGI(TAG, "Found CSS file with fallback path: %s", filepath);
                }
            }
            
            if (!found) {
                ESP_LOGE(TAG, "All fallback paths failed for asset: %s", uri_path);
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

    // Sett riktig content-type
    const char *dot = strrchr(filepath, '.');
    if (dot && !strcmp(dot, ".html")) {
        httpd_resp_set_type(req, "text/html");
    } else if (dot && !strcmp(dot, ".js")) {
        httpd_resp_set_type(req, "application/javascript");
        ESP_LOGI(TAG, "Setting content-type: application/javascript for %s", filepath);
    } else if (dot && !strcmp(dot, ".css")) {
        httpd_resp_set_type(req, "text/css");
        ESP_LOGI(TAG, "Setting content-type: text/css for %s", filepath);
    } else if (dot && !strcmp(dot, ".png")) {
        httpd_resp_set_type(req, "image/png");
    } else if (dot && !strcmp(dot, ".ico")) {
        httpd_resp_set_type(req, "image/x-icon");
    } else if (dot && !strcmp(dot, ".svg")) {
        httpd_resp_set_type(req, "image/svg+xml");
    } else {
        httpd_resp_set_type(req, "application/octet-stream");
    }

    // Sett caching-headers for bedre ytelse
    if (is_asset) {
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
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
    size_t total_sent = 0;
    ESP_LOGI(TAG, "Beginning file send: %s", filepath);
    
    do {
        read_bytes = fread(buffer, 1, SPIFFS_READ_SIZE, fd);
        if (read_bytes > 0) {
            esp_err_t res = httpd_resp_send_chunk(req, buffer, read_bytes);
            if (res != ESP_OK) {
                fclose(fd);
                free(buffer);
                ESP_LOGE(TAG, "File sending failed! Error: %s", esp_err_to_name(res));
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
            total_sent += read_bytes;
        }
    } while (read_bytes > 0);
    
    free(buffer);
    fclose(fd);
    ESP_LOGI(TAG, "File sent successfully: %s (Total bytes: %d)", filepath, total_sent);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t server = NULL;
static int ws_clients[MAX_WS_CLIENTS];
static SemaphoreHandle_t ws_mutex;

// char *generate_mock_data() {
//     static char json_msg[128];
//     snprintf(json_msg, sizeof(json_msg),
//              "{\"type\":\"data\",\"payload\":{\"gate\":1,\"timestamp\":%lld,\"status\":\"active\"}}",
//              esp_timer_get_time() / 1000);
//     return json_msg;
// }


char *generate_real_data() {
    static char json_msg[512];
    
    // Ta låsen før vi leser pole_data
    xSemaphoreTake(pole_data_mutex, portMAX_DELAY);
    
    // Finn en pole med data
    int idx = -1;
    for (int i = 0; i < pole_count; i++) {
        if (pole_data[i].k == 0) {  // Vi er interessert i ADC verdier
            idx = i;
            break;
        }
    }
    
    if (idx >= 0) {
        // Generer JSON med reelle data i det formatet GUI-et forventer
        char mac_str[18];
        sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
                pole_data[idx].mac[0], pole_data[idx].mac[1], pole_data[idx].mac[2],
                pole_data[idx].mac[3], pole_data[idx].mac[4], pole_data[idx].mac[5]);
        
        sprintf(json_msg, 
                "{\"M\":\"%s\",\"K\":%d,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"V\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],\"B\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}",
                mac_str, pole_data[idx].k, pole_data[idx].t, pole_data[idx].u,
                pole_data[idx].v[0], pole_data[idx].v[1], pole_data[idx].v[2], 
                pole_data[idx].v[3], pole_data[idx].v[4], pole_data[idx].v[5], pole_data[idx].v[6],
                pole_data[idx].b[0], pole_data[idx].b[1], pole_data[idx].b[2], 
                pole_data[idx].b[3], pole_data[idx].b[4], pole_data[idx].b[5], pole_data[idx].b[6]);
        
        ESP_LOGI(TAG, "Sending pole data: %s", json_msg);
    } else if (pole_count > 0) {
        // Vi har målestolper, men ingen ADC-data ennå
        sprintf(json_msg, "{\"type\":\"waiting\",\"message\":\"Waiting for ADC data from poles\"}");
    } else {
        // Ingen målestolper registrert ennå
        sprintf(json_msg, "{\"type\":\"no_poles\",\"message\":\"No poles connected yet\"}");
    }
    
    xSemaphoreGive(pole_data_mutex);
    return json_msg;
}




// void ws_broadcast_task(void *pvParameter) {
//     while (1) {
//         if (server == NULL) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             continue;
//         }

//         char *msg = generate_real_data();

//         httpd_ws_frame_t ws_pkt = {
//             .final = true,
//             .fragmented = false,
//             .type = HTTPD_WS_TYPE_TEXT,
//             .payload = (uint8_t *)msg,
//             .len = strlen(msg)
//         };

//         xSemaphoreTake(ws_mutex, portMAX_DELAY);
//         for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
//             if (ws_clients[i] != 0) {
//                 esp_err_t ret = httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
//                 if (ret != ESP_OK) {
//                     ESP_LOGW(TAG, "WebSocket send failed to fd %d: %s", ws_clients[i], esp_err_to_name(ret));
//                     ws_clients[i] = 0;
//                 }
//             }
//         }
//         xSemaphoreGive(ws_mutex);

//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }



void ws_broadcast_task(void *pvParameter) {
    while (1) {
        if (server == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Bruk reelle data i stedet for mock-data
        char *msg = generate_real_data();
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

        // Raskere oppdatering
        vTaskDelay(pdMS_TO_TICKS(200));  // 5 ganger per sekund i stedet for 1 gang per sekund
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
    // Øk maks URI-handlere
    config.max_uri_handlers = 12; // Økt fra 6 til 12 for å ha plass til flere handlere
    // Øk recv/send-bufferstørrelse og timeouts
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    
    httpd_handle_t server_handle = NULL;

    ESP_LOGI(TAG, "Starting HTTP server");
    
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
        
        // Debug handler for diagnostikk
        httpd_uri_t debug = {
            .uri = "/debug",
            .method = HTTP_GET,
            .handler = debug_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &debug);
        
        // Spesifikk handler for JS-filer
        httpd_uri_t js_files = {
            .uri = "/assets/*.js",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &js_files);
        
        // Spesifikk handler for CSS-filer
        httpd_uri_t css_files = {
            .uri = "/assets/*.css",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &css_files);
        
        // Root URI handler
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &root);
        
        // API handlere
        // Time sync API
        httpd_uri_t time_sync = {
            .uri = "/api/v1/time/sync",
            .method = HTTP_POST,
            .handler = time_sync_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &time_sync);
        
        // Time check API
        httpd_uri_t time_check = {
            .uri = "/api/v1/time/check",
            .method = HTTP_POST,
            .handler = time_check_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &time_check);
        
        // Highpoint search API
        httpd_uri_t time_hsearch = {
            .uri = "/api/v1/time/hsearch",
            .method = HTTP_POST,
            .handler = time_hsearch_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &time_hsearch);
        
        // Pole break configuration API
        httpd_uri_t pole_break = {
            .uri = "/api/v1/pole/break",
            .method = HTTP_POST,
            .handler = pole_break_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &pole_break);
        
        // Pole enabled configuration API
        httpd_uri_t pole_enabled = {
            .uri = "/api/v1/pole/enabled",
            .method = HTTP_POST,
            .handler = pole_enabled_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &pole_enabled);
        
        // OPTIONS handler for CORS
        httpd_uri_t options = {
            .uri = "/api/*",
            .method = HTTP_OPTIONS,
            .handler = options_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &options);
        
        // All other URIs
        httpd_uri_t common = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = spiffs_get_handler,
            .user_ctx = "/www"
        };
        httpd_register_uri_handler(server_handle, &common);

        ESP_LOGI(TAG, "Web server started");
    } else {
        ESP_LOGE(TAG, "Failed to start web server!");
    }

    return server_handle;
}



// Legg til denne funksjonen etter de eksisterende funksjonene, men før app_main

// ESP-NOW mottaker callback
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Hent MAC-adressen fra recv_info
    const uint8_t *mac_addr = recv_info->src_addr;
    
    ESP_LOGI(TAG, "ESP-NOW data mottatt fra %02x:%02x:%02x:%02x:%02x:%02x, len=%d", 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len);
    
    // Resten av funksjonen som før, men bruk mac_addr fra recv_info->src_addr
    
    // Sjekk at lengden er riktig for vår struktur (forenklet sjekk)
    if (len < 10) {  // Minst 1 byte for k + 4 bytes for t + 4 bytes for u + litt data
        ESP_LOGE(TAG, "Mottok data med feil lengde: %d", len);
        return;
    }
    
    xSemaphoreTake(pole_data_mutex, portMAX_DELAY);
    
    // Finn pole i vår liste eller legg til en ny
    int pole_idx = -1;
    for (int i = 0; i < pole_count; i++) {
        if (memcmp(pole_data[i].mac, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            pole_idx = i;
            break;
        }
    }
    
    if (pole_idx < 0 && pole_count < MAX_POLES) {
        pole_idx = pole_count++;
        memcpy(pole_data[pole_idx].mac, mac_addr, ESP_NOW_ETH_ALEN);
        ESP_LOGI(TAG, "Ny målestolpe lagt til, totalt: %d", pole_count);
    }
    
    if (pole_idx >= 0) {
        // Parse data (enkel implementasjon - dette må tilpasses det faktiske formatet)
        uint8_t k = data[0];
        pole_data[pole_idx].k = k;
        
        // Les timestamp
        memcpy(&pole_data[pole_idx].t, &data[1], sizeof(uint32_t));
        memcpy(&pole_data[pole_idx].u, &data[5], sizeof(uint32_t));
        
        ESP_LOGI(TAG, "Mottok data type %d fra målestolpe %d", k, pole_idx);
        
        // Les ADC verdier eller andre data basert på type melding
        if (k == 0 && len >= 9 + sizeof(int32_t) * 14) {  // ADC verdier
            memcpy(pole_data[pole_idx].v, &data[9], sizeof(int32_t) * 7);
            memcpy(pole_data[pole_idx].b, &data[9 + sizeof(int32_t) * 7], sizeof(int32_t) * 7);
            
            // Log noen verdier for debugging
            ESP_LOGI(TAG, "ADC verdier: %" PRId32 ", %" PRId32 ", %" PRId32 ", ...", 
                    pole_data[pole_idx].v[0], pole_data[pole_idx].v[1], pole_data[pole_idx].v[2]);
        } else if (k == 2 && len >= 9 + sizeof(int32_t) * 21) {  // Settings
            memcpy(pole_data[pole_idx].e, &data[9], sizeof(int32_t) * 7);
            memcpy(pole_data[pole_idx].o, &data[9 + sizeof(int32_t) * 7], sizeof(int32_t) * 7);
            memcpy(pole_data[pole_idx].b, &data[9 + sizeof(int32_t) * 14], sizeof(int32_t) * 7);
            
            ESP_LOGI(TAG, "Settings mottatt for målestolpe %d", pole_idx);
        }
    }
    
    xSemaphoreGive(pole_data_mutex);
}


// Funksjon for å initialisere ESP-NOW
void init_esp_now(void) {
    ESP_LOGI(TAG, "Initialiserer ESP-NOW");
    
    // Initialiser ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Feil ved initialisering av ESP-NOW");
        return;
    }
    
    // Registrer callback
    if (esp_now_register_recv_cb(esp_now_recv_cb) != ESP_OK) {
        ESP_LOGE(TAG, "Feil ved registrering av ESP-NOW mottaker callback");
        return;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialisert, venter på data fra målestolper...");
}




// void app_main(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }

//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = "Timergate",
//             .ssid_len = strlen("Timergate"),
//             .channel = 1,
//             .password = "12345678",
//             .max_connection = 4,
//             .authmode = WIFI_AUTH_WPA_WPA2_PSK
//         }
//     };

//     if (strlen((char *)wifi_config.ap.password) == 0) {
//         wifi_config.ap.authmode = WIFI_AUTH_OPEN;
//     }

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());

//     ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
//              "Timergate", "12345678", 1);

//     ws_mutex = xSemaphoreCreateMutex();
//     memset(ws_clients, 0, sizeof(ws_clients));

//     // Initialiser filsystemet
//     ESP_ERROR_CHECK(init_fs());

//     server = start_webserver();
//     xTaskCreate(ws_broadcast_task, "ws_broadcast_task", 4096, NULL, 5, NULL);
// }



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

    // Initialiser mutexer
    ws_mutex = xSemaphoreCreateMutex();
    pole_data_mutex = xSemaphoreCreateMutex();  // Nytt mutex for pole data
    
    memset(ws_clients, 0, sizeof(ws_clients));
    memset(pole_data, 0, sizeof(pole_data));  // Initialiser pole data array

    // Initialiser filsystemet
    ESP_ERROR_CHECK(init_fs());
    
    // Initialiser ESP-NOW (ny linje)
    init_esp_now();

    server = start_webserver();
    xTaskCreate(ws_broadcast_task, "ws_broadcast_task", 4096, NULL, 5, NULL);
}