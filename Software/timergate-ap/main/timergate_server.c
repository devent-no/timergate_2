//
//--------------- Del 1: Inkluderinger, definisjoner og strukturer ----------------
//


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

// TCP server definisjoner
#define PORT 3333
#define KEEPALIVE_IDLE 1
#define KEEPALIVE_INTERVAL 1
#define KEEPALIVE_COUNT 3

// Legg til disse nye definisjonene
#define MAX_POLES 5
#define MAX_TCP_POLES 2

// Struktur for å lagre informasjon om tilkoblede målestolper via TCP
typedef struct {
    int sock;
    bool connected;
    char mac[18];            // MAC-adresse format: "xx:xx:xx:xx:xx:xx"
    char nominal_mac[18];    // "Nominell" MAC-adresse som brukes i GUI
} tcp_pole_t;



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

static tcp_pole_t tcp_poles[MAX_TCP_POLES];
static SemaphoreHandle_t tcp_poles_mutex;

static const char *TAG = "timergate-ap";

// Funksjonsdeklarasjoner
void init_esp_now(void);
esp_err_t simulate_pole_data_handler(httpd_req_t *req);
esp_err_t websocket_test_page_handler(httpd_req_t *req);

static httpd_handle_t server = NULL;
static int ws_clients[MAX_WS_CLIENTS];
static SemaphoreHandle_t ws_mutex;


//
//
//--------------- Del 2: Hjelpefunksjoner for filsystem og TCP-kommunikasjon ----------------
//
//

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

// Funksjon for å sende kommando til en spesifikk målestolpe basert på MAC-adresse
bool send_command_to_pole_by_mac(const char *mac, const char *command) {
    bool sent = false;
    xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
    
    // Sjekk først for eksakt match på MAC-adresse eller nominell MAC
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        if (tcp_poles[i].connected && 
           (strcmp(tcp_poles[i].mac, mac) == 0 || 
            (strlen(tcp_poles[i].nominal_mac) > 0 && strcmp(tcp_poles[i].nominal_mac, mac) == 0))) {
            
            int sock = tcp_poles[i].sock;
            int to_write = strlen(command);
            int res = send(sock, command, to_write, 0);
            
            if (res == to_write) {
                ESP_LOGI(TAG, "Kommando sendt til målestolpe %d (MAC: %s): %s", 
                          i, tcp_poles[i].nominal_mac[0] ? tcp_poles[i].nominal_mac : mac, command);
                sent = true;
            } else {
                ESP_LOGE(TAG, "Feil ved sending av kommando: %d (errno: %d)", res, errno);
            }
            break;
        }
    }
    
    // Hvis ingen match, og dette er første gang vi ser denne MAC-adressen,
    // tildel den til første ledige målestolpe uten nominal_mac
    if (!sent) {
        for (int i = 0; i < MAX_TCP_POLES; i++) {
            if (tcp_poles[i].connected && strlen(tcp_poles[i].nominal_mac) == 0) {
                // Lagre den nominelle MAC-adressen
                strncpy(tcp_poles[i].nominal_mac, mac, sizeof(tcp_poles[i].nominal_mac) - 1);
                tcp_poles[i].nominal_mac[sizeof(tcp_poles[i].nominal_mac) - 1] = '\0'; // Sikre null-terminering
                
                ESP_LOGI(TAG, "Tildelt nominell MAC: %s til målestolpe %d", mac, i);
                
                int sock = tcp_poles[i].sock;
                int to_write = strlen(command);
                int res = send(sock, command, to_write, 0);
                
                if (res == to_write) {
                    ESP_LOGI(TAG, "Kommando sendt til målestolpe %d med nylig tildelt nominell MAC: %s: %s", 
                             i, mac, command);
                    sent = true;
                } else {
                    ESP_LOGE(TAG, "Feil ved sending av kommando: %d (errno: %d)", res, errno);
                }
                break;
            }
        }
    }
    
    // Siste utvei: Send til første tilkoblede målestolpe hvis ingen har fått tildelt MAC
    if (!sent) {
        for (int i = 0; i < MAX_TCP_POLES; i++) {
            if (tcp_poles[i].connected) {
                int sock = tcp_poles[i].sock;
                int to_write = strlen(command);
                int res = send(sock, command, to_write, 0);
                
                if (res == to_write) {
                    ESP_LOGI(TAG, "Siste utvei: Kommando sendt til første tilkoblede målestolpe %d (søkt MAC: %s): %s", 
                             i, mac, command);
                    
                    // Tildel MAC-adressen til denne målestolpen for fremtidige kall
                    strncpy(tcp_poles[i].nominal_mac, mac, sizeof(tcp_poles[i].nominal_mac) - 1);
                    tcp_poles[i].nominal_mac[sizeof(tcp_poles[i].nominal_mac) - 1] = '\0'; // Sikre null-terminering
                    
                    ESP_LOGI(TAG, "Tildelt nominell MAC: %s til målestolpe %d", mac, i);
                    
                    sent = true;
                } else {
                    ESP_LOGE(TAG, "Feil ved sending av kommando: %d (errno: %d)", res, errno);
                }
                break;
            }
        }
    }
    
    xSemaphoreGive(tcp_poles_mutex);
    
    if (!sent) {
        ESP_LOGW(TAG, "Fant ingen tilkoblet målestolpe å sende til med MAC: %s", mac);
    }
    
    return sent;
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


// 
// 
//--------------- Del 3: TCP Server og Client Handler ---------------- 
// 
//
// Funksjon som håndterer en målestolpes TCP-forbindelse
void tcp_client_handler(void *arg) {
    int sock = *(int *)arg;
    free(arg); // Frigjør minneallokeringen fra TCP server task
    
    char rx_buffer[100];
    char command[140];
    int cmd_index = 0;
    int pole_idx = -1;

    // Finn en ledig plass i tcp_poles-arrayet
    xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        if (!tcp_poles[i].connected) {
            tcp_poles[i].connected = true;
            tcp_poles[i].sock = sock;
            // Initialiser nominal_mac til tom streng
            tcp_poles[i].nominal_mac[0] = '\0';
            
            // Forsøk å hente IP-adressen til den tilkoblede klienten
            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            if (getpeername(sock, (struct sockaddr *)&source_addr, &addr_len) == 0) {
                char ip_str[INET_ADDRSTRLEN];
                struct sockaddr_in *source = (struct sockaddr_in *)&source_addr;
                inet_ntop(AF_INET, &(source->sin_addr), ip_str, INET_ADDRSTRLEN);
                
                ESP_LOGI(TAG, "Målestolpe på indeks %d har IP: %s", i, ip_str);
            }
            
            pole_idx = i;
            break;
        }
    }
    xSemaphoreGive(tcp_poles_mutex);







    if (pole_idx == -1) {
        ESP_LOGE(TAG, "Ingen ledige plasser for målestolper, avviser forbindelse");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Målestolpe tilkoblet på indeks %d, socket %d", pole_idx, sock);

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT);
        if (len < 0) {
            if (errno == 128 || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Målestolpe har koblet fra: errno %d", errno);
                break;
            }
        } else if (len == 0) {
            ESP_LOGW(TAG, "Forbindelse lukket");
            break;
        } else {
            rx_buffer[len] = 0; // Null-terminer strengen
            
            // Prosesser mottatte data
            for (int i = 0; i < len; i++) {
                command[cmd_index] = rx_buffer[i];
                if (rx_buffer[i] == '\n') {
                    command[cmd_index] = 0; // Null-terminer kommandostrengen
                    
                    // Logg mottatt kommando
                    //ESP_LOGI(TAG, "Mottatt kommando fra målestolpe %d: %s", pole_idx, command);
                    
                    // Tolke MAC-adresse fra "ID: MAC" kommando hvis den sendes
                    if (strncmp(command, "ID: ", 4) == 0) {
                        strncpy(tcp_poles[pole_idx].mac, command + 4, 17);
                        tcp_poles[pole_idx].mac[17] = 0; // Sikre null-terminering
                        ESP_LOGI(TAG, "Målestolpe %d har MAC: '%s'", pole_idx, tcp_poles[pole_idx].mac);
                    }
                    
                    // Send direkte til WebSocket-klienter
                    char ws_msg[256];
                    if (command[0] == '{' && strchr(command, '}')) {
                        // Kommandoen er allerede i JSON-format, send direkte
                        strncpy(ws_msg, command, sizeof(ws_msg) - 1);
                        ws_msg[sizeof(ws_msg) - 1] = '\0'; // Sikre null-terminering
                    } else {
                        // Ikke JSON-format, pakk inn kommandoen
                        sprintf(ws_msg, "{\"M\":\"%s\",\"K\":3,\"cmd\":\"%s\"}", 
                                tcp_poles[pole_idx].mac, command);
                    }
                    
                    // Send til alle WebSocket-klienter
                    httpd_ws_frame_t ws_pkt = {
                        .final = true,
                        .fragmented = false,
                        .type = HTTPD_WS_TYPE_TEXT,
                        .payload = (uint8_t *)ws_msg,
                        .len = strlen(ws_msg)
                    };

                    xSemaphoreTake(ws_mutex, portMAX_DELAY);
                    for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
                        if (ws_clients[i] != 0) {
                            httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
                        }
                    }
                    xSemaphoreGive(ws_mutex);
                    
                    cmd_index = 0;
                } else {
                    cmd_index++;
                    if (cmd_index >= sizeof(command) - 1) {
                        cmd_index = 0; // Unngå buffer overflow
                    }
                }
            }
        }
        
        // Håndtere utgående kommandoer vil bli implementert senere
    }

    // Lukk forbindelsen når løkken avsluttes
    xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
    tcp_poles[pole_idx].connected = false;
    
    // Logg MAC-adressen som ble frakoblet, men behold den for debugging
    if (strlen(tcp_poles[pole_idx].nominal_mac) > 0) {
        ESP_LOGI(TAG, "Målestolpe %d med nominell MAC: %s koblet fra", 
                 pole_idx, tcp_poles[pole_idx].nominal_mac);
    }
    
    // Behold nominal_mac for debugging, men sett mac til tom
    tcp_poles[pole_idx].mac[0] = '\0';
    xSemaphoreGive(tcp_poles_mutex);
    
    close(sock);
    ESP_LOGI(TAG, "Målestolpe %d koblet fra, frigjort", pole_idx);
    vTaskDelete(NULL);


}

// TCP-server som lytter etter innkommende forbindelser
static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Kunne ikke opprette socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket opprettet");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket kunne ikke bindes: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bundet, port %d", PORT);

    err = listen(listen_sock, 2);
    if (err != 0) {
        ESP_LOGE(TAG, "Feil under lytting: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "TCP-server lytter på port %d...", PORT);

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Kunne ikke akseptere forbindelse: errno %d", errno);
            break;
        }

        // Sett TCP keepalive
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        
        // Konverter IP-adresse til streng
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket akseptert, IP-adresse: %s", addr_str);

        // Opprett en ny task for å håndtere klienten
        int *socket_ptr = malloc(sizeof(int));
        if (socket_ptr == NULL) {
            ESP_LOGE(TAG, "Feil ved allokering av socket_ptr");
            close(sock);
            continue;
        }
        *socket_ptr = sock;
        xTaskCreatePinnedToCore(tcp_client_handler, "tcp_client_handler", 4096, socket_ptr, 5, NULL, 1);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}


// 
// 
//--------------- Del 4: API Handlers (oppdaterte versjoner) ---------------- 
// 
// 

// Funksjon for å håndtere tid-synkronisering API (oppdatert versjon)
esp_err_t time_sync_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "time_sync_handler called, content_len: %d", remaining);
    
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
    
    // Parse JSON data for timestamp
    int timestamp = 0;
    char *timestamp_start = strstr(buf, "\"timestamp\":");
    if (timestamp_start) {
        timestamp_start += 12; // Hopp over "timestamp":
        timestamp = atoi(timestamp_start);
    }
    
    ESP_LOGI(TAG, "Parsed timestamp: %d", timestamp);
    
    if (timestamp > 0) {
        // Vi sender tidsynkroniseringskommando til alle tilkoblede målestolper
        
        xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
        
        for (int i = 0; i < MAX_TCP_POLES; i++) {
            if (tcp_poles[i].connected) {
                // Bygg kommandoen som skal sendes til målestolpen
                char command[64];
                sprintf(command, "time: %d\n", timestamp + 1); // +1 for å kompensere for noe latens
                
                // Send direkte til socket
                int sock = tcp_poles[i].sock;
                int to_write = strlen(command);
                int res = send(sock, command, to_write, 0);
                
                if (res == to_write) {
                    ESP_LOGI(TAG, "Tid synkronisert til målestolpe %d (MAC: %s): %s", 
                             i, tcp_poles[i].mac, command);
                } else {
                    ESP_LOGE(TAG, "Feil ved sending av tidskommando: %d (errno: %d)", res, errno);
                }
            }
        }
        
        xSemaphoreGive(tcp_poles_mutex);
        
        // Sett også lokal tid
        struct timeval tv;
        tv.tv_sec = timestamp;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        
        // Send ESP-NOW-synkronisering også (vi beholder den eksisterende funksjonaliteten)
        init_esp_now(); // Dette vil sende en synkkommando til alle ESP-NOW-enheter
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Time synchronized\"}");
    
    return ESP_OK;
}

// Funksjon for å håndtere time/check API (beholder den eksisterende implementasjonen)
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

// Funksjon for å håndtere hsearch API (oppdatert for å sende til TCP-forbindelser)
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
    
    // Parse JSON data
    int channel = 0;
    char *channel_start = strstr(buf, "\"channel\":");
    if (channel_start) {
        channel_start += 10; // Hopp over "channel":
        channel = atoi(channel_start);
    }
    
    // Send hsearch kommando til alle tilkoblede målestolper
    xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
    
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        if (tcp_poles[i].connected) {
            // Bygg kommando
            char command[64];
            sprintf(command, "hsearch: %d\n", channel);
            
            // Send direkte til socket
            int sock = tcp_poles[i].sock;
            int to_write = strlen(command);
            int res = send(sock, command, to_write, 0);
            
            if (res == to_write) {
                ESP_LOGI(TAG, "Hsearch kommando sendt til målestolpe %d: %s", i, command);
            } else {
                ESP_LOGE(TAG, "Feil ved sending av hsearch kommando: %d (errno: %d)", res, errno);
            }
        }
    }
    
    xSemaphoreGive(tcp_poles_mutex);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Highpoint search initiated\"}");
    
    return ESP_OK;
}

// Oppdatert versjon av pole_break_handler
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
    
    // Parse JSON data (enkel implementasjon - bør bruke cJSON i en fullstendig løsning)
    char mac[18] = {0};
    int adc = -1;
    int break_val = -1;
    
    // Finn MAC-adressen i JSON-stringen
    char *mac_start = strstr(buf, "\"mac\":\"");
    if (mac_start) {
        mac_start += 7; // Hopp over "mac":"
        char *mac_end = strchr(mac_start, '\"');
        if (mac_end && (mac_end - mac_start) < 18) {
            strncpy(mac, mac_start, mac_end - mac_start);
            mac[mac_end - mac_start] = '\0';
        }
    }
    
    // Finn ADC-verdien
    char *adc_start = strstr(buf, "\"adc\":");
    if (adc_start) {
        adc_start += 6; // Hopp over "adc":
        adc = atoi(adc_start);
    }
    
    // Finn break-verdien
    char *break_start = strstr(buf, "\"break\":");
    if (break_start) {
        break_start += 8; // Hopp over "break":
        break_val = atoi(break_start);
    }
    
    ESP_LOGI(TAG, "Parsed values - MAC: %s, ADC: %d, Break: %d", mac, adc, break_val);
    
    if (strlen(mac) > 0 && adc >= 0 && break_val >= 0) {
        // Bygg kommandoen som skal sendes til målestolpen
        char command[64];
        sprintf(command, "break: %d %d\n", adc, break_val);
        
        // Send kommandoen
        if (send_command_to_pole_by_mac(mac, command)) {
            ESP_LOGI(TAG, "Kommando sendt: %s", command);
        } else {
            ESP_LOGW(TAG, "Kunne ikke sende kommando - målestolpe ikke tilkoblet");
        }
    } else {
        ESP_LOGW(TAG, "Ugyldig forespørsel - mangler data");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Break configuration updated\"}");
    
    return ESP_OK;
}


// 
// 
//--------------- Del 5: Flere API Handlers og WebSocket-håndtering ---------------- 
// 
// 

// Oppdatert versjon av pole_enabled_handler
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
   
   // Parse JSON data
   char mac[18] = {0};
   int sensor_nr = -1;
   int enabled = -1;
   
   // Finn MAC-adressen
   char *mac_start = strstr(buf, "\"mac\":\"");
   if (mac_start) {
       mac_start += 7; // Hopp over "mac":"
       char *mac_end = strchr(mac_start, '\"');
       if (mac_end && (mac_end - mac_start) < 18) {
           strncpy(mac, mac_start, mac_end - mac_start);
           mac[mac_end - mac_start] = '\0';
       }
   }
   
   // Finn sensor_nr
   char *sensor_start = strstr(buf, "\"sensor_nr\":");
   if (sensor_start) {
       sensor_start += 12; // Hopp over "sensor_nr":
       sensor_nr = atoi(sensor_start);
   }
   
   // Finn enabled
   char *enabled_start = strstr(buf, "\"enabled\":");
   if (enabled_start) {
       enabled_start += 10; // Hopp over "enabled":
       enabled = atoi(enabled_start);
   }
   
   ESP_LOGI(TAG, "Parsed values - MAC: %s, Sensor: %d, Enabled: %d", mac, sensor_nr, enabled);
   
   if (strlen(mac) > 0 && sensor_nr >= 0 && (enabled == 0 || enabled == 1)) {
       // Bygg kommandoen som skal sendes til målestolpen
       char command[64];
       sprintf(command, "enabled: %d %d\n", sensor_nr, enabled);
       
       // Send kommandoen
       if (send_command_to_pole_by_mac(mac, command)) {
           ESP_LOGI(TAG, "Kommando sendt: %s", command);
       } else {
           ESP_LOGW(TAG, "Kunne ikke sende kommando - målestolpe ikke tilkoblet");
       }
   } else {
       ESP_LOGW(TAG, "Ugyldig forespørsel - mangler data");
   }
   
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

// WebSocket-relaterte funksjoner
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



// 
// 
//--------------- Del 6: Filsystem og testing ---------------- 
// 
//

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
   
   // Sjekk /www/assets-mappen
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
   
   // Sjekk TCP-forbindelser
   httpd_resp_sendstr_chunk(req, "\nTCP Connection Status:\n");
   
   xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
   for (int i = 0; i < MAX_TCP_POLES; i++) {
       char pole_info[150];
       if (tcp_poles[i].connected) {
           snprintf(pole_info, sizeof(pole_info), "  Pole %d: Connected, Socket: %d, MAC: %s, Nominal MAC: %s\n", 
                    i, tcp_poles[i].sock, 
                    tcp_poles[i].mac[0] ? tcp_poles[i].mac : "Unknown",
                    tcp_poles[i].nominal_mac[0] ? tcp_poles[i].nominal_mac : "Not assigned");
       } else {
           snprintf(pole_info, sizeof(pole_info), "  Pole %d: Not connected (Last nominal MAC: %s)\n", 
                    i, tcp_poles[i].nominal_mac[0] ? tcp_poles[i].nominal_mac : "None");
       }
       httpd_resp_sendstr_chunk(req, pole_info);
   }
   xSemaphoreGive(tcp_poles_mutex);





   
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



// 
// 
//--------------- Del 7: ESP-NOW og Test-funksjoner ---------------- 
// 
//

// ESP-NOW mottaker callback
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
   // Hent MAC-adressen fra recv_info
   const uint8_t *mac_addr = recv_info->src_addr;
   
   ESP_LOGI(TAG, "ESP-NOW data mottatt fra %02x:%02x:%02x:%02x:%02x:%02x, len=%d", 
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len);
   
   // Sjekk at lengden er riktig for vår struktur
   if (len < 10) {
       ESP_LOGE(TAG, "Mottok data med feil lengde: %d (forventet minst 10 bytes)", len);
       return;
   }
   
   // Logg raw data for debugging
   ESP_LOGI(TAG, "Data header: k=%d, t=%02x%02x%02x%02x, u=%02x%02x%02x%02x", 
            data[0], 
            data[1], data[2], data[3], data[4],
            data[5], data[6], data[7], data[8]);
   
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
       ESP_LOGI(TAG, "Ny målestolpe lagt til, indeks=%d, totalt: %d", pole_idx, pole_count);
   } else if (pole_idx < 0) {
       ESP_LOGW(TAG, "Maks antall målestolper nådd (%d), ignorerer ny tilkobling", MAX_POLES);
       xSemaphoreGive(pole_data_mutex);
       return;
   }
   
   // Parse data
   uint8_t k = data[0];
   pole_data[pole_idx].k = k;
   
   // Les timestamp
   memcpy(&pole_data[pole_idx].t, &data[1], sizeof(uint32_t));
   memcpy(&pole_data[pole_idx].u, &data[5], sizeof(uint32_t));
   
   ESP_LOGI(TAG, "Mottok data type %d fra målestolpe %d, timestamp=%u.%06u", 
           k, pole_idx, pole_data[pole_idx].t, pole_data[pole_idx].u);
   
   // Les ADC verdier eller andre data basert på type melding
   if (k == 0 && len >= 9 + sizeof(int32_t) * 14) {  // ADC verdier
       memcpy(pole_data[pole_idx].v, &data[9], sizeof(int32_t) * 7);
       memcpy(pole_data[pole_idx].b, &data[9 + sizeof(int32_t) * 7], sizeof(int32_t) * 7);
       
       ESP_LOGI(TAG, "ADC verdier: [%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 "]", 
               pole_data[pole_idx].v[0], pole_data[pole_idx].v[1], pole_data[pole_idx].v[2],
               pole_data[pole_idx].v[3], pole_data[pole_idx].v[4], pole_data[pole_idx].v[5],
               pole_data[pole_idx].v[6]);
       ESP_LOGI(TAG, "Broken status: [%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 "]", 
               pole_data[pole_idx].b[0], pole_data[pole_idx].b[1], pole_data[pole_idx].b[2],
               pole_data[pole_idx].b[3], pole_data[pole_idx].b[4], pole_data[pole_idx].b[5],
               pole_data[pole_idx].b[6]);

       // Konverter mottatte ESP-NOW data til JSON og send via WebSocket
       char mac_str[18];
       sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
       
       char ws_msg[512];
       sprintf(ws_msg, 
               "{\"M\":\"%s\",\"K\":0,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"V\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],\"B\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}",
               mac_str, pole_data[pole_idx].t, pole_data[pole_idx].u,
               pole_data[pole_idx].v[0], pole_data[pole_idx].v[1], pole_data[pole_idx].v[2], 
               pole_data[pole_idx].v[3], pole_data[pole_idx].v[4], pole_data[pole_idx].v[5], pole_data[pole_idx].v[6],
               pole_data[pole_idx].b[0], pole_data[pole_idx].b[1], pole_data[pole_idx].b[2], 
               pole_data[pole_idx].b[3], pole_data[pole_idx].b[4], pole_data[pole_idx].b[5], pole_data[pole_idx].b[6]);
       
       // Send til alle WebSocket klienter
       httpd_ws_frame_t ws_pkt = {
           .final = true,
           .fragmented = false,
           .type = HTTPD_WS_TYPE_TEXT,
           .payload = (uint8_t *)ws_msg,
           .len = strlen(ws_msg)
       };

       xSemaphoreTake(ws_mutex, portMAX_DELAY);
       for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
           if (ws_clients[i] != 0) {
               httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
           }
       }
       xSemaphoreGive(ws_mutex);
   } else if (k == 1) {  // Break event
       // Antar at en enkelt break-verdi fins i data[9]
       int32_t break_value;
       memcpy(&break_value, &data[9], sizeof(int32_t));
       ESP_LOGI(TAG, "Break event mottatt: verdi=%" PRId32, break_value);

       // Send break event som JSON til WebSocket
       char mac_str[18];
       sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
       
       char ws_msg[256];
       sprintf(ws_msg, 
               "{\"M\":\"%s\",\"K\":1,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"B\":%" PRId32 "}",
               mac_str, pole_data[pole_idx].t, pole_data[pole_idx].u, break_value);
       
       // Send til alle WebSocket klienter
       httpd_ws_frame_t ws_pkt = {
           .final = true,
           .fragmented = false,
           .type = HTTPD_WS_TYPE_TEXT,
           .payload = (uint8_t *)ws_msg,
           .len = strlen(ws_msg)
       };

       xSemaphoreTake(ws_mutex, portMAX_DELAY);
       for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
           if (ws_clients[i] != 0) {
               httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
           }
       }
       xSemaphoreGive(ws_mutex);
   } else if (k == 2 && len >= 9 + sizeof(int32_t) * 21) {  // Settings
       memcpy(pole_data[pole_idx].e, &data[9], sizeof(int32_t) * 7);
       memcpy(pole_data[pole_idx].o, &data[9 + sizeof(int32_t) * 7], sizeof(int32_t) * 7);
       memcpy(pole_data[pole_idx].b, &data[9 + sizeof(int32_t) * 14], sizeof(int32_t) * 7);
       
       ESP_LOGI(TAG, "Settings mottatt for målestolpe %d:", pole_idx);
       ESP_LOGI(TAG, "  Enabled: [%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 "]", 
               pole_data[pole_idx].e[0], pole_data[pole_idx].e[1], pole_data[pole_idx].e[2],
               pole_data[pole_idx].e[3], pole_data[pole_idx].e[4], pole_data[pole_idx].e[5],
               pole_data[pole_idx].e[6]);

       // Send settings som JSON til WebSocket
       char mac_str[18];
       sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
       
       char ws_msg[512];
       sprintf(ws_msg, 
               "{\"M\":\"%s\",\"K\":2,\"E\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],\"O\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],\"B\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}",
               mac_str, 
               pole_data[pole_idx].e[0], pole_data[pole_idx].e[1], pole_data[pole_idx].e[2], 
               pole_data[pole_idx].e[3], pole_data[pole_idx].e[4], pole_data[pole_idx].e[5], pole_data[pole_idx].e[6],
               pole_data[pole_idx].o[0], pole_data[pole_idx].o[1], pole_data[pole_idx].o[2], 
               pole_data[pole_idx].o[3], pole_data[pole_idx].o[4], pole_data[pole_idx].o[5], pole_data[pole_idx].o[6],
               pole_data[pole_idx].b[0], pole_data[pole_idx].b[1], pole_data[pole_idx].b[2], 
               pole_data[pole_idx].b[3], pole_data[pole_idx].b[4], pole_data[pole_idx].b[5], pole_data[pole_idx].b[6]);
       
       // Send til alle WebSocket klienter
       httpd_ws_frame_t ws_pkt = {
           .final = true,
           .fragmented = false,
           .type = HTTPD_WS_TYPE_TEXT,
           .payload = (uint8_t *)ws_msg,
           .len = strlen(ws_msg)
       };

       xSemaphoreTake(ws_mutex, portMAX_DELAY);
       for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
           if (ws_clients[i] != 0) {
               httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
           }
       }
       xSemaphoreGive(ws_mutex);
   } else {
       ESP_LOGW(TAG, "Ukjent melding eller ugyldig lengde: k=%d, len=%d", k, len);
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

// WebSocket test page handler
esp_err_t websocket_test_page_handler(httpd_req_t *req) {
   ESP_LOGI(TAG, "WebSocket test page requested");
   
   // En enkel HTML-side med JavaScript for WebSocket-testing
   const char *html = "<!DOCTYPE html>\n"
       "<html>\n"
       "<head>\n"
       "    <title>Timergate WebSocket Test</title>\n"
       "    <style>\n"
       "        body { font-family: Arial, sans-serif; margin: 20px; }\n"
       "        #messages { border: 1px solid #ccc; padding: 10px; height: 300px; overflow-y: auto; margin-bottom: 10px; }\n"
       "        .time { color: #888; font-size: 0.8em; }\n"
       "        .error { color: red; }\n"
       "        .connected { color: green; }\n"
       "        .json { font-family: monospace; white-space: pre-wrap; }\n"
       "    </style>\n"
       "</head>\n"
       "<body>\n"
       "    <h1>Timergate WebSocket Test</h1>\n"
       "    <div>\n"
       "        <button id='connect'>Koble til</button>\n"
       "        <button id='disconnect'>Koble fra</button>\n"
       "        <span id='status'>Ikke tilkoblet</span>\n"
       "    </div>\n"
       "    <h2>Meldinger:</h2>\n"
       "    <div id='messages'></div>\n"
       "    <script>\n"
       "        let socket;\n"
       "        let messageCount = 0;\n"
       "        let maxMessages = 100;\n"
       "\n"
       "        function addMessage(text, className) {\n"
       "            const messages = document.getElementById('messages');\n"
       "            const time = new Date().toLocaleTimeString();\n"
       "            const msg = document.createElement('div');\n"
       "\n"
       "            let content = `<span class='time'>${time}</span> `;\n"
       "            \n"
       "            if (className === 'json') {\n"
       "                try {\n"
       "                    // Formater JSON for bedre lesbarhet\n"
       "                    const jsonObj = JSON.parse(text);\n"
       "                    text = JSON.stringify(jsonObj, null, 2);\n"
       "                } catch (e) {\n"
       "                    // Hvis ikke JSON, vis som vanlig tekst\n"
       "                }\n"
       "            }\n"
       "            \n"
       "            content += `<span class='${className}'>${text}</span>`;\n"
       "            msg.innerHTML = content;\n"
       "            messages.appendChild(msg);\n"
       "            messages.scrollTop = messages.scrollHeight;\n"
       "            \n"
       "            // Begrens antall meldinger for å unngå minneproblemer\n"
       "            messageCount++;\n"
       "            if (messageCount > maxMessages) {\n"
       "                messages.removeChild(messages.firstChild);\n"
       "                messageCount--;\n"
       "            }\n"
       "        }\n"
       "\n"
       "        document.getElementById('connect').addEventListener('click', function() {\n"
       "            if (socket && socket.readyState <= 1) {\n"
       "                addMessage('Allerede tilkoblet eller kobler til', 'error');\n"
       "                return;\n"
       "            }\n"
       "\n"
       "            const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';\n"
       "            const wsUrl = `${wsProtocol}//${window.location.host}/ws`;\n"
       "            addMessage(`Kobler til ${wsUrl}`, 'normal');\n"
       "\n"
       "            socket = new WebSocket(wsUrl);\n"
       "\n"
       "            socket.onopen = function() {\n"
       "                document.getElementById('status').textContent = 'Tilkoblet';\n"
       "                document.getElementById('status').style.color = 'green';\n"
       "                addMessage('WebSocket tilkoblet', 'connected');\n"
       "            };\n"
       "\n"
       "            socket.onmessage = function(event) {\n"
       "                addMessage(event.data, 'json');\n"
       "            };\n"
       "\n"
       "            socket.onclose = function() {\n"
       "                document.getElementById('status').textContent = 'Frakoblet';\n"
       "                document.getElementById('status').style.color = 'red';\n"
       "                addMessage('WebSocket frakoblet', 'error');\n"
       "            };\n"
       "\n"
       "            socket.onerror = function(error) {\n"
       "                addMessage('WebSocket feil: ' + error, 'error');\n"
       "            };\n"
       "        });\n"
       "\n"
       "        document.getElementById('disconnect').addEventListener('click', function() {\n"
       "            if (socket) {\n"
       "                socket.close();\n"
       "                addMessage('WebSocket lukket manuelt', 'normal');\n"
       "            }\n"
       "        });\n"
       "    </script>\n"
       "</body>\n"
       "</html>";
   
   httpd_resp_set_type(req, "text/html");
   httpd_resp_send(req, html, strlen(html));
   
   return ESP_OK;
}

// Funksjon for å simulere data for testing
esp_err_t simulate_pole_data_handler(httpd_req_t *req) {
   ESP_LOGI(TAG, "Simulate pole data handler called");
   
   // Opprett en simulert målestolpe hvis det ikke finnes noen
   xSemaphoreTake(pole_data_mutex, portMAX_DELAY);
   
   if (pole_count == 0) {
       // Lag en simulert stolpe med MAC-adresse aa:bb:cc:dd:ee:ff
       uint8_t mac[ESP_NOW_ETH_ALEN] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
       memcpy(pole_data[0].mac, mac, ESP_NOW_ETH_ALEN);
       pole_count = 1;
       ESP_LOGI(TAG, "Opprettet simulert målestolpe med MAC aa:bb:cc:dd:ee:ff");
   }
   
   // Fyll inn simulerte ADC-verdier
   pole_data[0].k = 0; // ADC-verdier
   pole_data[0].t = time(NULL);
   pole_data[0].u = 0;
   
   // Simulerte ADC-verdier i et mønster som vil være synlig i GUI
   for (int i = 0; i < 7; i++) {
       pole_data[0].v[i] = 2000 + (i * 300);  // Økende verdier
       pole_data[0].b[i] = (i % 2);  // Veksler mellom 0 og 1
   }
   
   // Simulerte innstillinger hvis de ikke er satt
   for (int i = 0; i < 7; i++) {
       pole_data[0].e[i] = 1;  // Alt er aktivert
       pole_data[0].o[i] = 4000;  // Standard offset
   }
   
   ESP_LOGI(TAG, "Simulerte ADC-verdier opprettet: [%d, %d, %d, %d, %d, %d, %d]",
           pole_data[0].v[0], pole_data[0].v[1], pole_data[0].v[2],
           pole_data[0].v[3], pole_data[0].v[4], pole_data[0].v[5],
           pole_data[0].v[6]);
   
   // Send simulerte ADC-verdier via WebSocket
   char mac_str[18];
   sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
           pole_data[0].mac[0], pole_data[0].mac[1], pole_data[0].mac[2],
           pole_data[0].mac[3], pole_data[0].mac[4], pole_data[0].mac[5]);
   
   char ws_msg[512];
   sprintf(ws_msg, 
           "{\"M\":\"%s\",\"K\":0,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"V\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],\"B\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}",
           mac_str, pole_data[0].t, pole_data[0].u,
           pole_data[0].v[0], pole_data[0].v[1], pole_data[0].v[2], 
           pole_data[0].v[3], pole_data[0].v[4], pole_data[0].v[5], pole_data[0].v[6],
           pole_data[0].b[0], pole_data[0].b[1], pole_data[0].b[2], 
           pole_data[0].b[3], pole_data[0].b[4], pole_data[0].b[5], pole_data[0].b[6]);
   
   xSemaphoreGive(pole_data_mutex);
   
   // Send til WebSocket-klienter
   httpd_ws_frame_t ws_pkt = {
       .final = true,
       .fragmented = false,
       .type = HTTPD_WS_TYPE_TEXT,
       .payload = (uint8_t *)ws_msg,
       .len = strlen(ws_msg)
   };

   xSemaphoreTake(ws_mutex, portMAX_DELAY);
   for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
       if (ws_clients[i] != 0) {
           httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
       }
   }
   xSemaphoreGive(ws_mutex);
   
   // Simuler et break event (k=1) etter et kort intervall
   vTaskDelay(pdMS_TO_TICKS(1000));
   
   xSemaphoreTake(pole_data_mutex, portMAX_DELAY);
   pole_data[0].k = 1;  // Break event
   pole_data[0].t = time(NULL);
   pole_data[0].u = 500000;  // 500 ms
   
   // Send et simulert break event via WebSocket
   char break_msg[256];
   sprintf(break_msg, 
           "{\"M\":\"%s\",\"K\":1,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"B\":1}",
           mac_str, pole_data[0].t, pole_data[0].u);
   
   xSemaphoreGive(pole_data_mutex);
   
   ws_pkt.payload = (uint8_t *)break_msg;
   ws_pkt.len = strlen(break_msg);
   
   xSemaphoreTake(ws_mutex, portMAX_DELAY);
   for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
       if (ws_clients[i] != 0) {
           httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
       }
   }
   xSemaphoreGive(ws_mutex);
   
   ESP_LOGI(TAG, "Simulert break event sendt: %s", break_msg);
   
   // Send respons til klienten
   httpd_resp_set_type(req, "application/json");
   httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
   httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Simulerte data opprettet\"}");
   
   return ESP_OK;
}



// 
// 
//--------------- Del 8: Server og Registrering av URL-handlers ---------------- 
// 
//

httpd_handle_t start_webserver(void) {
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   config.uri_match_fn = httpd_uri_match_wildcard; //Important when when gui-files is hashed

   // Øk maks URI-handlere
   config.max_uri_handlers = 20; // Økt fra 6 til 12 for å ha plass til flere handlere
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
           .uri = "/assets/*.js",  //when config.uri_match_fn is set
           .method = HTTP_GET,
           .handler = spiffs_get_handler,
           .user_ctx = "/www"
       };
       httpd_register_uri_handler(server_handle, &js_files);
       
       // Spesifikk handler for CSS-filer
       httpd_uri_t css_files = {
           .uri = "/assets/*.css", //when config.uri_match_fn is set
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
       
       // Simulere polling data API
       httpd_uri_t simulate_pole = {
           .uri = "/api/v1/debug/simulate",
           .method = HTTP_GET,
           .handler = simulate_pole_data_handler,
           .user_ctx = NULL
       };
       httpd_register_uri_handler(server_handle, &simulate_pole);

       // WebSocket test page
       httpd_uri_t ws_test = {
           .uri = "/ws_test",
           .method = HTTP_GET,
           .handler = websocket_test_page_handler,
           .user_ctx = NULL
       };
       httpd_register_uri_handler(server_handle, &ws_test);
       
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




// 
// 
//--------------- Del 9: Main-funksjon ---------------- 
// 
// 


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
    pole_data_mutex = xSemaphoreCreateMutex();
    tcp_poles_mutex = xSemaphoreCreateMutex();
    
    // Initialiser arrays
    memset(ws_clients, 0, sizeof(ws_clients));
    memset(pole_data, 0, sizeof(pole_data));
    memset(tcp_poles, 0, sizeof(tcp_poles));
    // Sikre at alle nominal_mac-felt er initialisert
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        tcp_poles[i].nominal_mac[0] = '\0';
    }

    // Initialiser filsystemet
    ESP_ERROR_CHECK(init_fs());
    
    // Start TCP-serveroppgaven
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    
    // Initialiser ESP-NOW
    init_esp_now();

    // Start webserveren
    server = start_webserver();
}