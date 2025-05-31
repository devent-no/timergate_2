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
#include "cJSON.h"
#include "lwip/sockets.h"
#include "esp_spiffs.h"
#include "lwip/sockets.h"
#include "esp_netif_net_stack.h"

#include "mdns.h"
#include "esp_timer.h"

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



// Struktur for å lagre siste brudd for debouncing
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];  // MAC-adresse til målestolpen
    int32_t sensor_id;              // Sensor-ID (0-6)
    uint32_t last_break_time;       // Tidspunkt for siste brudd (sekunder)
    uint32_t last_break_micros;     // Mikrosekunder del
} last_break_record_t;

// Array for å holde siste brudd per sensor/målestolpe
#define MAX_SENSORS_PER_POLE 7
static last_break_record_t last_breaks[MAX_POLES * MAX_SENSORS_PER_POLE];
static int last_break_count = 0;

// Konfigurerbar debounce-tid i millisekunder
static int break_debounce_ms = 1000; // Standard: 1 sekund





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



// Struktur for å spore passeringsdeteksjon
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];       // MAC-adresse til målestolpen
    bool sensor_triggered[MAX_SENSORS_PER_POLE]; // Status for hver sensor
    int unique_sensors_count;            // Antall unike sensorer utløst i sekvensen
    uint32_t first_break_time;           // Tidspunkt for første brudd i sekvensen (sekunder)
    uint32_t first_break_micros;         // Mikrosekunder del av tidspunkt for første brudd
    uint32_t sequence_start_time;        // Starttidspunkt for gjeldende sekvens
    uint32_t last_passage_time;          // Tidspunkt for siste passering (for debouncing)
    uint32_t last_passage_micros;        // Mikrosekunder del av tidspunkt for siste passering
    // Legg til disse nye variablene:
    bool use_sensor_time_scale;          // Sant hvis vi bruker sensor-tid for denne sekvensen
    uint32_t last_sensor_time;           // Siste tidspunkt fra sensoren
    uint32_t last_sensor_micros;         // Mikrosekunder del    
    int first_sensor_id_candidate;
} passage_detection_t;

// Array for å holde passeringsdeteksjonsstatus per målestolpe
#define MAX_PASSAGE_DETECTORS MAX_POLES
static passage_detection_t passage_detectors[MAX_PASSAGE_DETECTORS];
//static uint32_t passage_counter = 0; // Teller for unike passeringer, er vist ikke i bruk lenger...
static int passage_detector_count = 0;
static SemaphoreHandle_t passage_mutex;



// Ny struktur for stoppeklokke-passeringer med dual timing
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];     // MAC-adresse til målestolpen
    uint32_t server_time_sec;          // Server-tid for stoppeklokke-beregninger
    uint32_t server_time_usec;         // Server-mikrosekunder
    uint32_t pole_time_sec;            // Målestolpe-tid for logging/debugging
    uint32_t pole_time_usec;           // Målestolpe-mikrosekunder
    int sensor_count;                  // Antall sensorer som utløste
    uint32_t passage_id;               // Unik ID for denne passeringen
} stopwatch_passage_t;

#define MAX_STOPWATCH_PASSAGES 10      // Maks antall stoppeklokke-passeringer
static stopwatch_passage_t stopwatch_passages[MAX_STOPWATCH_PASSAGES];
static int stopwatch_passage_count = 0;
static uint32_t next_passage_id = 1;   // Teller for unike passage-ID-er
static SemaphoreHandle_t stopwatch_mutex;

// Strukt for å holde styr på sendte passeringer (duplikatdeteksjon)
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];     // MAC-adresse til målestolpen
    uint32_t timestamp_sec;            // Tidspunkt for passeringen (sekunder)
    uint32_t timestamp_micros;         // Mikrosekunder del
    int sensors_count;                 // Antall sensorer som utløste
    time_t send_time;                  // Når passeringen ble sendt (for cleanup)
} sent_passage_t;

#define MAX_SENT_PASSAGES 20           // Hvor mange tidligere passeringer vi holder styr på
static sent_passage_t sent_passages[MAX_SENT_PASSAGES];
static int sent_passage_count = 0;
static SemaphoreHandle_t sent_passages_mutex;





// Konfigurasjon for passeringsdeteksjon
static int passage_debounce_ms = 1000;   // Standard debounce-tid: 1 sekund
static int min_sensors_for_passage = 2;  // Standard minimum antall sensorer
static int sequence_timeout_ms = 500;   // Timeout for sekvens: 5 sekunder



// Datastruktur for passeringshistorikk
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint32_t timestamp_sec;
    uint32_t timestamp_usec;
    int sensors_count;
    int sensor_ids[MAX_SENSORS_PER_POLE];  // Hvilke sensorer som utløste
    uint32_t time_since_previous_ms;       // Tid siden forrige i millisekunder
} passage_history_t;

#define MAX_PASSAGE_HISTORY 20
static passage_history_t passage_history[MAX_PASSAGE_HISTORY];
static int passage_history_count = 0;
static uint32_t total_passages = 0;
static SemaphoreHandle_t passage_history_mutex;



// Globale variabler for å lagre data fra målestolpene
static pole_data_t pole_data[MAX_POLES];
static int pole_count = 0;
static SemaphoreHandle_t pole_data_mutex;

static tcp_pole_t tcp_poles[MAX_TCP_POLES];
static SemaphoreHandle_t tcp_poles_mutex;

static const char *TAG = "timergate-ap";


static void log_event_passering(const char *mac, int sensor_id, const char *status,
                                double timestamp, int count, int debounce_ms,
                                int sensors_used[], int sensor_count)
{
    if (strcmp(status, "PASSERING") == 0) {
        ESP_LOGI(TAG, "🎯 PASSERING: MAC=%s, sensor=%d, tid=%.3f, sensorer utløst=%d [",
                 mac, sensor_id, timestamp, sensor_count);
        for (int i = 0; i < sensor_count; i++) {
            printf("%d%s", sensors_used[i], (i < sensor_count - 1) ? "," : "");
        }
        printf("]\n");
    } else if (strcmp(status, "TIMEOUT") == 0) {
        ESP_LOGW(TAG, "⏱️ TIMEOUT: MAC=%s, sensor=%d, tid=%.3f, sensorer utløst=%d",
                 mac, sensor_id, timestamp, count);
    } else if (strcmp(status, "DEBOUNCE") == 0) {
        ESP_LOGI(TAG, "⛔ DEBOUNCE: MAC=%s, sensor=%d, %d ms etter forrige passering",
                 mac, sensor_id, debounce_ms);
    } else if (strcmp(status, "IGNORED") == 0) {
        ESP_LOGD(TAG, "ℹ️ Ignorert sensor-gjenoppretting: MAC=%s, sensor=%d", mac, sensor_id);
    } else {
        ESP_LOGI(TAG, "📍 EVENT: MAC=%s, sensor=%d, status=%s, tid=%.3f",
                 mac, sensor_id, status, timestamp);
    }
}



// Funksjonsdeklarasjoner
void init_esp_now(void);
esp_err_t simulate_pole_data_handler(httpd_req_t *req);
esp_err_t websocket_test_page_handler(httpd_req_t *req);

// Nye funksjonsdeklarasjoner for debounce-håndtering
bool should_ignore_break(const uint8_t *mac, int32_t sensor_id, uint32_t time_sec, uint32_t time_micros);
void send_break_to_websocket(const uint8_t *mac_addr, uint32_t time_sec, uint32_t time_micros, int32_t sensor_id, bool filtered);
//bool process_break_for_passage_detection(const uint8_t *mac_addr, int32_t sensor_id, uint32_t time_sec, uint32_t time_micros);
bool process_break_for_passage_detection(const uint8_t *mac_addr, int32_t sensor_id, 
                                        uint32_t sensor_time_sec, uint32_t sensor_time_micros,
                                        uint32_t server_time_sec, uint32_t server_time_micros);



// Funksjon for å legge til passering i historikk
void add_to_passage_history(const uint8_t *mac, uint32_t time_sec, uint32_t time_usec, 
                           int *sensor_ids, int sensor_count) {
    xSemaphoreTake(passage_history_mutex, portMAX_DELAY);
    
    // Beregn tid siden forrige passering
    uint32_t time_since_previous_ms = 0;
    if (passage_history_count > 0) {
        uint64_t current_time_ms = (uint64_t)time_sec * 1000 + time_usec / 1000;
        uint64_t last_time_ms = (uint64_t)passage_history[0].timestamp_sec * 1000 + 
                               passage_history[0].timestamp_usec / 1000;
        time_since_previous_ms = (uint32_t)(current_time_ms - last_time_ms);
    }
    
    // Flytt eksisterende data ned
    for (int i = MIN(passage_history_count, MAX_PASSAGE_HISTORY - 1); i > 0; i--) {
        passage_history[i] = passage_history[i-1];
    }
    
    // Legg til ny passering øverst
    memcpy(passage_history[0].mac, mac, ESP_NOW_ETH_ALEN);
    passage_history[0].timestamp_sec = time_sec;
    passage_history[0].timestamp_usec = time_usec;
    passage_history[0].sensors_count = sensor_count;
    passage_history[0].time_since_previous_ms = time_since_previous_ms;
    
    for (int i = 0; i < sensor_count && i < MAX_SENSORS_PER_POLE; i++) {
        passage_history[0].sensor_ids[i] = sensor_ids[i];
    }
    
    if (passage_history_count < MAX_PASSAGE_HISTORY) {
        passage_history_count++;
    }
    total_passages++;
    
    xSemaphoreGive(passage_history_mutex);
}


// Nye funksjonsdeklarasjoner for håndtering av sendte passeringer
bool is_passage_already_sent(const uint8_t *mac, uint32_t time_sec, uint32_t time_micros, int sensors_count);
void register_sent_passage(const uint8_t *mac, uint32_t time_sec, uint32_t time_micros, int sensors_count);
void clean_old_passages(void *pvParameters);

// Nye funksjonsdeklarasjoner for dual-timing stoppeklokke
void register_passage_for_stopwatch(const uint8_t *mac_addr, uint32_t server_sec, uint32_t server_usec,
                                   uint32_t pole_sec, uint32_t pole_usec, int sensor_count);
void log_passage_with_dual_timing(const char *mac_str, int sensor_id,
                                 uint32_t pole_time_sec, uint32_t pole_time_usec,
                                 uint32_t server_time_sec, uint32_t server_time_usec,
                                 int *sensors_used, int sensor_count);
uint64_t calculate_stopwatch_time_ms(const uint8_t *mac1, const uint8_t *mac2);


static httpd_handle_t global_server = NULL;
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
        .max_files = 20,  // Økt fra 5 til 10 for å støtte flere filer
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
                    
                    // Sjekk om dette er et break-event
                    if (strncmp(command, "break:", 6) == 0) {
                        // Parse sensor_id og break_value
                        int sensor_id = 0;
                        int break_value = 0;
                        if (sscanf(command + 6, "%d %d", &sensor_id, &break_value) == 2) {
                            // Få gjeldende tid
                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            
                            // Send alltid rådata (ufiltrert)
                            uint8_t mac_bytes[ESP_NOW_ETH_ALEN];
                            // Konverterer mac-stringen til bytes hvis nødvendig
                            // For enkelhets skyld bruker vi bare pole_idx her
                            mac_bytes[0] = 0xAA; // Dummy MAC for TCP-sensorer
                            mac_bytes[1] = 0xBB;
                            mac_bytes[2] = 0xCC;
                            mac_bytes[3] = 0xDD;
                            mac_bytes[4] = 0xEE;
                            mac_bytes[5] = pole_idx & 0xFF; // Bruk pole_idx som siste byte
                            
                            send_break_to_websocket(mac_bytes, tv.tv_sec, tv.tv_usec, break_value, false);
                            
                            // Behandle brudd for passeringsdeteksjon
                            struct timeval tv_now;
                            gettimeofday(&tv_now, NULL);
                            process_break_for_passage_detection(mac_bytes, sensor_id, 
                                                            tv.tv_sec, tv.tv_usec,           // Målestolpe-tid
                                                            tv_now.tv_sec, tv_now.tv_usec); // Server-tid
                        }
                    }
                   
                   

                    /* DEAKTIVERT: Bruk kun ESP-NOW for timing-data (TCP har for høy latens)
                    else if (strncmp(command, "{\"K\":1,", 7) == 0) {
                        // Parse JSON-strengen for K=1 (sensor break events)
                        char mac_str[18] = {0};
                        int32_t break_value = -1;
                        int32_t sensor_id = -1;
                        uint32_t time_sec = 0;
                        uint32_t time_usec = 0;
                        
                        ESP_LOGI(TAG, "Mottat K=1 JSON: %s", command);
                        
                        // Finn MAC-adresse
                        char *mac_start = strstr(command, "\"M\":\"");
                        if (mac_start) {
                            mac_start += 5; // Hopp over "M":"
                            char *mac_end = strchr(mac_start, '\"');
                            if (mac_end && (mac_end - mac_start) < 18) {
                                strncpy(mac_str, mac_start, mac_end - mac_start);
                                mac_str[mac_end - mac_start] = '\0';
                            }
                        }
                        
                        // Finn sensor ID (S-feltet)
                        char *sensor_start = strstr(command, "\"S\":");
                        if (sensor_start) {
                            sensor_start += 4; // Hopp over "S":
                            sensor_id = atoi(sensor_start);
                        }
                        
                        // Finn break-verdien (B-feltet)
                        char *break_start = strstr(command, "\"B\":");
                        if (break_start) {
                            break_start += 4; // Hopp over "B":
                            break_value = atoi(break_start);
                        }
                        
                        // Finn tidspunktet (T-feltet)
                        char *time_start = strstr(command, "\"T\":");
                        if (time_start) {
                            time_start += 4; // Hopp over "T":
                            time_sec = atoi(time_start);
                        }
                        
                        // Finn mikrosekunder (U-feltet)
                        char *usec_start = strstr(command, "\"U\":");
                        if (usec_start) {
                            usec_start += 4; // Hopp over "U":
                            time_usec = atoi(usec_start);
                        }
                        
                        ESP_LOGI(TAG, "JSON-parsing resultat: MAC=%s, sensor_id=%d, break_value=%d, time_sec=%u, time_usec=%u",
                                mac_str, sensor_id, break_value, time_sec, time_usec);
                        
                        // Valider at vi har nødvendige data og at det er et faktisk brudd
                        if (break_value == 1 && strlen(mac_str) > 0 && sensor_id >= 0 && sensor_id < MAX_SENSORS_PER_POLE) {
                            // Konverter MAC-adressen til bytes
                            uint8_t mac_bytes[ESP_NOW_ETH_ALEN];
                            sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                                &mac_bytes[0], &mac_bytes[1], &mac_bytes[2],
                                &mac_bytes[3], &mac_bytes[4], &mac_bytes[5]);
                            
                            ESP_LOGI(TAG, "Sensorbrudd mottatt: MAC=%s, sensor_id=%d, break_value=%d, tid=%u.%06u",
                                    mac_str, sensor_id, break_value, time_sec, time_usec);
                            
                            // Send alltid rådata til WebSocket først
                            send_break_to_websocket(mac_bytes, time_sec, time_usec, sensor_id, false);
                            
                            // Bruk GJELDENDE SERVERTID i stedet for målestolpe-tid for å unngå tidsproblemer
                            struct timeval tv_now;
                            gettimeofday(&tv_now, NULL);
                            uint32_t server_time_sec = tv_now.tv_sec;
                            uint32_t server_time_usec = tv_now.tv_usec;

                            ESP_LOGI(TAG, "Passeringsdeteksjon: målestolpe-tid=%u.%06u, server-tid=%u.%06u", 
                                    time_sec, time_usec, server_time_sec, server_time_usec);

                            // Send til passeringsdeteksjon med BÅDE målestolpe-tid og server-tid
                            ESP_LOGI(TAG, "Sender til passeringsdeteksjon: sensor_id=%d", sensor_id);
                            bool passage_detected = process_break_for_passage_detection(mac_bytes, sensor_id, 
                                                                                    time_sec, time_usec,           // Målestolpe-tid
                                                                                    server_time_sec, server_time_usec); // Server-tid

                            if (passage_detected) {
                                ESP_LOGI(TAG, "🎯 PASSERING DETEKTERT fra sensor %d på MAC %s", sensor_id, mac_str);
                            }

                        } else {
                            // Forbedret logging for debugging
                            if (break_value == 0) {
                                //ESP_LOGI(TAG, "Ignorerer sensor-gjenoppretting: sensor_id=%d, mac=%s", sensor_id, mac_str);
                                log_event_passering(mac_str, sensor_id, "IGNORED", 0.0, 0, 0, NULL, 0);
                            } else if (sensor_id < 0) {
                                ESP_LOGW(TAG, "JSON parsing feilet - sensor_id ikke funnet! JSON: %s", command);
                            } else {
                                ESP_LOGW(TAG, "Ugyldig break-melding: break_value=%d, sensor_id=%d, mac_length=%d", 
                                        break_value, sensor_id, strlen(mac_str));
                            }
                        }
                    }*/




                    /* DEAKTIVERT: Bruk kun ESP-NOW for timing-data (TCP har for høy latens)
                    else if (strncmp(command, "{\"K\":0,", 7) == 0 && strstr(command, "\"B\":[") != NULL) {
                        // Parse JSON-strengen for K=0 med B-array
                        char mac_str[18] = {0};
                        uint32_t time_sec = 0;
                        uint32_t time_usec = 0;
                        
                        // Finn MAC
                        char *mac_start = strstr(command, "\"M\":\"");
                        if (mac_start) {
                            mac_start += 5; // Hopp over "M":"
                            char *mac_end = strchr(mac_start, '\"');
                            if (mac_end && (mac_end - mac_start) < 18) {
                                strncpy(mac_str, mac_start, mac_end - mac_start);
                                mac_str[mac_end - mac_start] = '\0';
                            }
                        }
                        
                        // Vi bruker gjeldende tid siden T og U vanligvis ikke er i K=0 meldinger
                        struct timeval tv;
                        gettimeofday(&tv, NULL);
                        time_sec = tv.tv_sec;
                        time_usec = tv.tv_usec;
                        
                        // Finn B-array
                        char *b_array_start = strstr(command, "\"B\":[");
                        if (b_array_start && strlen(mac_str) > 0) {
                            b_array_start += 5; // Hopp over "B":[
                            
                            // Konverter MAC-adressen til bytes
                            uint8_t mac_bytes[ESP_NOW_ETH_ALEN];
                            sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                                &mac_bytes[0], &mac_bytes[1], &mac_bytes[2],
                                &mac_bytes[3], &mac_bytes[4], &mac_bytes[5]);
                            
                            // Prosessere B-arrayet for å finne utløste sensorer
                            char *ptr = b_array_start;
                            for (int i = 0; i < 7; i++) {
                                while (*ptr && (*ptr < '0' || *ptr > '9')) ptr++; // Hopp til neste tall
                                if (*ptr) {
                                    int value = *ptr - '0'; // Konverter til tall
                                    if (value == 1) {
                                        //ESP_LOGI(TAG, "Fant utløst sensor fra B-array: sensor_id=%d, MAC=%s", i, mac_str);
                                        // Kall passeringsdeteksjon for denne sensoren
                                        //process_break_for_passage_detection(mac_bytes, i, time_sec, time_usec);
                                        // Få server-tid
                                        struct timeval tv_server;
                                        gettimeofday(&tv_server, NULL);

                                        // Kall passeringsdeteksjon for denne sensoren
                                        process_break_for_passage_detection(mac_bytes, i, 
                                                                        time_sec, time_usec,                    // Målestolpe-tid
                                                                        tv_server.tv_sec, tv_server.tv_usec);   // Server-tid
                                    }
                                    ptr++;
                                }
                            }
                        }
                    }*/












                    
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


// Handler for å restarte en målestolpe
esp_err_t pole_restart_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "pole_restart_handler called, content_len: %d", remaining);
    
    // Legg til CORS-headere
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Hvis dette er en OPTIONS forespørsel, bare returner 200 OK med CORS-headere
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
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
    
    // Parse JSON data for MAC-adresse
    char mac[18] = {0};
    char *mac_start = strstr(buf, "\"mac\":\"");
    if (mac_start) {
        mac_start += 7; // Hopp over "mac":"
        char *mac_end = strchr(mac_start, '\"');
        if (mac_end && (mac_end - mac_start) < 18) {
            strncpy(mac, mac_start, mac_end - mac_start);
            mac[mac_end - mac_start] = '\0';
        }
    }
    
    // Parse restart-type (hvis angitt)
    int restart_type = 0;  // Standard restart
    char *type_start = strstr(buf, "\"type\":");
    if (type_start) {
        type_start += 7; // Hopp over "type":
        restart_type = atoi(type_start);
    }
    
    ESP_LOGI(TAG, "Restart forespørsel for MAC: %s, type: %d", mac, restart_type);
    
    if (strlen(mac) > 0) {
        // Bygger kommandoen basert på restart-typen
        char command[64];
        
        switch (restart_type) {
            case 1:
                // Myk restart (standard)
                sprintf(command, "restart: soft\n");
                break;
            case 2:
                // Hard restart
                sprintf(command, "restart: hard\n");
                break;
            case 3:
                // Factory reset (sletter lagrede innstillinger)
                sprintf(command, "restart: factory\n");
                break;
            default:
                // Standard restart
                sprintf(command, "restart: soft\n");
                break;
        }
        
        // Send kommandoen til målestolpen
        if (send_command_to_pole_by_mac(mac, command)) {
            ESP_LOGI(TAG, "Restart-kommando sendt: %s", command);
        } else {
            ESP_LOGW(TAG, "Kunne ikke sende restart-kommando - målestolpe ikke tilkoblet");
        }
    } else {
        ESP_LOGW(TAG, "Ugyldig MAC-adresse mottatt");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Restart command sent\"}");
    
    return ESP_OK;
}



// Handler for å kontrollere strømtilstand til en målestolpe
esp_err_t pole_power_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "pole_power_handler called, content_len: %d", remaining);
    
    // Legg til CORS-headere
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Hvis dette er en OPTIONS forespørsel, bare returner 200 OK med CORS-headere
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
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
    
    // Parse JSON data for MAC-adresse og strømtilstand
    char mac[18] = {0};
    char power_state[4] = {0};  // "on" eller "off"
    
    // Hent MAC-adresse
    char *mac_start = strstr(buf, "\"mac\":\"");
    if (mac_start) {
        mac_start += 7; // Hopp over "mac":"
        char *mac_end = strchr(mac_start, '\"');
        if (mac_end && (mac_end - mac_start) < 18) {
            strncpy(mac, mac_start, mac_end - mac_start);
            mac[mac_end - mac_start] = '\0';
        }
    }
    
    // Hent strømtilstand
    char *power_start = strstr(buf, "\"power\":\"");
    if (power_start) {
        power_start += 9; // Hopp over "power":"
        char *power_end = strchr(power_start, '\"');
        if (power_end && (power_end - power_start) < 4) {
            strncpy(power_state, power_start, power_end - power_start);
            power_state[power_end - power_start] = '\0';
        }
    }
    
    ESP_LOGI(TAG, "Strømkontroll for MAC: %s, tilstand: %s", mac, power_state);
    
    if (strlen(mac) > 0 && strlen(power_state) > 0) {
        // Bygger kommandoen basert på ønsket strømtilstand
        char command[32];
        
        if (strcmp(power_state, "on") == 0) {
            sprintf(command, "power: on\n");
        } else if (strcmp(power_state, "off") == 0) {
            sprintf(command, "power: off\n");
        } else {
            ESP_LOGW(TAG, "Ugyldig strømtilstand: %s", power_state);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Ugyldig strømtilstand\"}");
            return ESP_OK;
        }
        
        // Send kommandoen til målestolpen
        if (send_command_to_pole_by_mac(mac, command)) {
            ESP_LOGI(TAG, "Strømkommando sendt: %s", command);
        } else {
            ESP_LOGW(TAG, "Kunne ikke sende strømkommando - målestolpe ikke tilkoblet");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Målestolpe ikke tilkoblet\"}");
            return ESP_OK;
        }
    } else {
        ESP_LOGW(TAG, "Ugyldig forespørsel - mangler data");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Mangler MAC eller strømtilstand\"}");
        return ESP_OK;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Strømkommando sendt\"}");
    
    return ESP_OK;
}




// Funksjon for å håndtere tid-synkronisering API (oppdatert versjon)
esp_err_t time_sync_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "time_sync_handler called, content_len: %d", remaining);
    
    // Legg til CORS-headere først
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    //httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    //httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Accept");
    
    // Hvis dette er en OPTIONS forespørsel (preflight), bare returner 200 OK med CORS-headere
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
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
    
    
    // Hvis dette er en OPTIONS forespørsel, bare returner 200 OK med CORS-headere
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
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
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"diff\":0}");
    
    return ESP_OK;
}

// Funksjon for å håndtere hsearch API (oppdatert for å sende til TCP-forbindelser)
// Funksjon for å håndtere hsearch API (oppdatert for å sende til spesifikk målestolpe)
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
    char mac[18] = {0};
    
    // Parse channel parameter
    char *channel_start = strstr(buf, "\"channel\":");
    if (channel_start) {
        channel_start += 10; // Hopp over "channel":
        channel = atoi(channel_start);
    }
    
    // Parse MAC address parameter
    char *mac_start = strstr(buf, "\"mac\":\"");
    if (mac_start) {
        mac_start += 7; // Hopp over "mac":":
        char *mac_end = strchr(mac_start, '\"');
        if (mac_end && (mac_end - mac_start) < 18) {
            strncpy(mac, mac_start, mac_end - mac_start);
            mac[mac_end - mac_start] = '\0';
        }
    }
    
    ESP_LOGI(TAG, "Parsed: channel=%d, mac=\"%s\"", channel, mac);
    
    // Bygg kommandoen
    char command[64];
    sprintf(command, "hsearch: %d\n", channel);
    
    // Send kommando til spesifikk målestolpe hvis MAC er angitt
    if (strlen(mac) > 0) {
        if (send_command_to_pole_by_mac(mac, command)) {
            ESP_LOGI(TAG, "Hsearch kommando sendt til målestolpe med MAC %s: %s", mac, command);
        } else {
            ESP_LOGE(TAG, "Kunne ikke sende hsearch kommando til MAC: %s", mac);
        }
    } 
    else {
        ESP_LOGI(TAG, "Ingen MAC angitt, sender til alle stolper");
        // Hvis ingen MAC er oppgitt, send til alle tilkoblede målestolper
        xSemaphoreTake(tcp_poles_mutex, portMAX_DELAY);
        
        for (int i = 0; i < MAX_TCP_POLES; i++) {
            if (tcp_poles[i].connected) {
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
    }
    
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


// Handler for å sette konfigurasjon for passerings-deteksjon
esp_err_t set_passage_config_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "set_passage_config_handler called, content_len: %d", remaining);
    
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
    int debounce_ms = -1;
    int min_sensors = -1;
    int timeout_ms = -1;
    
    // Sjekk debounce_ms
    char *debounce_start = strstr(buf, "\"debounce_ms\":");
    if (debounce_start) {
        debounce_start += 14; // Hopp over "debounce_ms":
        debounce_ms = atoi(debounce_start);
    }
    
    // Sjekk min_sensors
    char *min_sensors_start = strstr(buf, "\"min_sensors\":");
    if (min_sensors_start) {
        min_sensors_start += 14; // Hopp over "min_sensors":
        min_sensors = atoi(min_sensors_start);
    }
    
    // Sjekk timeout_ms
    char *timeout_start = strstr(buf, "\"timeout_ms\":");
    if (timeout_start) {
        timeout_start += 13; // Hopp over "timeout_ms":
        timeout_ms = atoi(timeout_start);
    }
    
    // Oppdater konfigurasjon
    if (debounce_ms >= 0 && debounce_ms <= 30000) { // Maks 30 sekunder
        passage_debounce_ms = debounce_ms;
        ESP_LOGI(TAG, "Passering debounce-tid satt til %d ms", passage_debounce_ms);
    }
    
    if (min_sensors > 0 && min_sensors <= MAX_SENSORS_PER_POLE) {
        min_sensors_for_passage = min_sensors;
        ESP_LOGI(TAG, "Minimum sensorer for passering satt til %d", min_sensors_for_passage);
    }
    
    if (timeout_ms >= 1000 && timeout_ms <= 5000) { // Mellom 1 og 5 sekunder
        sequence_timeout_ms = timeout_ms;
        ESP_LOGI(TAG, "Timeout for sekvens satt til %d ms", sequence_timeout_ms);
    }
    
    // Send respons
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char response[256];
    sprintf(response, "{\"status\":\"success\",\"config\":{\"debounce_ms\":%d,\"min_sensors\":%d,\"timeout_ms\":%d}}",
            passage_debounce_ms, min_sensors_for_passage, sequence_timeout_ms);
    
    httpd_resp_sendstr(req, response);
    
    return ESP_OK;
}

// Handler for å hente nåværende konfigurasjon for passerings-deteksjon
esp_err_t get_passage_config_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "get_passage_config_handler called");
    
    // Send gjeldende konfigurasjon
    char response[256];
    sprintf(response, "{\"status\":\"success\",\"config\":{\"debounce_ms\":%d,\"min_sensors\":%d,\"timeout_ms\":%d}}",
            passage_debounce_ms, min_sensors_for_passage, sequence_timeout_ms);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, response);
    
    return ESP_OK;
}



// Handler for å beregne stoppeklokke-tid mellom to målestolper
esp_err_t calculate_stopwatch_handler(httpd_req_t *req) {
    char buf[200];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "calculate_stopwatch_handler called, content_len: %d", remaining);
    
    // Legg til CORS-headere
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
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
    
    // Parse JSON data for MAC-adressene
    char mac1[18] = {0}, mac2[18] = {0};
    
    // Finn første MAC-adresse
    char *mac1_start = strstr(buf, "\"mac1\":\"");
    if (mac1_start) {
        mac1_start += 8; // Hopp over "mac1":"
        char *mac1_end = strchr(mac1_start, '\"');
        if (mac1_end && (mac1_end - mac1_start) < 18) {
            strncpy(mac1, mac1_start, mac1_end - mac1_start);
            mac1[mac1_end - mac1_start] = '\0';
        }
    }
    
    // Finn andre MAC-adresse
    char *mac2_start = strstr(buf, "\"mac2\":\"");
    if (mac2_start) {
        mac2_start += 8; // Hopp over "mac2":"
        char *mac2_end = strchr(mac2_start, '\"');
        if (mac2_end && (mac2_end - mac2_start) < 18) {
            strncpy(mac2, mac2_start, mac2_end - mac2_start);
            mac2[mac2_end - mac2_start] = '\0';
        }
    }
    
    ESP_LOGI(TAG, "Beregner stoppeklokke-tid mellom MAC1: %s og MAC2: %s", mac1, mac2);
    
    if (strlen(mac1) > 0 && strlen(mac2) > 0) {
        // Konverter MAC-adressene til bytes
        uint8_t mac1_bytes[ESP_NOW_ETH_ALEN], mac2_bytes[ESP_NOW_ETH_ALEN];
        
        sscanf(mac1, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
               &mac1_bytes[0], &mac1_bytes[1], &mac1_bytes[2],
               &mac1_bytes[3], &mac1_bytes[4], &mac1_bytes[5]);
        
        sscanf(mac2, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
               &mac2_bytes[0], &mac2_bytes[1], &mac2_bytes[2],
               &mac2_bytes[3], &mac2_bytes[4], &mac2_bytes[5]);
        
        // Beregn stoppeklokke-tid
        uint64_t elapsed_ms = calculate_stopwatch_time_ms(mac1_bytes, mac2_bytes);
        
        // Send respons
        char response[256];
        sprintf(response, "{\"status\":\"success\",\"elapsed_ms\":%llu,\"mac1\":\"%s\",\"mac2\":\"%s\"}", 
                elapsed_ms, mac1, mac2);
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_sendstr(req, response);
    } else {
        ESP_LOGW(TAG, "Ugyldig forespørsel - mangler MAC-adresser");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Mangler MAC-adresser\"}");
    }
    
    return ESP_OK;
}








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
   
   // Finn enabled - FIKSET VERSJON
   char *enabled_start = strstr(buf, "\"enabled\":");
   if (enabled_start) {
       enabled_start += 10; // Hopp over "enabled":
       if (strncmp(enabled_start, "true", 4) == 0) {
           enabled = 1;
       } else {
           enabled = 0;
       }
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



    // I debug_handler eller en annen diagnose-funksjon:
    ESP_LOGI(TAG, "Tilkoblede målestolper:");
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        if (tcp_poles[i].connected) {
            ESP_LOGI(TAG, "Stolpe %d: Socket=%d, MAC=%s, Nominell MAC=%s", 
                    i, tcp_poles[i].sock, 
                    tcp_poles[i].mac[0] ? tcp_poles[i].mac : "Ukjent", 
                    tcp_poles[i].nominal_mac[0] ? tcp_poles[i].nominal_mac : "Ikke tildelt");
        }
    }


  
   // Avslutt responsen med en tom chunk
   httpd_resp_sendstr_chunk(req, NULL);
   return ESP_OK;
}


esp_err_t passages_monitor_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Passages monitor handler called");
    
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    
    // Auto-refresh header
    httpd_resp_set_hdr(req, "Refresh", "2");
    
    char buffer[512];
    
    // Header
    httpd_resp_sendstr_chunk(req, "Timergate - Passeringsovervåkning\n");
    httpd_resp_sendstr_chunk(req, "Automatisk refresh hver 2 sekund\n\n");
    
    snprintf(buffer, sizeof(buffer), "Siste passeringer (%d nyeste):\n\n", 
             MIN(passage_history_count, MAX_PASSAGE_HISTORY));
    httpd_resp_sendstr_chunk(req, buffer);
    
    xSemaphoreTake(passage_history_mutex, portMAX_DELAY);
    
    if (passage_history_count == 0) {
        httpd_resp_sendstr_chunk(req, "Ingen passeringer registrert ennå.\n");
    } else {
        for (int i = 0; i < passage_history_count; i++) {
            // Tidsstempel
            time_t timestamp = passage_history[i].timestamp_sec;
            struct tm *tm_info = localtime(&timestamp);
            snprintf(buffer, sizeof(buffer), 
                    "[%04d-%02d-%02d %02d:%02d:%02d.%03" PRIu32 "] MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                    tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                    tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                    passage_history[i].timestamp_usec / 1000,
                    passage_history[i].mac[0], passage_history[i].mac[1], 
                    passage_history[i].mac[2], passage_history[i].mac[3],
                    passage_history[i].mac[4], passage_history[i].mac[5]);
            httpd_resp_sendstr_chunk(req, buffer);
            
            // Sensorer
            httpd_resp_sendstr_chunk(req, "  Sensorer utløst: [");
            for (int j = 0; j < passage_history[i].sensors_count; j++) {
                snprintf(buffer, sizeof(buffer), "%d%s", 
                        passage_history[i].sensor_ids[j],
                        (j < passage_history[i].sensors_count - 1) ? ", " : "");
                httpd_resp_sendstr_chunk(req, buffer);
            }
            snprintf(buffer, sizeof(buffer), "] (%d av 7 sensorer)\n",
                    passage_history[i].sensors_count);
            httpd_resp_sendstr_chunk(req, buffer);
            
            // Tid siden forrige
            if (passage_history[i].time_since_previous_ms > 0) {
                snprintf(buffer, sizeof(buffer), "  Tid siden forrige: %.3f sekunder\n\n",
                        passage_history[i].time_since_previous_ms / 1000.0);
                httpd_resp_sendstr_chunk(req, buffer);
            } else {
                httpd_resp_sendstr_chunk(req, "  Tid siden forrige: N/A (første passering)\n\n");
            }
        }
    }
    
    // Statistikk
    snprintf(buffer, sizeof(buffer), 
            "\nStatistikk:\n  Total passeringer: %" PRIu32 "\n", total_passages);
    httpd_resp_sendstr_chunk(req, buffer);
    
    xSemaphoreGive(passage_history_mutex);
    
    httpd_resp_sendstr_chunk(req, NULL); // Avslutt response
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

// Funksjon for å håndtere sensorbrudd og detektere passeringer
bool process_break_for_passage_detection(const uint8_t *mac_addr, int32_t sensor_id,
                                        uint32_t sensor_time_sec, uint32_t sensor_time_micros,
                                        uint32_t server_time_sec, uint32_t server_time_micros) {
    
    // Sjekk at sensor_id er gyldig
    if (sensor_id < 0 || sensor_id >= MAX_SENSORS_PER_POLE) {
        ESP_LOGW(TAG, "Ugyldig sensor_id for passeringsdeteksjon: %d", sensor_id);
        return false;
    }
    
    xSemaphoreTake(passage_mutex, portMAX_DELAY);
    char mac_str[18];
    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);
    
    // Søk etter målestolpe i vår liste
    int detector_idx = -1;
    for (int i = 0; i < passage_detector_count; i++) {
        if (memcmp(passage_detectors[i].mac, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            detector_idx = i;
            break;
        }
    }
    
    // Opprett ny detektor hvis den ikke finnes
    if (detector_idx < 0) {
        if (passage_detector_count >= MAX_PASSAGE_DETECTORS) {
            ESP_LOGW(TAG, "Maks antall passeringsdetektorer nådd (%d)", MAX_PASSAGE_DETECTORS);
            xSemaphoreGive(passage_mutex);
            return false;
        }
        
        detector_idx = passage_detector_count++;
        memcpy(passage_detectors[detector_idx].mac, mac_addr, ESP_NOW_ETH_ALEN);
        for (int i = 0; i < MAX_SENSORS_PER_POLE; i++) {
            passage_detectors[detector_idx].sensor_triggered[i] = false;
        }
        passage_detectors[detector_idx].unique_sensors_count = 0;
        passage_detectors[detector_idx].first_break_time = 0;
        passage_detectors[detector_idx].first_break_micros = 0;
        passage_detectors[detector_idx].sequence_start_time = 0;
        passage_detectors[detector_idx].last_passage_time = 0;
        passage_detectors[detector_idx].last_passage_micros = 0;

        // Initialiser nye felt for robust tidshåndtering - FIKSET: Bruk sensor_time_*
        passage_detectors[detector_idx].last_sensor_time = sensor_time_sec;
        passage_detectors[detector_idx].last_sensor_micros = sensor_time_micros;
    }

    // KRITISK: Bruk målestolpe-tid for debounce-beregninger
    uint64_t current_time_ms = (uint64_t)sensor_time_sec * 1000 + sensor_time_micros / 1000;
    uint64_t last_passage_ms = (uint64_t)passage_detectors[detector_idx].last_passage_time * 1000 + 
                            passage_detectors[detector_idx].last_passage_micros / 1000;

    // Sjekk om vi er i debounce-perioden etter en passering (bruk målestolpe-tid)
    if (passage_detectors[detector_idx].last_passage_time > 0) {
        uint64_t time_since_last_passage_ms = current_time_ms - last_passage_ms;
            
        if (time_since_last_passage_ms < passage_debounce_ms) {
            log_event_passering(mac_str, sensor_id, "DEBOUNCE", 0.0, 0, time_since_last_passage_ms, NULL, 0);
            xSemaphoreGive(passage_mutex);
            return false;
        }
    }
    
    // Sjekk om vi har en ny sekvens (telleren er 0) eller en eksisterende sekvens
    if (passage_detectors[detector_idx].unique_sensors_count == 0) {
        // Dette er første sensor i en ny sekvens - bruk målestolpe-tid
        passage_detectors[detector_idx].first_break_time = sensor_time_sec;
        passage_detectors[detector_idx].first_break_micros = sensor_time_micros;
        passage_detectors[detector_idx].sequence_start_time = sensor_time_sec;
    } else {
        // Vi har en eksisterende sekvens
        // Sjekk om vi kan bruke tid direkte fra sensoren for timeout
        uint32_t sensor_elapsed_time = 0;
        
        // Beregn tid basert på målestolpe-tidsstempler (som er den mest konsistente kilden)
        if (passage_detectors[detector_idx].last_sensor_time > 0) {
            if (sensor_time_sec >= passage_detectors[detector_idx].last_sensor_time) {
                // Vanlig tilfelle - sensortid går fremover
                sensor_elapsed_time = (sensor_time_sec - passage_detectors[detector_idx].last_sensor_time) * 1000;
                
                // Legg til mikrosekunder-delen
                if (sensor_time_micros >= passage_detectors[detector_idx].last_sensor_micros) {
                    sensor_elapsed_time += (sensor_time_micros - passage_detectors[detector_idx].last_sensor_micros) / 1000;
                } else {
                    // Håndter mikrosekund-overgang
                    sensor_elapsed_time += (1000000 + sensor_time_micros - passage_detectors[detector_idx].last_sensor_micros) / 1000 - 1000;
                }
            } 
            else {
                // Sensortid hopper bakover
                ESP_LOGW(TAG, "Sensor-tid hopper bakover: %u -> %u, ignorerer denne hendelsen", 
                        passage_detectors[detector_idx].last_sensor_time, sensor_time_sec);
                
                // Ikke oppdater last_sensor_time for å unngå gjentatte advarsler om samme hopp
                xSemaphoreGive(passage_mutex);
                return false; // Ignorer denne sensoren for passeringsdeteksjon
            }
        }
        
        // FIKSET: Oppdater siste sensortid for neste beregning - bruk målestolpe-tid
        passage_detectors[detector_idx].last_sensor_time = sensor_time_sec;
        passage_detectors[detector_idx].last_sensor_micros = sensor_time_micros;
                
        // Sjekk om sekvensen har timed ut basert på sensortid
        if (sensor_elapsed_time > sequence_timeout_ms) {
            log_event_passering(mac_str, sensor_id, "TIMEOUT",
                passage_detectors[detector_idx].first_break_time +
                (passage_detectors[detector_idx].first_break_micros / 1000000.0),
                passage_detectors[detector_idx].unique_sensors_count, 0, NULL, 0);
                        
            // Tilbakestill tellere
            for (int i = 0; i < MAX_SENSORS_PER_POLE; i++) {
                passage_detectors[detector_idx].sensor_triggered[i] = false;
            }
            passage_detectors[detector_idx].unique_sensors_count = 0;
            
            // Sett nye verdier for første brudd i ny sekvens - bruk målestolpe-tid
            passage_detectors[detector_idx].first_break_time = sensor_time_sec;
            passage_detectors[detector_idx].first_break_micros = sensor_time_micros;
            passage_detectors[detector_idx].sequence_start_time = sensor_time_sec;
        }
    }
    
    // Registrer denne sensoren hvis den ikke allerede er registrert i gjeldende sekvens
    if (!passage_detectors[detector_idx].sensor_triggered[sensor_id]) {
        passage_detectors[detector_idx].sensor_triggered[sensor_id] = true;
        passage_detectors[detector_idx].unique_sensors_count++;

        // FIKSET: Oppdater siste sensor-tid - bruk målestolpe-tid
        passage_detectors[detector_idx].last_sensor_time = sensor_time_sec;
        passage_detectors[detector_idx].last_sensor_micros = sensor_time_micros;

        ESP_LOGI(TAG, "Sensor %d registrert, sensorteller nå: %d, min for passering: %d", 
            sensor_id, passage_detectors[detector_idx].unique_sensors_count, min_sensors_for_passage);
        
        // Sjekk om vi har nådd terskelen for en passering
        if (passage_detectors[detector_idx].unique_sensors_count >= min_sensors_for_passage) {
            // Sjekk om denne passeringen allerede er sendt
            if (is_passage_already_sent(mac_addr, 
                                    passage_detectors[detector_idx].first_break_time, 
                                    passage_detectors[detector_idx].first_break_micros, 
                                    passage_detectors[detector_idx].unique_sensors_count)) {
                
                ESP_LOGI(TAG, "DUPLIKAT: Ignorerer duplisert passering: MAC: %s, tidspunkt: %u.%06u", 
                        mac_str, passage_detectors[detector_idx].first_break_time, 
                        passage_detectors[detector_idx].first_break_micros);
                
                // Vi tilbakestiller fortsatt telleren for å være klar for neste sekvens
                for (int i = 0; i < MAX_SENSORS_PER_POLE; i++) {
                    passage_detectors[detector_idx].sensor_triggered[i] = false;
                }
                passage_detectors[detector_idx].unique_sensors_count = 0;
                
                xSemaphoreGive(passage_mutex);
                return false; // Ingen ny passering å rapportere
            }
            
            // Bygg liste over utløste sensorer
            int sensors_used[MAX_SENSORS_PER_POLE];
            int used_count = 0;
            for (int i = 0; i < MAX_SENSORS_PER_POLE; i++) {
                if (passage_detectors[detector_idx].sensor_triggered[i]) {
                    sensors_used[used_count++] = i;
                }
            }

            // log_event_passering(mac_str, sensor_id, "PASSERING",
            //     passage_detectors[detector_idx].first_break_time +
            //     (passage_detectors[detector_idx].first_break_micros / 1000000.0),
            //     used_count, 0, sensors_used, used_count);
                        
            // // Registrer denne passeringen som sendt
            // register_sent_passage(mac_addr, 
            //                 passage_detectors[detector_idx].first_break_time, 
            //                 passage_detectors[detector_idx].first_break_micros, 
            //                 passage_detectors[detector_idx].unique_sensors_count);



            log_event_passering(mac_str, sensor_id, "PASSERING",
                passage_detectors[detector_idx].first_break_time +
                (passage_detectors[detector_idx].first_break_micros / 1000000.0),
                used_count, 0, sensors_used, used_count);
            
            // NYTT: Log med dual timing for debugging
            log_passage_with_dual_timing(mac_str, sensor_id,
                                       passage_detectors[detector_idx].first_break_time,
                                       passage_detectors[detector_idx].first_break_micros,
                                       server_time_sec, server_time_micros,
                                       sensors_used, used_count);
            
            // NYTT: Registrer for stoppeklokke-beregninger med dual timing
            register_passage_for_stopwatch(mac_addr, 
                                         server_time_sec, server_time_micros,
                                         passage_detectors[detector_idx].first_break_time,
                                         passage_detectors[detector_idx].first_break_micros,
                                         used_count);
                        
            // Registrer denne passeringen som sendt (eksisterende funksjonalitet)
            register_sent_passage(mac_addr, 
                            passage_detectors[detector_idx].first_break_time, 
                            passage_detectors[detector_idx].first_break_micros, 
                            passage_detectors[detector_idx].unique_sensors_count);








            
            // Lagre i passage history
            add_to_passage_history(mac_addr,
                      passage_detectors[detector_idx].first_break_time,
                      passage_detectors[detector_idx].first_break_micros,
                      sensors_used, used_count);

            // Send passering til WebSocket-klienter
            char passage_msg[256];
            sprintf(passage_msg, 
                "{\"M\":\"%s\",\"K\":4,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"S\":%d}",
                mac_str, passage_detectors[detector_idx].first_break_time, 
                passage_detectors[detector_idx].first_break_micros, 
                passage_detectors[detector_idx].unique_sensors_count);
            
            // Lagre denne passeringens tidspunkt for debouncing - bruk målestolpe-tid
            passage_detectors[detector_idx].last_passage_time = passage_detectors[detector_idx].first_break_time;
            passage_detectors[detector_idx].last_passage_micros = passage_detectors[detector_idx].first_break_micros;
            
            // Tilbakestill tellere for neste sekvens
            for (int i = 0; i < MAX_SENSORS_PER_POLE; i++) {
                passage_detectors[detector_idx].sensor_triggered[i] = false;
            }
            passage_detectors[detector_idx].unique_sensors_count = 0;

            // Send til alle WebSocket-klienter
            httpd_ws_frame_t ws_pkt = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t *)passage_msg,
                .len = strlen(passage_msg)
            };
            
            xSemaphoreTake(ws_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
                if (ws_clients[i] != 0) {
                    httpd_ws_send_frame_async(server, ws_clients[i], &ws_pkt);
                }
            }
            xSemaphoreGive(ws_mutex);
            
            xSemaphoreGive(passage_mutex);
            return true;
        }
    }
    
    xSemaphoreGive(passage_mutex);
    return false; // Ingen passering detektert ennå
}


// Funksjon for å sjekke om et brudd skal ignoreres pga. debouncing
bool should_ignore_break(const uint8_t *mac, int32_t sensor_id, uint32_t time_sec, uint32_t time_micros) {
    // Konverter til total millisekunder for enkel sammenligning
    uint64_t current_time_ms = (uint64_t)time_sec * 1000 + time_micros / 1000;
    
    // Sjekk om vi har et tidligere brudd for denne sensoren
    for (int i = 0; i < last_break_count; i++) {
        if (memcmp(last_breaks[i].mac, mac, ESP_NOW_ETH_ALEN) == 0 && 
            last_breaks[i].sensor_id == sensor_id) {
            
            // Beregn tid siden siste brudd
            uint64_t last_time_ms = (uint64_t)last_breaks[i].last_break_time * 1000 + 
                                 last_breaks[i].last_break_micros / 1000;
            uint64_t time_diff_ms = current_time_ms - last_time_ms;
            
            // Hvis tiden er innenfor debounce-vinduet, ignorer
            if (time_diff_ms < break_debounce_ms) {
                ESP_LOGI(TAG, "Ignorerer brudd innenfor debounce-tid (%llu ms)", time_diff_ms);
                return true; // Ignorer dette bruddet
            }
            
            // Oppdater siste brudd-tid
            last_breaks[i].last_break_time = time_sec;
            last_breaks[i].last_break_micros = time_micros;
            return false; // Ikke ignorer - første brudd etter debounce-perioden
        }
    }
    
    // Første brudd for denne sensoren, legg til i oversikten
    if (last_break_count < MAX_POLES * MAX_SENSORS_PER_POLE) {
        memcpy(last_breaks[last_break_count].mac, mac, ESP_NOW_ETH_ALEN);
        last_breaks[last_break_count].sensor_id = sensor_id;
        last_breaks[last_break_count].last_break_time = time_sec;
        last_breaks[last_break_count].last_break_micros = time_micros;
        last_break_count++;
    }
    
    return false; // Ikke ignorer - første kjente brudd på denne sensoren
}



// Funksjon for å sende brudd til WebSocket-klienter
void send_break_to_websocket(const uint8_t *mac_addr, uint32_t time_sec, uint32_t time_micros, 
                             int32_t sensor_id, bool filtered) {
    char mac_str[18];
    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
    
    char ws_msg[256];
    if (filtered) {
        sprintf(ws_msg, 
            "{\"M\":\"%s\",\"K\":1,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"B\":%" PRId32 ",\"F\":true}",
            mac_str, time_sec, time_micros, sensor_id);
    } else {
        sprintf(ws_msg, 
            "{\"M\":\"%s\",\"K\":1,\"T\":%" PRIu32 ",\"U\":%" PRIu32 ",\"B\":%" PRId32 "}",
            mac_str, time_sec, time_micros, sensor_id);
    }



    ESP_LOGI(TAG, "SENSORBRUDD: MAC=%s, Tid=%u.%06u, Sensor=%d, Filtrert=%s",
        mac_str, time_sec, time_micros, sensor_id, filtered ? "Ja" : "Nei");

    
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
}




// Sjekker om en passering allerede er sendt
bool is_passage_already_sent(const uint8_t *mac, uint32_t time_sec, uint32_t time_micros, int sensors_count) {
    xSemaphoreTake(sent_passages_mutex, portMAX_DELAY);
    
    // Sjekk om vi har en identisk passering i nylig sendte passeringer
    for (int i = 0; i < sent_passage_count; i++) {
        // Sjekk om MAC-adresse og tidspunkt (innenfor en liten margin) er like
        if (memcmp(sent_passages[i].mac, mac, ESP_NOW_ETH_ALEN) == 0 &&
            sent_passages[i].sensors_count == sensors_count) {
            
            // Beregn tidsdifferanse i millisekunder
            uint64_t t1 = (uint64_t)sent_passages[i].timestamp_sec * 1000 + sent_passages[i].timestamp_micros / 1000;
            uint64_t t2 = (uint64_t)time_sec * 1000 + time_micros / 1000;
            uint64_t diff_ms = (t1 > t2) ? (t1 - t2) : (t2 - t1);
            
            // Hvis tidspunktene er innenfor 50ms anser vi det som duplisert
            if (diff_ms < 50) {
                xSemaphoreGive(sent_passages_mutex);
                return true;
            }
        }
    }
    
    xSemaphoreGive(sent_passages_mutex);
    return false;
}

// Registrerer en ny sendt passering
void register_sent_passage(const uint8_t *mac, uint32_t time_sec, uint32_t time_micros, int sensors_count) {
    xSemaphoreTake(sent_passages_mutex, portMAX_DELAY);
    
    int idx = sent_passage_count;
    if (sent_passage_count >= MAX_SENT_PASSAGES) {
        // Hvis array er fullt, lag plass ved å fjerne eldste innslag
        idx = 0;
        for (int i = 1; i < MAX_SENT_PASSAGES; i++) {
            sent_passages[i-1] = sent_passages[i];
        }
        sent_passage_count = MAX_SENT_PASSAGES - 1;
    }
    
    // Legg til ny passering
    memcpy(sent_passages[idx].mac, mac, ESP_NOW_ETH_ALEN);
    sent_passages[idx].timestamp_sec = time_sec;
    sent_passages[idx].timestamp_micros = time_micros;
    sent_passages[idx].sensors_count = sensors_count;
    sent_passages[idx].send_time = time(NULL);
    
    sent_passage_count++;
    
    xSemaphoreGive(sent_passages_mutex);
}



// Registrerer en passering for stoppeklokke-beregninger med dual timing
void register_passage_for_stopwatch(const uint8_t *mac_addr, uint32_t server_sec, uint32_t server_usec,
                                   uint32_t pole_sec, uint32_t pole_usec, int sensor_count) {
    xSemaphoreTake(stopwatch_mutex, portMAX_DELAY);
    
    int idx = stopwatch_passage_count;
    if (stopwatch_passage_count >= MAX_STOPWATCH_PASSAGES) {
        // Hvis array er fullt, lag plass ved å fjerne eldste innslag
        for (int i = 1; i < MAX_STOPWATCH_PASSAGES; i++) {
            stopwatch_passages[i-1] = stopwatch_passages[i];
        }
        idx = MAX_STOPWATCH_PASSAGES - 1;
    } else {
        stopwatch_passage_count++;
    }
    
    // Legg til ny stoppeklokke-passering
    memcpy(stopwatch_passages[idx].mac, mac_addr, ESP_NOW_ETH_ALEN);
    stopwatch_passages[idx].server_time_sec = server_sec;
    stopwatch_passages[idx].server_time_usec = server_usec;
    stopwatch_passages[idx].pole_time_sec = pole_sec;
    stopwatch_passages[idx].pole_time_usec = pole_usec;
    stopwatch_passages[idx].sensor_count = sensor_count;
    stopwatch_passages[idx].passage_id = next_passage_id++;
    
    char mac_str[18];
    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    ESP_LOGI(TAG, "🕒 STOPPEKLOKKE: Registrert passering ID=%u, MAC=%s, server-tid=%u.%06u, stolpe-tid=%u.%06u", 
             stopwatch_passages[idx].passage_id, mac_str, server_sec, server_usec, pole_sec, pole_usec);
    
    xSemaphoreGive(stopwatch_mutex);
}



// Logger passering med dual timing for debugging og analyse
void log_passage_with_dual_timing(const char *mac_str, int sensor_id,
                                 uint32_t pole_time_sec, uint32_t pole_time_usec,
                                 uint32_t server_time_sec, uint32_t server_time_usec,
                                 int *sensors_used, int sensor_count) {
    
    // Beregn tidsdifferanse mellom målestolpe og server
    uint64_t pole_time_ms = (uint64_t)pole_time_sec * 1000 + pole_time_usec / 1000;
    uint64_t server_time_ms = (uint64_t)server_time_sec * 1000 + server_time_usec / 1000;
    int64_t time_diff_ms = (int64_t)server_time_ms - (int64_t)pole_time_ms;
    
    // Bygg sensor-liste som streng
    char sensor_list[32] = {0};
    int pos = 0;
    for (int i = 0; i < sensor_count && pos < 30; i++) {
        if (i > 0) sensor_list[pos++] = ',';
        pos += snprintf(sensor_list + pos, sizeof(sensor_list) - pos, "%d", sensors_used[i]);
    }
    
    ESP_LOGI(TAG, "🎯 DUAL-TIMING PASSERING: MAC=%s, trigger_sensor=%d, sensorer=[%s]", 
             mac_str, sensor_id, sensor_list);
    ESP_LOGI(TAG, "   📍 Målestolpe-tid: %u.%06u", pole_time_sec, pole_time_usec);
    ESP_LOGI(TAG, "   🖥️  Server-tid: %u.%06u", server_time_sec, server_time_usec);
    ESP_LOGI(TAG, "   ⏱️  Tidsdiff: %+lld ms (server minus målestolpe)", time_diff_ms);
}


// Beregner stoppeklokke-tid mellom to målestolper basert på server-tid
uint64_t calculate_stopwatch_time_ms(const uint8_t *mac1, const uint8_t *mac2) {
    xSemaphoreTake(stopwatch_mutex, portMAX_DELAY);
    
    uint64_t time1_ms = 0, time2_ms = 0;
    bool found1 = false, found2 = false;
    uint32_t passage_id1 = 0, passage_id2 = 0;
    
    // Finn siste passering for hver målestolpe (nyeste først)
    for (int i = stopwatch_passage_count - 1; i >= 0; i--) {
        if (!found1 && memcmp(stopwatch_passages[i].mac, mac1, ESP_NOW_ETH_ALEN) == 0) {
            time1_ms = (uint64_t)stopwatch_passages[i].server_time_sec * 1000 + 
                      stopwatch_passages[i].server_time_usec / 1000;
            passage_id1 = stopwatch_passages[i].passage_id;
            found1 = true;
        }
        if (!found2 && memcmp(stopwatch_passages[i].mac, mac2, ESP_NOW_ETH_ALEN) == 0) {
            time2_ms = (uint64_t)stopwatch_passages[i].server_time_sec * 1000 + 
                      stopwatch_passages[i].server_time_usec / 1000;
            passage_id2 = stopwatch_passages[i].passage_id;
            found2 = true;
        }
        
        // Stopp når vi har funnet begge
        if (found1 && found2) break;
    }
    
    xSemaphoreGive(stopwatch_mutex);
    
    if (found1 && found2) {
        uint64_t elapsed_ms = (time2_ms > time1_ms) ? (time2_ms - time1_ms) : (time1_ms - time2_ms);
        
        char mac1_str[18], mac2_str[18];
        sprintf(mac1_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
               mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
        sprintf(mac2_str, "%02x:%02x:%02x:%02x:%02x:%02x", 
               mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]);
        
        ESP_LOGI(TAG, "⏱️ STOPPEKLOKKE-BEREGNING: %s (ID=%u) til %s (ID=%u) = %llu ms", 
                 mac1_str, passage_id1, mac2_str, passage_id2, elapsed_ms);
        
        return elapsed_ms;
    } else {
        ESP_LOGW(TAG, "⚠️ STOPPEKLOKKE: Kunne ikke finne passeringer for begge målestolper (funnet1=%d, funnet2=%d)", 
                 found1, found2);
        return 0;
    }
}



// Task som fjerner gamle passeringer
void clean_old_passages(void *pvParameters) {
    const uint32_t MAX_AGE_SEC = 300; // 5 minutter
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Kjør hvert minutt
        
        uint32_t now = time(NULL);
        xSemaphoreTake(sent_passages_mutex, portMAX_DELAY);
        
        int valid_count = 0;
        for (int i = 0; i < sent_passage_count; i++) {
            if (now - sent_passages[i].send_time < MAX_AGE_SEC) {
                // Behold denne passeringen (flytt framover hvis nødvendig)
                if (valid_count != i) {
                    sent_passages[valid_count] = sent_passages[i];
                }
                valid_count++;
            }
        }
        
        // Oppdater antall gyldige passeringer
        if (valid_count != sent_passage_count) {
            ESP_LOGI(TAG, "Fjernet %d gamle passeringer, %d igjen", 
                     sent_passage_count - valid_count, valid_count);
            sent_passage_count = valid_count;
        }
        
        xSemaphoreGive(sent_passages_mutex);
    }
}


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
       int32_t sensor_id = break_value; // Eller hvordan sensor_id faktisk bestemmes
       
       ESP_LOGI(TAG, "Break event mottatt: verdi=%" PRId32, break_value);

       // Send alltid rådata (ufiltrert)
       send_break_to_websocket(mac_addr, pole_data[pole_idx].t, pole_data[pole_idx].u, break_value, false);
       

       // Behandle brudd for passeringsdeteksjon
       //process_break_for_passage_detection(mac_addr, sensor_id, pole_data[pole_idx].t, pole_data[pole_idx].u);
       // Få server-tid ved mottak
        struct timeval server_time;
        gettimeofday(&server_time, NULL);

        // Behandle brudd for passeringsdeteksjon med både målestolpe-tid og server-tid
        process_break_for_passage_detection(mac_addr, sensor_id, 
                                        pole_data[pole_idx].t, pole_data[pole_idx].u,     // Målestolpe-tid
                                        server_time.tv_sec, server_time.tv_usec);         // Server-tid


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



// Handler for å sette debounce-tid
esp_err_t set_debounce_time_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;
    
    ESP_LOGI(TAG, "set_debounce_time_handler called, content_len: %d", remaining);
    
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
    int debounce_ms = 0;
    char *debounce_start = strstr(buf, "\"debounce_ms\":");
    if (debounce_start) {
        debounce_start += 14; // Hopp over "debounce_ms":
        debounce_ms = atoi(debounce_start);
    }
    
    // Valider og sett ny debounce-tid
    if (debounce_ms >= 0 && debounce_ms <= 5000) { // Maks 5 sekunder
        break_debounce_ms = debounce_ms;
        ESP_LOGI(TAG, "Debounce-tid satt til %d ms", break_debounce_ms);
    } else {
        ESP_LOGW(TAG, "Ugyldig debounce-tid: %d ms", debounce_ms);
    }
    
    // Send respons
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Debounce time updated\"}");
    
    return ESP_OK;
}




httpd_handle_t start_webserver(void) {
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   config.uri_match_fn = httpd_uri_match_wildcard; //Important when when gui-files is hashed
   config.max_open_sockets = 7;

   // Øk maks URI-handlere
   config.max_uri_handlers = 30; // Økt fra 6 til 12 for å ha plass til flere handlere
   // Øk recv/send-bufferstørrelse og timeouts
   config.recv_wait_timeout = 10;
   config.send_wait_timeout = 10;
   
   httpd_handle_t server_handle = NULL;

   ESP_LOGI(TAG, "Starting HTTP server");
   
   if (httpd_start(&server_handle, &config) == ESP_OK) {
        global_server = server_handle;


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


        // Passages monitor handler
        httpd_uri_t passages = {
            .uri = "/passages",
            .method = HTTP_GET,
            .handler = passages_monitor_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &passages);

       
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
       

        // Debounce-innstillinger API
        httpd_uri_t debounce_time = {
            .uri = "/api/v1/config/debounce",
            .method = HTTP_POST,
            .handler = set_debounce_time_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &debounce_time);



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
       
  
        // Passerings-konfigurasjon API
        httpd_uri_t set_passage_config = {
            .uri = "/api/v1/config/passage",
            .method = HTTP_POST,
            .handler = set_passage_config_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &set_passage_config);

        httpd_uri_t get_passage_config = {
            .uri = "/api/v1/config/passage",
            .method = HTTP_GET,
            .handler = get_passage_config_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &get_passage_config);


        // Stoppeklokke-beregning API
        httpd_uri_t calculate_stopwatch = {
            .uri = "/api/v1/stopwatch/calculate",
            .method = HTTP_POST,
            .handler = calculate_stopwatch_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &calculate_stopwatch);




        // Pole restart API
        httpd_uri_t pole_restart = {
            .uri = "/api/v1/pole/restart",
            .method = HTTP_POST,
            .handler = pole_restart_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &pole_restart);


        // Pole power API (av/på-funksjonalitet)
        httpd_uri_t pole_power = {
            .uri = "/api/v1/pole/power",
            .method = HTTP_POST,
            .handler = pole_power_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &pole_power);







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
        .channel = 6,
        .password = "12345678",  // Tomt passord
        .max_connection = 8,
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
         (char *)wifi_config.ap.ssid, (char *)wifi_config.ap.password, wifi_config.ap.channel);

    // Initialiser mutexer
    ws_mutex = xSemaphoreCreateMutex();
    pole_data_mutex = xSemaphoreCreateMutex();
    tcp_poles_mutex = xSemaphoreCreateMutex();
    passage_mutex = xSemaphoreCreateMutex();
    sent_passages_mutex = xSemaphoreCreateMutex();
    passage_history_mutex = xSemaphoreCreateMutex();
    stopwatch_mutex = xSemaphoreCreateMutex();




    
    // Initialiser arrays
    memset(ws_clients, 0, sizeof(ws_clients));
    memset(pole_data, 0, sizeof(pole_data));
    memset(tcp_poles, 0, sizeof(tcp_poles));
    memset(sent_passages, 0, sizeof(sent_passages));
    sent_passage_count = 0;

    memset(stopwatch_passages, 0, sizeof(stopwatch_passages));
    stopwatch_passage_count = 0;
    next_passage_id = 1; 


    // Sikre at alle nominal_mac-felt er initialisert
    for (int i = 0; i < MAX_TCP_POLES; i++) {
        tcp_poles[i].nominal_mac[0] = '\0';
    }

    // Initialiser filsystemet
    ESP_ERROR_CHECK(init_fs());
    
    // Start TCP-serveroppgaven
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    // Start oppgave for å rydde i gamle passeringer
    xTaskCreate(clean_old_passages, "clean_passages", 4096, NULL, 1, NULL);
    
    // Initialiser ESP-NOW
    init_esp_now();


    


    // Start webserveren
    server = start_webserver();
}