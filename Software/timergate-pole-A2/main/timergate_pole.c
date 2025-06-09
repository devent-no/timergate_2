/* Timergate Pole for hardware revision A2 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "led_strip.h"
#include "esp_now.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "esp_timer.h"  // For esp_timer_get_time()
#include "esp_sleep.h"




// ESP-NOW datastruktur for sensorbrudd
typedef struct {
    uint8_t k;           // Meldingstype (1 for sensorbrudd)
    uint32_t t;          // Timestamp sekunder  
    uint32_t u;          // Timestamp mikrosekunder
    int32_t sensor_id;   // Sensor som utløste (0-6)
    int32_t break_state; // 1=brudd, 0=gjenopprettet
} __attribute__((packed)) esp_now_sensor_break_t;

// ESP-NOW peer informasjon
static uint8_t ap_mac_addr[ESP_NOW_ETH_ALEN] = {0};
static bool esp_now_peer_added = false;


/* Constants that aren't configurable in menuconfig */
#define HOST_IP_ADDR "192.168.4.1"
#define PORT 3333
#define NUM_SENSORS 7
#define LED_GPIO 39
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define PWM_SEND_0 9
#define PWM_RCV_0 18
#define PWM_RCV_1 21
#define PWM_RCV_2 38
#define PWM_RCV_3 45
#define PWM_RCV_4 46
#define PWM_RCV_5 47
#define PWM_RCV_6 48
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define PWM_0_FREQUENCY (9000)          // Frequency in Hertz.

static const char *TAG = "timergate-pole";
static int sock;
char host_ip[] = HOST_IP_ADDR;

QueueHandle_t xQueue;
QueueHandle_t xQueueBreak;
char mac_addr[18];

struct sockaddr_in dest_addr;

static int adc_raw[NUM_SENSORS][10];
adc_oneshot_unit_handle_t adc1_handle;

static led_strip_handle_t led_strip;

int32_t restart_counter;
uint8_t led_val[NUM_SENSORS] = {0};
uint8_t adc_channel[NUM_SENSORS] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7};
uint16_t break_limit[NUM_SENSORS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
uint8_t rcv_gpios[NUM_SENSORS] = {PWM_RCV_0, PWM_RCV_1, PWM_RCV_2, PWM_RCV_3, PWM_RCV_4, PWM_RCV_5, PWM_RCV_6};
ledc_channel_t rcv_channels[NUM_SENSORS] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7};
uint16_t offsets[NUM_SENSORS];
bool enabled[NUM_SENSORS] = {true, true, true, false, true, true, true};
bool sensor_break[NUM_SENSORS] = {false};

uint16_t break_time;
bool curr_broken = false;
bool prev_broken = false;

bool connected = false;

char *cmd;
int64_t timestamp;

bool highpoint_search = false;
int highpoint_offset = 0;
int highpoint_max = 0;
int highpoint_offset_max = 0;
int highpoint_channel = 0;

// Nye variabler for blinkelogikk
bool blink_mode = false;
uint32_t blink_start_time = 0;
uint32_t all_sensors_broken_start_time = 0;
uint32_t min_time_for_alert = 5000;  // 5 sekunder i millisekunder
uint32_t blink_interval = 500;       // Blinkehastighet i millisekunder
bool blink_state = false;            // Av/på tilstand for blinking
static int min_broken_sensors_for_alert = 2;  // Antall sensorer som må være brutt for å utløse varsel


bool calibration_completed = false;  // Flagg for å holde styr på om kalibrering er fullført


// Nye variabler for discovery og system assignment
static uint32_t my_assigned_system_id = 0x00000000;  // 0 = ikke tilknyttet
static char assigned_system_name[32] = {0};
static bool announcement_active = true;              // Send announce-meldinger
static uint32_t last_announce_time = 0;
static const uint32_t ANNOUNCE_INTERVAL_MS = 5000;  // Send announce hvert 5. sekund


// Variabler for identify-funksjonalitet
static bool identification_active = false;
static uint64_t identification_end_time = 0;
static const uint8_t IDENTIFY_BLINK_INTERVAL_MS = 200; // Rask blinking hver 200ms




// Discovery protokoll strukturer (samme som i AP)
typedef struct {
    uint32_t system_id;      // 0x00000000 = søker system
    uint8_t msg_type;        // MSG_POLE_ANNOUNCE
    uint8_t mac[6];          // Målestolpens MAC
    char device_name[16];    // "TimerGate Pole"
    uint8_t firmware_ver[3]; // [1, 2, 3]
    int8_t rssi_estimate;    // For avstandsestimering
} __attribute__((packed)) pole_announce_t;

typedef struct {
    uint32_t system_id;      // AP's System ID
    uint8_t msg_type;        // MSG_SYSTEM_ASSIGN
    uint8_t target_mac[6];   // Målestolpens MAC
    char system_name[32];    // "Timergate Nord"
} __attribute__((packed)) system_assign_t;


typedef struct {
    uint32_t system_id;      // 0x00000000 for broadcast eller current_system_id
    uint8_t msg_type;        // MSG_IDENTIFY_REQUEST
    uint8_t target_mac[6];   // Spesifikk målestolpe
    uint8_t duration_sec;    // Hvor lenge den skal blinke (5-30 sekunder)
} __attribute__((packed)) identify_request_t;


// Kommando-struktur for ESP-NOW
typedef struct {
    uint32_t system_id;
    uint8_t msg_type;        // MSG_COMMAND
    uint8_t target_mac[6];   // Spesifikk målestolpe eller broadcast
    uint8_t command_type;    // TIME_SYNC, RESTART, HSEARCH, etc.
    uint8_t data[];          // Kommando-spesifikk data
} __attribute__((packed)) command_msg_t;

// Kommandotyper (samme som i AP)
#define CMD_TIME_SYNC      0x01
#define CMD_RESTART        0x02
#define CMD_HSEARCH        0x03
#define CMD_SET_BREAK      0x04
#define CMD_SET_ENABLED    0x05
#define CMD_POWER_OFF      0x06




// Meldingstyper (samme som i AP)
#define MSG_SENSOR_DATA        0x01
#define MSG_PASSAGE_DETECTED   0x02
#define MSG_POLE_ANNOUNCE      0x10
#define MSG_SYSTEM_ASSIGN      0x11
#define MSG_IDENTIFY_REQUEST   0x20
#define MSG_COMMAND            0x30






// Nye globale enum for systemstatus
typedef enum {
    STATUS_INITIALIZING,
    STATUS_WIFI_CONNECTING,
    STATUS_SERVER_CONNECTING,
    STATUS_READY,
    STATUS_ERROR_WIFI,
    STATUS_ERROR_SERVER,
    STATUS_ERROR_SENSORS_BLOCKED,
    STATUS_CALIBRATING // Ny status for kalibrering
} system_status_t;

void set_system_status(system_status_t status);

// Globale variabler for statushåndtering
system_status_t current_status = STATUS_INITIALIZING;
uint32_t status_animation_start_time = 0;
uint32_t status_animation_step = 0;
bool normal_led_control = false;  // Om vanlig sensorvisning skal overstyre animasjoner

// Variabler for tidsstyrt automatisk kalibrering
uint32_t ready_start_time = 0;
bool auto_calibration_started = false;


// For å holde styr på forrige tilstand per sensor
bool prev_sensor_break[NUM_SENSORS] = {false}; 


// Nye variabler for WiFi-gjenoppkobling
static bool wifi_connected = false;
static int wifi_reconnect_attempts = 0;
static const int MAX_WIFI_RECONNECT_ATTEMPTS = 10; // Maksimalt antall forsøk før timeout
//static TaskHandle_t wifi_reconnect_task_handle = NULL;

// Utvidede WiFi-overvåkingsvariabler
static bool wifi_connection_stable = false;
static uint32_t wifi_disconnect_time = 0;
static const uint32_t WIFI_RECONNECT_DELAY_MS = 5000;  // 5 sekunder mellom forsøk
static TaskHandle_t wifi_monitor_task_handle = NULL;





// LED-farger for de ulike tilstandene (R, G, B)
const uint8_t STATUS_COLORS[8][3] = {
    {64, 64, 64},  // Initialisering - Hvit
    {0, 0, 255},   // WiFi-tilkobling - Blå
    {255, 255, 0}, // Servertilkobling - Gul
    {0, 255, 0},   // Klar - Grønn
    {255, 0, 0},   // WiFi-feil - Rød
    {255, 0, 0},   // Serverfeil - Rød
    {255, 0, 0},   // Sensorer blokkert - Rød
    {128, 0, 128}  // Kalibrering - Lilla
};

led_strip_config_t strip_config = {
    .max_leds = 7, // at least one LED on board
    .strip_gpio_num = LED_GPIO,
};

led_strip_rmt_config_t rmt_config = {
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
};


// Nye funksjonsdeklarasjoner for discovery og assignment
static void send_pole_announce(void);
static void handle_system_assignment(const system_assign_t *assign);
static void load_system_assignment_from_nvs(void);
static void save_system_assignment_to_nvs(void);
static bool is_assigned_to_system(void);


static void handle_identify_request(const identify_request_t *msg);
static void start_identification_blink(uint8_t duration);
static void handle_identification_animation(void);


// Funksjonsdeklarasjoner for ESP-NOW kommando-håndtering
//static void handle_esp_now_command(const command_msg_t *cmd);
static void handle_esp_now_command(const command_msg_t *cmd, int len);
static void handle_time_sync_command(const uint8_t *data, size_t data_len);
static void handle_restart_command(const uint8_t *data, size_t data_len);
static void handle_hsearch_command(const uint8_t *data, size_t data_len);
static void handle_set_break_command(const uint8_t *data, size_t data_len);
static void handle_set_enabled_command(const uint8_t *data, size_t data_len);
static void handle_power_off_command(const uint8_t *data, size_t data_len);
static void send_k0_adc_data_esp_now(void);


typedef struct
{
    uint8_t type;     // Broadcast or unicast ESPNOW data.
    uint8_t state;    // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num; // Sequence number of ESPNOW data.
    uint16_t crc;     // CRC16 value of ESPNOW data.
    uint32_t magic;   // Magic number which is used to determine which device to send unicast ESPNOW data.
    char payload[0];  // Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

static void pwm_init(int timer_num, int frequency, int channel, int gpio, int hpoint)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = timer_num,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = timer_num,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = LEDC_DUTY, // Set duty to 50%
        .hpoint = hpoint};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// Modifisert LED sett funksjon med forbedret fargekontroll
void led_set(uint8_t led_nr, uint8_t value, uint8_t r, uint8_t g, uint8_t b)
{
    if (value)
    {
        led_strip_set_pixel(led_strip, led_nr, r, g, b);
    }
    else
    {
        led_strip_set_pixel(led_strip, led_nr, 0, 0, 0);
    }
    
    // Error handling for LED refresh - unngå RMT-feil ved WiFi-problemer
    esp_err_t refresh_result = led_strip_refresh(led_strip);
    if (refresh_result != ESP_OK) {
        static uint32_t last_led_error_log = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        
        // Logg kun hvert 5. sekund for å unngå spam
        if (now - last_led_error_log > 5000) {
            ESP_LOGW(TAG, "⚠️ LED refresh feilet: %s (undertrykker lignende feil i 5 sek)", 
                     esp_err_to_name(refresh_result));
            last_led_error_log = now;
        }
        return; // Ikke oppdater led_val hvis refresh feilet
    }

    led_val[led_nr] = value;
}






// Enkel overladning av led_set for å opprettholde bakoverkompatibilitet
void led_set_simple(uint8_t led_nr, uint8_t value)
{
    led_set(led_nr, value, 16, 16, 16); // Standard hvit farge når ikke angitt
}

// Funksjon for å sette alle LED-er til samme farge
void set_all_leds(uint8_t value, uint8_t r, uint8_t g, uint8_t b) 
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (enabled[i]) {
            led_strip_set_pixel(led_strip, i, value ? r : 0, value ? g : 0, value ? b : 0);
        }
    }

    // Error handling for LED refresh - unngå RMT-feil ved WiFi-problemer
    esp_err_t refresh_result = led_strip_refresh(led_strip);
    if (refresh_result != ESP_OK) {
        static uint32_t last_led_error_log_all = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        
        // Logg kun hvert 5. sekund for å unngå spam
        if (now - last_led_error_log_all > 5000) {
            ESP_LOGW(TAG, "⚠️ set_all_leds refresh feilet: %s (undertrykker lignende feil i 5 sek)", 
                     esp_err_to_name(refresh_result));
            last_led_error_log_all = now;
        }
        return; // Avbryt hvis refresh feilet
    }
}

// Funksjon for å håndtere blinkemodus for sensorblokkeringssvarsel
void handle_blink_mode() 
{

    uint32_t current_time = esp_timer_get_time() / 1000;  // Konverter til millisekunder
    
    // Sjekk om alle sensorer er brutt
    bool all_sensors_broken = false;  // Start med false
    int active_sensors = 0;
    int broken_sensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (enabled[i]) {
            active_sensors++;
            if (sensor_break[i]) {
                broken_sensors++;
            }
        }
    }
    
    // Sett all_sensors_broken basert på tellerne
    if (active_sensors > 0 && broken_sensors >= min_broken_sensors_for_alert) {
        all_sensors_broken = true;
    }
    

    if (!blink_mode) {
        // Ikke i blinkemodus ennå
        if (all_sensors_broken) {
            // Første gang vi oppdager at alle er brutt
            if (all_sensors_broken_start_time == 0) {
                all_sensors_broken_start_time = current_time;
                ESP_LOGI(TAG, "Alle sensorer er brutt, starter timer");
            } 
            // Sjekk om det har gått nok tid
            else if (current_time - all_sensors_broken_start_time >= min_time_for_alert) {
                // LOGGMELDING 2: Legg til her, ved aktivering av blinkemodus
                ESP_LOGI(TAG, "Alle sensorer har vært brutt i %d ms - aktiverer blinkemodus", min_time_for_alert);
                blink_mode = true;
                blink_start_time = current_time;
                blink_state = true;
                set_system_status(STATUS_ERROR_SENSORS_BLOCKED); // Sett systemstatus
                set_all_leds(1, 255, 0, 0); // Start med rødt
            }
            
            // LOGGMELDING 3: Legg til her, etter tidsjekken
            ESP_LOGI(TAG, "Timer aktiv: %lu ms av %d ms", 
                     current_time - all_sensors_broken_start_time, min_time_for_alert);
        } else {
            // Hvis ikke alle sensorer er brutt, nullstill timeren
            if (all_sensors_broken_start_time != 0) {
                ESP_LOGI(TAG, "Ikke alle sensorer er brutt lenger, nullstiller timer");
                all_sensors_broken_start_time = 0;
            }
        }
    } else {
        // Vi er allerede i blinkemodus
        // Sjekk om det er på tide å blinke
        if (current_time - blink_start_time >= blink_interval) {
            blink_start_time = current_time;
            blink_state = !blink_state;
            
            if (blink_state) {
                normal_led_control = false;
                set_all_leds(1, 255, 0, 0);  // Rødt på
            } else {
                normal_led_control = false;
                set_all_leds(0, 0, 0, 0);    // Av
            }
        }
        
        // Sjekk om vi skal avslutte blinkemodus (når ikke alle sensorer lenger er brutt)
        if (!all_sensors_broken) {
            ESP_LOGI(TAG, "Ikke alle sensorer er brutt lenger, avslutter blinkemodus");
            blink_mode = false;
            all_sensors_broken_start_time = 0;
            
            // Tilbakestill status til READY
            set_system_status(STATUS_READY);
        }
    }
}

// Funksjon for å sette systemstatus med forbedrede statusoverganger og LED-stabilitet
void set_system_status(system_status_t status) {
    if (current_status != status) {
        ESP_LOGI(TAG, "System status endret: %d -> %d", current_status, status);
        
        // VIKTIG: Logg ytterligere informasjon om statusendringen
        const char* status_names[] = {
            "INITIALIZING", "WIFI_CONNECTING", "SERVER_CONNECTING", 
            "READY", "ERROR_WIFI", "ERROR_SERVER", "ERROR_SENSORS_BLOCKED", "CALIBRATING"
        };
        ESP_LOGI(TAG, "Status endret fra %s til %s", 
                status_names[current_status], status_names[status]);
        
        // KRITISK FIX: Sikre LED-stripe er i gyldig tilstand
        normal_led_control = false;
        
        // Robust LED reset-sekvens
        for (int attempt = 0; attempt < 3; attempt++) {
            esp_err_t led_result = ESP_OK;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    led_result = led_strip_set_pixel(led_strip, i, 0, 0, 0);
                    if (led_result != ESP_OK) {
                        ESP_LOGW(TAG, "LED reset attempt %d failed for LED %d: %s", 
                                attempt + 1, i, esp_err_to_name(led_result));
                        break;
                    }
                }
            }
            
            if (led_result == ESP_OK) {
                led_result = led_strip_refresh(led_strip);
                if (led_result == ESP_OK) {
                    break; // Success
                } else {
                    ESP_LOGW(TAG, "LED refresh attempt %d failed: %s", 
                            attempt + 1, esp_err_to_name(led_result));
                }
            }
            
            // Hvis forsøk feilet, vent og prøv igjen
            if (attempt < 2) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Kortere pause
        
        // Forbedret visning for spesifikke overganger
        if (status == STATUS_ERROR_WIFI) {
            ESP_LOGW(TAG, "Viser WiFi-feil-indikasjon (robust versjon)");
            // Enklere indikasjon uten mange blink
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    esp_err_t result = led_strip_set_pixel(led_strip, i, 255, 0, 0);
                    if (result != ESP_OK) {
                        ESP_LOGE(TAG, "Kritisk LED-feil: %s", esp_err_to_name(result));
                    }
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGE(TAG, "Kritisk LED refresh-feil: %s", esp_err_to_name(refresh_result));
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // Hold rødt i 1 sekund
        }
        
        if (status == STATUS_ERROR_SERVER) {
            ESP_LOGW(TAG, "Setter LED-er til ERROR_SERVER-mønster");
            // Finn første og siste LED og sett disse til rødt
            int first = -1, last = -1;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    if (first == -1) first = i;
                    last = i;
                }
            }
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    if (i == first || i == last) {
                        led_strip_set_pixel(led_strip, i, 255, 0, 0);
                    } else {
                        led_strip_set_pixel(led_strip, i, 0, 0, 0);
                    }
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGW(TAG, "LED refresh feil i ERROR_SERVER: %s", esp_err_to_name(refresh_result));
            }
        }




        
        if (status == STATUS_WIFI_CONNECTING) {
            ESP_LOGI(TAG, "Setter LED-er til WIFI_CONNECTING-mønster");
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    led_strip_set_pixel(led_strip, i, 0, 0, 255); // Blå farge for WiFi-tilkobling
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGW(TAG, "LED refresh feil i WIFI_CONNECTING: %s", esp_err_to_name(refresh_result));
            }
        }


        
        if (status == STATUS_READY) {
            ESP_LOGI(TAG, "Setter LED-er til READY-mønster");
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    led_strip_set_pixel(led_strip, i, 0, 255, 0); // Grønn for klar
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGW(TAG, "LED refresh feil i READY: %s", esp_err_to_name(refresh_result));
            }
        }
        
        if (status == STATUS_INITIALIZING) {
            ESP_LOGI(TAG, "Setter LED-er til INITIALIZING-mønster (hvitt)");
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    led_strip_set_pixel(led_strip, i, 64, 64, 64); // Hvit for initialisering
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGW(TAG, "LED refresh feil i READY: %s", esp_err_to_name(refresh_result));
            };
        }
        
        if (status == STATUS_CALIBRATING) {
            ESP_LOGI(TAG, "Setter LED-er til CALIBRATING-mønster");
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    led_strip_set_pixel(led_strip, i, 128, 0, 128); // Lilla for kalibrering
                }
            }
            esp_err_t refresh_result = led_strip_refresh(led_strip);
            if (refresh_result != ESP_OK) {
                ESP_LOGW(TAG, "LED refresh feil i READY: %s", esp_err_to_name(refresh_result));
            }
        }
        
        // VIKTIG: Sett statusvariabelen FØR vi kaller andre funksjoner
        current_status = status;
        status_animation_start_time = esp_timer_get_time() / 1000;
        status_animation_step = 0;
        
        // Tilbakestill variabler
        if (status != STATUS_READY) {
            ready_start_time = 0; 
        }
        
        if (current_status == STATUS_CALIBRATING && status == STATUS_READY) {
            calibration_completed = true;
        }
        
        if (status != STATUS_ERROR_SENSORS_BLOCKED) {
            blink_mode = false;
            all_sensors_broken_start_time = 0;
        }
        
        // ENDRE: Deaktiver normal_led_control for alle status-animasjoner
        normal_led_control = false;
        ESP_LOGI(TAG, "Status-overgang fullført til %s (LED-kontroll deaktivert for animasjoner)", 
                status_names[status]);
    }
}



// Forbedret funksjon for status-animasjoner
void handle_status_animation() {
    uint32_t current_time = esp_timer_get_time() / 1000; // Millisekunder
    uint32_t elapsed_time = current_time - status_animation_start_time;
    
    // Håndter sensorblokkering separat
    if (current_status == STATUS_ERROR_SENSORS_BLOCKED) {
        handle_blink_mode();
        return;
    }
    
    // Logge statusinformasjon med fast intervall (hvert 10. sekund)
    if (elapsed_time % 10000 < 50) {
        ESP_LOGI(TAG, "Status: %d - Kjører animasjon for %d ms", current_status, elapsed_time);
    }
    
    switch (current_status) {
        case STATUS_INITIALIZING: {
            // Oppstartssekvens: LED-er tennes én etter én, så slukkes alle
            uint32_t step_time = 200; // 200ms per LED
            uint32_t current_step = elapsed_time / step_time;
            
            if (current_step <= NUM_SENSORS) {
                // Tenn LED-er én etter én
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i]) {
                        led_set(i, (i < current_step), 
                               STATUS_COLORS[STATUS_INITIALIZING][0],
                               STATUS_COLORS[STATUS_INITIALIZING][1],
                               STATUS_COLORS[STATUS_INITIALIZING][2]);
                    }
                }
            } else if (current_step <= NUM_SENSORS * 2) {
                // Slukk LED-er én etter én
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i]) {
                        led_set(i, (i >= (current_step - NUM_SENSORS)), 
                               STATUS_COLORS[STATUS_INITIALIZING][0],
                               STATUS_COLORS[STATUS_INITIALIZING][1],
                               STATUS_COLORS[STATUS_INITIALIZING][2]);
                    }
                }
            } else if (current_step > NUM_SENSORS * 2 + 5) {
                // Automatisk overgang til neste status etter fullført oppstartssekvens
                // Legg til litt forsinkelse (5 steg) før bytte
                // Men bare hvis WiFi-tilkobling er nødvendig
                if (wifi_connected == false) {
                    ESP_LOGI(TAG, "Oppstartssekvens fullført, går videre til WiFi-tilkobling");
                    set_system_status(STATUS_WIFI_CONNECTING);
                }
            }
            break;
        }
        
        case STATUS_WIFI_CONNECTING: {
            // Roterende LED-er (blå)
            uint32_t step_time = 150; // Raskere rotasjon: 150ms per steg
            uint32_t active_sensors = 0;
            
            // Tell antall aktive sensorer
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    active_sensors++;
                }
            }
            
            if (active_sensors > 0) {
                // Beregn hvilken LED som skal være aktiv nå
                uint32_t current_led = (elapsed_time / step_time) % active_sensors;
                
                // Slå på den riktige LED-en basert på indeks
                int led_index = 0;
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i]) {
                        bool should_be_on = (led_index == current_led);
                        led_set(i, should_be_on, 
                              STATUS_COLORS[STATUS_WIFI_CONNECTING][0],
                              STATUS_COLORS[STATUS_WIFI_CONNECTING][1],
                              STATUS_COLORS[STATUS_WIFI_CONNECTING][2]);
                        led_index++;
                    } else {
                        // Slå av deaktiverte sensorer
                        led_set(i, 0, 0, 0, 0);
                    }
                }
            }
            break;
        }
        
        case STATUS_SERVER_CONNECTING: {
            // Alle LED-er blinker samtidig (gul)
            uint32_t blink_cycle = 500; // 0.5 sekund på/av
            bool on = ((elapsed_time / blink_cycle) % 2) == 0;
            
            set_all_leds(on, 
                       STATUS_COLORS[STATUS_SERVER_CONNECTING][0],
                       STATUS_COLORS[STATUS_SERVER_CONNECTING][1],
                       STATUS_COLORS[STATUS_SERVER_CONNECTING][2]);
            break;
        }
        
        case STATUS_READY: {
            // Standard grønn pulsering hver 30. sekund for kalibrert system
            uint32_t pulse_cycle = 60000; // 30 sekunder mellom hver pulsering
            uint32_t pulse_duration = 100; // 200ms pulsering
            
            uint32_t time_in_cycle = elapsed_time % pulse_cycle;
            bool pulse_on = time_in_cycle < pulse_duration;
            
            if (pulse_on) {
                // Vis kort pulsering (grønn) bare i sensor 0 med dempet lys
                if (enabled[0]) {
                    led_set(0, 1, 0, 100, 0);  // Dempet grønn (100 i stedet for 255)
                }
                normal_led_control = false;
            } else if (time_in_cycle < pulse_duration + 50) {
                // Slå av LED-er umiddelbart etter pulsering
                if (enabled[0]) {
                    led_set(0, 0, 0, 0, 0);
                }
                normal_led_control = false;
            } else {
                // Resten av tiden, aktiver normal LED-kontroll
                normal_led_control = true;
            }
            break;
        }
        
        case STATUS_ERROR_WIFI: {
            // Rytmisk blinking av annenhver LED (rødt mønster)
            uint32_t blink_cycle = 800; // 0.8 sekund per full syklus
            bool odd_leds = ((elapsed_time / (blink_cycle/2)) % 2) == 0;
            
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    bool should_be_on = (i % 2 == 0) ? odd_leds : !odd_leds;
                    led_set(i, should_be_on, 
                           STATUS_COLORS[STATUS_ERROR_WIFI][0],
                           STATUS_COLORS[STATUS_ERROR_WIFI][1],
                           STATUS_COLORS[STATUS_ERROR_WIFI][2]);
                }
            }
            
            // Logger periodisk for å bekrefte at vi fortsatt er i denne tilstanden
            if (elapsed_time % 10000 < 100) {
                ESP_LOGW(TAG, "WiFi-feil vedvarer etter %u ms", elapsed_time);
            }
            break;
        }
        
        case STATUS_ERROR_SERVER: {
            // Blinking av første og siste LED (rødt mønster)
            uint32_t blink_cycle = 800; // 0.8 sekund per full syklus
            bool on = ((elapsed_time / (blink_cycle/2)) % 2) == 0;
            
            // Finn først og siste aktive sensor
            int first_active = -1;
            int last_active = -1;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    if (first_active == -1) first_active = i;
                    last_active = i;
                }
            }
            
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    bool should_be_on = (i == first_active || i == last_active) ? on : false;
                    led_set(i, should_be_on, 
                           STATUS_COLORS[STATUS_ERROR_SERVER][0],
                           STATUS_COLORS[STATUS_ERROR_SERVER][1],
                           STATUS_COLORS[STATUS_ERROR_SERVER][2]);
                }
            }
            break;
        }
        
        case STATUS_CALIBRATING: {
            // Pulserende lilla for kalibrering
            uint32_t pulse_period = 1000; // 1 sekund pulsering
            uint32_t phase = (elapsed_time % pulse_period);
            uint8_t brightness = phase < (pulse_period / 2) 
                ? 50 + (phase * 200 / (pulse_period / 2))  // Øk fra 50 til 250
                : 250 - ((phase - (pulse_period / 2)) * 200 / (pulse_period / 2));  // Reduser fra 250 til 50
            
            // Sett fargene for hver sensor basert på kalibreringsstatus
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i]) {
                    if (i == highpoint_channel) {
                        // Aktiv sensor som kalibreres: Pulserende hvit/lilla
                        led_set(i, 1, brightness / 2, brightness / 8, brightness / 2);
                    } else if (i < highpoint_channel) {
                        // Kalibrert sensor: Grønn
                        led_set(i, 1, 0, 100, 0);
                    } else {
                        // Ikke kalibrert ennå: Dempet lilla
                        led_set(i, 1, 20, 0, 20);
                    }
                }
            }
            
            // Legg til kalibreringsprogresjon (indikator for hvor langt vi er kommet)
            float progress = (float)highpoint_offset / 8092.0;
            if (progress > 0.95) {
                // Når nesten ferdig med gjeldende sensor, blink hvitt/grønt
                if ((elapsed_time / 100) % 2 == 0) {
                    led_set(highpoint_channel, 1, 255, 255, 255);
                } else {
                    led_set(highpoint_channel, 1, 0, 255, 0);
                }
            }
            break;
        }
        
        default:
            break;
    }
}





void nvs_setup()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void nvs_read()
{
    esp_err_t err;
    // Open
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        switch (err)
        {
        case ESP_OK:
            printf("Done\n");
            printf("Restart counter = %" PRIu32 "\n", restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        char var_s[30];
        printf("Reading offsets in NVS ... ");
        err = ESP_OK;
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            sprintf(var_s, "offset_%d", i);
            err |= nvs_get_u16(my_handle, var_s, &offsets[i]);
            
            if (err != ESP_OK || offsets[i] == 0) {
                offsets[i] = 4000;  // fallback-verdi
                ESP_LOGW(TAG, "Setter offset[%d] til fallback-verdi: %d", i, offsets[i]);
                err = ESP_OK;  // Unngå at én feil blokkerer alle
            }
            
            sprintf(var_s, "break_limit_%d", i);
            err |= nvs_get_u16(my_handle, var_s, &break_limit[i]);

            sprintf(var_s, "enabled_%d", i);
            uint16_t tmp_enabled;
            err |= nvs_get_u16(my_handle, var_s, &tmp_enabled);
            enabled[i] = (tmp_enabled != 0);
        }
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
}

void nvs_update_restart()
{
    esp_err_t err;
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        // Write
        printf("Updating restart counter in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        // Close
        nvs_close(my_handle);
    }
}

void nvs_update_offsets()
{
    esp_err_t err;
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        char var_s[30];
        printf("Updating offsets in NVS ... ");
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            sprintf(var_s, "offset_%d", i);
            err = nvs_set_u16(my_handle, var_s, offsets[i]);
            sprintf(var_s, "break_limit_%d", i);
            err = nvs_set_u16(my_handle, var_s, break_limit[i]);
            sprintf(var_s, "enabled_%d", i);
            err = nvs_set_u16(my_handle, var_s, enabled[i]);
        }

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        // Close
        nvs_close(my_handle);
    }
}








// Hjelpefunksjon for å sjekke WiFi-status (kan kalles fra hovedløkke)
static bool is_wifi_healthy(void) {
    if (!wifi_connected || !wifi_connection_stable) {
        return false;
    }
    
    // Ekstra validering: sjekk faktisk tilkoblingsstatus
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "📶 WiFi health check feilet - ikke tilkoblet AP");
        return false;
    }
    
    return true;
}



// Ny WiFi monitor task for intelligent reconnect
static void wifi_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "🔄 WiFi monitor task startet");
    
    while (1) {
        // Sjekk om vi fortsatt trenger å kjøre
        if (wifi_connected && wifi_connection_stable) {
            ESP_LOGI(TAG, "🔄 WiFi stabil igjen, avslutter monitor task");
            break;
        }
        
        // Sjekk maksimalt antall forsøk
        if (wifi_reconnect_attempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
            ESP_LOGE(TAG, "🔄 Maks WiFi reconnect-forsøk nådd (%d), går til langvarig retry-modus", 
                     MAX_WIFI_RECONNECT_ATTEMPTS);
            
            // Gå til langvarig retry (hvert 30. sekund)
            while (!wifi_connected) {
                ESP_LOGI(TAG, "🔄 Langvarig WiFi retry-forsøk...");
                
                // Visuell indikasjon for langvarig retry
                normal_led_control = false;
                set_all_leds(1, 255, 0, 0);     // Rød
                vTaskDelay(500 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);       // Av
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                esp_wifi_connect();
                vTaskDelay(30000 / portTICK_PERIOD_MS);  // Vent 30 sekunder
            }
            break;
        }
        
        // Normal reconnect-forsøk
        wifi_reconnect_attempts++;
        ESP_LOGI(TAG, "🔄 WiFi reconnect-forsøk %d/%d", 
                 wifi_reconnect_attempts, MAX_WIFI_RECONNECT_ATTEMPTS);
        
        // Visuell indikasjon på reconnect-forsøk
        normal_led_control = false;
        for (int i = 0; i < wifi_reconnect_attempts; i++) {
            set_all_leds(1, 0, 0, 255);     // Blå for reconnect
            vTaskDelay(200 / portTICK_PERIOD_MS);
            set_all_leds(0, 0, 0, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        
        // Forsøk reconnect
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "🔄 WiFi reconnect-forsøk feilet: %s", esp_err_to_name(err));
        }
        
        // Vent før neste forsøk
        vTaskDelay(WIFI_RECONNECT_DELAY_MS / portTICK_PERIOD_MS);
    }
    
    // Cleanup ved avslutning
    ESP_LOGI(TAG, "🔄 WiFi monitor task avsluttet");
    wifi_monitor_task_handle = NULL;  // VIKTIG: Nullstill handle FØR delete
    vTaskDelete(NULL);


}







// Forbedret WiFi-event handler med robust disconnect/reconnect håndtering
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    // Håndtere WiFi-hendelser
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "📶 WiFi STA startet, forsøker å koble til AP");
                wifi_connected = false;
                wifi_connection_stable = false;
                esp_wifi_connect();
                set_system_status(STATUS_WIFI_CONNECTING);
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "📶 WiFi FRAKOBLET (årsak: %d) - starter overvåket reconnect", disconnected->reason);
                
                // Oppdater tilkoblingsstatus
                wifi_connected = false;
                wifi_connection_stable = false;
                wifi_disconnect_time = esp_timer_get_time() / 1000;  // Timestamp i ms
                
                // Visuell indikasjon på tap av WiFi
                normal_led_control = false;
                for (int i = 0; i < 3; i++) {
                    set_all_leds(1, 255, 100, 0);  // Oransje for "mister tilkobling"
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    set_all_leds(0, 0, 0, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                
                // Sett systemstatus til WiFi-feil
                set_system_status(STATUS_ERROR_WIFI);
                
                // Start WiFi monitor task (kun én gang)
                if (wifi_monitor_task_handle == NULL) {
                    BaseType_t task_result = xTaskCreate(wifi_monitor_task, "wifi_monitor", 3072, NULL, 4, &wifi_monitor_task_handle);
                    if (task_result == pdPASS) {
                        ESP_LOGI(TAG, "🔄 WiFi monitor task startet");
                    } else {
                        ESP_LOGE(TAG, "🔄 Feil ved opprettelse av WiFi monitor task");
                        wifi_monitor_task_handle = NULL;
                    }
                } else {
                    ESP_LOGW(TAG, "🔄 WiFi monitor task kjører allerede - hopper over");
                }

                break;
            }
            
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "📶 WiFi TILKOBLET - venter på IP-adresse");
                break;
                
            default:
                ESP_LOGD(TAG, "📶 WiFi event: %ld", event_id);
                break;
        }
    } 
    // Håndtere IP-hendelser
    else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "✅ WiFi HELT TILKOBLET! IP: " IPSTR, IP2STR(&event->ip_info.ip));
                
                // Oppdater tilkoblingsstatus
                wifi_connected = true;
                wifi_connection_stable = true;
                wifi_reconnect_attempts = 0;  // Reset forsøksteller
                
                // Stopp WiFi monitor task hvis den kjører
                if (wifi_monitor_task_handle != NULL) {
                    vTaskDelete(wifi_monitor_task_handle);
                    wifi_monitor_task_handle = NULL;
                    ESP_LOGI(TAG, "🔄 WiFi monitor task stoppet (tilkobling gjenopprettet)");
                }
                
                // Visuell bekreftelse på gjenopprettet tilkobling
                normal_led_control = false;
                for (int i = 0; i < 3; i++) {
                    set_all_leds(1, 0, 255, 0);  // Grønt for "tilkobling OK"
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    set_all_leds(0, 0, 0, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                
                // Gå tilbake til READY-status
                set_system_status(STATUS_READY);
                break;
            }
            
            case IP_EVENT_STA_LOST_IP:
                ESP_LOGW(TAG, "📶 Mistet IP-adresse");
                wifi_connected = false;
                wifi_connection_stable = false;
                break;
                
            default:
                ESP_LOGD(TAG, "📶 IP event: %ld", event_id);
                break;
        }
    }
}







static void adc_init()
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < 7; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channel[i], &config));
    }
}

void tcp_setup(void)
{
    int addr_family = 0;
    int ip_protocol = 0;

    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");
}

static void tcp_close()
{
    ESP_LOGI(TAG, "Socket closing");
    int err = close(sock);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to close: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Successfully closed");
}

static void tcp_connect()
{
    // Med ESP-NOW er TCP valgfritt, så ikke vis server connecting status
    // vTaskDelay(500 / portTICK_PERIOD_MS); // Fjernet

    ESP_LOGI(TAG, "Socket connecting to %s:%d (valgfritt med ESP-NOW)", host_ip, PORT);
    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGW(TAG, "TCP tilkobling feilet: errno %d (OK med ESP-NOW)", errno);
        // IKKE sett ERROR_SERVER status - ESP-NOW fungerer uavhengig
        connected = false;
        return;
    }
    
    ESP_LOGI(TAG, "TCP tilkobling vellykket (ekstra funksjonalitet)");
    connected = true;
    

    // Send vår MAC-adresse til serveren for ESP-NOW peer-registrering
    uint8_t mac[6];
    esp_err_t mac_err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (mac_err == ESP_OK) {
        char id_msg[32];
        sprintf(id_msg, "ID: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        int written = send(sock, id_msg, strlen(id_msg), 0);
        if (written > 0) {
            ESP_LOGI(TAG, "Sendt MAC-adresse til server: %s", id_msg);
        } else {
            ESP_LOGW(TAG, "Kunne ikke sende MAC-adresse til server");
        }
    }




}


static void publish_settings()
{
    char *event = malloc(200); // Freed after sending
    char enabled_s[40];
    char offset_s[50];
    char break_s[50];

    sprintf(enabled_s, "[%1d,%1d,%1d,%1d,%1d,%1d,%1d]",
            enabled[0],
            enabled[1],
            enabled[2],
            enabled[3],
            enabled[4],
            enabled[5],
            enabled[6]);
    sprintf(offset_s, "[%4d,%4d,%4d,%4d,%4d,%4d,%4d]",
            offsets[0],
            offsets[1],
            offsets[2],
            offsets[3],
            offsets[4],
            offsets[5],
            offsets[6]);
    sprintf(break_s, "[%4d,%4d,%4d,%4d,%4d,%4d,%4d]",
            break_limit[0],
            break_limit[1],
            break_limit[2],
            break_limit[3],
            break_limit[4],
            break_limit[5],
            break_limit[6]);
    sprintf(event, "{\"K\":2,\"M\":\"%s\",\"E\":%s,\"O\":%s,\"B\":%s,\"P\":true}\n", mac_addr, enabled_s, offset_s, break_s);
    ESP_LOGI(TAG, "%s", event);
    xQueueSendToBack(xQueueBreak, &event, portMAX_DELAY);
}

static void store_mac()
{
    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = ESP_OK;
    ret = esp_read_mac(base_mac_addr, ESP_MAC_EFUSE_FACTORY);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Unable to read MAC: errno %d", ret);
    }
    sprintf(mac_addr, "%02x:%02x:%02x:%02x:%02x:%02x",
            base_mac_addr[0],
            base_mac_addr[1],
            base_mac_addr[2],
            base_mac_addr[3],
            base_mac_addr[4],
            base_mac_addr[5]);
    ESP_LOGI(TAG, "Store MAC: %s", mac_addr);
}

static void sync_time()
{
    struct timeval tv;
    tv.tv_sec = timestamp; // POSIX timestamp in seconds
    tv.tv_usec = 0;        // microseconds
    settimeofday(&tv, NULL);

    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;
    struct timeval tv_now;

    time(&now);
    // Set timezone to Norway Standard Time
    setenv("TZ", "GMT+2", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Norway is: %s", strftime_buf);
    gettimeofday(&tv_now, NULL);
    ESP_LOGI(TAG, "Unix timestamp is: %lld", tv_now.tv_sec);
}



// Funksjon for å sende sensorbrudd via ESP-NOW
static void send_sensor_break_esp_now(int sensor_id, int break_state) {
    if (!esp_now_peer_added) {
        ESP_LOGW(TAG, "ESP-NOW peer ikke registrert, kan ikke sende sensorbrudd");
        return;
    }
    
    // Opprett datastruktur
    esp_now_sensor_break_t sensor_data = {0};
    sensor_data.k = 1;  // Meldingstype for sensorbrudd
    sensor_data.sensor_id = sensor_id;
    sensor_data.break_state = break_state;
    
    // Få nåværende tidsstempel
    struct timeval tv;
    gettimeofday(&tv, NULL);
    sensor_data.t = tv.tv_sec;
    sensor_data.u = tv.tv_usec;
    
    ESP_LOGI(TAG, "📡 ESP-NOW SEND: sensor_id=%d, break_state=%d, tid=%u.%06u", 
             sensor_id, break_state, sensor_data.t, sensor_data.u);
    
    // Send ESP-NOW melding
    esp_err_t result = esp_now_send(ap_mac_addr, (uint8_t*)&sensor_data, sizeof(sensor_data));
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "✅ ESP-NOW sensorbrudd sendt vellykket");
    } else {
        ESP_LOGE(TAG, "❌ ESP-NOW sending feilet: %s", esp_err_to_name(result));
    }
}


// Funksjon for å sjekke ESP-NOW status
static void log_esp_now_status(void) {
    ESP_LOGI(TAG, "📊 ESP-NOW Status:");
    ESP_LOGI(TAG, "   Peer registrert: %s", esp_now_peer_added ? "✅ Ja" : "❌ Nei");
    
    if (esp_now_peer_added) {
        ESP_LOGI(TAG, "   AP MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 ap_mac_addr[0], ap_mac_addr[1], ap_mac_addr[2],
                 ap_mac_addr[3], ap_mac_addr[4], ap_mac_addr[5]);
    }
    
    // Test ESP-NOW tilgjengelighet
    esp_now_peer_num_t peer_num = {0};
    esp_err_t ret = esp_now_get_peer_num(&peer_num);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "   Antall peers: %d", peer_num.total_num);
    }
}



// Funksjon for å sende K=0 ADC-data via ESP-NOW
static void send_k0_adc_data_esp_now(void) {
    if (!esp_now_peer_added) {
        ESP_LOGW(TAG, "ESP-NOW peer ikke registrert, kan ikke sende K=0 data");
        return;
    }
    
    // Beregn størrelse: 1 byte (K) + 4 bytes (T) + 4 bytes (U) + 7*4 bytes (ADC) + 7*4 bytes (Break status)
    size_t data_size = 1 + 4 + 4 + (7 * 4) + (7 * 4);
    uint8_t *esp_now_data = malloc(data_size);
    if (!esp_now_data) {
        ESP_LOGE(TAG, "Minneallokering feilet for K=0 ESP-NOW data");
        return;
    }
    
    // Få nåværende tidsstempel
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Bygg K=0 melding
    int pos = 0;
    esp_now_data[pos++] = 0; // K=0 for ADC-data
    
    // Tidsstempel (T og U)
    memcpy(&esp_now_data[pos], &tv.tv_sec, 4);
    pos += 4;
    memcpy(&esp_now_data[pos], &tv.tv_usec, 4);
    pos += 4;
    
    // ADC-verdier (V array)
    for (int i = 0; i < NUM_SENSORS; i++) {
        memcpy(&esp_now_data[pos], &adc_raw[0][i], 4);
        pos += 4;
    }
    
    // Break status (B array)
    for (int i = 0; i < NUM_SENSORS; i++) {
        int32_t break_val = sensor_break[i] ? 1 : 0;
        memcpy(&esp_now_data[pos], &break_val, 4);
        pos += 4;
    }
    
    ESP_LOGI(TAG, "📊 Sender K=0 ADC-data via ESP-NOW (størrelse: %d bytes)", data_size);
    
    // Send ESP-NOW melding
    esp_err_t result = esp_now_send(ap_mac_addr, esp_now_data, data_size);
    
    if (result == ESP_OK) {
        ESP_LOGD(TAG, "✅ K=0 ESP-NOW data sendt vellykket");
    } else {
        ESP_LOGE(TAG, "❌ K=0 ESP-NOW sending feilet: %s", esp_err_to_name(result));
    }
    
    free(esp_now_data);
}




static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    // Sjekk om dette er en system assignment-melding
    if (len >= sizeof(system_assign_t)) {
        system_assign_t *assign = (system_assign_t*)data;
        if (assign->msg_type == MSG_SYSTEM_ASSIGN) {
            ESP_LOGI(TAG, "📨 Mottok system assignment via ESP-NOW");
            handle_system_assignment(assign);
            return;
        }
    }

    // Sjekk om dette er en identify-forespørsel
    if (len >= sizeof(identify_request_t)) {
        identify_request_t *identify = (identify_request_t*)data;
        if (identify->msg_type == MSG_IDENTIFY_REQUEST) {
            ESP_LOGI(TAG, "✨ Mottok identify-forespørsel via ESP-NOW");
            handle_identify_request(identify);
            return;
        }
    }


    // Sjekk om dette er en kommando-melding
    if (len >= sizeof(command_msg_t)) {
        command_msg_t *command = (command_msg_t*)data;
        if (command->msg_type == MSG_COMMAND) {
            ESP_LOGI(TAG, "📨 Mottok kommando via ESP-NOW: type=%d", command->command_type);
            handle_esp_now_command(command, len);
            return;
        }
    }




    // Behandle vanlige ESP-NOW meldinger (eksisterende kode)
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    int payload_len = len - sizeof(example_espnow_data_t);

    ESP_LOGI(TAG, "Receive broadcast ESPNOW data, len = %d", len);
    ESP_LOGI(TAG, "Payload len = %d, payload %s", payload_len, buf->payload);
    if (strncmp(buf->payload, "SYNC", 4) == 0)
    {
        sync_time();
    }
}


static esp_err_t example_espnow_init(void)
{
    ESP_LOGI(TAG, "Initialiserer ESP-NOW med peer management");
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));

    // Hent AP's MAC-adresse fra WiFi-tilkoblingen
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret == ESP_OK) {
        // Kopier AP's MAC-adresse
        memcpy(ap_mac_addr, ap_info.bssid, ESP_NOW_ETH_ALEN);
        
        ESP_LOGI(TAG, "🔍 AP MAC-adresse hentet: %02x:%02x:%02x:%02x:%02x:%02x",
                 ap_mac_addr[0], ap_mac_addr[1], ap_mac_addr[2],
                 ap_mac_addr[3], ap_mac_addr[4], ap_mac_addr[5]);
        
        // Legg til AP som ESP-NOW peer
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, ap_mac_addr, ESP_NOW_ETH_ALEN);
        peer_info.channel = 0;  // Samme kanal som WiFi
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;  // Ingen kryptering for nå
        
        ret = esp_now_add_peer(&peer_info);
        if (ret == ESP_OK) {
            esp_now_peer_added = true;
            ESP_LOGI(TAG, "✅ ESP-NOW peer (AP) registrert vellykket");
        } else if (ret == ESP_ERR_ESPNOW_EXIST) {
            esp_now_peer_added = true;
            ESP_LOGI(TAG, "ℹ️ ESP-NOW peer (AP) allerede registrert");
        } else {
            ESP_LOGE(TAG, "❌ Feil ved registrering av ESP-NOW peer: %s", esp_err_to_name(ret));
        }
        
        // Legg også til broadcast peer for announce-meldinger
        esp_now_peer_info_t broadcast_peer = {0};
        uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        memcpy(broadcast_peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
        broadcast_peer.channel = 0;
        broadcast_peer.ifidx = WIFI_IF_STA;
        broadcast_peer.encrypt = false;
        
        esp_err_t broadcast_result = esp_now_add_peer(&broadcast_peer);
        if (broadcast_result == ESP_OK) {
            ESP_LOGI(TAG, "✅ ESP-NOW broadcast peer registrert for announce-meldinger");
        } else if (broadcast_result == ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGI(TAG, "ℹ️ ESP-NOW broadcast peer allerede registrert");
        } else {
            ESP_LOGE(TAG, "❌ Feil ved registrering av broadcast peer: %s", esp_err_to_name(broadcast_result));
        }
        
    } else {
        ESP_LOGE(TAG, "❌ Kunne ikke hente AP-informasjon: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "ESP-NOW sending vil ikke fungere uten AP MAC-adresse");
        
        // Legg til broadcast peer likevel for announce-meldinger
        esp_now_peer_info_t broadcast_peer = {0};
        uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        memcpy(broadcast_peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
        broadcast_peer.channel = 0;
        broadcast_peer.ifidx = WIFI_IF_STA;
        broadcast_peer.encrypt = false;
        
        esp_err_t broadcast_result = esp_now_add_peer(&broadcast_peer);
        if (broadcast_result == ESP_OK) {
            ESP_LOGI(TAG, "✅ ESP-NOW broadcast peer registrert (uten AP)");
        }
    }

    return ESP_OK;
}




// Funksjon for å sjekke om målestolpen er tilknyttet et system
static bool is_assigned_to_system(void) {
    return (my_assigned_system_id != 0x00000000);
}

// Funksjon for å sende announce-melding
static void send_pole_announce(void) {
    if (!esp_now_peer_added || is_assigned_to_system()) {
        return; // Ikke send hvis ikke ESP-NOW er klar eller vi allerede er tilknyttet
    }
    
    pole_announce_t announce = {0};
    announce.system_id = 0x00000000;  // Søker system
    announce.msg_type = MSG_POLE_ANNOUNCE;
    
    // Få egen MAC-adresse
    esp_read_mac(announce.mac, ESP_MAC_WIFI_STA);
    
    // Sett enhetsnavn og firmware-versjon
    strncpy(announce.device_name, "TimerGate Pole", sizeof(announce.device_name) - 1);
    announce.firmware_ver[0] = 1;
    announce.firmware_ver[1] = 0;
    announce.firmware_ver[2] = 0;
    announce.rssi_estimate = -50; // Estimat
    
    ESP_LOGI(TAG, "📢 Sender pole announce (søker system-tilknytning)");
    
    // Send via ESP-NOW til broadcast
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcast_mac, (uint8_t*)&announce, sizeof(announce));
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "❌ Feil ved sending av announce: %s", esp_err_to_name(result));
    }
}




// Funksjon for å håndtere system assignment
static void handle_system_assignment(const system_assign_t *assign) {
    // Sjekk at meldingen er rettet mot oss
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    
    if (memcmp(assign->target_mac, my_mac, 6) != 0) {
        return; // Ikke for oss
    }
    
    ESP_LOGI(TAG, "✅ Mottok system assignment fra System ID: %08X", assign->system_id);
    ESP_LOGI(TAG, "   System navn: %s", assign->system_name);
    
    // Lagre assignment
    my_assigned_system_id = assign->system_id;
    strncpy(assigned_system_name, assign->system_name, sizeof(assigned_system_name) - 1);
    assigned_system_name[sizeof(assigned_system_name) - 1] = '\0';
    
    // Stopp announce-meldinger
    announcement_active = false;
    
    // Lagre i NVS
    save_system_assignment_to_nvs();
    
    ESP_LOGI(TAG, "🔗 Målestolpe tilknyttet system: %s (ID: %08X)", 
             assigned_system_name, my_assigned_system_id);
}





// Funksjon for å laste system assignment fra NVS
static void load_system_assignment_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("assignment", NVS_READONLY, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Ingen lagret system assignment funnet - starter i discovery-modus");
        my_assigned_system_id = 0x00000000;
        announcement_active = true;
        return;
    }
    
    // Les system ID
    err = nvs_get_u32(nvs_handle, "system_id", &my_assigned_system_id);
    if (err != ESP_OK) {
        my_assigned_system_id = 0x00000000;
        announcement_active = true;
    }
    
    // Les system navn
    size_t required_size = sizeof(assigned_system_name);
    err = nvs_get_str(nvs_handle, "system_name", assigned_system_name, &required_size);
    if (err != ESP_OK) {
        assigned_system_name[0] = '\0';
    }
    
    nvs_close(nvs_handle);
    
    if (my_assigned_system_id != 0x00000000) {
        announcement_active = false; // Stopp announce hvis vi er tilknyttet
        ESP_LOGI(TAG, "🔗 Lastet system assignment: %s (ID: %08X)", 
                 assigned_system_name, my_assigned_system_id);
    } else {
        announcement_active = true;
        ESP_LOGI(TAG, "📢 Starter i discovery-modus (ikke tilknyttet system)");
    }
}

// Funksjon for å lagre system assignment til NVS
static void save_system_assignment_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("assignment", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Kunne ikke åpne NVS for assignment: %s", esp_err_to_name(err));
        return;
    }
    
    // Lagre system ID
    err = nvs_set_u32(nvs_handle, "system_id", my_assigned_system_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Kunne ikke lagre system_id: %s", esp_err_to_name(err));
    }
    
    // Lagre system navn
    err = nvs_set_str(nvs_handle, "system_name", assigned_system_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Kunne ikke lagre system_name: %s", esp_err_to_name(err));
    }
    
    // Commit endringer
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Kunne ikke committe NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "💾 System assignment lagret i NVS");
    }
    
    nvs_close(nvs_handle);
}



// Funksjon for å håndtere identify-forespørsler
static void handle_identify_request(const identify_request_t *msg) {
    // Sjekk at meldingen er rettet mot oss
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    
    if (memcmp(msg->target_mac, my_mac, 6) != 0) {
        return; // Ikke for oss
    }
    
    ESP_LOGI(TAG, "✨ Mottok identify-forespørsel fra System ID: %08X", msg->system_id);
    ESP_LOGI(TAG, "   Varighet: %d sekunder", msg->duration_sec);
    
    // Start identifikasjonsblinking
    start_identification_blink(msg->duration_sec);
}

// Funksjon for å starte identifikasjonsblinking
static void start_identification_blink(uint8_t duration) {
    if (duration < 5) duration = 5;   // Minimum 5 sekunder
    if (duration > 30) duration = 30; // Maksimum 30 sekunder
    
    identification_active = true;
    identification_end_time = esp_timer_get_time() + ((uint64_t)duration * 1000000);
    
    ESP_LOGI(TAG, "🔶 Starter identifikasjonsblinking i %d sekunder", duration);
    
    // Sett alle LED-er til kraftig oransje for å starte
    normal_led_control = false;
    set_all_leds(1, 255, 165, 0);  // Kraftig oransje
}

// Funksjon for å håndtere identifikasjonsanimasjon
static void handle_identification_animation(void) {
    if (!identification_active) {
        return;
    }
    
    uint64_t current_time = esp_timer_get_time();
    
    // Sjekk om identifikasjonsperioden er over
    if (current_time >= identification_end_time) {
        ESP_LOGI(TAG, "🔶 Identifikasjonsblinking fullført");
        identification_active = false;
        
        // Gå tilbake til normal LED-kontroll
        normal_led_control = true;
        
        // Slå av alle LED-er for å returnere til normal tilstand
        set_all_leds(0, 0, 0, 0);
        return;
    }
    
    // Kraftig oransje blinking med rask frekvens
    uint32_t blink_cycle = IDENTIFY_BLINK_INTERVAL_MS * 2; // 400ms total syklus
    uint32_t time_in_cycle = (current_time / 1000) % blink_cycle;
    bool blink_on = time_in_cycle < IDENTIFY_BLINK_INTERVAL_MS;
    
    if (blink_on) {
        // Kraftig oransje når på
        set_all_leds(1, 255, 165, 0);
    } else {
        // Av når blink er av
        set_all_leds(0, 0, 0, 0);
    }
    
    // Deaktiver normal LED-kontroll mens identifikasjon pågår
    normal_led_control = false;
}



// Hovedfunksjon for å håndtere ESP-NOW kommandoer
static void handle_esp_now_command(const command_msg_t *cmd, int len) {
    // Valider at kommandoen er for oss
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    
    // Sjekk om kommandoen er broadcast (alle nuller) eller spesifikt for oss
    bool is_broadcast = true;
    bool is_for_us = false;
    
    for (int i = 0; i < 6; i++) {
        if (cmd->target_mac[i] != 0x00) {
            is_broadcast = false;
            break;
        }
    }
    
    if (!is_broadcast) {
        is_for_us = (memcmp(cmd->target_mac, my_mac, 6) == 0);
    }
    
    if (!is_broadcast && !is_for_us) {
        ESP_LOGD(TAG, "Kommando ikke rettet mot oss, ignorerer");
        return;
    }
    
    // Beregn data-lengde
    size_t header_size = sizeof(command_msg_t);
    size_t data_len = (len > header_size) ? (len - header_size) : 0;
    const uint8_t *data = (len > header_size) ? cmd->data : NULL;
    
    ESP_LOGI(TAG, "🔧 Behandler ESP-NOW kommando type %d, data_len=%d", 
             cmd->command_type, data_len);
    
    // Dispatch til riktig handler basert på kommando-type
    switch (cmd->command_type) {
        case CMD_TIME_SYNC:
            handle_time_sync_command(data, data_len);
            break;
        case CMD_RESTART:
            handle_restart_command(data, data_len);
            break;
        case CMD_HSEARCH:
            handle_hsearch_command(data, data_len);
            break;
        case CMD_SET_BREAK:
            handle_set_break_command(data, data_len);
            break;
        case CMD_SET_ENABLED:
            handle_set_enabled_command(data, data_len);
            break;
        case CMD_POWER_OFF:
            handle_power_off_command(data, data_len);
            break;
        default:
            ESP_LOGW(TAG, "Ukjent kommando-type: %d", cmd->command_type);
            break;
    }
}



// Handler for tidssynkronisering via ESP-NOW
static void handle_time_sync_command(const uint8_t *data, size_t data_len) {
    if (data_len < sizeof(uint32_t)) {
        ESP_LOGW(TAG, "TIME_SYNC kommando har ugyldig data-lengde: %d", data_len);
        return;
    }
    
    // Les timestamp fra kommando-data
    uint32_t timestamp;
    memcpy(&timestamp, data, sizeof(uint32_t));
    
    ESP_LOGI(TAG, "⏰ ESP-NOW TIME_SYNC: Mottok timestamp %u", timestamp);
    
    // Sett lokal tid (samme logikk som TCP-versjon)
    struct timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    
    // Logg ny tid for bekreftelse
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;
    
    time(&now);
    setenv("TZ", "GMT+2", 1);
    tzset();
    
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "✅ ESP-NOW tid synkronisert: %s (Unix: %lld)", strftime_buf, (long long)now);
}



// Handler for restart-kommando via ESP-NOW
static void handle_restart_command(const uint8_t *data, size_t data_len) {
    uint8_t restart_type = 1; // Standard: soft restart
    
    if (data_len >= sizeof(uint8_t)) {
        restart_type = data[0];
    }
    
    ESP_LOGI(TAG, "🔄 ESP-NOW RESTART: Type %d", restart_type);
    
    // Vis visuell indikasjon basert på restart-type
    normal_led_control = false;
    
    switch (restart_type) {
        case 1: // Soft restart
            ESP_LOGI(TAG, "Utfører myk restart via ESP-NOW...");
            for (int i = 0; i < 3; i++) {
                set_all_leds(1, 0, 0, 255);  // Blå
                vTaskDelay(200 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            break;
            
        case 2: // Hard restart
            ESP_LOGI(TAG, "Utfører hard restart via ESP-NOW...");
            for (int i = 0; i < 5; i++) {
                set_all_leds(1, 255, 165, 0);  // Oransje
                vTaskDelay(150 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);
                vTaskDelay(150 / portTICK_PERIOD_MS);
            }
            break;
            
        case 3: // Factory reset
            ESP_LOGI(TAG, "Utfører factory reset via ESP-NOW...");
            for (int i = 0; i < 10; i++) {
                set_all_leds(1, 255, 0, 0);  // Rød
                vTaskDelay(100 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            
            // Slett NVS-innstillinger for factory reset
            if (restart_type == 3) {
                nvs_handle_t my_handle;
                esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Sletter NVS-innstillinger for factory reset...");
                    nvs_erase_all(my_handle);
                    nvs_commit(my_handle);
                    nvs_close(my_handle);
                }
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Ukjent restart-type: %d", restart_type);
            return;
    }
    
    ESP_LOGI(TAG, "✅ Restart kommando behandlet, restarter om 500ms...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_restart();
}



// Handler for highpoint search kommando via ESP-NOW
static void handle_hsearch_command(const uint8_t *data, size_t data_len) {
    uint8_t channel = 0; // Standard kanal
    
    if (data_len >= sizeof(uint8_t)) {
        channel = data[0];
    }
    
    ESP_LOGI(TAG, "🔍 ESP-NOW HSEARCH: Kanal %d", channel);
    
    // Start highpoint search (samme logikk som TCP-versjon)
    highpoint_search = true;
    highpoint_offset = 0;
    highpoint_channel = channel;
    highpoint_max = 0;
    
    // Sett status til kalibrering
    set_system_status(STATUS_CALIBRATING);
    
    ESP_LOGI(TAG, "✅ Highpoint search startet via ESP-NOW på kanal %d", channel);
}




// Handler for set break kommando via ESP-NOW
static void handle_set_break_command(const uint8_t *data, size_t data_len) {
    if (data_len < sizeof(uint8_t) + sizeof(uint16_t)) {
        ESP_LOGW(TAG, "SET_BREAK kommando har ugyldig data-lengde: %d", data_len);
        return;
    }
    
    // Les ADC nummer og break-verdi fra data
    uint8_t adc_nr = data[0];
    uint16_t break_val;
    memcpy(&break_val, &data[1], sizeof(uint16_t));
    
    ESP_LOGI(TAG, "⚙️ ESP-NOW SET_BREAK: ADC %d = %d", adc_nr, break_val);
    
    // Valider verdier (samme logikk som TCP-versjon)
    if (adc_nr < 7 && break_val < 4906) {
        break_limit[adc_nr] = break_val;
        ESP_LOGI(TAG, "✅ Break limit oppdatert: sensor %d = %d", adc_nr, break_val);
        
        // Hvis dette er siste sensor (6), lagre alle innstillinger
        if (adc_nr == 6) {
            nvs_update_offsets();
            publish_settings();
            ESP_LOGI(TAG, "💾 Alle break limits lagret og publisert");
        }
    } else {
        ESP_LOGW(TAG, "❌ Ugyldig break-verdier: ADC=%d, verdi=%d", adc_nr, break_val);
    }
}




// Handler for set enabled kommando via ESP-NOW
static void handle_set_enabled_command(const uint8_t *data, size_t data_len) {
    if (data_len < sizeof(uint8_t) + sizeof(uint8_t)) {
        ESP_LOGW(TAG, "SET_ENABLED kommando har ugyldig data-lengde: %d", data_len);
        return;
    }
    
    // Les sensor nummer og enabled-status fra data
    uint8_t sensor_nr = data[0];
    uint8_t is_enabled = data[1];
    
    ESP_LOGI(TAG, "🔧 ESP-NOW SET_ENABLED: Sensor %d = %s", 
             sensor_nr, is_enabled ? "aktivert" : "deaktivert");
    
    // Valider verdier (samme logikk som TCP-versjon)
    if (sensor_nr < 7) {
        enabled[sensor_nr] = (is_enabled != 0);
        
        // Slå av LED for denne sensoren
        led_set_simple(sensor_nr, 0);
        
        // Lagre innstillinger
        nvs_update_offsets();
        publish_settings();
        
        ESP_LOGI(TAG, "✅ Sensor %d %s via ESP-NOW", 
                 sensor_nr, enabled[sensor_nr] ? "aktivert" : "deaktivert");
    } else {
        ESP_LOGW(TAG, "❌ Ugyldig sensor nummer: %d", sensor_nr);
    }
}




// Handler for power off kommando via ESP-NOW
static void handle_power_off_command(const uint8_t *data, size_t data_len) {
    ESP_LOGI(TAG, "⚡ ESP-NOW POWER_OFF: Setter ESP32 i deep sleep");
    
    // Vis visuell indikasjon på at enheten skal soves (samme som TCP-versjon)
    normal_led_control = false;
    for (int i = 0; i < 3; i++) {
        set_all_leds(1, 255, 0, 0);  // Rød for avstengning
        vTaskDelay(200 / portTICK_PERIOD_MS);
        set_all_leds(0, 0, 0, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    // Lukk TCP-forbindelse på en ren måte (hvis tilkoblet)
    if (connected) {
        const char *shutdown_msg = "{\"K\":3,\"cmd\":\"powering_off_espnow\"}\n";
        send(sock, shutdown_msg, strlen(shutdown_msg), 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        close(sock);
        connected = false;
        ESP_LOGI(TAG, "TCP-forbindelse lukket før deep sleep");
    }
    
    // Slå av alle LED-er før deep sleep
    set_all_leds(0, 0, 0, 0);
    
    ESP_LOGI(TAG, "✅ Går inn i deep sleep modus via ESP-NOW kommando");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Sett deep sleep (samme som TCP-versjon)
    esp_deep_sleep_start();
}











static void check_socket()
{
    char rx_buffer[128];
    char command[50];
    int cmd_index = 0;

    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT);
    if (len < 0)
    {
        // ESP_LOGE(TAG, "recv failed: errno %d", errno);
    }
    else
    {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s: %s", len, host_ip, rx_buffer);
        for (int i = 0; i < len; i++)
        {
            command[cmd_index] = rx_buffer[i];
            if (rx_buffer[i] == '\n')
            {
                command[cmd_index] = 0;
                
                // Logg mottatt kommando for debugging
                ESP_LOGI(TAG, "Prosesserer kommando: '%s'", command);
                
                if (strncmp(command, "break", 5) == 0)
                {
                    int adc_nr = (uint8_t)atoi(command + 6);
                    uint16_t val = (uint16_t)strtol(command + 9, NULL, 10);
                    ESP_LOGI(TAG, "Got break: %d = %d", adc_nr, val);
                    if (adc_nr < 7 && val < 4906)
                    {
                        break_limit[adc_nr] = val;
                    }
                    if (adc_nr == 6)
                    {
                        nvs_update_offsets();
                        // DEAKTIVERT: TCP publish_settings() - ikke lenger nødvendig med ren ESP-NOW
                        // publish_settings();
                    }
                }


                else if (strncmp(command, "time", 4) == 0)
                {
                    timestamp = strtol(command + 5, NULL, 10);
                    ESP_LOGI(TAG, "Got timestamp: %lld", timestamp);
                    
                    // Initialiserer ESP-NOW med forbedret error handling
                    esp_err_t espnow_result = example_espnow_init();
                    if (espnow_result != ESP_OK) {
                        ESP_LOGE(TAG, "❌ ESP-NOW initialisering feilet: %s", esp_err_to_name(espnow_result));
                        ESP_LOGW(TAG, "⚠️ Sensorbrudd vil kun sendes via TCP");
                    } else {
                        ESP_LOGI(TAG, "✅ ESP-NOW initialisert - sensorbrudd sendes via både TCP og ESP-NOW");
                    }
                }



                else if (strncmp(command, "hsearch", 7) == 0)
                {
                    int channel = strtol(command + 8, NULL, 10);
                    ESP_LOGI(TAG, "Hsearch channel: %d", channel);
                    highpoint_search = true;
                    highpoint_offset = 0;
                    highpoint_channel = channel;
                    highpoint_max = 0;
                    
                    // Sett status til kalibrering når hsearch starter
                    set_system_status(STATUS_CALIBRATING);
                }
                else if (strncmp(command, "enabled", 7) == 0)
                {
                    int sensor_nr = (uint8_t)atoi(command + 8);
                    bool is_enabled = (bool)atoi(command + 11);
                    ESP_LOGI(TAG, "Got enabled: %d = %d", sensor_nr, is_enabled);
                    if (sensor_nr < 7)
                    {
                        enabled[sensor_nr] = is_enabled;
                        led_set_simple(sensor_nr, 0);
                        nvs_update_offsets();
                        // DEAKTIVERT: TCP publish_settings() - ikke lenger nødvendig med ren ESP-NOW
                        // publish_settings();    
                    }
                }
                else if (strncmp(command, "power", 5) == 0)
                {
                    ESP_LOGI(TAG, "Mottok power kommando: '%s'", command);
                    
                    if (strncmp(command + 7, "off", 3) == 0)  // Endret fra +6 til +7 for å hoppe over mellomrom
                    {
                        ESP_LOGI(TAG, "Mottok power off kommando - setter ESP32 i deep sleep");
                        
                        // Vis visuell indikasjon på at enheten skal soves
                        normal_led_control = false;
                        for (int i = 0; i < 3; i++) {
                            set_all_leds(1, 255, 0, 0);  // Rød for avstengning
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                        }
                        
                        // Lukk TCP-forbindelse på en ren måte
                        if (connected) {
                            const char *shutdown_msg = "{\"K\":3,\"cmd\":\"powering_off\"}\n";
                            send(sock, shutdown_msg, strlen(shutdown_msg), 0);
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            tcp_close();
                            connected = false;
                        }
                        
                        // Slå av alle LED-er før deep sleep
                        set_all_leds(0, 0, 0, 0);
                        
                        ESP_LOGI(TAG, "Går inn i deep sleep modus");
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        
                        //esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 sekunder i mikrosekunder, ønsker ikke automatisk opvåking fra deepsleep, må gjøres fysisk
                        esp_deep_sleep_start();
                    }
                    else if (strncmp(command + 7, "on", 2) == 0)  // Endret fra +6 til +7 for å hoppe over mellomrom
                    {
                        ESP_LOGI(TAG, "Mottok power on kommando - enheten er allerede våken");
                        
                        // Vis visuell indikasjon på at enheten er aktiv
                        normal_led_control = false;
                        for (int i = 0; i < 3; i++) {
                            set_all_leds(1, 0, 255, 0);  // Grønn for aktivering
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                        }
                        
                        // Send bekreftelse tilbake til server
                        const char *response_msg = "{\"K\":3,\"cmd\":\"power_on_confirmed\"}\n";
                        if (connected) {
                            send(sock, response_msg, strlen(response_msg), 0);
                        }
                        
                        // Gjenoppta normal drift
                        set_system_status(STATUS_READY);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Ukjent power kommando: '%s'", command);
                    }
                }
                else if (strncmp(command, "restart:", 8) == 0)
                {
                    // Finn restart-typen
                    char restart_type[10] = {0};
                    sscanf(command + 8, " %s", restart_type);
                    
                    ESP_LOGI(TAG, "Mottok restart-kommando: '%s'", restart_type);
                    
                    // Send bekreftelse tilbake til server før restart
                    char response_msg[100];
                    snprintf(response_msg, sizeof(response_msg), 
                             "{\"K\":3,\"cmd\":\"restart_%s_initiated\"}\n", restart_type);
                    
                    if (connected) {
                        send(sock, response_msg, strlen(response_msg), 0);
                        vTaskDelay(100 / portTICK_PERIOD_MS); // Gi tid til å sende respons
                    }
                    
                    if (strcmp(restart_type, "soft") == 0)
                    {
                        // Myk restart - normal restart som beholder alle innstillinger
                        ESP_LOGI(TAG, "Utfører myk restart...");
                        
                        // Vis visuell indikasjon (blå blinking)
                        normal_led_control = false;
                        for (int i = 0; i < 3; i++) {
                            set_all_leds(1, 0, 0, 255);  // Blå for myk restart
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                        }
                        
                        // Lukk TCP-forbindelse på en ren måte
                        if (connected) {
                            tcp_close();
                            connected = false;
                        }
                        
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        esp_restart();
                    } 
                    else if (strcmp(restart_type, "hard") == 0)
                    {
                        // Hard restart - mer aggressiv restart
                        ESP_LOGI(TAG, "Utfører hard restart...");
                        
                        // Vis visuell indikasjon (oransje blinking)
                        normal_led_control = false;
                        for (int i = 0; i < 5; i++) {
                            set_all_leds(1, 255, 165, 0);  // Oransje for hard restart
                            vTaskDelay(150 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(150 / portTICK_PERIOD_MS);
                        }
                        
                        // Ikke prøv å lukke TCP-forbindelse pent - bare restart
                        vTaskDelay(300 / portTICK_PERIOD_MS);
                        esp_restart();
                    }
                    else if (strcmp(restart_type, "factory") == 0)
                    {
                        // Factory reset - slett NVS innhold før restart
                        ESP_LOGI(TAG, "Utfører factory reset...");
                        
                        // Vis visuell advarsel (rød blinking)
                        normal_led_control = false;
                        for (int i = 0; i < 10; i++) {
                            set_all_leds(1, 255, 0, 0);  // Rød for factory reset
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                        }
                        
                        // Slett alle NVS-innstillinger
                        nvs_handle_t my_handle;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "Sletter alle NVS-innstillinger...");
                            err = nvs_erase_all(my_handle);
                            if (err == ESP_OK) {
                                ESP_LOGI(TAG, "NVS-innhold slettet. Committer endringer...");
                                err = nvs_commit(my_handle);
                                if (err == ESP_OK) {
                                    ESP_LOGI(TAG, "NVS-endringer committed vellykket");
                                } else {
                                    ESP_LOGE(TAG, "Feil ved commit av NVS-endringer: %s", esp_err_to_name(err));
                                }
                            } else {
                                ESP_LOGE(TAG, "Feil ved sletting av NVS: %s", esp_err_to_name(err));
                            }
                            nvs_close(my_handle);
                        } else {
                            ESP_LOGE(TAG, "Kunne ikke åpne NVS for factory reset: %s", esp_err_to_name(err));
                        }
                        
                        // Lukk TCP-forbindelse
                        if (connected) {
                            tcp_close();
                            connected = false;
                        }
                        
                        // Vis siste advarsel før restart
                        for (int i = 0; i < 3; i++) {
                            set_all_leds(1, 255, 0, 0);
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                            set_all_leds(0, 0, 0, 0);
                            vTaskDelay(200 / portTICK_PERIOD_MS);
                        }
                        
                        ESP_LOGI(TAG, "Factory reset fullført. Restarter...");
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        esp_restart();
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Ukjent restart-type: '%s'", restart_type);
                        
                        // Send feilmelding tilbake
                        const char *error_msg = "{\"K\":3,\"cmd\":\"restart_error_unknown_type\"}\n";
                        if (connected) {
                            send(sock, error_msg, strlen(error_msg), 0);
                        }
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Ukjent kommando: '%s'", command);
                }

                cmd_index = 0;
            }
            else
            {
                cmd_index++;
            }
        }
    }
}







static void tcp_client_task(void *pvParameters)
{
    char *cmd;
    char *break_cmd;

    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Økt delay for mindre CPU-bruk
        
        // Sjekk WiFi-status før TCP-operasjoner
        if (!is_wifi_healthy()) {
            if (connected) {
                ESP_LOGW(TAG, "📶 WiFi ikke tilgjengelig - avslutter TCP-forbindelse");
                connected = false;
                tcp_close();
            }
            continue;  // Hopp over TCP-logikk hvis WiFi er nede
        }

        if (!connected)
        {
            ESP_LOGI(TAG, "TCP ikke tilkoblet, forsøker å koble til (WiFi OK)");
            tcp_close();
            vTaskDelay(100 / portTICK_PERIOD_MS);
            tcp_setup();
            
            // Dobbeltsjekk WiFi før TCP connect
            if (is_wifi_healthy()) {
                set_system_status(STATUS_SERVER_CONNECTING);
                tcp_connect();
            } else {
                ESP_LOGW(TAG, "WiFi ble utilgjengelig under TCP-oppsett");
                continue;
            }
        }
        else
        {
            // Normal TCP-operasjoner (eksisterende kode)
            if (xQueueReceive(xQueue, &cmd, 0))
            {
                int written = send(sock, cmd, strlen(cmd), 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "TCP send feilet: errno %d", errno);
                    connected = false;
                }
            }
            if (xQueueReceive(xQueueBreak, &break_cmd, 0) == pdPASS)
            {
                int written = send(sock, break_cmd, strlen(break_cmd), 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "TCP send feilet: errno %d", errno);
                    connected = false;
                }
                if (break_cmd != NULL)
                {
                    free(break_cmd);
                    break_cmd = NULL;
                }
            }
            check_socket();
        }
    }


}



// Forbedret wifi_init funksjon med robust event handling
static void wifi_init() {
    ESP_LOGI(TAG, "📶 Initialiserer WiFi med robust overvåking");
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Registrer event-handlere FØRST
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL));

    esp_netif_create_default_wifi_sta();
    
    // Sett status til WiFi connecting
    set_system_status(STATUS_WIFI_CONNECTING);
    
    // Initialiser WiFi med optimaliserte innstillinger
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 1;
    cfg.nano_enable = 0;
    cfg.rx_ba_win = 16;   // Redusert for raskere respons
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Sett power save til none for raskere disconnect detection
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    // Robust WiFi-tilkobling uten example_connect
    ESP_LOGI(TAG, "📶 Starter initial WiFi-tilkobling med robust håndtering...");
    
    // Sett WiFi-konfigurasjon manuelt
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Timergate",
            .password = "12345678",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "📶 WiFi konfigurert - venter på event-driven tilkobling");
    
    // La event handler håndtere tilkoblingen
    wifi_connected = false;
    wifi_connection_stable = false;
}





static void pwm_setup()
{
    // Initialiser senderkanal
    pwm_init(LEDC_TIMER_0, PWM_0_FREQUENCY, LEDC_CHANNEL_0, PWM_SEND_0, 0);

    // Initialiser mottakerkanaler
    for (int i = 0; i < NUM_SENSORS; i++) {
        pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, rcv_channels[i], rcv_gpios[i], offsets[i]);
    }

    // Aktiver duty MED offset (hpoint) for alle mottakerkanaler
    for (int i = 0; i < NUM_SENSORS; i++) {
        ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[i], LEDC_DUTY, offsets[i]);
        ledc_update_duty(LEDC_MODE, rcv_channels[i]);
    }

    // Aktiver duty for senderkanal
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}



// Deklarasjon av WiFi-event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data); 

// Deklarasjoner for WiFi-overvåking
static void wifi_monitor_task(void *pvParameters);
static bool is_wifi_healthy(void);





void app_main(void)
{
    // Initialiser komponenter
    nvs_setup();
    nvs_read();

    // Last system assignment fra NVS
    load_system_assignment_from_nvs();



    // Sjekk oppvåkningsårsak etter deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool should_stay_awake = false;

    switch(wakeup_reason) {
            
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Normal oppstart (ikke fra deep sleep)");
            should_stay_awake = true;
            break;
    }

    // Hvis vi ikke skal forbli våkne, gå tilbake til deep sleep
    if (!should_stay_awake) {
        ESP_LOGI(TAG, "Går tilbake til deep sleep");
        esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 sekunder
        esp_deep_sleep_start();
    }



    pwm_setup();

    bool system_busy = false; 

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    // Sett oppstartsstatus
    set_system_status(STATUS_INITIALIZING);   

    esp_log_level_set("gpio", ESP_LOG_WARN);

    cmd = malloc(200);

    adc_init();

    // WiFi init MED robust error handling
    wifi_init();
    
    // KRITISK: Ikke start TCP eller ESP-NOW hvis WiFi feilet
    bool should_attempt_network = is_wifi_healthy();
    
    if (should_attempt_network) {
        ESP_LOGI(TAG, "✅ WiFi OK - starter nettverkstjenester");
        tcp_setup();
        tcp_connect();
    } else {
        ESP_LOGW(TAG, "⚠️ WiFi ikke tilgjengelig - hopper over nettverkstjenester");
        ESP_LOGI(TAG, "   Sensorer vil fungere lokalt, nettverksfunksjoner utilgjengelig");
        connected = false;  // Sikre at TCP-status er korrekt
    }
    
    store_mac();
    nvs_update_restart();

    // ESP-NOW kun hvis WiFi fungerer
    if (should_attempt_network) {
        ESP_LOGI(TAG, "📡 Starter ESP-NOW (WiFi OK)");
        esp_err_t espnow_result = example_espnow_init();
        if (espnow_result != ESP_OK) {
            ESP_LOGE(TAG, "❌ ESP-NOW feilet: %s", esp_err_to_name(espnow_result));
        } else {
            ESP_LOGI(TAG, "✅ ESP-NOW klar");
            log_esp_now_status();
        }
    }







    xQueue = xQueueCreate(1, sizeof(char *));
    xQueueBreak = xQueueCreate(30, sizeof(char *));
    
    // Start TCP task kun hvis WiFi fungerer
    if (should_attempt_network) {
        xTaskCreatePinnedToCore(tcp_client_task, "tcp_client", 4096, (void *)AF_INET, 5, NULL, 1);
    }


    publish_settings();


    // Legg til kode for å starte kalibrering umiddelbart
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint32_t wait_start = esp_timer_get_time() / 1000;
    while (true) {
        // Sjekk om systemet er klart - her kan du legge til mer spesifikke vilkår
        if (wifi_connected && connected && !system_busy) {
            break;
        }
        
        // Maksimal ventetid uansett
        if ((esp_timer_get_time() / 1000) - wait_start > 2000) {
            ESP_LOGW(TAG, "Tidsavbrudd ved venting på systemklargjøring");
            break;
        }
        
        // Kort pause for å ikke låse CPU
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Fortsett med kalibrering hvis systemet er klart
    if (wifi_connected && connected) {
        ESP_LOGI(TAG, "*** STARTER HIGHPOINT SEARCH ***");
        // Resten av kalibreringskoden...
    }







    // Sjekk om WiFi og server er tilkoblet før kalibrering startes
    if (wifi_connected && connected) {
        ESP_LOGI(TAG, "*** STARTER HIGHPOINT SEARCH MANUELT VED OPPSTART ***");
        highpoint_search = true;
        // Logg ESP-NOW status etter initialisering
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        log_esp_now_status();
        highpoint_offset = 0;
        highpoint_channel = 0;
        highpoint_max = 0;
        calibration_completed = false;
        set_system_status(STATUS_CALIBRATING);
    } else {
        ESP_LOGW(TAG, "Kan ikke starte kalibrering: WiFi %s", 
                wifi_connected ? "tilkoblet" : "frakoblet");
                
        // Sett riktig tilstand basert på WiFi-status
        if (!wifi_connected) {
            set_system_status(STATUS_ERROR_WIFI);
        }
    }


    // Sett høyere initialverdier for break_limit før kalibrering starter
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Behold eksisterende verdier hvis de er rimelige, ellers sett en konservativ verdi
        if (enabled[i] && break_limit[i] < 500) {
            ESP_LOGI(TAG, "Sensor %d: Endrer break_limit fra %d til 2000 (konservativ initialverdi)", 
                    i, break_limit[i]);
            break_limit[i] = 2000;  // Høyere initialverdi
        }
    }



   
    // Hovedløkke
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // ═══ PERIODISK WIFI HEALTH CHECK ═══
        static uint32_t last_wifi_check = 0;
        const uint32_t WIFI_CHECK_INTERVAL_MS = 5000;  // Sjekk WiFi hvert 5. sekund
        
        if (current_time - last_wifi_check >= WIFI_CHECK_INTERVAL_MS) {
            last_wifi_check = current_time;
            
            bool wifi_healthy = is_wifi_healthy();
            static bool prev_wifi_state = true;  // Anta OK ved oppstart
            
            if (wifi_healthy != prev_wifi_state) {
                if (wifi_healthy) {
                    ESP_LOGI(TAG, "📶 WiFi-tilkobling gjenopprettet - aktiverer nettverkstjenester");
                    
                    // Restart nettverkstjenester hvis de ikke kjører
                    if (!connected) {
                        tcp_setup();
                        tcp_connect();
                    }
                    
                    // Sikre ESP-NOW er initialisert
                    if (!esp_now_peer_added) {
                        example_espnow_init();
                    }
                    
                } else {
                    ESP_LOGW(TAG, "📶 WiFi-tilkobling tapt - deaktiverer nettverkstjenester");
                    connected = false;  // Stopp TCP-forsøk
                }
                prev_wifi_state = wifi_healthy;
            }
        }



        // ═══ DISCOVERY OG ANNOUNCE (kun med WiFi) ═══
        if (wifi_connected && announcement_active && !is_assigned_to_system()) {
            if (current_time - last_announce_time >= ANNOUNCE_INTERVAL_MS) {
                send_pole_announce();
                last_announce_time = current_time;
            }
        }


        curr_broken = false;
        int active_sensors = 0;
        
        // Les sensorverdier og oppdater sensor_break-tilstand
        for (int i = 0; i < 7; i++) {
            if (enabled[i]) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel[i], &adc_raw[0][i]));
                if (adc_raw[0][i] > 0) {
                    //ESP_LOGI(TAG, "ADC verdi for sensor %d: %d", i, adc_raw[0][i]);
                }

                
                // Hvis vi er i kalibrering, ikke still sensorer
                if (!highpoint_search) {
                    sensor_break[i] = (adc_raw[0][i] < break_limit[i]);
                    curr_broken |= sensor_break[i];
                } else {
                    // Under kalibrering, angi alle sensorer som ikke brutt
                    sensor_break[i] = false;
                }
                
                // Oppdater tellere
                active_sensors++;
            }
        }

        
        // Sjekk sensorblokkeringsstatus - men kun når vi IKKE er i kalibrering
        if (!highpoint_search && active_sensors > 0) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            // Tell antall brutte sensorer
            int broken_sensors = 0;
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i] && sensor_break[i]) {
                    broken_sensors++;
                }
            }
            
            // Logg for å se telleren
            //ESP_LOGI(TAG, "Sensortelling: %d brutt av %d aktive (krav: %d for alarm)", 
            //        broken_sensors, active_sensors, min_broken_sensors_for_alert);
            
            // Sjekk om nok sensorer er brutt
            if (broken_sensors >= min_broken_sensors_for_alert) {
                // Første gang nok sensorer er brutt
                if (all_sensors_broken_start_time == 0) {
                    all_sensors_broken_start_time = current_time;
                    ESP_LOGI(TAG, "Minst %d sensorer er brutt, starter timer", min_broken_sensors_for_alert);
                } 
                // Sjekk om det har gått nok tid
                else if (current_time - all_sensors_broken_start_time >= min_time_for_alert && 
                        current_status != STATUS_ERROR_SENSORS_BLOCKED) {
                    ESP_LOGI(TAG, "Minst %d sensorer har vært brutt i %d ms - aktiverer blinkemodus", 
                            min_broken_sensors_for_alert, min_time_for_alert);
                    blink_mode = true;  // Sett blink_mode direkte
                    set_system_status(STATUS_ERROR_SENSORS_BLOCKED);
                }
            } else if (all_sensors_broken_start_time != 0) {
                // Nullstill timer hvis ikke nok sensorer er brutt
                ESP_LOGI(TAG, "Færre enn %d sensorer er brutt nå, nullstiller timer", min_broken_sensors_for_alert);
                all_sensors_broken_start_time = 0;
                
                // Hvis vi er i feil-status, tilbakestill til READY
                if (current_status == STATUS_ERROR_SENSORS_BLOCKED) {
                    blink_mode = false;  // Stopp blink_mode direkte
                    set_system_status(STATUS_READY);
                }
            }
        }

        // Håndter status-animasjoner
        handle_status_animation();

        // Håndter identifikasjonsanimasjon (prioriteres over normal LED-kontroll)
        handle_identification_animation();



        // Oppdater LED-er basert på sensortilstand men bare hvis normal_led_control er aktivert og ikke under kalibrering
        if (normal_led_control && !highpoint_search) {
            for (int i = 0; i < 7; i++) {
                if (enabled[i]) {
                    led_set_simple(i, sensor_break[i]);
                }
            }
        }

     // ═══ ESP-NOW DATA SENDING (kun med WiFi) ═══
        if (!blink_mode && current_status != STATUS_ERROR_SENSORS_BLOCKED && 
            esp_now_peer_added && wifi_connected) {
            
            static uint32_t last_espnow_k0_send = 0;
            if (current_time - last_espnow_k0_send >= 1000) {
                send_k0_adc_data_esp_now();
                last_espnow_k0_send = current_time;
            }
        }






        // Sjekk om vi skal starte automatisk kalibrering
        if (current_status == STATUS_READY && !auto_calibration_started && connected) {
            if (ready_start_time == 0) {
                // Første gang vi er i READY-tilstand, start tid
                ready_start_time = esp_timer_get_time() / 1000; // Millisekunder
                ESP_LOGI(TAG, "System i READY-tilstand, starter timer for automatisk kalibrering");
            } else {
                // Sjekk om det har gått nok tid siden vi ble READY
                uint32_t current_time = esp_timer_get_time() / 1000;
                if (current_time - ready_start_time > 5000) { // 5 sekunder etter READY
                    ESP_LOGI(TAG, "Starter automatisk kalibrering etter 5 sekunder i READY-tilstand");
                    highpoint_search = true;
                    highpoint_offset = 0;
                    highpoint_channel = 0;
                    highpoint_max = 0;
                    calibration_completed = false;
                    set_system_status(STATUS_CALIBRATING);
                    auto_calibration_started = true;
                }
            }
        }



        // I koden for å starte automatisk kalibrering
        if (current_status == STATUS_READY && !auto_calibration_started && connected) {
            // Logg mer detaljer om tilstandene
            ESP_LOGI(TAG, "Sjekker kalibrering - Status: %d, Auto: %d, Connected: %d", 
                    current_status, auto_calibration_started, connected);
            
            if (ready_start_time == 0) {
                ready_start_time = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "System i READY-tilstand, starter timer for automatisk kalibrering: %lu", ready_start_time);
            } else {
                uint32_t current_time = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "Kalibrerings-timer: %lu/%lu (%lu ms siden READY)", 
                        current_time - ready_start_time, 5000, current_time - ready_start_time);
                
                if (current_time - ready_start_time > 5000) {
                    ESP_LOGI(TAG, "*** STARTER AUTOMATISK KALIBRERING ETTER 5 SEKUNDER I READY-TILSTAND ***");
                    highpoint_search = true;
                    highpoint_offset = 0;
                    highpoint_channel = 0;
                    highpoint_max = 0;
                    calibration_completed = false;
                    set_system_status(STATUS_CALIBRATING);
                    auto_calibration_started = true;
                }
            }
        }








        // ═══ AVBRYT KALIBRERING VED WIFI-TAP ═══
        if (highpoint_search && !wifi_connected) {
            ESP_LOGW(TAG, "Avbryter kalibrering på grunn av WiFi-tap");
            highpoint_search = false;
            set_system_status(STATUS_ERROR_WIFI);
        }




        // Håndter highpoint_search
        if (highpoint_search)
        {
            // Sikre at statusen er satt til kalibrering
            if (current_status != STATUS_CALIBRATING) {
                set_system_status(STATUS_CALIBRATING);
            }

            // Vis status for kalibrering
            set_system_status(STATUS_CALIBRATING);
            
            // Sett current_led til den sensoren som for øyeblikket kalibreres
            led_set(highpoint_channel, 1, 255, 0, 255); // Sett aktiv LED til lilla
            
            
            // Gi tid til å stabilisere
            vTaskDelay(5 / portTICK_PERIOD_MS);
            
            // Sett PWM for gjeldende sensor
            ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[highpoint_channel], LEDC_DUTY, highpoint_offset);
            ledc_update_duty(LEDC_MODE, rcv_channels[highpoint_channel]);
            
            // Gi tid til å stabilisere før måling
            vTaskDelay(15 / portTICK_PERIOD_MS);
            
            // Ta flere målinger og bruk gjennomsnitt for å redusere støy
            int sum_readings = 0;
            int valid_readings = 0;
            
            for (int j = 0; j < 3; j++) {
                int current_reading = 0;
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel[highpoint_channel], &current_reading));
                
                // Ignorer nullverdier og for lave verdier
                if (current_reading > 100) {
                    sum_readings += current_reading;
                    valid_readings++;
                }
                
                vTaskDelay(5 / portTICK_PERIOD_MS);  // Kort pause mellom målinger
            }
            


            //vTaskDelay(pdMS_TO_TICKS(10));  // Settling time etter offset-endring
            vTaskDelay(pdMS_TO_TICKS(2));  // Settling time etter offset-endring

            // Ekstra ADC-stabilisering: mål 8 ganger og ta snitt
            int stabilized_sum = 0;
            for (int i = 0; i < 8; ++i) {
                stabilized_sum += adc_raw[0][highpoint_channel];
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            adc_raw[0][highpoint_channel] = stabilized_sum / 8;





            // Oppdater adc_raw med gjennomsnittlig verdi hvis vi har gyldige målinger
            if (valid_readings > 0) {
                //adc_raw[0][highpoint_channel] = sum_readings / valid_readings;

                vTaskDelay(pdMS_TO_TICKS(10));  // Settling time etter offset-endring

                // Ekstra ADC-stabilisering: ny gjennomsnittsberegning over 8 målinger
                int stabilized_sum = 0;
                for (int i = 0; i < 8; ++i) {
                    stabilized_sum += sum_readings / valid_readings;
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
                adc_raw[0][highpoint_channel] = stabilized_sum / 8;

                // Oppdater max hvis den nye verdien er høyere
                if (adc_raw[0][highpoint_channel] > highpoint_max) {
                    // Logg betydelige nye høydepunkter
                    if (adc_raw[0][highpoint_channel] > 1000) {
                        ESP_LOGI(TAG, "Ny høyere verdi for sensor %d: %d ved offset %d", 
                                highpoint_channel, adc_raw[0][highpoint_channel], highpoint_offset);
                    }
                    
                    highpoint_max = adc_raw[0][highpoint_channel];
                    highpoint_offset_max = highpoint_offset;
                }
            }


        // Endre dette i highpoint_search-delen av koden
        if (highpoint_offset >= 8092) {
            // Sett optimal offset for denne sensoren
            ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[highpoint_channel], LEDC_DUTY, highpoint_offset_max);
            ledc_update_duty(LEDC_MODE, rcv_channels[highpoint_channel]);
            
            // Gi tid til å stabilisere
            vTaskDelay(20 / portTICK_PERIOD_MS);
            
            // Verifiser at sensoren fungerer med ny offset
            int verification_value = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel[highpoint_channel], &verification_value));
            
            // Sett break_limit basert på highpoint_max (robust versjon)
            if (highpoint_max > 500 && verification_value > 300) {  // Krev at både max OG verifikasjon er bra
                // Sett grensen til 80% av maksimumsverdien
                break_limit[highpoint_channel] = (uint16_t)(highpoint_max * 0.8);
                ESP_LOGI(TAG, "Satt ny break_limit for sensor %d: %d (80%% av maks %d)", 
                        highpoint_channel, break_limit[highpoint_channel], highpoint_max);
            } else {
                // Behold gjeldende break_limit hvis den finnes, ellers sett standardverdi
                if (break_limit[highpoint_channel] < 500) {
                    break_limit[highpoint_channel] = 1000;  // Standard break-grense
                }
                ESP_LOGW(TAG, "Sensor %d har for lav maksverdi (%d), beholder eksisterende verdi: %d", 
                        highpoint_channel, highpoint_max, break_limit[highpoint_channel]);
            }
            
            ESP_LOGI(TAG, "hsearch done for channel %d, max adc value: %d, offset: %d, break_limit: %d", 
                    highpoint_channel, highpoint_max, highpoint_offset_max, break_limit[highpoint_channel]);

            if (highpoint_channel == 6) {
                highpoint_search = false;
                
                // Sjekk om noen sensorer har fått brukbare verdier (Punkt 3, del 2)
                bool any_valid_sensors = false;
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i] && break_limit[i] > 100) {
                        any_valid_sensors = true;
                        break;
                    }
                }
                
                if (!any_valid_sensors) {
                    ESP_LOGW(TAG, "Ingen sensorer fikk gyldige kalibreringsverdier - bruker standardverdier");
                    // Sett standardverdier for alle sensorer
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        if (enabled[i]) {
                            break_limit[i] = 1000;  // Standard break-grense
                            offsets[i] = 4500;      // Standard offset
                        }
                    }
                }
                
                nvs_update_offsets();
                // DEAKTIVERT: TCP publish_settings() - ikke lenger nødvendig med ren ESP-NOW
                // publish_settings();
                calibration_completed = true;



                auto_calibration_started = true;  // Forhindre at automatisk kalibrering starter igjen
                ESP_LOGI(TAG, "Kalibrering fullført!");

  
                // Punkt 4 - Deaktiver sensor-blokkeringsfeil midlertidig
                all_sensors_broken_start_time = 0;
                blink_mode = false;  // Sikre at blinkemodus er deaktivert

                // Gi systemet litt tid til å stabilisere seg etter kalibrering
                ESP_LOGI(TAG, "Venter 0,5 sekunder for å stabilisere systemet etter kalibrering");
                vTaskDelay(500 / portTICK_PERIOD_MS);

                // Nullstill sensorbrudd-status etter kalibrering
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i]) {
                        sensor_break[i] = false;
                    }
                }
                curr_broken = false;
                prev_broken = false;

                set_system_status(STATUS_READY);
            }
            else {
                highpoint_channel++;
                highpoint_offset = 0;
                highpoint_max = 0;
            }
        }
                    
            //highpoint_offset += 30;
            highpoint_offset += 1000;
            //vTaskDelay(15 / portTICK_PERIOD_MS);  // Legg til en kort pause mellom justeringene
            vTaskDelay(5 / portTICK_PERIOD_MS);  // Legg til en kort pause mellom justeringene

        }
        else
        {

            // ═══ SENSOR BREAK DETECTION (kun med ESP-NOW/WiFi) ═══
            if (wifi_connected && esp_now_peer_added) {
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i] && !highpoint_search) {
                        bool current_sensor_break = (adc_raw[0][i] < break_limit[i]);
                        
                        if (current_sensor_break != prev_sensor_break[i]) {
                            send_sensor_break_esp_now(i, current_sensor_break ? 1 : 0);
                            prev_sensor_break[i] = current_sensor_break;
                            
                            ESP_LOGI(TAG, "🔄 Sensor %d endret: %s (ADC: %d) - Sendt via ESP-NOW", 
                                    i, current_sensor_break ? "BRUTT" : "OK", adc_raw[0][i]);
                        }
                    }
                }
            } else if (!wifi_connected) {
                // Logg sensorendringer lokalt selv uten nettverk
                static uint32_t last_local_log = 0;
                if (current_time - last_local_log >= 2000) {  // Hver 2. sekund
                    ESP_LOGI(TAG, "📊 OFFLINE: Sensor-status [%d,%d,%d,%d,%d,%d,%d] (WiFi ikke tilgjengelig)",
                            sensor_break[0], sensor_break[1], sensor_break[2], sensor_break[3],
                            sensor_break[4], sensor_break[5], sensor_break[6]);
                    last_local_log = current_time;
                }
            }





        }
    }
}