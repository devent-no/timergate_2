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
static TaskHandle_t wifi_reconnect_task_handle = NULL;


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
    // Fjern denne sjekken som kan forhindre oppdatering
    // if (led_val[led_nr] == value)
    //    return;

    if (value)
    {
        led_strip_set_pixel(led_strip, led_nr, r, g, b);
    }
    else
    {
        led_strip_set_pixel(led_strip, led_nr, 0, 0, 0);
    }
    led_strip_refresh(led_strip);
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
    led_strip_refresh(led_strip);
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
    
    // LEGG TIL LOGGMELDINGEN HER, rett etter koden ovenfor
    // if (broken_sensors >= min_broken_sensors_for_alert) {
    //     ESP_LOGI(TAG, "!!!SENSOR ALARM!!! %d av %d sensorer er brutt (krav: %d) - all_sensors_broken=%d", 
    //              broken_sensors, active_sensors, min_broken_sensors_for_alert, all_sensors_broken);
    // }

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

// Funksjon for å sette systemstatus med forbedrede statusoverganger
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
        
        // ENDRE: Slå av alle LED-er først, vent, og deretter vis tydelig indikasjon
        normal_led_control = false;
        set_all_leds(0, 0, 0, 0);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        
        // Forbedret visning for spesifikke overganger
        if (status == STATUS_ERROR_WIFI) {
            // ENDRE: Mer tydelig WiFi-feil-indikasjon
            ESP_LOGW(TAG, "Viser tydelig WiFi-feil-indikasjon");
            for (int i = 0; i < 3; i++) {
                set_all_leds(1, 255, 0, 0);  // Rødt
                vTaskDelay(300 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);  // Av
                vTaskDelay(300 / portTICK_PERIOD_MS);
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
        
        // ENDRE: Sett normal_led_control=false med tydelig indikasjon
        normal_led_control = false;
        ESP_LOGI(TAG, "Deaktiverte normal LED-kontroll for status %s", status_names[status]);
        
        // LEGG TIL: Initial visualisering av status med konstant farge
        switch (status) {
            case STATUS_ERROR_WIFI:
                ESP_LOGW(TAG, "Setter LED-er til ERROR_WIFI-mønster");
                // Alternevende rødt-av mønster for WiFi-feil
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (enabled[i]) {
                        led_set(i, i % 2 == 0, 255, 0, 0);
                    }
                }
                break;
                
            case STATUS_ERROR_SERVER:
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
                        led_set(i, (i == first || i == last), 255, 0, 0);
                    }
                }
                break;
                
            case STATUS_WIFI_CONNECTING:
                ESP_LOGI(TAG, "Setter LED-er til WIFI_CONNECTING-mønster");
                set_all_leds(1, 0, 0, 255); // Blå farge for WiFi-tilkobling
                break;
                
            case STATUS_READY:
                ESP_LOGI(TAG, "Setter LED-er til READY-mønster");
                set_all_leds(1, 0, 255, 0); // Grønn for klar
                break;
                
            case STATUS_INITIALIZING:
                ESP_LOGI(TAG, "Setter LED-er til INITIALIZING-mønster (hvitt)");
                set_all_leds(1, 64, 64, 64); // Hvit for initialisering
                break;
                
            case STATUS_CALIBRATING:
                ESP_LOGI(TAG, "Setter LED-er til CALIBRATING-mønster");
                set_all_leds(1, 128, 0, 128); // Lilla for kalibrering
                break;
                
            default:
                break;
        }
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




// Task for å håndtere WiFi-gjenoppkobling
static void wifi_reconnect_task(void *pvParameters) {
    while (1) {
        if (!wifi_connected) {
            ESP_LOGI(TAG, "WiFi ikke tilkoblet, forsøker å koble til igjen (forsøk %d/%d)",
                   wifi_reconnect_attempts + 1, MAX_WIFI_RECONNECT_ATTEMPTS);
            
            // LEGG TIL: Tydelig blinking før hvert reconnect-forsøk
            normal_led_control = false;
            for (int i = 0; i < 2; i++) {
                set_all_leds(1, 255, 165, 0);  // Oransje for reconnect-forsøk
                vTaskDelay(200 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            
            // Forsøke tilkobling
            esp_err_t err = esp_wifi_connect();
            
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "WiFi tilkoblingsforsøk feilet med feilkode %d", err);
                set_system_status(STATUS_ERROR_WIFI);
            } else {
                set_system_status(STATUS_WIFI_CONNECTING);
            }
            
            wifi_reconnect_attempts++;
            
            if (wifi_reconnect_attempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
                ESP_LOGE(TAG, "Nådde maksimalt antall WiFi-gjenoppkoblingsforsøk. Restarter enheten...");
                
                // LEGG TIL: Tydelig indikasjon på at enheten skal restarte
                normal_led_control = false;
                for (int i = 0; i < 10; i++) {
                    set_all_leds(1, 255, 0, 0);  // Rød
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    set_all_leds(0, 0, 0, 0);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                esp_restart();
            }
        } else {
            wifi_reconnect_attempts = 0;
        }
        //Ventetid mellom hvert forsøk på å koble til wifi
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


// WiFi-event handler for å håndtere tilkoblings- og frakoblingshendelser
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    // Håndtere WiFi-hendelser
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "WiFi STA startet, forsøker å koble til AP");
            esp_wifi_connect();
            
            // LEGG TIL: Tydelig blå indikasjon på at WiFi starter tilkobling
            normal_led_control = false;
            set_all_leds(0, 0, 0, 0); // Slå av alle LED-er først
            vTaskDelay(200 / portTICK_PERIOD_MS);
            set_all_leds(1, 0, 0, 255); // Blått lys for WiFi-tilkobling
            
            set_system_status(STATUS_WIFI_CONNECTING);
        } 
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGW(TAG, "WiFi frakoblet (reason: %d), starter gjenoppkoblingsprosess", disconnected->reason);
            
            // Oppdater WiFi-tilkoblingsstatus
            wifi_connected = false;
            
            // ENDRE: Bruk forsinket triggering av LED-indikasjonsfunksjoner
            normal_led_control = false;
            set_all_leds(0, 0, 0, 0); // Slå av alle LED-er
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // KRITISK: Vis rød blinking direkte uten å bruke set_system_status ennå
            for (int i = 0; i < 5; i++) {  // Flere blink for å være mer synlig
                set_all_leds(1, 255, 0, 0);  // Rødt
                vTaskDelay(200 / portTICK_PERIOD_MS); 
                set_all_leds(0, 0, 0, 0);    // Av
                vTaskDelay(200 / portTICK_PERIOD_MS);
                
                // Logg hver blinking så vi ser om den gjennomføres
                ESP_LOGW(TAG, "WiFi-feil blinkindikasjon %d/5", i+1);
            }
            
            // Sett systemstatus til ERROR_WIFI etter synlig indikasjon
            set_system_status(STATUS_ERROR_WIFI);
            
            // Start reconnect task i stedet for å prøve umiddelbart
            if (wifi_reconnect_task_handle == NULL) {
                xTaskCreate(wifi_reconnect_task, "wifi_reconnect", 4096, NULL, 3, &wifi_reconnect_task_handle);
                ESP_LOGI(TAG, "WiFi-gjenoppkoblings-task startet");
            }
        }
    } 
    // Håndtere IP-hendelser
    else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "WiFi tilkoblet! IP-adresse: " IPSTR, IP2STR(&event->ip_info.ip));
            
            // Oppdater WiFi-tilkoblingsstatus
            wifi_connected = true;
            
            // LEGG TIL: Visuell indikasjon på vellykket WiFi-tilkobling
            normal_led_control = false;
            for (int i = 0; i < 3; i++) {
                set_all_leds(1, 0, 255, 0);  // Grønt for vellykket
                vTaskDelay(100 / portTICK_PERIOD_MS);
                set_all_leds(0, 0, 0, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            
            // Endre status til SERVER_CONNECTING
            if (current_status == STATUS_ERROR_WIFI || current_status == STATUS_WIFI_CONNECTING) {
                set_system_status(STATUS_SERVER_CONNECTING);
            }
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
    // Vis STATUS_SERVER_CONNECTING lenger før vi forsøker tilkobling
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Socket connecting to %s:%d", host_ip, PORT);
    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        set_system_status(STATUS_ERROR_SERVER);
        return;
    }
    
    ESP_LOGI(TAG, "Successfully connected to server");
    connected = true;
    
    // Vent lenger før status endres til READY
    vTaskDelay(500 / portTICK_PERIOD_MS);
    set_system_status(STATUS_READY);
}

static void add_to_queue(char *msg)
{
    strncpy(cmd, msg, strlen(msg) + 1);
    xQueueOverwrite(xQueue, &cmd);
}

static void publish_sensor()
{
    char event[140];
    char adc_vals_s[40];
    char broken_s[40];

    sprintf(adc_vals_s, "[%4d,%4d,%4d,%4d,%4d,%4d,%4d]",
            adc_raw[0][0],
            adc_raw[0][1],
            adc_raw[0][2],
            adc_raw[0][3],
            adc_raw[0][4],
            adc_raw[0][5],
            adc_raw[0][6]);
    sprintf(broken_s, "[%1d,%1d,%1d,%1d,%1d,%1d,%1d]",
            sensor_break[0],
            sensor_break[1],
            sensor_break[2],
            sensor_break[3],
            sensor_break[4],
            sensor_break[5],
            sensor_break[6]);
    sprintf(event, "{\"K\":0,\"M\":\"%s\",\"V\":%s,\"B\":%s}\n", mac_addr, adc_vals_s, broken_s);
    add_to_queue(event);
    //ESP_LOGI(TAG, "%s", adc_vals_s); //MIDLERTIDIG for å ikke drukne loggen
}


static void publish_break(int sensor_id, int broken)
{
    char *event = malloc(140);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    ESP_LOGI(TAG, "Break state change sensor %d: %d @ %lld", sensor_id, broken, tv_now.tv_sec);
    sprintf(event, "{\"K\":1,\"M\":\"%s\",\"S\":%d,\"B\":%d,\"T\":%lld, \"U\":%ld}\n", 
            mac_addr, sensor_id, broken, tv_now.tv_sec, tv_now.tv_usec);
    xQueueSendToBack(xQueueBreak, &event, portMAX_DELAY);
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

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

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
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));

    return ESP_OK;
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
                        publish_settings();
                    }
                }
                else if (strncmp(command, "time", 4) == 0)
                {
                    timestamp = strtol(command + 5, NULL, 10);
                    ESP_LOGI(TAG, "Got timestamp: %lld", timestamp);
                    example_espnow_init();
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
                        publish_settings();    
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
                        
                        // Sett deep sleep med timer wakeup (f.eks. hver 30 sekund for å sjekke om vi skal våkne)
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
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (!connected)
        {
            ESP_LOGI(TAG, "Not connected, trying to connect");
            tcp_close();
            vTaskDelay(100 / portTICK_PERIOD_MS);
            tcp_setup();
            
            // Sjekk WiFi-status før TCP-tilkobling
            wifi_ap_record_t ap_info;
            esp_err_t wifi_status = esp_wifi_sta_get_ap_info(&ap_info);
            
            if (wifi_status == ESP_OK) {
                // WiFi er tilkoblet, sett server connecting
                set_system_status(STATUS_SERVER_CONNECTING);
                tcp_connect();
            } else {
                // WiFi er ikke tilkoblet, sett WiFi error
                set_system_status(STATUS_ERROR_WIFI);
            }
        }
        else
        {
            if (xQueueReceive(xQueue, &cmd, 0))
            {
                // ESP_LOGI(TAG, "Sending from queue: %s", cmd);
                int written = send(sock, cmd, strlen(cmd), 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    connected = false;
                }
            }
            if (xQueueReceive(xQueueBreak, &break_cmd, 0) == pdPASS)
            {
                // ESP_LOGI(TAG, "Sending from xQueueBreak: %s", break_cmd);
                int written = send(sock, break_cmd, strlen(break_cmd), 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
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



static void wifi_init()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Registrer event-handlere
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    // Sett status til WiFi connecting før vi starter tilkobling
    set_system_status(STATUS_WIFI_CONNECTING);

    // Gi tid til å vise WiFi-tilkoblingsstatus, men ikke for lenge
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Initialiser WiFi med optimaliserte innstillinger
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 1;
    cfg.nano_enable = 0;
    cfg.rx_ba_win = 32;   // Redusere buffer for raskere håndtering
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Deaktiver strømsparing for raskere tilkobling
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    
    // Bruk example_connect
    esp_err_t ret = example_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi tilkobling feilet med kode %d", ret);
        set_system_status(STATUS_ERROR_WIFI);
        return;
    }
    
    // Status blir oppdatert i event handler når tilkoblingen er ferdig
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




void app_main(void)
{
    // Initialiser komponenter
    nvs_setup();
    nvs_read();



    // Sjekk oppvåkningsårsak etter deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool should_stay_awake = false;

    switch(wakeup_reason) {
        // case ESP_SLEEP_WAKEUP_TIMER:
        //     ESP_LOGI(TAG, "Våknet fra deep sleep på grunn av timer");
            
        //     // Prøv å koble til nettverk for å sjekke om det er en "power on"-kommando
        //     // Dette er en forenklet tilnærming - i praksis kan vi implementere mer sofistikert logikk
        //     should_stay_awake = true; // For nå, anta at vi skal forbli våkne
        //     break;
            
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
    // for (int i = 0; i < NUM_SENSORS; i++) {
    //     //ledc_set_duty(LEDC_MODE, rcv_channels[i], LEDC_DUTY);
    //     ledc_update_duty(LEDC_MODE, rcv_channels[i]);
    // }




    bool system_busy = false; 


    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    // Sett oppstartsstatus
    set_system_status(STATUS_INITIALIZING);   

    esp_log_level_set("gpio", ESP_LOG_WARN);

    cmd = malloc(200);

    adc_init();
    wifi_init();
    tcp_setup();
    tcp_connect();
    store_mac();
    nvs_update_restart();

    xQueue = xQueueCreate(1, sizeof(char *));
    xQueueBreak = xQueueCreate(30, sizeof(char *));
    xTaskCreatePinnedToCore(tcp_client_task, "tcp_client", 4096, (void *)AF_INET, 5, NULL, 1);

       
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
        highpoint_offset = 0;
        highpoint_channel = 0;
        highpoint_max = 0;
        calibration_completed = false;
        set_system_status(STATUS_CALIBRATING);
    } else {
        ESP_LOGW(TAG, "Kan ikke starte kalibrering: WiFi %s, Server %s", 
                wifi_connected ? "tilkoblet" : "frakoblet",
                connected ? "tilkoblet" : "frakoblet");
                
        // Sett riktig tilstand basert på tilkoblingsstatus
        if (!wifi_connected) {
            set_system_status(STATUS_ERROR_WIFI);
        } else if (!connected) {
            set_system_status(STATUS_ERROR_SERVER);
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

        curr_broken = false;
        bool all_sensors_broken = true;
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
                
                // Oppdater tellere for all_sensors_broken sjekk
                active_sensors++;
                if (!sensor_break[i]) {
                    all_sensors_broken = false;
                }
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

        // Oppdater LED-er basert på sensortilstand men bare hvis normal_led_control er aktivert og ikke under kalibrering
        if (normal_led_control && !highpoint_search) {
            for (int i = 0; i < 7; i++) {
                if (enabled[i]) {
                    led_set_simple(i, sensor_break[i]);
                }
            }
        }

        // Send sensor data til serveren kun hvis vi ikke er i rødblink-modus
        // if (!blink_mode && current_status != STATUS_ERROR_SENSORS_BLOCKED) {
        //     publish_sensor();
        // }

                // Send sensor data til serveren kun hvis vi ikke er i rødblink-modus
        if (!blink_mode && current_status != STATUS_ERROR_SENSORS_BLOCKED) {
            // Reduser frekvens fra 10ms til 50ms (5x reduksjon, ikke 10x)
            static uint32_t last_publish = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            //if (now - last_publish >= 50) {  // 50ms = 20Hz i stedet for 100Hz
            if (now - last_publish >= 200) {  // 50ms = 20Hz i stedet for 100Hz
                publish_sensor();
                last_publish = now;
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




        // I hovedløkken, rett før sjekk av highpoint_search
        // Legg til jevnlig logging av systemtilstand
        //if (esp_timer_get_time() / 1000000 % 5 == 0) { // Logg hvert 5. sekund
            //ESP_LOGI(TAG, "Systemstatus: %d, Connected: %d, Auto Calibration Started: %d, Ready Time: %lu", 
            //        current_status, connected, auto_calibration_started, ready_start_time);
        //}

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








        if (highpoint_search && (!wifi_connected || !connected)) {
            ESP_LOGW(TAG, "Avbryter highpoint search: WiFi %s, Server %s", 
                    wifi_connected ? "tilkoblet" : "frakoblet",
                    connected ? "tilkoblet" : "frakoblet");
                    
            // Avbryt kalibrering ved nettverksproblemer
            highpoint_search = false;
            
            // Sett riktig feilstatus
            if (!wifi_connected) {
                set_system_status(STATUS_ERROR_WIFI);
            } else {
                set_system_status(STATUS_ERROR_SERVER);
            }
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
            
            // Deaktiver andre sensorer midlertidig for å redusere interferens
            // for (int i = 0; i < NUM_SENSORS; i++) {
            //     if (i != highpoint_channel && enabled[i]) {
            //         // Sett PWM til 0 for alle andre sensorer
            //         ledc_set_duty(LEDC_MODE, rcv_channels[i], 0);
            //         ledc_update_duty(LEDC_MODE, rcv_channels[i]);
            //     }
            // }
            
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
                publish_settings();
                calibration_completed = true;

                // Gjenopprett PWM-signaler for alle sensorer med deres optimale offset
                // for (int i = 0; i < NUM_SENSORS; i++) {
                //     if (enabled[i]) {
                //         ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[i], LEDC_DUTY, offsets[i]);
                //         ledc_update_duty(LEDC_MODE, rcv_channels[i]);
                //         ESP_LOGI(TAG, "Gjenopprettet PWM for sensor %d med offset %d", i, offsets[i]);
                //     }
                // }


                auto_calibration_started = true;  // Forhindre at automatisk kalibrering starter igjen
                ESP_LOGI(TAG, "Kalibrering fullført!");

                // Gjenopprett PWM-signaler for alle sensorer med deres optimale offset
                // for (int i = 0; i < NUM_SENSORS; i++) {
                //     if (enabled[i]) {
                //         ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[i], LEDC_DUTY, offsets[i]);
                //         ledc_update_duty(LEDC_MODE, rcv_channels[i]);
                //         ESP_LOGI(TAG, "Gjenopprettet PWM for sensor %d med offset %d", i, offsets[i]);
                //     }
                // }

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

            // Sjekk hver sensor individuelt for endringer og send break-events
            for (int i = 0; i < NUM_SENSORS; i++) {
                if (enabled[i] && !highpoint_search) {
                    bool current_sensor_break = (adc_raw[0][i] < break_limit[i]);
                    
                    // Send event hvis denne sensoren har endret tilstand
                    if (current_sensor_break != prev_sensor_break[i]) {
                        publish_break(i, current_sensor_break ? 1 : 0);
                        prev_sensor_break[i] = current_sensor_break;
                        
                        ESP_LOGI(TAG, "Sensor %d endret tilstand: %s (ADC: %d, limit: %d)", 
                                i, current_sensor_break ? "BRUTT" : "OK", 
                                adc_raw[0][i], break_limit[i]);
                    }
                }
            }

        }
    }
}