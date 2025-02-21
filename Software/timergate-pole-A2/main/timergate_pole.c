/* Timergate Pole for hardware revision A2 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

void led_set(uint8_t led_nr, uint8_t value)
{
    if (led_val[led_nr] == value)
        return;

    if (value)
    {
        led_strip_set_pixel(led_strip, led_nr, 16, 16, 16);
        led_strip_refresh(led_strip);
    }
    else
    {
        led_strip_set_pixel(led_strip, led_nr, 0, 0, 0);
        led_strip_refresh(led_strip);
    }

    led_val[led_nr] = value;
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
            
            sprintf(var_s, "break_limit_%d", i);
            err |= nvs_get_u16(my_handle, var_s, &break_limit[i]);
            sprintf(var_s, "enabled_%d", i);
            err |= nvs_get_u16(my_handle, var_s, &enabled[i]);
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
        .atten = ADC_ATTEN_DB_11,
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
    ESP_LOGI(TAG, "Socket connecting to %s:%d", host_ip, PORT);
    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Successfully connected");
    connected = true;
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
    ESP_LOGI(TAG, "%s", adc_vals_s);
}

static void publish_break(int broken)
{
    char *event = malloc(140);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    ESP_LOGI(TAG, "Break state change: %d @ %lld", curr_broken, tv_now.tv_sec);
    sprintf(event, "{\"K\":1,\"M\":\"%s\",\"B\":%d,\"T\":%lld, \"U\":%ld}\n", mac_addr, broken, tv_now.tv_sec, tv_now.tv_usec);
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
    sprintf(event, "{\"K\":2,\"M\":\"%s\",\"E\":%s,\"O\":%s,\"B\":%s}\n", mac_addr, enabled_s, offset_s, break_s);
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
                if (strncmp(command, "time", 4) == 0)
                {
                    timestamp = strtol(command + 5, NULL, 10);
                    ESP_LOGI(TAG, "Got timestamp: %lld", timestamp);
                    example_espnow_init();
                }
                if (strncmp(command, "hsearch", 7) == 0)
                {
                    int channel = strtol(command + 8, NULL, 10);
                    ESP_LOGI(TAG, "Hsearch channel: %d", channel);
                    highpoint_search = true;
                    highpoint_offset = 0;
                    highpoint_channel = channel;
                    highpoint_max = 0;
                }
                if (strncmp(command, "enabled", 7) == 0)
                {
                    int sensor_nr = (uint8_t)atoi(command + 8);
                    bool is_enabled = (bool)atoi(command + 11);
                    ESP_LOGI(TAG, "Got enabled: %d = %d", sensor_nr, is_enabled);
                    if (sensor_nr < 7)
                    {
                        enabled[sensor_nr] = is_enabled;
                        led_set(sensor_nr, 0);
                        nvs_update_offsets();
                        publish_settings();    
                    }
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
            tcp_connect();
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
    ESP_ERROR_CHECK(example_connect());
}

static void pwm_setup(){
    pwm_init(LEDC_TIMER_0, PWM_0_FREQUENCY, LEDC_CHANNEL_0, PWM_SEND_0, 0);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        ESP_LOGI(TAG, "Offset for sensor %d = %d", i, (int)offsets[i]);
        pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, rcv_channels[i], rcv_gpios[i], offsets[i]);

        ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[i], LEDC_DUTY, offsets[i]);
        ledc_update_duty(LEDC_MODE, rcv_channels[i]);
        ESP_LOGI(TAG, "Break for sensor %d = %d", i, (int)break_limit[i]);
        ESP_LOGI(TAG, "Sensor %d enabled = %d", i, (int)enabled[i]);
    }
}

void app_main(void)
{

    nvs_setup();
    nvs_read();
    pwm_setup();

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

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

    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        curr_broken = false;
        for (int i = 0; i < 7; i++)
        {
            if (enabled[i])
            {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel[i], &adc_raw[0][i]));
                sensor_break[i] = (adc_raw[0][i] < break_limit[i]);
                led_set(i, sensor_break[i]);
                curr_broken |= sensor_break[i];
            }
        }

        publish_sensor();

        if (highpoint_search)
        {
            // ESP_LOGI(TAG, "offset: %d", highpoint_offset);
            ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1 + highpoint_channel, LEDC_DUTY, highpoint_offset);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1 + highpoint_channel);
            if (adc_raw[0][highpoint_channel] > highpoint_max)
            {
                highpoint_max = adc_raw[0][highpoint_channel];
                highpoint_offset_max = highpoint_offset;
            }
            if (highpoint_offset >= 8092)
            {
                ledc_set_duty_with_hpoint(LEDC_MODE, rcv_channels[highpoint_channel], LEDC_DUTY, highpoint_offset_max);
                ledc_update_duty(LEDC_MODE, rcv_channels[highpoint_channel]);
                ESP_LOGI(TAG, "hsearch done for channel %d, max adc value: %d, offset: %d", highpoint_channel, highpoint_max, highpoint_offset_max);

                if (highpoint_channel == 6)
                {
                    highpoint_search = false;
                    nvs_update_offsets();
                    publish_settings();
                }
                else
                {
                    highpoint_channel++;
                    highpoint_offset = 0;
                    highpoint_max = 0;
                }
            }
            highpoint_offset += 30;
        }
        else
        {
            // Send event if there is a change in state
            if (curr_broken != prev_broken)
            {
                publish_break(curr_broken);
            }
            prev_broken = curr_broken;
        }
    }
}
