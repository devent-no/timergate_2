/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
//#define HOST_IP_ADDR "192.168.4.1"
#define HOST_IP_ADDR "192.168.4.1"
#define PORT 3333
#define NUM_SENSORS 7
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
uint8_t led_gpio[NUM_SENSORS] = {39, 40, 41, 42, 35, 36, 37};
uint8_t led_val[NUM_SENSORS] = {0};

uint8_t adc_channel[NUM_SENSORS] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7};

uint16_t break_limit[NUM_SENSORS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000};

bool enabled[NUM_SENSORS] = {true, true, true, true, true, true, true};

uint8_t sensor_break[NUM_SENSORS] = {0};

uint16_t break_time;
bool curr_broken = false;
bool prev_broken = false;

bool connected = false;

char *cmd;
int64_t timestamp;

led_strip_config_t strip_config = {
    .max_leds = 1, // at least one LED on board
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

    strip_config.strip_gpio_num = led_gpio[led_nr];
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    if (value)
    {
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        led_strip_refresh(led_strip);
    }
    else
    {
        led_strip_clear(led_strip);
    }
    led_strip_del(led_strip);

    led_val[led_nr] = value;
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
                }
                if (strncmp(command, "time", 4) == 0)
                {
                    timestamp = strtol(command + 5, NULL, 10);
                    ESP_LOGI(TAG, "Got timestamp: %lld", timestamp);
                    example_espnow_init();
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
                //ESP_LOGI(TAG, "Sending from xQueueBreak: %s", break_cmd);
                int written = send(sock, break_cmd, strlen(break_cmd), 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    connected = false;
                }
                if (break_cmd != NULL){
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

void app_main(void)
{
    pwm_init(LEDC_TIMER_0, PWM_0_FREQUENCY, LEDC_CHANNEL_0, PWM_SEND_0, 0);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_1, PWM_RCV_0, 0);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_2, PWM_RCV_1, 1000);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_3, PWM_RCV_2, 2000);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_4, PWM_RCV_3, 3000);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_5, PWM_RCV_4, 4000);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_6, PWM_RCV_5, 5000);
    pwm_init(LEDC_TIMER_1, PWM_0_FREQUENCY, LEDC_CHANNEL_7, PWM_RCV_6, 6000);

    esp_log_level_set("gpio", ESP_LOG_WARN);

    cmd = malloc(100);

    adc_init();
    wifi_init();
    tcp_setup();
    tcp_connect();
    store_mac();

    xQueue = xQueueCreate(1, sizeof(char *));
    xQueueBreak = xQueueCreate(10, sizeof(char *));
    xTaskCreatePinnedToCore(tcp_client_task, "tcp_client", 4096, (void *)AF_INET, 5, NULL, 1);

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

        // Send event if there is a change in state
        if (curr_broken != prev_broken)
        {
            publish_break(curr_broken);
        }
        prev_broken = curr_broken;
    }
}
