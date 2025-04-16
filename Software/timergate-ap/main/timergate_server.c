#include <string.h>
#include <fcntl.h>
#include "esp_http_server.h"
#include "esp_chip_info.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "freertos/ringbuf.h"



#define PORT 3333
#define KEEPALIVE_IDLE 1
#define KEEPALIVE_INTERVAL 1
#define KEEPALIVE_COUNT 3
#define STACK_SIZE 5000

#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#define ESPNOW_SEND_LEN 15

static const char *REST_TAG = "timergate-ap";
#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            ESP_LOGE(REST_TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context
{
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

enum
{
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* Parameters of sending ESPNOW data. */
typedef struct
{
    bool unicast;                       // Send unicast ESPNOW data.
    bool broadcast;                     // Send broadcast ESPNOW data.
    uint8_t state;                      // Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                     // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                     // Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                     // Delay between sending two ESPNOW data, unit: ms.
    int len;                            // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                    // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} example_espnow_send_param_t;

/* User defined field of ESPNOW data in this example. */
typedef struct
{
    uint8_t type;       // Broadcast or unicast ESPNOW data.
    uint8_t state;      // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;   // Sequence number of ESPNOW data.
    uint16_t crc;       // CRC16 value of ESPNOW data.
    uint32_t magic;     // Magic number which is used to determine which device to send unicast ESPNOW data.
    char payload[0]; // Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq = 0;

example_espnow_send_param_t *send_param;

httpd_handle_t server = NULL;
int handshake_done = 0;
int fd;
SemaphoreHandle_t ws_send_semaphore;
QueueHandle_t xQueue[2];

int pole_ids[2];
char macs[2][20];
int sync_timestamp;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html"))
    {
        type = "text/html";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".js"))
    {
        type = "application/javascript";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".css"))
    {
        type = "text/css";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".png"))
    {
        type = "image/png";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".ico"))
    {
        type = "image/x-icon";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".svg"))
    {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* Send HTTP response with the contents of the requested file */
static esp_err_t rest_common_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/')
    {
        strlcat(filepath, "/index.html", sizeof(filepath));
    }
    else
    {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1)
    {
        ESP_LOGE(REST_TAG, "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do
    {
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1)
        {
            ESP_LOGE(REST_TAG, "Failed to read file : %s", filepath);
        }
        else if (read_bytes > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
            {
                close(fd);
                ESP_LOGE(REST_TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    ESP_LOGI(REST_TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

int get_pole_id_by_mac(char *mac)
{
    if (strncmp(macs[0], mac, 17) == 0)
    {
        return 0;
    }
    else if (strncmp(macs[1], mac, 17) == 0)
    {
        return 1;
    }
    return -1;
}

void save_pole_id(int pole_id, char *command)
{
    char *mac = command + 12;
    if (command[8] == 'M' && get_pole_id_by_mac(mac) < 0)
    {
        strncpy(macs[pole_id], mac, 17);
        macs[pole_id][18] = 0;
        ESP_LOGI(REST_TAG, "Pole %d has mac: '%s'", pole_id, macs[pole_id]);
    }
}

static esp_err_t pole_break_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    int adc = cJSON_GetObjectItem(root, "adc")->valueint;
    int breakVal = cJSON_GetObjectItem(root, "break")->valueint;
    char *mac = cJSON_GetObjectItem(root, "mac")->valuestring;

    ESP_LOGI(REST_TAG, "Pole break: adc = %d, break = %d, mac = %s", adc, breakVal, mac);

    char *cal_command = malloc(30);
    sprintf(cal_command, "break: %d %d\n", adc, breakVal);

    int pole_id = get_pole_id_by_mac(mac);
    if (pole_id == 0 || pole_id == 1)
    {
        xQueueSendToBack(xQueue[pole_id], &cal_command, portMAX_DELAY);
    }
    else
    {
        ESP_LOGI(REST_TAG, "Not valid mac: '%s'", mac);
    }
    httpd_resp_sendstr(req, "{\"response\": \"OK\"}");

    cJSON_Delete(root);

    return ESP_OK;
}

static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(REST_TAG, "Send cb arg error");
        return;
    }
    ESP_LOGI(REST_TAG, "Send cb OK");
}

void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = EXAMPLE_ESPNOW_DATA_BROADCAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    sprintf(buf->payload, "SYNC");
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
    ESP_LOGI(REST_TAG, "Payload filled with len %d", send_param->len - sizeof(example_espnow_data_t));
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    esp_now_deinit();
}

static esp_err_t example_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
    // ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(REST_TAG, "Malloc peer information fail");
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(REST_TAG, "Malloc send parameter fail");
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = 100;
    send_param->delay = 1000;
    send_param->len = ESPNOW_SEND_LEN;
    send_param->buffer = malloc(ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(REST_TAG, "Malloc send buffer fail");
        free(send_param);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    // Wait one second before sending sync signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(REST_TAG, "Send error");
    }
    else{
        ESP_LOGI(REST_TAG, "ESPNOW data sent");
    }
    example_espnow_deinit(send_param);

    // Sync local time
    struct timeval tv;
    tv.tv_sec = sync_timestamp;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    return ESP_OK;
}

static esp_err_t time_sync_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);

    // We send the ESPnow sync one second after the local time.    
    int sync_timestamp = cJSON_GetObjectItem(root, "timestamp")->valueint + 1;

    // Sync connected poles. We need two because the are freed elsewhere.
    char *sync_command0 = malloc(30);
    char *sync_command1 = malloc(30);
    sprintf(sync_command0, "time: %d\n", sync_timestamp);
    sprintf(sync_command1, "time: %d\n", sync_timestamp);
    if (pole_ids[0])
    {
        xQueueSendToBack(xQueue[0], &sync_command0, portMAX_DELAY);
    }
    if (pole_ids[1])
    {
        xQueueSendToBack(xQueue[1], &sync_command1, portMAX_DELAY);
    }

    ESP_LOGI(REST_TAG, "Time stamp set to: '%d'", sync_timestamp);

    example_espnow_init();

    httpd_resp_sendstr(req, "{\"response\": \"OK\"}");
    return ESP_OK;
}

static esp_err_t hsearch_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);

    int channel = cJSON_GetObjectItem(root, "channel")->valueint;

    // Sync connected poles. We need two because the are freed elsewhere.
    char *hsearch_command0 = malloc(30);
    char *hsearch_command1 = malloc(30);
    sprintf(hsearch_command0, "hsearch: %d\n", channel);
    sprintf(hsearch_command1, "hsearch: %d\n", channel);
    if (pole_ids[0])
    {
        xQueueSendToBack(xQueue[0], &hsearch_command0, portMAX_DELAY);
    }
    if (pole_ids[1])
    {
        xQueueSendToBack(xQueue[1], &hsearch_command1, portMAX_DELAY);
    }

    ESP_LOGI(REST_TAG, "Hsync command sent");

    httpd_resp_sendstr(req, "{\"response\": \"OK\"}");
    return ESP_OK;
}


static esp_err_t enabled_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);

    int sensor_nr = cJSON_GetObjectItem(root, "sensor_nr")->valueint;
    int enabled = cJSON_GetObjectItem(root, "enabled")->valueint;
    char *mac = cJSON_GetObjectItem(root, "mac")->valuestring;

    ESP_LOGI(REST_TAG, "Pole sensor enabled: sensor_nr = %d, enabled = %d, mac = %s", sensor_nr, enabled, mac);

    char *en_cmd = malloc(30);
    sprintf(en_cmd, "enabled: %d %d\n", sensor_nr, enabled);

    int pole_id = get_pole_id_by_mac(mac);
    if (pole_id == 0 || pole_id == 1)
    {
        xQueueSendToBack(xQueue[pole_id], &en_cmd, portMAX_DELAY);
    }
    else
    {
        ESP_LOGI(REST_TAG, "Not valid mac: '%s'", mac);
    }
    httpd_resp_sendstr(req, "{\"response\": \"OK\"}");

    cJSON_Delete(root);

    return ESP_OK;
}


static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(REST_TAG, "Handshake done, the new connection was opened");
        handshake_done = 1;
        fd = httpd_req_to_sockfd(req);
        
        // Sende en enkel Welcome-melding for å teste WebSocket
        char *welcome_msg = "{\"type\": \"welcome\", \"message\": \"WebSocket connected\"}";
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)welcome_msg;
        ws_pkt.len = strlen(welcome_msg);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        
        httpd_ws_send_frame(req, &ws_pkt);
    }
    
    // Håndter også innkommende meldinger (om nødvendig)
    if (req->method == HTTP_POST) {
        httpd_ws_frame_t ws_pkt;
        uint8_t *buf = NULL;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        
        if (ret != ESP_OK) {
            ESP_LOGE(REST_TAG, "httpd_ws_recv_frame failed with %d", ret);
            return ret;
        }
        
        if (ws_pkt.len) {
            buf = calloc(1, ws_pkt.len + 1);
            if (buf == NULL) {
                ESP_LOGE(REST_TAG, "Failed to calloc memory for buf");
                return ESP_ERR_NO_MEM;
            }
            ws_pkt.payload = buf;
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret != ESP_OK) {
                ESP_LOGE(REST_TAG, "httpd_ws_recv_frame failed with %d", ret);
                free(buf);
                return ret;
            }
            
            // Her kan du behandle mottatte meldinger om nødvendig
            ESP_LOGI(REST_TAG, "Received packet with message: %s", ws_pkt.payload);
        }
        free(buf);
    }
    
    return ESP_OK;
}




static void ws_send_message(char *msg)
{
    if (!handshake_done) {
        return;
    }
    
    httpd_handle_t server_handle = server; // Du må ha en global server-handle
    
    if (xSemaphoreTake(ws_send_semaphore, portMAX_DELAY) == pdTRUE) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)msg;
        ws_pkt.len = strlen(msg);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        
        esp_err_t ret = httpd_ws_send_frame_async(server_handle, fd, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(REST_TAG, "httpd_ws_send_frame_async failed with %d", ret);
        }
        
        xSemaphoreGive(ws_send_semaphore);
    } else {
        ESP_LOGI(REST_TAG, "Unable to take semaphore");
    }
}




static int get_pole_id()
{
    if (pole_ids[0] == 0)
    {
        pole_ids[0] = 1;
        return 0;
    }
    if (pole_ids[1] == 0)
    {
        pole_ids[1] = 1;
        return 1;
    }
    return 2;
}
void tcp_client_handler(void *arg)
{
    int sock = *(int *)arg;
    char rx_buffer[100];
    char command[140];
    int cmd_index = 0;
    int pole_id = get_pole_id();

    if (pole_id > 1)
    {
        ESP_LOGE(REST_TAG, "Pole ID: %d not valid, returning", pole_id);
        return;
    }

    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT);
        if (len < 0)
        {
            if (errno == 128)
            {
                ESP_LOGE(REST_TAG, "Pole has disconnected: errno %d", errno);
                pole_ids[pole_id] = 0;
                break;
            }
        }
        else if (len == 0)
        {
            ESP_LOGW(REST_TAG, "Connection closed");
            break;
        }
        else
        {
            rx_buffer[len] = 0;
            for (int i = 0; i < len; i++)
            {
                command[cmd_index] = rx_buffer[i];
                if (rx_buffer[i] == '\n')
                {
                    command[cmd_index] = 0;
                    save_pole_id(pole_id, command);
                    ws_send_message(command);
                    cmd_index = 0;
                }
                else
                {
                    cmd_index++;
                }
            }
        }
        char *cmd;
        if (xQueueReceive(xQueue[pole_id], &cmd, 0))
        {
            int to_write = strlen(cmd);
            len = to_write;
            while (to_write > 0)
            {
                int written = send(sock, cmd + (len - to_write), to_write, 0);
                if (written < 0)
                {
                    ESP_LOGE(REST_TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                to_write -= written;
            }
            free(cmd);
        }
    }
    close(sock);
    vTaskDelete(NULL);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(REST_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(REST_TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(REST_TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(REST_TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(REST_TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 2);
    if (err != 0)
    {
        ESP_LOGE(REST_TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1)
    {

        ESP_LOGI(REST_TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(REST_TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(REST_TAG, "Socket accepted ip address: %s", addr_str);

        // Create a new task to handle the client
        xTaskCreatePinnedToCore(tcp_client_handler, "tcp_client_handler", 4096, &sock, 5, NULL, 1);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

esp_err_t start_rest_server(const char *base_path)
{
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "No memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    xQueue[0] = xQueueCreate(10, sizeof(char *));
    xQueue[1] = xQueueCreate(10, sizeof(char *));

    ws_send_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(ws_send_semaphore);

    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 5, NULL);

    ESP_LOGI(REST_TAG, "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err_start);

    httpd_uri_t pole_break_post_uri = {
        .uri = "/api/v1/pole/break",
        .method = HTTP_POST,
        .handler = pole_break_post_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &pole_break_post_uri);

    httpd_uri_t time_sync_post_uri = {
        .uri = "/api/v1/time/sync",
        .method = HTTP_POST,
        .handler = time_sync_post_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &time_sync_post_uri);

    httpd_uri_t hsearch_post_uri = {
        .uri = "/api/v1/time/hsearch",
        .method = HTTP_POST,
        .handler = hsearch_post_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &hsearch_post_uri);

    httpd_uri_t pole_enabled_post_uri = {
        .uri = "/api/v1/pole/enabled",
        .method = HTTP_POST,
        .handler = enabled_post_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &pole_enabled_post_uri);

httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL
};
httpd_register_uri_handler(server, &ws);


    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = rest_common_get_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &common_get_uri);

    return ESP_OK;
err_start:
    free(rest_context);
err:
    return ESP_FAIL;
}
