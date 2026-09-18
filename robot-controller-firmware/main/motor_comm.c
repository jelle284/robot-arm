#include <string.h>
#include <stdbool.h>

#include "motor_comm.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

static const char *TAG = "motor_comm";

#define TCP_RETRY_MS        2000
#define FEEDBACK_PERIOD_MS  20   // 50 Hz
#define UDP_RECV_TIMEOUT_MS 20
#define MAX_DATA_LEN        64

QueueHandle_t xSetpointsQueue = NULL;
QueueHandle_t xFeedbackQueue = NULL;

// Shared state, guarded by s_mutex.
static SemaphoreHandle_t s_mutex = NULL;
static bool s_active = false;
static ParamsData s_params = {
    .kp = 1.0f, .ki = 0.2f, .kd = 0.0f,
    .output_min = -5000, .output_max = 5000,
};

static int s_tcp_sock = -1;
// Timestamp of the last "the link is alive" event: either activation
// itself, or the most recent valid setpoint packet. Starting the clock
// at activation (not at 0/"never") gives a full SETPOINT_TIMEOUT_MS
// grace period for the first packet to arrive before anything trips.
static int64_t s_last_setpoint_ms = 0;

static uint32_t s_tcp_seq = 0;
static uint32_t s_udp_seq = 0;

static int64_t now_ms(void);
static bool tcp_send(int sock, uint8_t type, const void *data, uint16_t len);

// ---------------------------------------------------------------------
// Shared state accessors
// ---------------------------------------------------------------------
static void set_active(bool active)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_active != active) {
        ESP_LOGI(TAG, "%s", active ? "ACTIVE" : "INACTIVE");
    }
    s_active = active;
    s_last_setpoint_ms = now_ms();  // (re)start the grace window
    xSemaphoreGive(s_mutex);
}

int64_t motor_comm_setpoint_age_ms(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int64_t last = s_last_setpoint_ms;
    xSemaphoreGive(s_mutex);
    return now_ms() - last;
}

void motor_comm_deactivate(const char *reason)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool was_active = s_active;
    int sock = s_tcp_sock;
    s_active = false;
    xSemaphoreGive(s_mutex);

    if (!was_active) {
        return;
    }
    ESP_LOGW(TAG, "deactivated locally: %s", reason ? reason : "unspecified");

    // Tell the server, so the HMI does not keep showing us as active.
    if (sock >= 0) {
        ActivateAckData ack = { .accepted = 1, .active = 0 };
        tcp_send(sock, MSG_ACTIVATE_ACK, &ack, sizeof(ack));
    }
}

bool motor_comm_is_active(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool active = s_active;
    xSemaphoreGive(s_mutex);
    return active;
}

void motor_comm_get_params(ParamsData *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_params;
    xSemaphoreGive(s_mutex);
}

static void set_params(const ParamsData *params)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_params = *params;
    xSemaphoreGive(s_mutex);
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// ---------------------------------------------------------------------
// TCP session task (we are the client)
// ---------------------------------------------------------------------
static bool send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len > 0) {
        int sent = send(sock, p, len, 0);
        if (sent <= 0) {
            return false;
        }
        p += sent;
        len -= sent;
    }
    return true;
}

static bool recv_all(int sock, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len > 0) {
        int got = recv(sock, p, len, 0);
        if (got <= 0) {
            return false;  // peer closed or error
        }
        p += got;
        len -= got;
    }
    return true;
}

static bool tcp_send(int sock, uint8_t type, const void *data, uint16_t len)
{
    uint8_t frame[sizeof(MsgHeader) + MAX_DATA_LEN];
    if (len > MAX_DATA_LEN) {
        return false;
    }
    MsgHeader header = {
        .type = type,
        .version = PROTOCOL_VERSION,
        .length = len,
        .seq = s_tcp_seq++,
    };
    memcpy(frame, &header, sizeof(header));
    if (len > 0) {
        memcpy(frame + sizeof(header), data, len);
    }
    return send_all(sock, frame, sizeof(header) + len);
}

// Handles one framed message. Returns false to drop the connection.
static bool handle_tcp_message(int sock, const MsgHeader *header, const uint8_t *data)
{
    switch (header->type) {
    case MSG_HELLO_ACK:
        ESP_LOGI(TAG, "server acknowledged session");
        return true;

    case MSG_SET_PARAMS: {
        if (header->length != sizeof(ParamsData)) {
            ESP_LOGW(TAG, "bad SET_PARAMS length %u", header->length);
            return true;
        }
        ParamsData params;
        memcpy(&params, data, sizeof(params));
        set_params(&params);
        ESP_LOGI(TAG, "params: kp=%.3f ki=%.3f kd=%.3f limits=[%d,%d]",
                 params.kp, params.ki, params.kd,
                 params.output_min, params.output_max);
        AckData ack = { .ok = 1 };
        return tcp_send(sock, MSG_PARAMS_ACK, &ack, sizeof(ack));
    }

    case MSG_ACTIVATE_REQ: {
        if (header->length != sizeof(ActivateReqData)) {
            return true;
        }
        ActivateReqData req;
        memcpy(&req, data, sizeof(req));
        set_active(req.enable != 0);
        ActivateAckData ack = { .accepted = 1, .active = req.enable != 0 };
        return tcp_send(sock, MSG_ACTIVATE_ACK, &ack, sizeof(ack));
    }

    default:
        ESP_LOGW(TAG, "ignoring TCP message type 0x%02x", header->type);
        return true;
    }
}

static void tcp_task(void *pvParameters)
{
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MOTOR_LINK_TCP_PORT),
    };
    inet_pton(AF_INET, MOTOR_LINK_SERVER_IP, &server_addr.sin_addr);

    for (;;) {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_MS));
            continue;
        }

        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
            ESP_LOGW(TAG, "connect to %s:%d failed: errno %d",
                     MOTOR_LINK_SERVER_IP, MOTOR_LINK_TCP_PORT, errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_MS));
            continue;
        }

        ESP_LOGI(TAG, "connected to server %s:%d",
                 MOTOR_LINK_SERVER_IP, MOTOR_LINK_TCP_PORT);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_tcp_sock = sock;
        xSemaphoreGive(s_mutex);

        HelloData hello = { .axis_count = AXIS_NUM };
        if (tcp_send(sock, MSG_HELLO, &hello, sizeof(hello))) {
            // Blocking read loop: the session lives until the server hangs up.
            for (;;) {
                MsgHeader header;
                uint8_t data[MAX_DATA_LEN];

                if (!recv_all(sock, &header, sizeof(header))) {
                    break;
                }
                if (header.version != PROTOCOL_VERSION || header.length > MAX_DATA_LEN) {
                    ESP_LOGE(TAG, "bad frame (v%u len %u), closing",
                             header.version, header.length);
                    break;
                }
                if (header.length > 0 && !recv_all(sock, data, header.length)) {
                    break;
                }
                if (!handle_tcp_message(sock, &header, data)) {
                    break;
                }
            }
        }

        // Losing the session always disarms the motors.
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_tcp_sock = -1;
        xSemaphoreGive(s_mutex);
        set_active(false);
        ESP_LOGW(TAG, "session closed");
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(TCP_RETRY_MS));
    }
}

// ---------------------------------------------------------------------
// UDP streaming task
// ---------------------------------------------------------------------
static void udp_task(void *pvParameters)
{
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MOTOR_LINK_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MOTOR_LINK_UDP_PORT),
    };
    inet_pton(AF_INET, MOTOR_LINK_SERVER_IP, &server_addr.sin_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket() failed: errno %d", errno);
        vTaskDelete(NULL);
    }

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = UDP_RECV_TIMEOUT_MS * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind() failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "UDP ready on port %d", MOTOR_LINK_UDP_PORT);

    int64_t last_send = 0;

    for (;;) {
        SetpointsMessage setpoints;
        int n = recvfrom(sock, &setpoints, sizeof(setpoints), 0, NULL, NULL);

        if (n == (int)sizeof(setpoints) &&
            setpoints.header.type == MSG_SETPOINTS &&
            setpoints.header.version == PROTOCOL_VERSION) {
            if (motor_comm_is_active()) {
                xQueueOverwrite(xSetpointsQueue, &setpoints);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                s_last_setpoint_ms = now_ms();
                xSemaphoreGive(s_mutex);
            }
        }

        // No watchdog here. Judging the link is the control loop's job:
        // it runs on a fixed clock, whereas this task only wakes when a
        // packet arrives or the socket times out.
        if (!motor_comm_is_active()) {
            continue;
        }

        if (now_ms() - last_send >= FEEDBACK_PERIOD_MS) {
            FeedbackMessage feedback;
            if (xQueueReceive(xFeedbackQueue, &feedback, 0) == pdTRUE) {
                feedback.header.type = MSG_FEEDBACK;
                feedback.header.version = PROTOCOL_VERSION;
                feedback.header.length = sizeof(FeedbackData);
                feedback.header.seq = s_udp_seq++;
                feedback.checksum = 0;  // reserved

                if (sendto(sock, &feedback, sizeof(feedback), 0,
                           (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                    ESP_LOGW(TAG, "sendto() failed: errno %d", errno);
                }
                last_send = now_ms();
            }
        }
    }
}

// ---------------------------------------------------------------------
bool motor_comm_init(void)
{
    s_tcp_sock = -1;
    s_mutex = xSemaphoreCreateMutex();
    xSetpointsQueue = xQueueCreate(1, sizeof(SetpointsMessage));
    xFeedbackQueue = xQueueCreate(1, sizeof(FeedbackMessage));

    if (s_mutex == NULL || xSetpointsQueue == NULL || xFeedbackQueue == NULL) {
        ESP_LOGE(TAG, "failed to allocate queues/mutex");
        return false;
    }

    if (xTaskCreate(tcp_task, "motor_tcp", 4096, NULL, 5, NULL) != pdPASS ||
        xTaskCreate(udp_task, "motor_udp", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create comm tasks");
        return false;
    }

    return true;
}