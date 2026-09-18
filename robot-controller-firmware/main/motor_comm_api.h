#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define AXIS_NUM 6
#define PROTOCOL_VERSION 1

// --- Control timing ---------------------------------------------------
// The control loop runs on its own clock at CONTROL_PERIOD_MS and never
// waits on the network. These two thresholds only decide how it treats
// the setpoint it already holds.
#define CONTROL_PERIOD_MS   10   // 100 Hz, free-running
#define SETPOINT_STALE_MS   100  // link degraded: hold last target, keep controlling
#define SETPOINT_TIMEOUT_MS 500  // link lost: ramp to a stop, then disarm
#define STOP_RAMP_MS        200  // time to bring a commanded speed to zero

// The ESP32 is the client: it dials out to the server, so the server
// address lives here and the server never needs to know ours in advance.
#define MOTOR_LINK_SERVER_IP "192.168.0.10"
#define MOTOR_LINK_TCP_PORT  8001
#define MOTOR_LINK_UDP_PORT  8002

// --- Message types ----------------------------------------------------
// 0x1x travels over TCP (session + parameters), 0x2x over UDP (streaming).
#define MSG_HELLO        0x10u  // client -> server, on connect
#define MSG_HELLO_ACK    0x11u  // server -> client
#define MSG_SET_PARAMS   0x12u  // server -> client
#define MSG_PARAMS_ACK   0x13u  // client -> server
#define MSG_ACTIVATE_REQ 0x14u  // server -> client
#define MSG_ACTIVATE_ACK 0x15u  // client -> server
#define MSG_SETPOINTS    0x20u  // server -> client, periodic
#define MSG_FEEDBACK     0x21u  // client -> server, periodic

// --- Header -----------------------------------------------------------
// Same 8-byte header on both transports. On TCP it also acts as the frame
// length prefix, so the stream can always be parsed unambiguously.
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  version;
    uint16_t length;  // bytes in the data section that follows
    uint32_t seq;     // per-sender counter, useful for spotting drops
} MsgHeader;

// --- Data sections ----------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t axis_count;
} HelloData;

typedef struct __attribute__((packed)) {
    float   kp;
    float   ki;
    float   kd;
    int16_t output_min;
    int16_t output_max;
} ParamsData;

typedef struct __attribute__((packed)) {
    uint8_t ok;
} AckData;

typedef struct __attribute__((packed)) {
    uint8_t enable;  // 1 = activate, 0 = deactivate
} ActivateReqData;

typedef struct __attribute__((packed)) {
    uint8_t accepted;  // 1 = request honoured
    uint8_t active;    // resulting state
} ActivateAckData;

typedef struct __attribute__((packed)) {
    int32_t positions[AXIS_NUM];
    int16_t speed_limits[AXIS_NUM];  // reserved, not applied yet
} SetpointsData;

typedef struct __attribute__((packed)) {
    int32_t positions[AXIS_NUM];
} FeedbackData;

// --- UDP frames: header + data + checksum ------------------------------
// checksum is reserved: always written as 0, never verified yet.
typedef struct __attribute__((packed)) {
    MsgHeader     header;
    SetpointsData data;
    uint16_t      checksum;
} SetpointsMessage;

typedef struct __attribute__((packed)) {
    MsgHeader    header;
    FeedbackData data;
    uint16_t     checksum;
} FeedbackMessage;

_Static_assert(sizeof(MsgHeader) == 8, "MsgHeader must be 8 bytes");
_Static_assert(sizeof(ParamsData) == 16, "ParamsData must be 16 bytes");
_Static_assert(sizeof(SetpointsMessage) == 46, "SetpointsMessage must be 46 bytes");
_Static_assert(sizeof(FeedbackMessage) == 34, "FeedbackMessage must be 34 bytes");

// Length-1 mailboxes, written with xQueueOverwrite(): newest wins, never blocks.
extern QueueHandle_t xSetpointsQueue;  // UDP task     -> control loop
extern QueueHandle_t xFeedbackQueue;   // control loop -> UDP task