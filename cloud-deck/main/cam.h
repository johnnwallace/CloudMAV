#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

#define CAM_WIFI_TRANSPORT_MTU 1022

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[CAM_WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) WifiTransportPayload_t;

typedef struct {
    uint16_t payloadLength;
    union {
        WifiTransportPayload_t routablePayload;
        uint8_t payload[CAM_WIFI_TRANSPORT_MTU];
    };
} __attribute__((packed)) WifiTransportPacket_t;

void cam_init();