#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CPX_VERSION (0)

// This enum is used to identify source and destination for CPX routing information
typedef enum {
  CPX_T_STM32 = 1, // The STM in the Crazyflie
  CPX_T_ESP32 = 2, // The ESP on the AI-deck
  CPX_T_WIFI_HOST = 3,  // A remote computer connected via Wifi
  CPX_T_GAP8 = 4   // The GAP8 on the AI-deck
} CPXTarget_t;

typedef enum {
  CPX_F_SYSTEM = 1,
  CPX_F_CONSOLE = 2,
  CPX_F_CRTP = 3,
  CPX_F_WIFI_CTRL = 4,
  CPX_F_APP = 5,
  CPX_F_TEST = 0x0E,
  CPX_F_BOOTLOADER = 0x0F,
} CPXFunction_t;

typedef struct {
  CPXTarget_t destination;
  CPXTarget_t source;
  bool lastPacket;
  CPXFunction_t function;
  uint8_t version;
} CPXRouting_t;

// This struct contains routing information in a packed format. This struct
// should mainly be used to serialize data when tranfering. Unpacked formats
// should be preferred in application code.
typedef struct {
  CPXTarget_t destination : 3;
  CPXTarget_t source : 3;
  bool lastPacket : 1;
  bool reserved : 1;
  CPXFunction_t function : 6;
  uint8_t version : 2;
} __attribute__((packed)) CPXRoutingPacked_t;

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

// The maximum MTU of any link
#define CPX_MAX_PAYLOAD_SIZE 1022

typedef struct {
  CPXRouting_t route;

  uint16_t dataLength;
  uint8_t data[CPX_MAX_PAYLOAD_SIZE - CPX_ROUTING_PACKED_SIZE];
} CPXRoutablePacket_t;

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route);
void cpxRouteToPacked(const CPXRouting_t* route, CPXRoutingPacked_t* packed);
void cpxPackedToRoute(const CPXRoutingPacked_t* packed, CPXRouting_t* route);