#pragma once

#include "cpx.h"

#define CRTP_MAX_DATA_SIZE 30
#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

typedef struct _CRTPPacket
{
  uint8_t size;                         //< Size of data
  union {
    struct {
      union {
        uint8_t header;                 //< Header selecting channel and port
        struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;      //< Selected channel within port
          uint8_t reserved    : 2;
          uint8_t port        : 4;      //< Selected port
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
  };
} __attribute__((packed)) CRTPPacket;

void espTransportReceive(CPXRoutablePacket_t* packet);
void espTransportInit(void*);