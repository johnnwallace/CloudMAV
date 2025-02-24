#pragma once

#include "cpx.h"

#define ESP_TRANSPORT_MTU 1022

#if ESP_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "ESP MTU bigger than defined by CPX"
#endif

void espTransportInit(void*);

// Interface used by ESP applications to exchange CPX packets with other part of the system
void espAppSendToRouterBlocking(const CPXRoutablePacket_t* packet);
void espAppReceiveFromRouter(CPXRoutablePacket_t* packet);

// Interface used by the router
void espTransportSend(const CPXRoutablePacket_t* packet);
void espTransportReceive(CPXRoutablePacket_t* packet);