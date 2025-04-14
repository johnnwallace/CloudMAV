#pragma once

// The UART transport module represents the transport link between the router and the STM on the Crazyflie.

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

#define UART_TRANSPORT_MTU 100

#if UART_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "UART MTU bigger than defined by CPX"
#endif

void uart_transport_init();

// Interface used by the router
void uart_transport_send(const CPXRoutablePacket_t* packet);
void uart_transport_receive(CPXRoutablePacket_t* packet);
