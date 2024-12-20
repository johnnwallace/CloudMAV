#pragma once

#include "cpx.h"

#define UART_TRANSPORT_MTU 100

void uart_transport_send(const CPXRoutablePacket_t* packet);
void uartTransportInit(void*);