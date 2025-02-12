# pragma once

#include "cpx.h"

void init_webserver(void*);

// Interface used by the router
void serverTransportSend(const CPXRoutablePacket_t* packet);
void serverTransportReceive(CPXRoutablePacket_t* packet);