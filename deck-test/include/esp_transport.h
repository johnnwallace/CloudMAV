#pragma once

#include <arduino_freertos.h>
#include <queue.h>

#include "cpx.h"

void espTransportReceive(CPXRoutablePacket_t* packet);
void espTransportInit();