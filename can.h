/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "usbd_gs_can.h"

#define CAN_MODULE_FREQUENCY     144000000  //XMC4100: 80000000


void can_init(uint8_t channel);
bool can_set_bittiming(uint8_t channel, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_enable(uint8_t channel, bool loop_back, bool listen_only, bool one_shot);
void can_disable(uint8_t channel);
bool can_is_enabled(uint8_t channel);

bool can_receive(uint8_t channel, struct gs_host_frame *rx_frame);
bool can_is_rx_pending(uint8_t channel);

bool can_send(uint8_t channel, struct gs_host_frame *frame);

uint32_t can_get_error_status(uint8_t channel);
bool can_parse_error_status(uint32_t err, struct gs_host_frame *frame);
