#pragma once

#include "esp_err.h"
#include "redmond_protocol.h"

esp_err_t zigbee_bridge_start(void);
void zigbee_bridge_update(const r4s_status_t *status, bool available);
bool zigbee_bridge_is_joined(void);
void zigbee_bridge_factory_reset(void);
