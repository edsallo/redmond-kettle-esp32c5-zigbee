#include "redmond_protocol.h"
#include <string.h>

size_t r4s_make_packet(uint8_t counter, uint8_t command,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *out, size_t out_size)
{
    if (!out || out_size < payload_len + 4) return 0;
    out[0] = 0x55;
    out[1] = counter;
    out[2] = command;
    if (payload_len && payload) memcpy(out + 3, payload, payload_len);
    out[payload_len + 3] = 0xaa;
    return payload_len + 4;
}

void r4s_make_mode_payload(bool keep_warm, uint8_t target, uint8_t out[16])
{
    memset(out, 0, 16);
    out[0] = keep_warm ? 1 : 0;
    out[2] = target;
    out[13] = 0x80;
}

bool r4s_decode_status(const uint8_t *payload, size_t length, r4s_status_t *status)
{
    if (!payload || !status || length < 16) return false;
    status->mode = payload[0];
    status->target_temperature = payload[2];
    status->temperature = payload[5];
    /* RK-G200S/G211S state 4 means that the program has completed.  It is a
     * non-zero value, but the kettle is already off.  Mode 3 is the autonomous
     * night-light and must not be exposed as kettle power either. */
    const uint8_t state = payload[8];
    status->is_on = state != 0 && state != 4 && status->mode != 3;
    status->error = payload[9];
    return true;
}
