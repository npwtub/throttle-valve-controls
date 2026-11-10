#pragma once
#include "mcp2515.h"

static float sentMotorCommand = 0;
static unsigned commandsSent = 0, heartbeatsSent = 0;
namespace CanControl {
class SparkMax {
public:
    SparkMax(MCP2515&, uint8_t) {}
    void handle_received_frame(const can_frame&) {}
    MCP2515::ERROR set_position(float command, int = 0, int = 0, int = 0) {
        sentMotorCommand = command;
        ++commandsSent;
        return MCP2515::ERROR_OK;
    }
};
inline int default_heartbeat() { return 0; }
inline MCP2515::ERROR send_heartbeat(MCP2515&, int) {
    ++heartbeatsSent;
    return MCP2515::ERROR_OK;
}
}
