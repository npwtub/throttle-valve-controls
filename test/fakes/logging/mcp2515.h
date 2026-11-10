#pragma once
#include <stdint.h>

constexpr uint32_t CAN_RTR_FLAG = 0x40000000UL;
constexpr uint32_t CAN_EFF_FLAG = 0x80000000UL;
constexpr uint32_t CAN_EFF_MASK = 0x1fffffffUL;
struct can_frame { uint32_t can_id; uint8_t can_dlc; uint8_t data[8]; };
constexpr int CAN_1000KBPS = 1, MCP_8MHZ = 1;
class MCP2515 {
public:
    enum ERROR { ERROR_OK, ERROR_NOMSG };
    MCP2515(uint8_t, unsigned long) {}
    void reset() {}
    void setBitrate(int, int) {}
    void setNormalOneShotMode() {}
    ERROR readMessage(can_frame*) { return ERROR_NOMSG; }
};
