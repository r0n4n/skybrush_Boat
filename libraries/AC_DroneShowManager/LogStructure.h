#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_DRONE_SHOW \
    LOG_DRONE_SHOW_MSG

// @LoggerMessage: SHOW
// @Description: Drone show mode information
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time on the show clock
// @Field: Stage: Current stage of the show
// @Field: R: Red component of current color in show
// @Field: G: Green component of current color in show
// @Field: B: Blue component of current color in show
// @Field: HDist: Horizontal distance from desired position
// @Field: VDist: Vertical distance from desired position

// drone show mode logging
struct PACKED log_DroneShowStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t show_clock_ms;
    uint8_t stage;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    float h_dist;
    float v_dist;
};

#define LOG_STRUCTURE_FROM_DRONE_SHOW \
    { LOG_DRONE_SHOW_MSG, sizeof(log_DroneShowStatus),                  \
      "SHOW", "QiBBBBff", "TimeUS,ClockMS,Stage,R,G,B,HDist,VDist", "ss----mm", "FC----BB" }
