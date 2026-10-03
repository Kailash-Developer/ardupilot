#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

namespace JiyiRadar {

enum class ChecksumType : uint8_t {
    B4_B5_B6,
    SUM_B0_B6
};

struct StreamConfig {
    uint16_t recv_id;
    uint8_t prefix[4];
    ChecksumType csum_type;
    uint8_t expected_orient;
    const char *name;
};

extern const StreamConfig STREAM_CONFIGS[];
extern const uint8_t NUM_STREAM_CONFIGS;

static constexpr float JUMP_THRESHOLD_M = 0.5f;

} // namespace JiyiRadar

class AP_RangeFinder_Jiyi_CAN : public AP_RangeFinder_Backend_CAN {
public:
    int32_t get_receive_id() const { return receive_id.get(); }
    AP_RangeFinder_Jiyi_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // handler for incoming CAN frames
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    // periodic update called by frontend to publish distance
    void update(void) override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // State machine for jump filter
    float _last_valid_m;
    float _candidate_m;
    uint8_t _candidate_count;
    
    // Thread-safe accumulation buffer
    float _pending_distance_m;
    bool _has_pending_distance;
    
    bool _was_idle;
    mutable bool _warned_orient;

    const JiyiRadar::StreamConfig* _config;
};

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
