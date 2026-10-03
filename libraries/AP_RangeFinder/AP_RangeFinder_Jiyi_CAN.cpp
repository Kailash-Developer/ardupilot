#include "AP_RangeFinder_Jiyi_CAN.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <string.h>

#define JIYI_DEBUG 1
#if JIYI_DEBUG
# define Debug(fmt, args...) ::printf("[JiyiCAN] " fmt "\n", ##args)
#else
# define Debug(fmt, args...)
#endif

namespace JiyiRadar {
extern const StreamConfig STREAM_CONFIGS[] = {
    { 214, {0xEA, 0x2D, 0x04, 0x00}, ChecksumType::B4_B5_B6, 25, "H30 Down" },
    { 220, {0xD3, 0x3F, 0x04, 0x01}, ChecksumType::SUM_B0_B6, 0,  "R21 Front" },
    { 221, {0xD3, 0x3F, 0x04, 0x02}, ChecksumType::SUM_B0_B6, 4,  "R21 Rear" }
};
extern const uint8_t NUM_STREAM_CONFIGS = ARRAY_SIZE(STREAM_CONFIGS);
}

AP_RangeFinder_Jiyi_CAN::AP_RangeFinder_Jiyi_CAN(RangeFinder::RangeFinder_State &_state,
                                                 AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::Jiyi, "jiyi"),
    _last_valid_m(0.0f),
    _candidate_m(0.0f),
    _candidate_count(0),
    _pending_distance_m(0.0f),
    _has_pending_distance(false),
    _was_idle(false),
    _warned_orient(false),
    _config(nullptr)
{
    // Do not set max_distance_default here because H30 is 27m but R21 is much shorter.
    // Let the user configure it via RNGFNDx_MAX.
    _params.min_distance.set_default(0.2f);
    Debug("Driver Initialized!");
}

bool AP_RangeFinder_Jiyi_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    const uint32_t can_id = frame.id & AP_HAL::CANFrame::MaskStdID;
    
    // Detailed CAN tracing (only print for our target ID to avoid console flooding from other CAN devices)
    if (can_id == (uint32_t)receive_id.get()) {
        Debug("--- Rx CAN ID: %u | DLC: %u | Data: %02X %02X %02X %02X %02X %02X %02X %02X ---", 
              (unsigned)can_id, frame.dlc, 
              frame.data[0], frame.data[1], frame.data[2], frame.data[3], 
              frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    }

    if (frame.isExtended() || frame.isRemoteTransmissionRequest() || frame.dlc != 8) {
        return false;
    }

    int32_t expected_id = receive_id.get();
    if (expected_id <= 0 || expected_id > 0x7FF) {
        return false;
    }

    if (_config == nullptr || _config->recv_id != expected_id) {
        _config = nullptr;
        for (uint8_t i = 0; i < JiyiRadar::NUM_STREAM_CONFIGS; i++) {
            if (JiyiRadar::STREAM_CONFIGS[i].recv_id == expected_id) {
                _config = &JiyiRadar::STREAM_CONFIGS[i];
                break;
            }
        }
        if (_config == nullptr) {
            return false;
        }
        Debug("Config matched: %s", _config->name);
        _warned_orient = false;
    }

    if (can_id != _config->recv_id) {
        return false;
    }

    if (memcmp(frame.data, _config->prefix, 4) != 0) {
        Debug("Rejected: Payload prefix mismatch");
        return false;
    }

    if (frame.data[6] < snr_min.get()) {
        Debug("Rejected: SNR %u < Min %u", frame.data[6], (unsigned)snr_min.get());
        return false;
    }

    if (!_warned_orient && _config->expected_orient != (uint8_t)orientation()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Jiyi_CAN: ORIENT mismatch (expected %u)", _config->expected_orient);
        _warned_orient = true;
    }

    if (_config->recv_id == 214 && frame.data[4] == 0 && frame.data[5] == 0 && frame.data[6] == 0) {
        Debug("H30: Dropped zero-range frame (prevented inversion)");
        return false; 
    }

    uint8_t calc_csum = 0;
    if (_config->csum_type == JiyiRadar::ChecksumType::B4_B5_B6) {
        calc_csum = static_cast<uint8_t>((frame.data[4] + frame.data[5] + frame.data[6]) & 0xFF);
    } else {
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 7; i++) {
            sum += frame.data[i];
        }
        calc_csum = static_cast<uint8_t>(sum & 0xFF);
    }

    if (calc_csum != frame.data[7]) {
        Debug("Rejected: Checksum Fail (Calc:%02X, Rx:%02X)", calc_csum, frame.data[7]);
        return false;
    }

    const uint16_t raw_range = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
    
    if (raw_range == 0) {
        if (_config->recv_id == 220 || _config->recv_id == 221) {
            Debug("R21: Idle state detected, pushing OutOfRangeHigh");
            _was_idle = true;
            _last_valid_m = 0.0f; // Reset jump filter
            _candidate_count = 0;
            _pending_distance_m = MAX(max_distance(), 0.1f) + 1.0f;
            _has_pending_distance = true;
            return true;
        } else {
            Debug("H30: Dropped raw_range == 0");
            return false;
        }
    }

    float distance_m = raw_range * 0.01f;
    Debug("Parsed Valid Range: %u cm (%.2f m)", raw_range, distance_m);

    float accepted_dist = distance_m;
    bool accepted = false;

    if (_last_valid_m > 0.0f) {
        float diff = fabsf(distance_m - _last_valid_m);
        if (diff > JiyiRadar::JUMP_THRESHOLD_M) {
            if (_candidate_count > 0 && fabsf(distance_m - _candidate_m) <= JiyiRadar::JUMP_THRESHOLD_M) {
                _candidate_count++;
                if (_candidate_count >= 2) {
                    _last_valid_m = distance_m;
                    _candidate_count = 0;
                    accepted_dist = distance_m;
                    accepted = true;
                    Debug("Jump filter: Accepted step change to %.2f m", distance_m);
                }
            } else {
                _candidate_m = distance_m;
                _candidate_count = 1;
                Debug("Jump filter: Spike rejected %.2f m (last: %.2f m)", distance_m, _last_valid_m);
            }
        } else {
            _last_valid_m = distance_m;
            _candidate_count = 0;
            accepted_dist = distance_m;
            accepted = true;
        }
    } else {
        _last_valid_m = distance_m;
        _candidate_count = 0;
        accepted_dist = distance_m;
        accepted = true;
    }

    if (accepted) {
        Debug("Enqueued for frontend: %.2f m", accepted_dist);
        _pending_distance_m = accepted_dist;
        _has_pending_distance = true;
    }

    return true;
}

void AP_RangeFinder_Jiyi_CAN::update(void)
{
    bool got_reading = false;
    float current_distance = 0.0f;
    bool idle_transition = false;

    {
        WITH_SEMAPHORE(_sem);
        if (_has_pending_distance) {
            current_distance = _pending_distance_m;
            got_reading = true;
            _has_pending_distance = false;
            
            if (_was_idle && current_distance <= max_distance()) {
                idle_transition = true;
                _was_idle = false;
            }
        }
    }

    if (idle_transition) {
        float dummy;
        get_reading(dummy);
        Debug("update(): Flushed accumulator for idle transition");
    }

    if (got_reading) {
        accumulate_distance_m(current_distance);
        Debug("update(): Accumulated %.2f m", current_distance);
    }

    if (get_reading(state.distance_m)) {
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        Debug("update(): Published %.2f m to ArduPilot Frontend", state.distance_m);
    } else if (AP_HAL::millis() - state.last_reading_ms >= read_timeout_ms()) {
        set_status(RangeFinder::Status::NoData);
        Debug("update(): TIMEOUT - NoData");
        // Reset jump filter safely under semaphore
        WITH_SEMAPHORE(_sem);
        _last_valid_m = 0.0f;
        _candidate_count = 0;
    }
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
