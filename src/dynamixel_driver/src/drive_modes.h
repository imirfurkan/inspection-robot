/*
 * drive_modes.h — Data-driven drive mode definitions
 * ===================================================
 * Each drive mode is a struct describing what every motor does,
 * with separate forward/reverse profiles and per-motor K gains.
 *
 * To add a new mode:
 *   1. Add an entry in drive_modes.cpp inside buildDefaultModes()
 *   2. That's it. No touching the driver node.
 */

#pragma once

#include <cstdint>
#include <string>
#include <map>
#include <vector>

// ── What a single motor does inside a mode ────────────────────
enum class ControlType : uint8_t {
    VELOCITY,   // motor in velocity mode, scaled by K * joystick
    CURRENT,    // motor in current mode,  scaled by K * joystick
    HOLD        // motor in velocity mode with goal = 0 (powered brake)
};

// Command descriptor for one motor in one direction of travel.
struct MotorCommand {
    ControlType type = ControlType::HOLD;
    float       k    = 0.0f;   // gain multiplier [0.0 – 1.0] applied to max
};

// ── A complete drive mode ─────────────────────────────────────
// forward_profile is used when linear.x >= 0
// reverse_profile is used when linear.x <  0
//
// Each profile maps motor_id → MotorCommand.
// If a motor ID is missing from a profile, it defaults to HOLD.
struct DriveModeDef {
    std::string name;
    std::map<uint8_t, MotorCommand> forward_profile;
    std::map<uint8_t, MotorCommand> reverse_profile;
};

// ── Motor ID config passed into the mode builder ──────────────
// This avoids hard-coding motor IDs inside the modes file.
struct MotorLayout {
    uint8_t front_left;
    uint8_t front_right;
    uint8_t rear_left;
    uint8_t rear_right;
};

// ── Public API ────────────────────────────────────────────────
// Returns the full table of available modes keyed by mode name.
// Caller provides the motor layout so modes aren't coupled to IDs.
std::map<std::string, DriveModeDef> buildDefaultModes(const MotorLayout& layout);

// Utility: does this profile require current mode for a given motor?
// Used by the driver to decide when to do the torque-off/switch/torque-on dance.
inline bool needsCurrentMode(const MotorCommand& cmd) {
    return cmd.type == ControlType::CURRENT;
}

// Utility: get the MotorCommand for a given ID from a profile,
// defaulting to HOLD if the motor isn't listed.
inline MotorCommand getCommand(const std::map<uint8_t, MotorCommand>& profile, uint8_t id) {
    auto it = profile.find(id);
    if (it != profile.end()) return it->second;
    return {ControlType::HOLD, 0.0f};
}

// Utility: which Dynamixel operating mode does this control type need?
// VELOCITY and HOLD both use velocity mode; CURRENT uses current mode.
inline uint8_t requiredOperatingMode(ControlType type) {
    return (type == ControlType::CURRENT) ? 0 : 1;  // 0 = current, 1 = velocity
}