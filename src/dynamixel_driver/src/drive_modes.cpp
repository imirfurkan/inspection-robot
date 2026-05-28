/*
 * drive_modes.cpp — Default mode table
 * =====================================
 * All drive mode definitions live here. To add a new mode, just add
 * another entry to the map inside buildDefaultModes(). Nothing else
 * needs to change — the driver node reads these descriptors generically.
 *
 * Each mode has a forward_profile (linear.x >= 0) and a reverse_profile
 * (linear.x < 0). Each profile maps motor_id → {ControlType, K_gain}.
 *
 * K gain is a multiplier [0.0–1.0] applied to max_velocity or max_current.
 * Use it to give different motors different strengths per mode.
 *
 * ControlType options:
 *   VELOCITY — motor in velocity mode, command = K * max_vel * joystick
 *   CURRENT  — motor in current mode,  command = K * max_cur * joystick
 *   HOLD     — motor in velocity mode with goal = 0 (powered brake)
 *              K is ignored for HOLD, but convention is 0.0.
 */

#include "drive_modes.h"

std::map<std::string, DriveModeDef> buildDefaultModes(const MotorLayout& m)
{
    // Shorthand aliases
    const uint8_t FL = m.front_left;
    const uint8_t FR = m.front_right;
    const uint8_t RL = m.rear_left;
    const uint8_t RR = m.rear_right;

    std::map<std::string, DriveModeDef> modes;

    // ────────────────────────────────────────────────────────────
    // drive_all — All 4 motors velocity, normal driving
    // ────────────────────────────────────────────────────────────
    modes["drive_all"] = {
        .name = "drive_all",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f}},
            {FR, {ControlType::VELOCITY, 1.0f}},
            {RL, {ControlType::VELOCITY, 1.0f}},
            {RR, {ControlType::VELOCITY, 1.0f}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f}},
            {FR, {ControlType::VELOCITY, 1.0f}},
            {RL, {ControlType::VELOCITY, 1.0f}},
            {RR, {ControlType::VELOCITY, 1.0f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_rear_assist — Rear velocity + front current (compliant)
    // ────────────────────────────────────────────────────────────
    modes["drive_rear_assist"] = {
        .name = "drive_rear_assist",
        .forward_profile = {
            {FL, {ControlType::CURRENT,  1.0f}},
            {FR, {ControlType::CURRENT,  1.0f}},
            {RL, {ControlType::VELOCITY, 1.0f}},
            {RR, {ControlType::VELOCITY, 1.0f}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY,  1.0f}},
            {FR, {ControlType::VELOCITY,  1.0f}},
            {RL, {ControlType::CURRENT, 1.0f}},
            {RR, {ControlType::CURRENT, 1.0f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_front_nudge — Rear hold (vel=0) + front current
    // ────────────────────────────────────────────────────────────
    modes["drive_front_nudge"] = {
        .name = "drive_front_nudge",
        .forward_profile = {
            {FL, {ControlType::CURRENT, 1.0f}},
            {FR, {ControlType::CURRENT, 1.0f}},
            {RL, {ControlType::HOLD,    0.0f}},
            {RR, {ControlType::HOLD,    0.0f}},
        },
        .reverse_profile = {
            {FL, {ControlType::CURRENT, 1.0f}},
            {FR, {ControlType::CURRENT, 1.0f}},
            {RL, {ControlType::HOLD,    0.0f}},
            {RR, {ControlType::HOLD,    0.0f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_front_only — Rear hold + front velocity
    // ────────────────────────────────────────────────────────────
    modes["drive_front_only"] = {
        .name = "drive_front_only",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f}},
            {FR, {ControlType::VELOCITY, 1.0f}},
            {RL, {ControlType::HOLD,     0.0f}},
            {RR, {ControlType::HOLD,     0.0f}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f}},
            {FR, {ControlType::VELOCITY, 1.0f}},
            {RL, {ControlType::HOLD,     0.0f}},
            {RR, {ControlType::HOLD,     0.0f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_rear_only — Rear velocity + front hold
    // ────────────────────────────────────────────────────────────
    modes["drive_rear_only"] = {
        .name = "drive_rear_only",
        .forward_profile = {
            {FL, {ControlType::HOLD,     0.0f}},
            {FR, {ControlType::HOLD,     0.0f}},
            {RL, {ControlType::VELOCITY, 1.0f}},
            {RR, {ControlType::VELOCITY, 1.0f}},
        },
        .reverse_profile = {
            {FL, {ControlType::HOLD,     0.0f}},
            {FR, {ControlType::HOLD,     0.0f}},
            {RL, {ControlType::VELOCITY, 1.0f}},
            {RR, {ControlType::VELOCITY, 1.0f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_pivot_left — Rear-left hold, rear-right velocity, front current
    //
    // NOTE: forward/reverse profiles are SYMMETRIC here.
    // This is your original behavior. When you're ready, tweak the
    // reverse_profile to do fronts→velocity, rear-left→current, etc.
    // ────────────────────────────────────────────────────────────
    modes["drive_pivot_left"] = {
        .name = "drive_pivot_left",
        .forward_profile = {
            {FL, {ControlType::VELOCITY,  0.85f}},
            {FR, {ControlType::VELOCITY,  1.0f}},
            {RL, {ControlType::VELOCITY,  0.79f}}, // 0.785
            {RR, {ControlType::VELOCITY, 0.94f}},
        },
        .reverse_profile = {
            // TODO: customize for reverse driving. Example:
            //   fronts → VELOCITY with lower K,
            //   rear-left → CURRENT,
            //   rear-right → HOLD
            // For now, mirrors forward.
            {FL, {ControlType::VELOCITY,  0.85f}},
            {FR, {ControlType::VELOCITY,  1.0f}},
            {RL, {ControlType::VELOCITY,  0.79f}},
            {RR, {ControlType::VELOCITY, 0.94f}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_pivot_right — Rear-right hold, rear-left velocity, front current
    //
    // Your example: forward → fronts current, RL vel, RR hold
    //               reverse → fronts velocity, RL current, RR hold
    // Uncomment the reverse_profile below when ready to test.
    // ────────────────────────────────────────────────────────────
    modes["drive_pivot_right"] = {
        .name = "drive_pivot_right",
        .forward_profile = {
            {FL, {ControlType::VELOCITY,  1.0f}},
            {FR, {ControlType::VELOCITY,  0.85f}},
            {RL, {ControlType::VELOCITY, 0.94f}},
            {RR, {ControlType::VELOCITY,     0.79f}},
        },
        .reverse_profile = {
            // TODO: your described reverse behavior would be:
            // {FL, {ControlType::VELOCITY, 0.8f}},  // fronts velocity, maybe lower K
            // {FR, {ControlType::VELOCITY, 0.8f}},
            // {RL, {ControlType::CURRENT,  1.0f}},  // rear-left current
            // {RR, {ControlType::HOLD,     0.0f}},  // rear-right still hold
            // For now, mirrors forward:
            {FL, {ControlType::VELOCITY,  1.0f}},
            {FR, {ControlType::VELOCITY,  0.85f}},
            {RL, {ControlType::VELOCITY, 0.94f}},
            {RR, {ControlType::VELOCITY,  0.79f}},
        }
    };

    return modes;
}