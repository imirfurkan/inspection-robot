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
 *
 * NOTE: drive_all K gains are NOT set here — the driver node overrides
 * them at runtime via Ackermann interpolation from /steering_angle.
 * The 1.0 values below are only the straight-ahead (center) baseline.
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
    // K values here are the straight-ahead baseline (all 1.0).
    // The driver node interpolates these per-motor based on
    // /steering_angle for Ackermann differential:
    //   full left  (servo_min): FL=0.85, FR=1.00, RL=0.79, RR=0.94
    //   center     (90°):       all 1.0
    //   full right (servo_max): FL=1.00, FR=0.85, RL=0.94, RR=0.79
    // ────────────────────────────────────────────────────────────
    modes["drive_all"] = {
        .name = "drive_all",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_rear_assist — Rear velocity + front current (compliant)
    // ────────────────────────────────────────────────────────────
    modes["drive_rear_assist"] = {
        .name = "drive_rear_assist",
        .forward_profile = {
            {FL, {ControlType::CURRENT,  1.0f, false}},
            {FR, {ControlType::CURRENT,  1.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY,  1.0f, false}},
            {FR, {ControlType::VELOCITY,  1.0f, false}},
            {RL, {ControlType::CURRENT, 1.0f, false}},
            {RR, {ControlType::CURRENT, 1.0f, false}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_front_nudge — Rear hold (vel=0) + front current
    // ────────────────────────────────────────────────────────────
    modes["drive_front_nudge"] = {
        .name = "drive_front_nudge",
        .forward_profile = {
            {FL, {ControlType::CURRENT, 1.0f, false}},
            {FR, {ControlType::CURRENT, 1.0f, false}},
            {RL, {ControlType::HOLD,    0.0f, false}},
            {RR, {ControlType::HOLD,    0.0f, false}},
        },
        .reverse_profile = {
            {FL, {ControlType::CURRENT, 1.0f, false}},
            {FR, {ControlType::CURRENT, 1.0f, false}},
            {RL, {ControlType::HOLD,    0.0f, false}},
            {RR, {ControlType::HOLD,    0.0f, false}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_front_only — Rear hold + front velocity
    // ────────────────────────────────────────────────────────────
    modes["drive_front_only"] = {
        .name = "drive_front_only",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::HOLD,     0.0f, false}},
            {RR, {ControlType::HOLD,     0.0f, false}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::HOLD,     0.0f, false}},
            {RR, {ControlType::HOLD,     0.0f, false}},
        }
    };

    // ────────────────────────────────────────────────────────────
    // drive_rear_only — Rear velocity + front hold
    // ────────────────────────────────────────────────────────────
    modes["drive_rear_only"] = {
        .name = "drive_rear_only",
        .forward_profile = {
            {FL, {ControlType::HOLD,     0.0f, false}},
            {FR, {ControlType::HOLD,     0.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        },
        .reverse_profile = {
            {FL, {ControlType::HOLD,     0.0f, false}},
            {FR, {ControlType::HOLD,     0.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        }
    };

    // ── tank_turn_right — left side forward, right side backward ──
    modes["tank_turn_left"] = {
        .name = "tank_turn_left",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, true}},
            {FR, {ControlType::VELOCITY, 1.0f, true}},   // reversed
            {RR, {ControlType::VELOCITY, 1.0f, false}},   // reversed
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, false}},
            {RL, {ControlType::VELOCITY, 1.0f, true}},
            {FR, {ControlType::VELOCITY, 1.0f, true}},
            {RR, {ControlType::VELOCITY, 1.0f, false}},
        }
    };

    modes["tank_turn_right"] = {
        .name = "tank_turn_right",
        .forward_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, true}},   // reversed
            {RL, {ControlType::VELOCITY, 1.0f, false}},   // reversed
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, true}},
        },
        .reverse_profile = {
            {FL, {ControlType::VELOCITY, 1.0f, true}},
            {RL, {ControlType::VELOCITY, 1.0f, false}},
            {FR, {ControlType::VELOCITY, 1.0f, false}},
            {RR, {ControlType::VELOCITY, 1.0f, true}},
        }
    };
    
    // // integeated into drive_all with Ackermann interpolation, so these "steer_trial" modes are not needed anymore.
    // // ────────────────────────────────────────────────────────────
    // // steer_trial —
    // // (This mode is not intended for regular use and may be removed later.)
    // modes["steer_trial_left"] = {
    //     .name = "steer_trial_left",
    //     .forward_profile = {
    //         {FL, {ControlType::CURRENT, 0.85f, false}},
    //         {FR, {ControlType::VELOCITY, 1.0f, false}},
    //         {RL, {ControlType::VELOCITY, 0.79f, false}},
    //         {RR, {ControlType::VELOCITY, 0.94f, false}},
    //     },
    //     .reverse_profile = {
    //         {FL, {ControlType::VELOCITY, 0.85f, false}},
    //         {FR, {ControlType::VELOCITY, 1.0f, false}},
    //         {RL, {ControlType::CURRENT, 0.79f, false}},
    //         {RR, {ControlType::VELOCITY, 0.94f, false}},
    //     }
    // };

    // // ────────────────────────────────────────────────────────────
    // modes["steer_trial_right"] = {
    //     .name = "steer_trial_right",
    //     .forward_profile = {
    //         {FL, {ControlType::VELOCITY, 1.0f, false}},
    //         {FR, {ControlType::CURRENT, 0.85f, false}},
    //         {RL, {ControlType::VELOCITY, 0.94f, false}},
    //         {RR, {ControlType::VELOCITY, 0.79f, false}},
    //     },
    //     .reverse_profile = {
    //         {FL, {ControlType::VELOCITY, 1.0f, false}},
    //         {FR, {ControlType::VELOCITY, 0.85f, false}},
    //         {RL, {ControlType::VELOCITY, 0.94f, false}},
    //         {RR, {ControlType::CURRENT, 0.79f, false}},
    //     }
    // };

    return modes;
}