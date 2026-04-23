/*
 * fsd_handler.cpp
 *
 * CAN frame manipulation logic for Tesla FSD unlock.
 * Ported from hypery11/flipper-tesla-fsd  fsd_logic/fsd_handler.c
 *
 * All bit operations, mux dispatch, speed profile mapping, and checksum
 * calculations are kept bit-for-bit identical to the upstream Flipper Zero
 * implementation.
 */

#include "fsd_handler.h"
#include <string.h>

// ── Internal helpers ──────────────────────────────────────────────────────────

static void set_bit(CanFrame *frame, int bit, bool value) {
    if (bit < 0 || bit >= 64) return;
    int byte_idx = bit / 8;
    int bit_idx  = bit % 8;
    uint8_t mask = (uint8_t)(1U << bit_idx);
    if (value)
        frame->data[byte_idx] |= mask;
    else
        frame->data[byte_idx] &= (uint8_t)(~mask);
}

static uint8_t read_mux_id(const CanFrame *frame) {
    // MUX ID is the lower 3 bits of byte 0
    return frame->data[0] & 0x07;
}

static bool is_fsd_selected(const CanFrame *frame, bool force_fsd) {
    if (force_fsd) return true;
    if (frame->dlc < 5) return false;
    // DAS_autopilotControl byte 4 bits [7:6] = UI "FSD selected" flag (bit 38 in the 64-bit data
    // field).  Note: bit 46 is the *output* FSD-activation bit written to the modified frame —
    // a different field at byte 5 bit 6.
    return (frame->data[4] >> 6) & 0x01;
}

// ── State init ────────────────────────────────────────────────────────────────

void fsd_state_init(FSDState *state, TeslaHWVersion hw) {
    memset(state, 0, sizeof(FSDState));
    state->hw_version = hw;
    state->op_mode    = OpMode_ListenOnly;  // safe default — never TX on boot

    // Feature flags: nag killer and chime suppress default ON; others OFF
    state->nag_killer           = true;
    state->suppress_speed_chime = true;
    state->emergency_vehicle_detect = false;
    state->force_fsd            = false;
    state->bms_output           = false;
    state->prng_state           = 0xABCDABCDu; // xorshift32 seed (must be non-zero)

    // Default speed profile per HW version
    if (hw == TeslaHW_HW4)
        state->speed_profile = 4;
    else if (hw == TeslaHW_Legacy)
        state->speed_profile = 1;
    else
        state->speed_profile = 2;
}

// ── Transmit gate ─────────────────────────────────────────────────────────────

bool fsd_can_transmit(const FSDState *state) {
    if (state->op_mode == OpMode_ListenOnly) return false;
    if (state->tesla_ota_in_progress)        return false;
    return true;
}

// ── HW version detection from GTW_carConfig (0x398) ──────────────────────────

TeslaHWVersion fsd_detect_hw_version(const CanFrame *frame) {
    if (frame->id != CAN_ID_GTW_CAR_CONFIG) return TeslaHW_Unknown;
    // DAS_HWversion field: bits 7:6 of byte 0  (das_hw)
    uint8_t das_hw = (frame->data[0] >> 6) & 0x03;
    switch (das_hw) {
        case 0:
        case 1:  return TeslaHW_Legacy;   // HW1/HW2/EAP retrofit — uses 0x3EE/0x045
        case 2:  return TeslaHW_HW3;
        case 3:  return TeslaHW_HW4;
        default: return TeslaHW_Unknown;
    }
}

// ── OTA detection from GTW_carState (0x318) ───────────────────────────────────

void fsd_handle_gtw_car_state(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 7) return;
    // GTW_updateInProgress: bits 1:0 of byte 6.
    // Filter transient / incompatible values to avoid false positives.
    uint8_t raw = frame->data[6] & 0x03u;
    state->ota_raw_state = raw;

    bool in_progress = (raw == OTA_IN_PROGRESS_RAW_VALUE);
    if (in_progress) {
        if (state->ota_assert_count < 255u) state->ota_assert_count++;
        state->ota_clear_count = 0;
        if (state->ota_assert_count >= OTA_ASSERT_FRAMES)
            state->tesla_ota_in_progress = true;
    } else {
        if (state->ota_clear_count < 255u) state->ota_clear_count++;
        state->ota_assert_count = 0;
        if (state->ota_clear_count >= OTA_CLEAR_FRAMES)
            state->tesla_ota_in_progress = false;
    }
}

// ── Follow distance → speed profile (DAS_followDistance 0x3F8) ───────────────

void fsd_handle_follow_distance(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    // Follow distance stalk position: bits 7:5 of byte 5
    uint8_t fd = (frame->data[5] & 0xE0) >> 5;

    if (state->hw_version == TeslaHW_HW3) {
        // HW3: 3 levels  (fd 1→profile 2, 2→1, 3→0)
        switch (fd) {
            case 1: state->speed_profile = 2; break;
            case 2: state->speed_profile = 1; break;
            case 3: state->speed_profile = 0; break;
            default: break;
        }
    } else {
        // HW4: 5 levels  (fd 1→3, 2→2, 3→1, 4→0, 5→4)
        switch (fd) {
            case 1: state->speed_profile = 3; break;
            case 2: state->speed_profile = 2; break;
            case 3: state->speed_profile = 1; break;
            case 4: state->speed_profile = 0; break;
            case 5: state->speed_profile = 4; break;
            default: break;
        }
    }
}

// ── HW3/HW4 autopilot control (DAS_autopilotControl 0x3FD) ───────────────────

bool fsd_handle_autopilot_frame(FSDState *state, CanFrame *frame) {
    if (frame->dlc < 8) return false;
    // Only process known HW versions to avoid corrupting frames for HW_Unknown
    if (state->hw_version != TeslaHW_HW3 && state->hw_version != TeslaHW_HW4)
        return false;

    uint8_t mux     = read_mux_id(frame);
    bool    fsd_ui  = is_fsd_selected(frame, state->force_fsd);
    bool    modified = false;

    // mux 0 is the authoritative "is FSD requested" mux
    if (mux == 0) state->fsd_enabled = fsd_ui;

    if (state->hw_version == TeslaHW_HW3) {
        // ── HW3 ──────────────────────────────────────────────────────────────
        if (mux == 0 && state->fsd_enabled) {
            // Compute speed offset from current speed signal (bits 6:1 of byte 3)
            int raw    = (int)((frame->data[3] >> 1) & 0x3F) - 30;
            int offset = raw * 5;
            if (offset < 0)   offset = 0;
            if (offset > 100) offset = 100;
            state->speed_offset = offset;

            // Activate FSD: set bit 46
            set_bit(frame, 46, true);

            // Write speed profile into bits 2:1 of byte 6
            frame->data[6] &= ~0x06u;
            frame->data[6] |= (uint8_t)((state->speed_profile & 0x03) << 1);
            modified = true;
        }
        if (mux == 1) {
            // Nag suppression via bit 19 (clear = no hands-on-wheel request)
            set_bit(frame, 19, false);
            // Enhanced Autopilot / summon unlock: set bit 46 on mux=1
            // Source: ev-open-can-tools HW3Handler enhancedAutopilotRuntime
            if (state->summon_eu_unlock)
                set_bit(frame, 46, true);
            state->nag_suppressed = true;
            modified = true;
        }
        if (mux == 2 && state->fsd_enabled) {
            // Write speed offset into bits 7:6 of byte 0 and bits 5:0 of byte 1
            frame->data[0] &= ~0xC0u;
            frame->data[1] &= ~0x3Fu;
            frame->data[0] |= (uint8_t)((state->speed_offset & 0x03) << 6);
            frame->data[1] |= (uint8_t)(state->speed_offset >> 2);
            modified = true;
        }
    } else {
        // ── HW4 ──────────────────────────────────────────────────────────────
        if (mux == 0 && state->fsd_enabled) {
            set_bit(frame, 46, true);   // FSD activation
            set_bit(frame, 60, true);   // HW4 additional FSD bit
            if (state->emergency_vehicle_detect)
                set_bit(frame, 59, true);  // emergency vehicle detection
            modified = true;
        }
        if (mux == 1) {
            set_bit(frame, 19, false);  // clear hands-on-wheel nag
            set_bit(frame, 47, true);   // HW4 nag-suppression confirmation bit
            state->nag_suppressed = true;
            modified = true;
        }
        if (mux == 2) {
            // Write speed profile into bits 6:4 of byte 7
            frame->data[7] &= ~(uint8_t)(0x07u << 4);
            frame->data[7] |=  (uint8_t)((state->speed_profile & 0x07u) << 4);
            modified = true;
        }
    }

    if (modified) state->frames_modified++;
    return modified;
}

// ── Legacy autopilot (DAS_autopilot 0x3EE) ───────────────────────────────────

void fsd_handle_legacy_stalk(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 2) return;
    // STW_ACTN_RQ: stalk position encoded in bits 7:5 of byte 1
    // 0x00=Pos1, 0x21=Pos2, 0x42=Pos3, 0x64=Pos4, 0x85=Pos5, 0xA6=Pos6, 0xC8=Pos7
    uint8_t pos = frame->data[1] >> 5;
    if (pos <= 1)
        state->speed_profile = 2;
    else if (pos == 2)
        state->speed_profile = 1;
    else
        state->speed_profile = 0;
}

bool fsd_handle_legacy_autopilot(FSDState *state, CanFrame *frame) {
    if (frame->dlc < 8) return false;

    uint8_t mux    = read_mux_id(frame);
    bool    fsd_ui = is_fsd_selected(frame, state->force_fsd);
    bool    modified = false;

    if (mux == 0) state->fsd_enabled = fsd_ui;

    if (mux == 0 && state->fsd_enabled) {
        set_bit(frame, 46, true);
        // Speed profile in bits 2:1 of byte 6 (same encoding as HW3)
        frame->data[6] &= ~0x06u;
        frame->data[6] |= (uint8_t)((state->speed_profile & 0x03) << 1);
        modified = true;
    }
    if (mux == 1) {
        set_bit(frame, 19, false);
        state->nag_suppressed = true;
        modified = true;
    }

    if (modified) state->frames_modified++;
    return modified;
}

// ── ISA speed chime suppress (0x399, HW4 only) ───────────────────────────────

bool fsd_handle_isa_speed_chime(CanFrame *frame) {
    if (frame->dlc < 8) return false;
    // Set "ISA_speedLimitSoundActive" flag: bit 5 of byte 1
    frame->data[1] |= 0x20u;
    // Recalculate Tesla checksum: sum(byte0..6) + low(CAN_ID) + high(CAN_ID)
    // CAN_ID_ISA_SPEED = 0x399 → low=0x99, high=0x03
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += frame->data[i];
    sum += (uint8_t)(CAN_ID_ISA_SPEED & 0xFFu) + (uint8_t)(CAN_ID_ISA_SPEED >> 8);
    frame->data[7] = sum;
    return true;
}

// ── NAG killer: PRNG organic-torque echo of EPAS3P_sysStatus (0x370) ────────
//
// Builds a spoofed EPAS frame with handsOnLevel=1 (hands confirmed) and
// a randomised torsion-bar torque so the DAS can't fingerprint a fixed value.
//
// Torque encoding in 0x370:
//   torsionBarTorque: 12-bit Motorola (bit19|12@0+), factor=0.01, offset=-20.5
//   raw_12bit = (Nm + 20.5) * 100
//   byte2[3:0] = high 4 bits of raw_12bit, byte3 = low 8 bits
//
// Ranges (verified against anchor: 0xB6 byte3 + nibble 8 = 1.80 Nm):
//   Normal walk  1.00–2.40 Nm: raw 2150–2290 → nibble=0x8, byte3=0x66–0xF2
//   Grip pulse   3.10–3.30 Nm: raw 2360–2380 → nibble=0x9, byte3=0x38–0x4C
//
// DAS-aware: when das_hands_on_state >= 3 (escalated nag), bias upper half.
// Grip pulse: every 5–9 s, 2 frames of 3.10–3.30 Nm simulating grip adjustment.
//
// Checksum formula (same as TSL6P module):
//   byte7 = (sum(byte0..6) + 0x73) & 0xFF  (0x73 = 0x70 + 0x03)

#define NAG_TORQUE_NORM_MIN_B3   0x66u   // 1.00 Nm
#define NAG_TORQUE_NORM_MAX_B3   0xF2u   // 2.40 Nm
#define NAG_TORQUE_GRIP_MIN_B3   0x38u   // 3.10 Nm
#define NAG_TORQUE_GRIP_MAX_B3   0x4Cu   // 3.30 Nm
#define NAG_TORQUE_GRIP_NIBBLE   0x09u   // byte2 high nibble for grip range
#define NAG_GRIP_INTERVAL_MIN    5000u   // ms between grip pulses (min)
#define NAG_GRIP_INTERVAL_RANGE  4000u   // + 0..4000 ms random
#define NAG_GRIP_PULSE_FRAMES    2u      // frames per grip excursion

static uint32_t xorshift32(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (*s = x);
}

bool fsd_handle_nag_killer(FSDState *state, const CanFrame *frame, CanFrame *out, uint32_t now_ms) {
    if (frame->dlc < 8)    return false;
    if (!state->nag_killer) return false;

    // Act when handsOnLevel == 0 (nag imminent) or >= 2 (escalated).
    // Level 1 = hands confirmed OK — no action needed.
    uint8_t hands_on_epas = (frame->data[4] >> 6) & 0x03;
    if (hands_on_epas == 1) return false;

    // Initialise grip-pulse timer on first call
    if (state->nag_grip_due_ms == 0) {
        uint32_t r = xorshift32(&state->prng_state);
        state->nag_grip_due_ms = now_ms + NAG_GRIP_INTERVAL_MIN + (r % NAG_GRIP_INTERVAL_RANGE);
    }

    out->id  = CAN_ID_EPAS_STATUS;
    out->dlc = 8;
    out->data[0] = frame->data[0];
    out->data[1] = frame->data[1];
    out->data[5] = frame->data[5];

    // counter+1: lower nibble of byte 6
    uint8_t cnt = ((frame->data[6] & 0x0Fu) + 1u) & 0x0Fu;
    out->data[6] = (frame->data[6] & 0xF0u) | cnt;

    // Grip pulse state machine
    bool grip = false;
    if (state->nag_grip_frames > 0) {
        grip = true;
        state->nag_grip_frames--;
        if (state->nag_grip_frames == 0) {
            uint32_t r = xorshift32(&state->prng_state);
            state->nag_grip_due_ms = now_ms + NAG_GRIP_INTERVAL_MIN + (r % NAG_GRIP_INTERVAL_RANGE);
        }
    } else if (now_ms >= state->nag_grip_due_ms) {
        grip = true;
        state->nag_grip_frames = NAG_GRIP_PULSE_FRAMES - 1u;
    }

    // Pick torque byte — DAS-aware: bias high half when escalation level >= 3
    uint8_t torque_b3;
    uint8_t b2_nibble;
    uint32_t rnd = xorshift32(&state->prng_state);
    if (grip) {
        torque_b3 = NAG_TORQUE_GRIP_MIN_B3 +
                    (uint8_t)(rnd % (NAG_TORQUE_GRIP_MAX_B3 - NAG_TORQUE_GRIP_MIN_B3 + 1u));
        b2_nibble = NAG_TORQUE_GRIP_NIBBLE;
    } else {
        uint8_t range = NAG_TORQUE_NORM_MAX_B3 - NAG_TORQUE_NORM_MIN_B3;
        if (state->das_hands_on_state >= 3) {
            // Escalated: bias upper half of normal range
            torque_b3 = NAG_TORQUE_NORM_MIN_B3 + (range / 2u) +
                        (uint8_t)(rnd % (range / 2u + 1u));
        } else {
            torque_b3 = NAG_TORQUE_NORM_MIN_B3 + (uint8_t)(rnd % (range + 1u));
        }
        b2_nibble = 0x08u;
    }

    out->data[2] = (frame->data[2] & 0xF0u) | b2_nibble;
    out->data[3] = torque_b3;
    out->data[4] = (frame->data[4] & ~0xC0u) | 0x40u;  // handsOnLevel = 1

    // Checksum
    uint16_t sum = 0;
    for (int i = 0; i < 7; i++) sum += out->data[i];
    sum += (CAN_ID_EPAS_STATUS & 0xFFu) + (CAN_ID_EPAS_STATUS >> 8);
    out->data[7] = (uint8_t)(sum & 0xFFu);

    state->nag_echo_count++;
    state->nag_suppressed = true;
    return true;
}

// ── BMS read-only parsers ─────────────────────────────────────────────────────

void fsd_handle_bms_hv(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 4) return;
    // Voltage: uint16 little-endian bytes 1:0, LSB = 0.01 V
    uint16_t raw_v = ((uint16_t)frame->data[1] << 8) | frame->data[0];
    // Current: int16 little-endian bytes 3:2, LSB = 0.1 A (signed)
    int16_t  raw_i = (int16_t)(((uint16_t)frame->data[3] << 8) | frame->data[2]);
    state->pack_voltage_v = raw_v * 0.01f;
    state->pack_current_a = raw_i * 0.1f;
    state->bms_seen = true;
}

void fsd_handle_bms_soc(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 2) return;
    // SoC: 10-bit little-endian (bits 9:0 across bytes 1:0), LSB = 0.1 %
    uint16_t raw = ((uint16_t)(frame->data[1] & 0x03u) << 8) | frame->data[0];
    state->soc_percent = raw * 0.1f;
    state->bms_seen = true;
}

void fsd_handle_bms_thermal(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    // Temperatures: raw byte − 40 = °C
    state->batt_temp_min_c = (int8_t)((int)frame->data[4] - 40);
    state->batt_temp_max_c = (int8_t)((int)frame->data[5] - 40);
    state->bms_seen = true;
}

// ── Precondition trigger ──────────────────────────────────────────────────────

void fsd_build_precondition_frame(CanFrame *frame) {
    memset(frame, 0, sizeof(CanFrame));
    frame->id  = CAN_ID_TRIP_PLANNING;
    frame->dlc = 8;
    // byte0: bit0 = tripPlanningActive, bit2 = requestActiveBatteryHeating
    frame->data[0] = 0x05u;
}

// ── TLSSC Restore (0x331) ─────────────────────────────────────────────────────

bool fsd_handle_tlssc_restore(FSDState *state, CanFrame *frame) {
    if (!state->tlssc_restore) return false;
    if (frame->dlc < 1) return false;

    uint8_t original = frame->data[0];
    uint8_t modified = (original & 0xC0u) | 0x1Bu;

    if (modified == original) return false;

    frame->data[0] = modified;
    state->tlssc_restore_count++;
    return true;
}

// ── DAS_status (0x39B) read-only sniff — nag escalation level ─────────────────
//
// DAS_autopilotHandsOnState: bit42|4 little-endian → byte5 bits[5:2]
// This is a 4-bit escalation level (0=none, higher=more urgent).
// More precise than the 2-bit EPAS field; used as feedback to adapt nag torque.
// Source: opendbc tesla_model3_party.dbc

void fsd_handle_das_status(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    state->das_hands_on_state = (frame->data[5] >> 2) & 0x0Fu;
    state->das_status_seen    = true;
}

// ── ULC stalk-confirm disable (0x3F8 bit 1) ───────────────────────────────────
//
// Clears bit 1 of DAS_followDistance to remove the "require blinker tap to
// confirm lane change" behaviour on Navigate-on-Autopilot.
// Source: ev-open-can-tools "ULC Stalk Confirm Disable" rule.

bool fsd_ulc_disable(const FSDState *state, CanFrame *frame) {
    if (!state->ulc_confirm_disable) return false;
    if (frame->dlc < 1) return false;
    if ((frame->data[0] & 0x02u) == 0) return false;  // already cleared — no TX needed
    frame->data[0] &= ~0x02u;
    return true;
}
