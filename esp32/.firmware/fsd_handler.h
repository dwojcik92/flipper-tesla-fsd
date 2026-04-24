#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ── CAN frame (shared by all drivers) ────────────────────────────────────────
struct CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// ── Hardware version ──────────────────────────────────────────────────────────
typedef enum {
    TeslaHW_Unknown = 0,
    TeslaHW_Legacy,   // HW1 / HW2
    TeslaHW_HW3,
    TeslaHW_HW4,
} TeslaHWVersion;

// ── Operation mode ────────────────────────────────────────────────────────────
typedef enum {
    OpMode_ListenOnly = 0,  // default on boot: no TX
    OpMode_Active,          // TX enabled
} OpMode;

// ── Full FSD state ────────────────────────────────────────────────────────────
struct FSDState {
    TeslaHWVersion hw_version;
    int            speed_profile;   // 0-4 depending on HW
    int            speed_offset;    // HW3 only, 0-100

    bool           fsd_enabled;     // true when car's UI has FSD selected (mux0)
    bool           nag_suppressed;  // true after first nag-killer echo sent

    uint32_t       frames_modified; // TX counter

    // ── Feature flags (runtime-toggleable) ───────────────────────────────────
    bool           force_fsd;               // bypass UI selection check
    bool           suppress_speed_chime;    // ISA chime suppress (HW4, 0x399)
    bool           emergency_vehicle_detect;// set bit59 in mux0 (HW4)
    bool           nag_killer;              // 0x370 counter+1 echo
    bool           summon_eu_unlock;      // set_bit 19=0 + bit46=1 on mux=1 — Summon EU Unlock HW3
    bool           ulc_confirm_disable;     // clear bit1 of 0x3F8 — auto-confirm lane changes
    uint32_t       nag_echo_count;

    // ── PRNG nag killer state (organic torque variation) ─────────────────────
    uint32_t       prng_state;          // xorshift32 state (must never be 0)
    uint32_t       nag_grip_due_ms;     // millis() when next grip pulse fires (0=uninit)
    uint8_t        nag_grip_frames;     // frames remaining in current grip excursion

    // ── DAS_status (0x39B) sniff — nag feedback ───────────────────────────────
    uint8_t        das_hands_on_state;  // 4-bit escalation level (0=none, higher=urgent)
    bool           das_status_seen;

    // ── Mode + diagnostics ────────────────────────────────────────────────────
    OpMode         op_mode;
    bool           tesla_ota_in_progress;   // pause TX during OTA
    uint8_t        ota_raw_state;           // raw GTW_updateInProgress bits [1:0]
    uint8_t        ota_assert_count;        // consecutive "in-progress" samples
    uint8_t        ota_clear_count;         // consecutive "not in-progress" samples
    uint32_t       crc_err_count;           // CAN bus error counter
    uint32_t       rx_count;                // total frames seen (wiring check)
    uint32_t       seen_gtw_car_state;      // 0x318 seen count
    uint32_t       seen_gtw_car_config;     // 0x398 seen count
    uint32_t       seen_ap_control;         // 0x3FD seen count
    uint32_t       seen_bms_hv;             // 0x132 seen count
    uint32_t       seen_bms_soc;            // 0x292 seen count
    uint32_t       seen_bms_thermal;        // 0x312 seen count
    uint32_t       seen_gtw_config_eth;     // 0x7FF seen count

    // ── BMS read-only sniff ───────────────────────────────────────────────────
    bool           bms_output;       // print BMS data to serial
    bool           bms_seen;
    float          pack_voltage_v;
    float          pack_current_a;
    float          soc_percent;
    int8_t         batt_temp_min_c;
    int8_t         batt_temp_max_c;

    // ── Precondition trigger ──────────────────────────────────────────────────
    bool           precondition;     // periodically inject 0x082

    // ── TLSSC Restore (0x331 DAS config spoof) ──────────────────────────────
    bool           tlssc_restore;
    uint32_t       tlssc_restore_count;

    // ── GTW_carConfig 0x7FF mux=2 autopilot tier readback ───────────────────
    int8_t         gtw_autopilot_tier;      // -1 unknown; 0 none; 1 highway; 2 enhanced; 3 self-driving; 4 basic
};

// ── API ───────────────────────────────────────────────────────────────────────

/** Initialise state with safe defaults for a given HW version. */
void fsd_state_init(FSDState *state, TeslaHWVersion hw);

/** Returns true if current state allows transmitting CAN frames. */
bool fsd_can_transmit(const FSDState *state);

/** Read GTW_carConfig (0x398) to detect HW version.
 *  Returns TeslaHW_Unknown if frame is not 0x398 or version unrecognised. */
TeslaHWVersion fsd_detect_hw_version(const CanFrame *frame);

/** Parse GTW_carState (0x318) — updates tesla_ota_in_progress. */
void fsd_handle_gtw_car_state(FSDState *state, const CanFrame *frame);

/** Parse DAS_followDistance (0x3F8) — updates speed_profile from stalk. */
void fsd_handle_follow_distance(FSDState *state, const CanFrame *frame);

/** Modify DAS_autopilotControl (0x3FD) for HW3/HW4.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_autopilot_frame(FSDState *state, CanFrame *frame);

/** Parse STW_ACTN_RQ (0x045) for Legacy stalk position → speed_profile. */
void fsd_handle_legacy_stalk(FSDState *state, const CanFrame *frame);

/** Modify DAS_autopilot (0x3EE) for Legacy/HW1/HW2.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_legacy_autopilot(FSDState *state, CanFrame *frame);

/** Modify ISA speed limit frame (0x399) to suppress speed chime (HW4).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_isa_speed_chime(CanFrame *frame);

/** Build an echo of EPAS3P_sysStatus (0x370) with PRNG torque variation and
 *  DAS-aware intensity.  now_ms = millis().  Returns true if echo should be sent. */
bool fsd_handle_nag_killer(FSDState *state, const CanFrame *frame, CanFrame *out, uint32_t now_ms);

/** Parse DAS_status (0x39B) — updates das_hands_on_state (4-bit nag escalation). */
void fsd_handle_das_status(FSDState *state, const CanFrame *frame);

/** Modify DAS_followDistance (0x3F8) to clear ULC stalk-confirm bit (bit 1).
 *  Disables "require blinker tap" for Navigate-on-Autopilot lane changes.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_ulc_disable(const FSDState *state, CanFrame *frame);

/** Parse BMS_hvBusStatus (0x132) — updates pack_voltage_v / pack_current_a. */
void fsd_handle_bms_hv(FSDState *state, const CanFrame *frame);

/** Parse BMS_socStatus (0x292) — updates soc_percent. */
void fsd_handle_bms_soc(FSDState *state, const CanFrame *frame);

/** Parse BMS_thermalStatus (0x312) — updates batt_temp_min/max_c. */
void fsd_handle_bms_thermal(FSDState *state, const CanFrame *frame);

/** Build a UI_tripPlanning (0x082) frame to trigger active battery heating. */
void fsd_build_precondition_frame(CanFrame *frame);

/** Handle CAN ID 0x331 — TLSSC Restore via DAS config spoof.
 *  Overwrites byte[0] lower 6 bits to 0x1B (SELF_DRIVING).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_tlssc_restore(FSDState *state, CanFrame *frame);

/** Parse GTW_carConfig (0x7FF) mux=2 autopilot tier readback.
 *  byte[5] bits 4:2: 0=NONE 1=HIGHWAY 2=ENHANCED 3=SELF_DRIVING 4=BASIC. */
void fsd_handle_gtw_autopilot_tier(FSDState *state, const CanFrame *frame);
