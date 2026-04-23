#pragma once

// ── CAN IDs ───────────────────────────────────────────────────────────────────
#define CAN_ID_STW_ACTN_RQ    0x045u  // 69   - STW_ACTN_RQ:  steering stalk (Legacy follow distance)
#define CAN_ID_TRIP_PLANNING  0x082u  // 130  - UI_tripPlanning: precondition trigger
#define CAN_ID_BMS_HV_BUS     0x132u  // 306  - BMS_hvBusStatus: pack voltage / current
#define CAN_ID_BMS_SOC        0x292u  // 658  - BMS_socStatus:   state of charge
#define CAN_ID_BMS_THERMAL    0x312u  // 786  - BMS_thermalStatus: battery temp
#define CAN_ID_GTW_CAR_STATE  0x318u  // 792  - GTW_carState:    OTA detection
#define CAN_ID_DAS_STATUS     0x39Bu  // 923  - DAS_status: AP hands-on state (4-bit, nag feedback)
#define CAN_ID_EPAS_STATUS    0x370u  // 880  - EPAS3P_sysStatus: nag killer target
#define CAN_ID_GTW_CAR_CONFIG 0x398u  // 920  - GTW_carConfig:   HW version detection
#define CAN_ID_ISA_SPEED      0x399u  // 921  - ISA speed limit:  HW4 chime suppress
#define CAN_ID_AP_LEGACY      0x3EEu  // 1006 - DAS_autopilot:   Legacy / HW1 / HW2
#define CAN_ID_FOLLOW_DIST    0x3F8u  // 1016 - DAS_followDistance: speed profile source
#define CAN_ID_DAS_AP_CONFIG  0x331u  // 817  - DAS autopilot config (tier restore target, ~1 Hz)
#define CAN_ID_AP_CONTROL     0x3FDu  // 1021 - DAS_autopilotControl: HW3 / HW4 core

// ── GPIO ──────────────────────────────────────────────────────────────────────
#if defined(BOARD_LILYGO)
  #define PIN_CAN_TX         27
  #define PIN_CAN_RX         26
  #define PIN_CAN_SPEED_MODE 23   // SN65HVD230 Rs — must be LOW for TX+RX
  #define PIN_LED            4    // SK6812 NeoPixel
  #define SD_MISO            2
  #define SD_MOSI            15
  #define SD_SCLK            14
  #define SD_CS              13
#elif defined(BOARD_FEATHER_RP2040_CAN)
  // Adafruit Feather RP2040 CAN — MCP25625 on SPI1
  // SPI1 pins (SCK=14, MOSI=15, MISO=8) are configured by the board variant;
  // SPI.begin() with no arguments uses them automatically.
  #define PIN_MCP_CS          10    // GPIO10 — CAN_CS
  #define PIN_CAN_INT          9    // GPIO9  — MCP25625 ~INT (active-low)
  #undef  PIN_LED                   // board variant defines PIN_LED=13; override for NeoPixel
  #define PIN_LED             16    // GPIO16 — NeoPixel data
  #define PIN_NEOPIXEL_POWER  17    // GPIO17 — NeoPixel power (active HIGH)
  // No dedicated user button on this board; button logic is disabled.
  // MCP25625 ships with a 16 MHz crystal on the Feather RP2040 CAN.
  #define MCP_CRYSTAL_MHZ   MCP_16MHZ
#else
  #ifndef PIN_CAN_TX
  #define PIN_CAN_TX   22   // TWAI TX → ATOMIC CAN Base TX
  #endif
  #ifndef PIN_CAN_RX
  #define PIN_CAN_RX   19   // TWAI RX ← ATOMIC CAN Base RX
  #endif
  #ifndef PIN_LED
  #define PIN_LED      27   // SK6812 NeoPixel (single LED)
  #endif
#endif
#if !defined(BOARD_FEATHER_RP2040_CAN) && !defined(PIN_BUTTON)
#define PIN_BUTTON   39   // Built-in button, active-LOW (no external pull-up needed)
#endif

// MCP2515 SPI — only used in CAN_DRIVER_MCP2515 build
#if !defined(BOARD_FEATHER_RP2040_CAN)
  // Generic ESP32: Standard VSPI pins: SCK=18, MISO=19, MOSI=23, CS=5
  #ifndef PIN_MCP_CS
  #define PIN_MCP_CS   5
  #endif
  #define PIN_MCP_SCK  18
  #define PIN_MCP_MISO 19
  #define PIN_MCP_MOSI 23
  // MCP2515 oscillator: common Chinese modules use 8 MHz
  #ifndef MCP_CRYSTAL_MHZ
  #define MCP_CRYSTAL_MHZ  MCP_8MHZ   // from autowp-mcp2515 CAN_CLOCK enum
  #endif
#endif

// ── Timing ────────────────────────────────────────────────────────────────────
#define WIRING_WARN_MS        5000u   // Red LED / serial warning if no CAN after this
#define PRECOND_INTERVAL_MS    500u   // Re-inject 0x082 precondition every N ms
#define BMS_PRINT_MS          1000u   // BMS serial print interval
#define BUTTON_DEBOUNCE_MS      50u
#define LONG_PRESS_MS         3000u   // Long press → toggle NAG killer
#define DOUBLE_CLICK_MS        400u   // Max gap between two clicks for double-click
#define STATUS_PRINT_MS       5000u   // Periodic status line when Active

// OTA detection hardening on GTW_carState (0x318)
// Some firmware versions keep non-zero states when no update is actively running.
// We only treat one specific raw value as "update in progress" and require
// consecutive-frame confirmation to avoid false positives.
#define OTA_IN_PROGRESS_RAW_VALUE  1u
#define OTA_ASSERT_FRAMES          3u
#define OTA_CLEAR_FRAMES           6u

#if defined(BOARD_LILYGO)
  #define ME2107_EN 16
#endif
