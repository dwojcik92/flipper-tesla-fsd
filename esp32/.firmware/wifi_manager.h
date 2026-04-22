#pragma once

/**
 * wifi_manager.h — WiFi Access Point initialisation
 *
 * Starts a soft-AP with the fixed credentials below.
 * Call once from setup(); non-fatal on failure.
 * Non-ESP32 targets (e.g. RP2040) get an inline no-op stub.
 */

#ifdef ESP32
bool wifi_ap_init();
#else
inline bool wifi_ap_init() { return false; }
#endif
