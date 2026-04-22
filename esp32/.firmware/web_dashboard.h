#pragma once

#include "fsd_handler.h"
#include "can_driver.h"

/**
 * web_dashboard.h — HTTP + WebSocket dashboard
 *
 * HTTP  port 80 : serves the embedded HTML dashboard
 * WebSocket port 81 : pushes JSON state every 1 s;
 *                     receives control commands from the browser
 *
 * Call web_dashboard_init() once after wifi_ap_init() succeeds.
 * Call web_dashboard_update() every loop iteration (after CAN processing).
 * If init was never called, update() is a safe no-op.
 * Non-ESP32 targets (e.g. RP2040) get inline no-op stubs.
 */

#ifdef ESP32
void web_dashboard_init(FSDState *state, CanDriver *can);
void web_dashboard_update();
#else
inline void web_dashboard_init(FSDState *, CanDriver *) {}
inline void web_dashboard_update() {}
#endif
