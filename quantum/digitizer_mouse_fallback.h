// Copyright 2025 George Norton (@george-norton)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#if defined(POINTING_DEVICE_DRIVER_digitizer)
#    include "pointing_device.h"

const pointing_device_driver_t digitizer_pointing_device_driver;
extern bool                    digitizer_send_mouse_reports;
extern bool                    force_digitizer_send_mouse_reports;

/* Swipe handling. The default (weak) implementation taps the
 * DIGITIZER_SWIPE_*_KC keycodes; a keymap can override it to source
 * actions elsewhere, e.g. VIA-assignable spare matrix positions. */
typedef enum { DIGITIZER_SWIPE_DIR_RIGHT, DIGITIZER_SWIPE_DIR_LEFT, DIGITIZER_SWIPE_DIR_DOWN, DIGITIZER_SWIPE_DIR_UP } digitizer_swipe_dir_t;
void digitizer_swipe_action(digitizer_swipe_dir_t dir);

__attribute__((weak)) void digitizer_update_mouse_report(report_digitizer_t *report);
__attribute__((weak)) bool digitizer_update_gesture_state(void);
#endif