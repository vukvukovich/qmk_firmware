// Copyright 2025 George Norton (@george-norton)
// SPDX-License-Identifier: GPL-2.0-or-later

#if defined(POINTING_DEVICE_DRIVER_digitizer)

// We can fallback to reporting as a mouse for hosts which do not implement trackpad support.

#    include <stdlib.h>
#    include <math.h>
#    include "digitizer.h"
#    include "digitizer_mouse_fallback.h"
#    include "debug.h"
#    include "print.h"
#    include "timer.h"
#    include "action.h"

#    ifndef DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT
#        define DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT 150
#    endif

#    ifndef DIGITIZER_MOUSE_TAP_DURATION
#        define DIGITIZER_MOUSE_TAP_DURATION 1
#    endif

#    ifndef DIGITIZER_MOUSE_TAP_DISTANCE
#        define DIGITIZER_MOUSE_TAP_DISTANCE (DIGITIZER_RESOLUTION_X / DIGITIZER_WIDTH_MM)
#    endif

#    ifndef DIGITIZER_SCROLL_DIVISOR
#        define DIGITIZER_SCROLL_DIVISOR 10
#    endif

#    ifndef DIGITIZER_MOUSE_SWIPE_TIMEOUT
#        define DIGITIZER_MOUSE_SWIPE_TIMEOUT 1000
#    endif

#    ifndef DIGITIZER_MOUSE_SWIPE_DISTANCE
#        define DIGITIZER_MOUSE_SWIPE_DISTANCE 500
#    endif

#    ifndef DIGITIZER_MOUSE_SWIPE_THRESHOLD
#        define DIGITIZER_MOUSE_SWIPE_THRESHOLD 300
#    endif

#    ifndef DIGITIZER_SWIPE_LEFT_KC
#        define DIGITIZER_SWIPE_LEFT_KC QK_MOUSE_BUTTON_3
#    endif

#    ifndef DIGITIZER_SWIPE_RIGHT_KC
#        define DIGITIZER_SWIPE_RIGHT_KC QK_MOUSE_BUTTON_4
#    endif

#    ifndef DIGITIZER_SWIPE_UP_KC
#        define DIGITIZER_SWIPE_UP_KC KC_LEFT_GUI
#    endif

#    ifndef DIGITIZER_SWIPE_DOWN_KC
#        define DIGITIZER_SWIPE_DOWN_KC KC_ESC
#    endif

/* 1-euro filter (Casiez/Roquet/Vogel, CHI 2012) on the absolute finger
 * position. Cutoff adapts to speed: near-still = heavy smoothing (rest
 * jitter crushed below one count), fast = light smoothing (no lag). */
#    ifndef ONE_EURO_MINCUTOFF
#        define ONE_EURO_MINCUTOFF 1.0f /* Hz; LOWER to kill at-rest jitter */
#    endif

#    ifndef ONE_EURO_BETA
#        define ONE_EURO_BETA 0.01f /* cutoff gain per unit/s; RAISE to kill fast-swipe lag */
#    endif

#    ifndef ONE_EURO_DCUTOFF
#        define ONE_EURO_DCUTOFF 1.0f /* Hz; smoothing of the speed estimate, leave at 1 */
#    endif

/* Rest clamp on top of the 1-euro filter: while the finger is judged at
 * rest, the cursor anchors and emits nothing (Apple-grade stillness).
 * Rest is detected from a SLOW EMA of the signed velocity - at rest the
 * noise velocities ping-pong and cancel toward zero, during a real slow
 * drag they all point one way and do not - so unlike a zero-delta or
 * magnitude test, this cannot re-arm mid-drag. On breakout the anchored
 * displacement is replayed over a few reports, never swallowed. */
#    ifndef ONE_EURO_REST_RADIUS
#        define ONE_EURO_REST_RADIUS 18.0f /* post-scale units the filtered position may wander while anchored */
#    endif

#    ifndef ONE_EURO_REST_VSLOW_CUTOFF
#        define ONE_EURO_REST_VSLOW_CUTOFF 0.25f /* Hz; slow signed-velocity EMA for the rest detector */
#    endif

#    ifndef ONE_EURO_REST_V_THRESH
#        define ONE_EURO_REST_V_THRESH 120.0f /* units/s; |vslow| sum below this counts as still */
#    endif

#    ifndef ONE_EURO_REST_MS
#        define ONE_EURO_REST_MS 250 /* sustained stillness before anchoring */
#    endif

#    ifndef ONE_EURO_REST_LEAK_MS
#        define ONE_EURO_REST_LEAK_MS 40 /* anchor follows slow sensor walks 1 unit per this many ms */
#    endif

// Runtime-tunable so the keymap can adjust them live from keycodes while
// feeling the result; the defines above are only the boot values.
float one_euro_mincutoff = ONE_EURO_MINCUTOFF;
float one_euro_beta      = ONE_EURO_BETA;
bool  one_euro_rest_clamp = true;

#    ifndef DIGITIZER_MIN_CPI
#        define DIGITIZER_MIN_CPI 50
#    endif

#    ifndef DIGITIZER_MAX_CPI
#        define DIGITIZER_MAX_CPI 1200
#    endif

#    ifdef DIGITIZER_REPORT_TAPS_AS_CLICKS
bool digitizer_taps_as_clicks = true;
#    else
bool digitizer_taps_as_clicks = false;
#    endif

#define CLIP(X, A, B) (X<A ? A : X>B ? B : X)

// This variable indicates that we are sending mouse reports. It will be updated
// during USB enumeration if the host sends a feature report indicating it supports
// Microsofts Precision Trackpad protocol. This variable can also be modified by users
// to force reporting as a mouse or as a digitizer.
bool                  digitizer_send_mouse_reports = true;
bool                  force_digitizer_send_mouse_reports = false;

static report_mouse_t mouse_report                 = {};

static report_mouse_t digitizer_get_mouse_report(report_mouse_t _mouse_report);
static uint16_t       digitizer_get_cpi(void);
static void           digitizer_set_cpi(uint16_t cpi);
static bool           digitizer_mouse_fallback_init(void);

const pointing_device_driver_t digitizer_pointing_device_driver = {.init = digitizer_mouse_fallback_init, .get_report = digitizer_get_mouse_report, .get_cpi = digitizer_get_cpi, .set_cpi = digitizer_set_cpi};

/**
 * @brief Initialize the pointing device driver. If this function does not return true, the pointing device
 * is not usable.
 *
 * @return report_mouse_t
 */
static bool digitizer_mouse_fallback_init(void)
{
    // TODO: Return false here, if we have a physical digitizer device and its initialization failed.
    return true;
}

/**
 * @brief Gets the current digitizer mouse report, the pointing device feature will send this is we
 * nave fallen back to mouse mode.
 *
 * @return report_mouse_t
 */
static report_mouse_t digitizer_get_mouse_report(report_mouse_t _mouse_report) {
    if (force_digitizer_send_mouse_reports || digitizer_send_mouse_reports) {
        report_mouse_t report = mouse_report;
        // Retain the button state, but drop any motion.
        memset(&mouse_report, 0, sizeof(report_mouse_t));
        mouse_report.buttons = report.buttons;
        return report;
    }
    return _mouse_report;
}

static uint16_t mouse_cpi = 400;

/**
 * @brief Gets the CPI used by the digitizer mouse fallback feature.
 *
 * @return the current CPI value
 */
static uint16_t digitizer_get_cpi(void) {
    return mouse_cpi;
}

/**
 * @brief Sets the CPI used by the digitizer mouse fallback feature.
 *
 *  @param[in] the new CPI value
 */
static void digitizer_set_cpi(uint16_t cpi) {
    mouse_cpi = CLIP(cpi, DIGITIZER_MIN_CPI, DIGITIZER_MAX_CPI);
    digitizer_set_scale((mouse_cpi * 100) / DIGITIZER_MAX_CPI);
}

// Touch-down counter from the driver (weak default for non-maxtouch
// builds). A change means the contact lifted and re-registered - position
// is discontinuous and must not be diffed across.
__attribute__((weak)) uint32_t maxtouch_contact_downs = 0;

// One low-pass step: alpha from the sample interval te (seconds) and the
// cutoff frequency (Hz), per the reference 1-euro implementation.
static inline float oe_alpha(float te, float cutoff) {
    const float tau = 1.0f / (6.2831853f * cutoff);
    return 1.0f / (1.0f + tau / te);
}

// Motion-masked drain of a breakout-replay accumulator: catch-up flows
// only while that axis is already moving, and never faster than the
// motion itself - hidden inside the movement, so no perceptible hop.
// Folded into the emitted-position reference so the normal delta path
// (and its sub-unit remainder handling) does the actual emission.
static void oe_drain_replay(float *acc, float motion, float *out_ref) {
    float rp = *acc;
    if (rp == 0.0f) return;
    float cap  = (motion < 0.0f) ? -motion : motion;
    float step = (rp > cap) ? cap : (rp < -cap) ? -cap : rp;
    *acc -= step;
    if (*acc > -0.01f && *acc < 0.01f) *acc = 0.0f;
    *out_ref -= step;
}

// The gesture detection state machine will transition between these states.
typedef enum { None, Down, MoveScroll, Tapped, DoubleTapped, Drag, Swipe, Finished } State;

static State state     = None;
static int   tap_count = 0;

/**
 * \brief Signals that a gesture is in progress so digitizer_update_mouse_report should be called,
 * even if no new digitizer data is available.
 * @return true if update_mouse_report should run.
 */
bool digitizer_update_gesture_state(void) {
    return tap_count || state != None;
}

/**
 * \brief Generate a mouse report from the digitizer report. This function implements
 * a state machine to detect gestures and handle them.
 * @param[in] report a new digitizer report
 */
void digitizer_update_mouse_report(report_digitizer_t *report) {
    static int      contact_start_time = 0;
    static int      contact_start_x    = 0;
    static int      contact_start_y    = 0;
    static int      tap_contacts       = 0;
    static int      last_contacts      = 0;
    static bool     oe_init = false;
    static uint32_t oe_seen_downs = 0;
    static uint32_t oe_t_last = 0;
    static uint32_t oe_last_run = 0;
    static float    oe_x = 0.0f, oe_y = 0.0f;         /* filtered position */
    static float    oe_dx = 0.0f, oe_dy = 0.0f;       /* filtered speed, units/s */
    static float    oe_vsx = 0.0f, oe_vsy = 0.0f;     /* slow signed velocity (rest detector) */
    static float    oe_out_x = 0.0f, oe_out_y = 0.0f; /* last emitted position */
    static bool     rc_anchored = false;
    static float    rc_ax = 0.0f, rc_ay = 0.0f;       /* rest anchor */
    static uint32_t rc_still_since = 0;
    static uint32_t rc_leak_t = 0;
    static float    rc_replay_x = 0.0f, rc_replay_y = 0.0f;
    static uint16_t last_x             = 0;
    static uint16_t last_y             = 0;
    uint16_t  x                        = 0;
    uint16_t  y                        = 0;
    const uint32_t  duration           = timer_elapsed32(contact_start_time);
    int             contacts           = 0;

#if defined(DIGITIZER_REPORT_FINGER_PRESSURE) || defined(DIGITIZER_REPORT_FINGER_SIZE)
    // Buffer ovement while the contact is shrinking so it can be
    // dropped if we are lifting off.
    int             strength           = 0;
    static bool     buffering          = false;
    static int      buffered_x         = 0;
    static int      buffered_y         = 0;
    static int      strength_buffer    = 0;
    static uint8_t  strength_counter   = 0;
    static int      average_strength   = 0;
#endif

    memset(&mouse_report, 0, sizeof(report_mouse_t));

    for (int i = 0; i < DIGITIZER_FINGER_COUNT; i++) {
        if (report->fingers[i].tip) {
            if (contacts == 0) {
                x = report->fingers[i].x;
                y = report->fingers[i].y;
#if defined(DIGITIZER_REPORT_FINGER_PRESSURE)
                strength =  report->fingers[i].pressure;
#elif defined(DIGITIZER_REPORT_FINGER_SIZE)
                strength = report->fingers[i].width * report->fingers[i].height;
#endif
            }
            contacts++;
        }
    }

    switch (state) {
        case None: {
            oe_init = false;
            if (contacts != 0) {
                state              = Down;
                contact_start_time = timer_read32();
                contact_start_x    = x;
                contact_start_y    = y;
                tap_contacts       = contacts;
            }
            break;
        }
        case Down: {
            const uint16_t distance_x = abs(contact_start_x - x);
            const uint16_t distance_y = abs(contact_start_y - y);
            tap_contacts              = MAX(contacts, tap_contacts);

            if (contacts == 0) {
                state              = Tapped;
                contact_start_time = timer_read32();
            } else if (contacts >= 3) {
                state = Swipe;
            } else if (duration > DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT || distance_x > DIGITIZER_MOUSE_TAP_DISTANCE || distance_y > DIGITIZER_MOUSE_TAP_DISTANCE) {
                state = MoveScroll;
            }
            break;
        }
        case Drag:
        case MoveScroll: {
            if (contacts == 0) {
                state = None;
            } else if (contacts == 1) {
#if defined(DIGITIZER_REPORT_FINGER_PRESSURE) || defined(DIGITIZER_REPORT_FINGER_SIZE)
                // Reset our liftoff detection state if the number of contacts changed
                if (contacts != last_contacts)
                {
                    buffered_x = 0;
                    buffered_y = 0;
                    buffering = false;
                    strength_buffer    = 0;
                    strength_counter   = 0;
                    average_strength   = 0; 
                }

                // A 10% drop in strength is more than just noise, treat it as a potental lift off.
                if (average_strength)
                {
                    if (strength * 10 < average_strength * 9)
                    {
                        buffering = true;
                    }
                    if (strength >= average_strength)
                    {
                        buffering = false;
                    }
                }
                strength_buffer += strength;
                strength_counter ++;
                if ((!buffering && strength_counter > 5) || strength_counter > 16)
                {
                    average_strength = strength_buffer / strength_counter;
                    strength_buffer = 0;
                    strength_counter = 0;
                }
                if (buffering)
                {
                    buffered_x += (x - last_x);
                    buffered_y += (y - last_y);
                    break;
                }
                if (last_contacts == 1)
                {
                    mouse_report.x = x - last_x + buffered_x;
                    mouse_report.y = y - last_y + buffered_y;
                    buffered_x = 0;
                    buffered_y = 0;
                }
#else
                if (last_contacts == 1)
                {
                    /* Contact re-registration (lift + immediate re-touch,
                     * often within one processing cycle so contacts never
                     * reads 0 here): the centroid teleports. Reseed the
                     * filter instead of slewing across it - measured jumps
                     * up to 250 units. */
                    if (maxtouch_contact_downs != oe_seen_downs) {
                        oe_seen_downs = maxtouch_contact_downs;
                        oe_init       = false;
                    }

                    const uint32_t now = timer_read32();
                    /* A long gap means this branch stopped running (finger
                     * count changed, gesture states): position is
                     * discontinuous, do not slew across it. */
                    if (oe_init && (uint32_t)(now - oe_last_run) > 100) {
                        oe_init = false;
                    }
                    oe_last_run = now;
                    if (!oe_init) {
                        oe_init     = true;
                        oe_x        = (float)x;
                        oe_y        = (float)y;
                        oe_dx       = 0.0f;
                        oe_dy       = 0.0f;
                        oe_vsx      = 0.0f;
                        oe_vsy      = 0.0f;
                        oe_out_x    = oe_x;
                        oe_out_y    = oe_y;
                        oe_t_last   = now;
                        /* start at rest: anchored at the touch point */
                        rc_anchored    = true;
                        rc_ax          = oe_x;
                        rc_ay          = oe_y;
                        rc_still_since = 0;
                        rc_leak_t      = now;
                        rc_replay_x    = 0.0f;
                        rc_replay_y    = 0.0f;
                    } else {
                        /* Reports arrive irregularly (sensor IRQ plus stale
                         * gesture-timeout calls), so alpha is derived from
                         * the real sample interval each time. */
                        uint32_t te_ms = now - oe_t_last;
                        oe_t_last      = now;
                        if (te_ms < 1) te_ms = 1;
                        if (te_ms > 50) te_ms = 50;
                        const float te = (float)te_ms * 0.001f;

                        /* Speed estimate from the previous FILTERED position,
                         * itself low-passed at ONE_EURO_DCUTOFF: one noise
                         * spike barely moves it, sustained motion builds it,
                         * so spikes cannot inflate the cutoff. */
                        const float a_d = oe_alpha(te, ONE_EURO_DCUTOFF);
                        oe_dx += a_d * ((((float)x - oe_x) / te) - oe_dx);
                        oe_dy += a_d * ((((float)y - oe_y) / te) - oe_dy);

                        /* Much slower EMA of the SIGNED velocity: rest noise
                         * cancels itself here, coherent slow motion does not.
                         * This is the rest/move classifier. */
                        const float a_s = oe_alpha(te, ONE_EURO_REST_VSLOW_CUTOFF);
                        oe_vsx += a_s * (oe_dx - oe_vsx);
                        oe_vsy += a_s * (oe_dy - oe_vsy);

                        oe_x += oe_alpha(te, one_euro_mincutoff + one_euro_beta * fabsf(oe_dx)) * ((float)x - oe_x);
                        oe_y += oe_alpha(te, one_euro_mincutoff + one_euro_beta * fabsf(oe_dy)) * ((float)y - oe_y);
                    }

                    if (one_euro_rest_clamp && rc_anchored) {
                        const float ex = oe_x - rc_ax;
                        const float ey = oe_y - rc_ay;
                        if (ex * ex + ey * ey > ONE_EURO_REST_RADIUS * ONE_EURO_REST_RADIUS) {
                            /* breakout: hand the anchored displacement to the
                             * replay drain - nothing is ever swallowed */
                            rc_anchored    = false;
                            rc_still_since = 0;
                            rc_replay_x    = ex;
                            rc_replay_y    = ey;
#ifdef MAXTOUCH_EVENT_TRACE
                            uprintf("BRK %d,%d\n", (int)ex, (int)ey);
#endif
                        } else {
                            /* absorb: emit nothing, let the anchor slowly
                             * follow sensor walks so they never accumulate
                             * into a spurious breakout */
                            if (timer_elapsed32(rc_leak_t) >= ONE_EURO_REST_LEAK_MS) {
                                rc_leak_t = now;
                                if (ex > 0.5f) rc_ax += 1.0f; else if (ex < -0.5f) rc_ax -= 1.0f;
                                if (ey > 0.5f) rc_ay += 1.0f; else if (ey < -0.5f) rc_ay -= 1.0f;
                            }
                            oe_out_x = oe_x;
                            oe_out_y = oe_y;
                        }
                    }
                    if (!(one_euro_rest_clamp && rc_anchored)) {
                        /* re-arm only when the slow signed velocity says the
                         * finger is genuinely still - a slow coherent drag
                         * keeps this high, so it cannot quantize mid-drag */
                        if (fabsf(oe_vsx) + fabsf(oe_vsy) < ONE_EURO_REST_V_THRESH) {
                            if (rc_still_since == 0) {
                                rc_still_since = now | 1;
                            } else if (timer_elapsed32(rc_still_since) > ONE_EURO_REST_MS) {
                                rc_anchored    = true;
                                rc_ax          = oe_x;
                                rc_ay          = oe_y;
                                rc_leak_t      = now;
                                rc_replay_x    = 0.0f;
                                rc_replay_y    = 0.0f;
#ifdef MAXTOUCH_EVENT_TRACE
                                uprintf("ANC %d,%d\n", (int)oe_x, (int)oe_y);
#endif
                            }
                        } else {
                            rc_still_since = 0;
                        }
                        oe_drain_replay(&rc_replay_x, oe_x - oe_out_x, &oe_out_x);
                        oe_drain_replay(&rc_replay_y, oe_y - oe_out_y, &oe_out_y);
                    }

                    /* HID delta = motion of the FILTERED position. Emit the
                     * integer part, retain the sub-unit remainder so slow
                     * drags accumulate instead of truncating to nothing. */
                    const int odx = (int)(oe_x - oe_out_x);
                    const int ody = (int)(oe_y - oe_out_y);
                    oe_out_x += (float)odx;
                    oe_out_y += (float)ody;
                    mouse_report.x = odx;
                    mouse_report.y = ody;
#ifdef MAXTOUCH_EVENT_TRACE
                    if (odx || ody) uprintf("OUT %d %d\n", odx, ody);
#endif
                }
#endif
            } else if (contacts == 3 && duration < DIGITIZER_MOUSE_SWIPE_TIMEOUT) {
                state = Swipe;
            } else {
                static int carry_h = 0;
                static int carry_v = 0;
                const int  h       = x - last_x + carry_h;
                const int  v       = y - last_y + carry_v;

                carry_h = h % DIGITIZER_SCROLL_DIVISOR;
                carry_v = v % DIGITIZER_SCROLL_DIVISOR;

                mouse_report.h = h / DIGITIZER_SCROLL_DIVISOR;
                mouse_report.v = v / DIGITIZER_SCROLL_DIVISOR;
            }
            break;
        }
        case DoubleTapped:
        case Tapped: {
            tap_contacts = MAX(contacts, tap_contacts);
            if (contacts == 0 && last_contacts != contacts) {
                tap_count++;
                state              = DoubleTapped;
                contact_start_time = timer_read32();
            } else if (duration > DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT) {
                if (contacts > 0 && state == Tapped) {
                    state = Drag;
                } else {
                    tap_count++;
                    state = Finished;
                }
            }
            break;
        }
        case Swipe: {
            const int32_t distance_x = x - contact_start_x;
            const int32_t distance_y = y - contact_start_y;
            if (contacts == 0) {
                state = None;
            } else if (duration > DIGITIZER_MOUSE_SWIPE_TIMEOUT) {
                state = MoveScroll;
            } else if (force_digitizer_send_mouse_reports || digitizer_send_mouse_reports) {
                if (distance_x > DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_y) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe right
                    tap_code(DIGITIZER_SWIPE_RIGHT_KC);
                    state = Finished;
                } else if (distance_x < -DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_y) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe left
                    tap_code(DIGITIZER_SWIPE_LEFT_KC);
                    state = Finished;
                } else if (distance_y > DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_x) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe down
                    tap_code(DIGITIZER_SWIPE_DOWN_KC);
                    state = Finished;
                } else if (distance_y < -DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_x) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe up
                    tap_code(DIGITIZER_SWIPE_UP_KC);
                    state = Finished;
                }
            }
            break;
        }
        case Finished: {
            if (contacts == 0) {
                state = None;
            }
            break;
        }
    }
    static bool     tap      = false;
    static uint32_t tap_time = 0;
    if (tap_count) {
        if (timer_elapsed32(tap_time) > DIGITIZER_MOUSE_TAP_DURATION) {
            tap = !tap;
            if (!tap) {
                tap_count--;
            }
            tap_time = timer_read32();
        }
    }
    const bool button_pressed = tap || (state == Drag);
    if (report->button1 || (tap_contacts == 1 && button_pressed)) {
        mouse_report.buttons |= 0x1;
        if (digitizer_taps_as_clicks) report->button1 = 1;
    }
    if (report->button2 || (tap_contacts == 2 && button_pressed)) {
        mouse_report.buttons |= 0x2;
        if (digitizer_taps_as_clicks) report->button2 = 1;
    }
    if (report->button3 || (tap_contacts == 3 && button_pressed)) {
        mouse_report.buttons |= 0x4;
        if (digitizer_taps_as_clicks) report->button3 = 1;
    }
    last_contacts = contacts;
    last_x        = x;
    last_y        = y;
}
#endif
