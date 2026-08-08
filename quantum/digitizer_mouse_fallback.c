// Copyright 2025 George Norton (@george-norton)
// SPDX-License-Identifier: GPL-2.0-or-later

#if defined(POINTING_DEVICE_DRIVER_digitizer)

// We can fallback to reporting as a mouse for hosts which do not implement trackpad support.

#    include <stdlib.h>
#    include <math.h>
#    include "quantum.h"
#    include "digitizer.h"
#    include "digitizer_mouse_fallback.h"
#    include "debug.h"
#    include "print.h"
#    include "timer.h"
#    include "action.h"

#    ifndef DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT
#        define DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT 150
#    endif

/* Two-finger tap judged at lift: window measured from when the PAIR
 * formed (not the first finger), so a staggered second finger still
 * right-clicks. Wider than the one-finger window, like Apple. */
#    ifndef DIGITIZER_MOUSE_TAP_SALVAGE_MS
#        define DIGITIZER_MOUSE_TAP_SALVAGE_MS 250
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

#    ifndef DIGITIZER_SCROLL_INTERVAL_MS
#        define DIGITIZER_SCROLL_INTERVAL_MS 50 /* emission tick; the divisor, not the tick, sets the speed */
#    endif

/* Fingers of a two-finger gesture never land or lift simultaneously.
 * Pointer output is suppressed briefly after touch-down (the second
 * finger of a scroll may still be coming) and after a scroll ends (the
 * remaining finger is lifting, not pointing). Suppressed travel is not
 * lost - it rides out through the rest anchor's breakout replay. */
#    ifndef DIGITIZER_MOUSE_TOUCH_GRACE_MS
#        define DIGITIZER_MOUSE_TOUCH_GRACE_MS 50
#    endif

#    ifndef DIGITIZER_MOUSE_SCROLL_END_GRACE_MS
#        define DIGITIZER_MOUSE_SCROLL_END_GRACE_MS 150
#    endif

/* Close fingers periodically MERGE into one tracker contact (electrode
 * pitch; measured ~15% of frames even at tuned gain). A merge during an
 * engaged scroll must not stall it: route the lone blob through the
 * scroll path while a pair was seen recently - the blob's centroid
 * tracks the fingers, and the finger-count resync absorbs the 2->1
 * centroid jump. Window unrefreshed: a true single finger regains the
 * pointer after this long. */
#    ifndef DIGITIZER_MOUSE_MERGE_SCROLL_MS
#        define DIGITIZER_MOUSE_MERGE_SCROLL_MS 800
#    endif

/* Pointer output scale: fraction of filtered motion emitted as HID
 * counts (percent). Sub-count remainders accumulate, so precision is
 * unaffected - only speed. Scroll has its own divisor. */
#    ifndef DIGITIZER_MOUSE_POINTER_SCALE_PCT
#        define DIGITIZER_MOUSE_POINTER_SCALE_PCT 100
#    endif

/* Rapid scroll-spinning: fingers lift and re-land in rhythm, and a
 * re-landing close pair often registers MERGED (one contact) - which
 * used to reset the gesture and slip. A touch landing while the
 * scroll rhythm is warm and near the previous scroll position is the
 * scroll continuing, whatever the contact count says. */
#    ifndef DIGITIZER_MOUSE_RESCROLL_MS
#        define DIGITIZER_MOUSE_RESCROLL_MS 500
#    endif
#    ifndef DIGITIZER_MOUSE_RESCROLL_UNITS
#        define DIGITIZER_MOUSE_RESCROLL_UNITS 350
#    endif

/* Scroll emission is LINEAR: travel/divisor clicks per tick, magnitude
 * intact, remainder carried - designed for a host-side pixel renderer
 * (each click = fixed pixels) that keeps the OS's per-click line
 * inflation and rate acceleration out of the path. The accumulator is
 * clamped so a backlog can never outlive the gesture: a flick's excess
 * speed lives on as coast velocity instead of queued clicks. */

/* Teleport rejection: while a second finger descends, the sensor's
 * tracker sometimes snaps an existing contact to the incoming finger's
 * position and back within single reports (measured ping-pong of
 * 170-640 units; real flicks stay under ~40 units/report). Such a
 * delta is physically impossible finger motion - reseed, never emit. */
#    ifndef DIGITIZER_MOUSE_TELEPORT_UNITS
#        define DIGITIZER_MOUSE_TELEPORT_UNITS 100
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
 * Rest is detected from the FILTERED position going nowhere: a candidate
 * point plus a still timer. A slow coherent drag keeps outrunning the
 * still radius and resetting the candidate, so it cannot re-arm and
 * quantize mid-drag; a genuine stop arms within ~half a second. On
 * breakout the anchored displacement is replayed motion-masked, never
 * swallowed. */
#    ifndef ONE_EURO_REST_RADIUS
#        define ONE_EURO_REST_RADIUS 18.0f /* post-scale units the filtered position may wander while anchored */
#    endif

#    ifndef ONE_EURO_REST_STILL_RADIUS
#        define ONE_EURO_REST_STILL_RADIUS 6.0f /* post-scale units of drift allowed while judging stillness */
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

// Natural (content-follows-finger) scroll direction, as on Apple
// trackpads. Runtime-toggleable; keymaps can default it from OS detection.
#    ifdef DIGITIZER_NATURAL_SCROLL
bool digitizer_natural_scroll = true;
#    else
bool digitizer_natural_scroll = false;
#    endif

// Scroll sensitivity. The divisor sets wheel-click granularity; the
// INTERVAL is the perceived-speed lever on macOS, which weighs scroll
// by event rate and largely ignores magnitude (measured: 120x magnitude
// changes imperceptible, rate changes always felt). Both runtime.
uint8_t  digitizer_scroll_divisor     = DIGITIZER_SCROLL_DIVISOR;
uint16_t digitizer_scroll_interval_ms = DIGITIZER_SCROLL_INTERVAL_MS;

// Swipe dispatch, keymap-overridable (see digitizer_mouse_fallback.h).
__attribute__((weak)) void digitizer_swipe_action(digitizer_swipe_dir_t dir) {
    switch (dir) {
        case DIGITIZER_SWIPE_DIR_RIGHT: tap_code16(DIGITIZER_SWIPE_RIGHT_KC); break;
        case DIGITIZER_SWIPE_DIR_LEFT: tap_code16(DIGITIZER_SWIPE_LEFT_KC); break;
        case DIGITIZER_SWIPE_DIR_DOWN: tap_code16(DIGITIZER_SWIPE_DOWN_KC); break;
        case DIGITIZER_SWIPE_DIR_UP: tap_code16(DIGITIZER_SWIPE_UP_KC); break;
    }
}

/* With hi-res scrolling the host treats wheel units as 1/RESOLUTION of
 * a detent, so finger travel is scaled up by the resolution instead of
 * being crushed into whole detents - the host then pixel-scrolls
 * smoothly and the divisor becomes a true linear speed knob. */
#    ifdef POINTING_DEVICE_HIRES_SCROLL_ENABLE
#        include "usb_descriptor_common.h"
#        define DIGITIZER_SCROLL_SCALE POINTING_DEVICE_HIRES_SCROLL_MULTIPLIER
#    else
#        define DIGITIZER_SCROLL_SCALE 1
#    endif

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

static State state           = None;
static int   tap_count       = 0;
static bool  scroll_coasting = false;

/**
 * \brief Signals that a gesture is in progress so digitizer_update_mouse_report should be called,
 * even if no new digitizer data is available.
 * @return true if update_mouse_report should run.
 */
bool digitizer_update_gesture_state(void) {
    return tap_count || state != None || scroll_coasting;
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
    static float    oe_out_x = 0.0f, oe_out_y = 0.0f; /* last emitted position */
    static int      scroll_acc_h = 0, scroll_acc_v = 0;
    static uint32_t scroll_click_t = 0; /* time of the previous emitted click */
    static uint32_t scroll_last_t = 0;  /* last time the 2-finger scroll branch ran */
    static bool     scroll_touched = false;
    static bool     scroll_clicked = false; /* did this gesture emit any scroll? */
    static uint32_t two_seen_t = 0; /* last time two or more contacts were present */
    static uint32_t pair_t     = 0; /* when the contact count last rose to two */
    static int      prev_n     = 0; /* contact count on the previous report */
    static int      sc_cx = 0, sc_cy = 0; /* previous scroll centroid */
    static int      sc_n = 0;             /* finger count it was valid for */
    static bool     scr_anch = false;     /* scroll rest anchor (wiggle absorber) */
    static bool     scr_esc = false;      /* anchor already escaped this gesture */
    static int      scr_ax = 0, scr_ay = 0;
    static int      scr_cand_x = 0, scr_cand_y = 0;
    static uint32_t scr_still_t = 0;
    static int      coast_v16h = 0, coast_v16v = 0; /* scroll velocity, clicks/s x16 */
    static uint32_t tri_t = 0; /* when a third contact first appeared */
    static int      down2_cx = 0, down2_cy = 0; /* centroid at 2-finger placement */
    static bool     down2_set = false;
    static bool     rc_anchored = false;
    static float    rc_ax = 0.0f, rc_ay = 0.0f;       /* rest anchor */
    static float    rc_cand_x = 0.0f, rc_cand_y = 0.0f; /* stillness candidate */
    static uint32_t rc_still_since = 0;
    static uint32_t rc_leak_t = 0;
    static float    rc_replay_x = 0.0f, rc_replay_y = 0.0f;
    static uint16_t last_x             = 0;
    static uint16_t last_y             = 0;
    uint16_t  x                        = 0;
    uint16_t  y                        = 0;
    bool      scroll_branch_live       = false; /* this call routed a contact through the scroll path */
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

    int cx_sum = 0;
    int cy_sum = 0;
    for (int i = 0; i < DIGITIZER_FINGER_COUNT; i++) {
        if (report->fingers[i].tip) {
            cx_sum += report->fingers[i].x;
            cy_sum += report->fingers[i].y;
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

    /* 3-tap median on the tracked position: deletes single-report
     * position impulses outright (measured X snaps of 100-1800 units on
     * a still finger, environment-dependent) which the low-pass filter
     * would smear into visible wander and the teleport gate only
     * catches above 100 units. Costs one sensor report (~8ms). */
    {
        static uint16_t med_x[3], med_y[3];
        static uint8_t  med_n = 0;
        if (contacts == 1) {
            med_x[0] = med_x[1]; med_x[1] = med_x[2]; med_x[2] = x;
            med_y[0] = med_y[1]; med_y[1] = med_y[2]; med_y[2] = y;
            if (med_n < 3) med_n++;
            if (med_n == 3) {
#define MED3(a, b, c) MAX(MIN(a, b), MIN(MAX(a, b), c))
                x = MED3(med_x[0], med_x[1], med_x[2]);
                y = MED3(med_y[0], med_y[1], med_y[2]);
#undef MED3
            }
        } else {
            med_n = 0;
        }
    }

    /* A third contact only counts as a swipe when SUSTAINED: heavy slow
     * fingers momentarily split into phantom contacts (single-report
     * blips) that must never fire Spaces switches mid-scroll. */
    if (contacts == 3) {
        if (tri_t == 0) tri_t = timer_read32() | 1;
    } else {
        tri_t = 0;
    }
    if (contacts >= 2) two_seen_t = timer_read32() | 1;
    if (contacts >= 2 && prev_n < 2) pair_t = timer_read32() | 1;
    prev_n = contacts;
    const bool tri_sustained = tri_t != 0 && timer_elapsed32(tri_t) > 40;

    switch (state) {
        case None: {
            oe_init   = false;
            sc_n      = 0;
            down2_set = false;
            if (contacts != 0) {
                /* a new touch cancels any scroll momentum */
                scroll_acc_h       = 0;
                scroll_acc_v       = 0;
                coast_v16h         = 0;
                coast_v16v         = 0;
                scroll_coasting    = false;
                scroll_clicked     = false;
                scr_esc            = false;
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
            /* Apple-style stagger tolerance: the tap window restarts when
             * another finger lands, so a slightly-delayed second finger
             * still yields a clean two-finger tap. */
            if (contacts > tap_contacts) {
                contact_start_time = timer_read32();
            }
            tap_contacts              = MAX(contacts, tap_contacts);

            if (contacts == 0) {
                state              = Tapped;
                contact_start_time = timer_read32();
                uprintf("TAPJ down tc=%d dur=%lu -> tap\n", tap_contacts, duration);
            } else if (contacts > 3 || (contacts == 3 && tri_sustained)) {
                state = Swipe;
            } else if (contacts == 2) {
                /* Two fingers: scroll must respond the moment they move -
                 * judged by the CENTROID (immune to tracker index swaps),
                 * while a motionless pair can still become a tap. */
                const int cx = cx_sum / contacts;
                const int cy = cy_sum / contacts;
                if (!down2_set) {
                    down2_set = true;
                    down2_cx  = cx;
                    down2_cy  = cy;
                }
                if (abs(cx - down2_cx) > 14 || abs(cy - down2_cy) > 14 || duration > DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT) {
                    state = MoveScroll;
                }
            } else if (duration > DIGITIZER_MOUSE_TAP_DETECTION_TIMEOUT || distance_x > DIGITIZER_MOUSE_TAP_DISTANCE || distance_y > DIGITIZER_MOUSE_TAP_DISTANCE) {
                state = MoveScroll;
            }
            break;
        }
        case Drag:
        case MoveScroll: {
            /* a second finger arriving after the Down state has passed
             * must still count toward the tap-at-lift judgement */
            tap_contacts = MAX(contacts, tap_contacts);
            if (contacts == 0) {
                const State prior = state;
                state             = None;
                /* coast only engages when the fingers left MID-MOTION:
                 * a gesture that stopped before lifting has a stale last
                 * click and must leave no momentum */
                if (timer_elapsed32(scroll_click_t) > 250) {
                    coast_v16h = 0;
                    coast_v16v = 0;
                }
                /* Tap salvage: two fingers LANDING wobble enough to get
                 * promoted to MoveScroll, which used to silently eat the
                 * tap. Judge at lift, like Apple: brief touch, nothing
                 * scrolled - it was a right-click tap all along. */
                if (prior == MoveScroll && tap_contacts == 2 && !scroll_clicked && pair_t != 0 && timer_elapsed32(pair_t) < DIGITIZER_MOUSE_TAP_SALVAGE_MS) {
                    state              = Tapped;
                    contact_start_time = timer_read32();
                }
                uprintf("TAPJ lift tc=%d sc=%d dur=%lu pe=%lu -> %s\n", tap_contacts, (int)scroll_clicked, duration, pair_t ? timer_elapsed32(pair_t) : 0, state == Tapped ? "tap" : "miss");
            } else if (contacts == 1 && !scr_esc && !(tap_contacts >= 2 && duration < 500 && two_seen_t != 0 && timer_elapsed32(two_seen_t) < DIGITIZER_MOUSE_MERGE_SCROLL_MS) && !(timer_elapsed32(scroll_click_t) < DIGITIZER_MOUSE_RESCROLL_MS && abs((int)x - sc_cx) < DIGITIZER_MOUSE_RESCROLL_UNITS && abs((int)y - sc_cy) < DIGITIZER_MOUSE_RESCROLL_UNITS)) {
                /* scr_esc alone is decisive: an ENGAGED scroll stays a
                 * scroll until every finger lifts, like Apple - a touch
                 * never mutates into pointer motion mid-gesture, however
                 * long the fingers stay merged. */
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
                     * discontinuous, do not slew across it. Returning from
                     * a scroll always reseeds - the tracked finger may have
                     * swapped, however short the scroll was. */
                    if (oe_init && ((uint32_t)(now - oe_last_run) > 100 || scroll_touched)) {
                        oe_init = false;
                    }
                    scroll_touched = false;
                    /* Teleport rejection (see define): a ping-pong snap
                     * reseeds on every hop and therefore emits nothing. */
                    if (oe_init && (abs((int)x - (int)last_x) > DIGITIZER_MOUSE_TELEPORT_UNITS || abs((int)y - (int)last_y) > DIGITIZER_MOUSE_TELEPORT_UNITS)) {
                        oe_init = false;
#ifdef MAXTOUCH_EVENT_TRACE
                        uprintf("TELE %d,%d\n", (int)x - (int)last_x, (int)y - (int)last_y);
#endif
                    }
                    oe_last_run = now;
                    if (!oe_init) {
                        oe_init     = true;
                        oe_x        = (float)x;
                        oe_y        = (float)y;
                        oe_dx       = 0.0f;
                        oe_dy       = 0.0f;
                        oe_out_x    = oe_x;
                        oe_out_y    = oe_y;
                        oe_t_last   = now;
                        /* start at rest: anchored at the touch point */
                        rc_anchored    = true;
                        rc_ax          = oe_x;
                        rc_ay          = oe_y;
                        rc_cand_x      = oe_x;
                        rc_cand_y      = oe_y;
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

                        oe_x += oe_alpha(te, one_euro_mincutoff + one_euro_beta * fabsf(oe_dx)) * ((float)x - oe_x);
                        oe_y += oe_alpha(te, one_euro_mincutoff + one_euro_beta * fabsf(oe_dy)) * ((float)y - oe_y);
                    }

                    /* While a second finger may still land (touch grace) or
                     * has just lifted (scroll-end grace), the anchor may not
                     * break: a scroll must never begin or end with a pointer
                     * jump. Travel accumulated during grace replays through
                     * the normal motion-masked breakout afterward. */
                    const bool tf_grace = duration < DIGITIZER_MOUSE_TOUCH_GRACE_MS || timer_elapsed32(scroll_last_t) < DIGITIZER_MOUSE_SCROLL_END_GRACE_MS || (two_seen_t != 0 && timer_elapsed32(two_seen_t) < DIGITIZER_MOUSE_SCROLL_END_GRACE_MS);
                    if (one_euro_rest_clamp && rc_anchored) {
                        const float ex = oe_x - rc_ax;
                        const float ey = oe_y - rc_ay;
                        if (!tf_grace && ex * ex + ey * ey > ONE_EURO_REST_RADIUS * ONE_EURO_REST_RADIUS) {
                            /* breakout: hand the anchored displacement to the
                             * replay drain - nothing is ever swallowed */
                            rc_anchored    = false;
                            rc_cand_x      = oe_x;
                            rc_cand_y      = oe_y;
                            rc_still_since = now | 1;
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
                        /* re-arm when the FILTERED position stops going
                         * anywhere: within the still radius of the candidate
                         * point for the rest interval. A slow coherent drag
                         * keeps outrunning the radius and resetting the
                         * candidate, so it cannot quantize mid-drag. */
                        const float cdx = oe_x - rc_cand_x;
                        const float cdy = oe_y - rc_cand_y;
                        if (cdx * cdx + cdy * cdy > ONE_EURO_REST_STILL_RADIUS * ONE_EURO_REST_STILL_RADIUS || rc_still_since == 0) {
                            rc_cand_x      = oe_x;
                            rc_cand_y      = oe_y;
                            rc_still_since = now | 1;
                        } else if (timer_elapsed32(rc_still_since) > ONE_EURO_REST_MS) {
                            rc_anchored = true;
                            rc_ax       = oe_x;
                            rc_ay       = oe_y;
                            rc_leak_t   = now;
                            rc_replay_x = 0.0f;
                            rc_replay_y = 0.0f;
#ifdef MAXTOUCH_EVENT_TRACE
                            uprintf("ANC %d,%d\n", (int)oe_x, (int)oe_y);
#endif
                        }
                        oe_drain_replay(&rc_replay_x, oe_x - oe_out_x, &oe_out_x);
                        oe_drain_replay(&rc_replay_y, oe_y - oe_out_y, &oe_out_y);
                    }

                    /* HID delta = motion of the FILTERED position. Emit the
                     * integer part, retain the sub-unit remainder so slow
                     * drags accumulate instead of truncating to nothing. */
                    const float pscale = DIGITIZER_MOUSE_POINTER_SCALE_PCT / 100.0f;
                    const int odx = (int)((oe_x - oe_out_x) * pscale);
                    const int ody = (int)((oe_y - oe_out_y) * pscale);
                    oe_out_x += (float)odx / pscale;
                    oe_out_y += (float)ody / pscale;
                    mouse_report.x = odx;
                    mouse_report.y = ody;
#ifdef MAXTOUCH_EVENT_TRACE
                    if (odx || ody) uprintf("OUT %d %d\n", odx, ody);
#endif
                }
#endif
            } else if (contacts == 3 && tri_sustained && duration < DIGITIZER_MOUSE_SWIPE_TIMEOUT) {
                state = Swipe;
            } else {
                scroll_branch_live = true;
                scroll_last_t  = timer_read32();
                scroll_touched = true;
                /* Scroll follows the CENTROID of the fingers: the tracker
                 * recycles contact ids mid-gesture, so any single finger's
                 * slot can swap identity and teleport - the midpoint is
                 * invariant to that. Resync without a delta whenever the
                 * finger count changes (the centroid legitimately jumps). */
                const int cx = cx_sum / contacts;
                const int cy = cy_sum / contacts;
                if (sc_n != contacts) {
                    sc_n        = contacts;
                    sc_cx       = cx;
                    sc_cy       = cy;
                    /* fresh gesture starts ANCHORED (wiggle absorber),
                     * EXCEPT during rapid successive scrolling, and never
                     * re-arms mid-gesture once escaped (count flapping) */
                    scr_anch    = timer_elapsed32(scroll_click_t) > 500 && !scr_esc;
                    scr_ax      = cx;
                    scr_ay      = cy;
                    scr_still_t = 0;
                }
                if (scr_anch) {
                    const int adx = cx - scr_ax;
                    const int ady = cy - scr_ay;
                    if (adx * adx + ady * ady > 25 * 25) {
                        scr_anch = false; /* deliberate travel: scroll begins here */
                        scr_esc  = true;
                    }
                } else {
                    /* re-arm on stillness so a held pause absorbs wiggle again */
                    if (abs(cx - scr_cand_x) > 6 || abs(cy - scr_cand_y) > 6) {
                        scr_cand_x  = cx;
                        scr_cand_y  = cy;
                        scr_still_t = timer_read32() | 1;
                    } else if (scr_still_t != 0 && timer_elapsed32(scr_still_t) > 300) {
                        scr_anch     = true;
                        scr_ax       = cx;
                        scr_ay       = cy;
                        scroll_acc_h = 0;
                        scroll_acc_v = 0;
                        coast_v16h   = 0;
                        coast_v16v   = 0;
                    }
                }
                if (!scr_anch) {
                    scroll_acc_h += (cx - sc_cx) * DIGITIZER_SCROLL_SCALE;
                    scroll_acc_v += (cy - sc_cy) * DIGITIZER_SCROLL_SCALE;
                    /* a MERGED blob that keeps travelling keeps the scroll
                     * alive: control returns to the pointer on stillness
                     * or lift, never mid-motion (no cursor jumping out of
                     * a close-finger scroll) */
                    if (contacts == 1 && (abs(cx - sc_cx) > 2 || abs(cy - sc_cy) > 2)) two_seen_t = timer_read32() | 1;
                }
                sc_cx = cx;
                sc_cy = cy;
                /* Clamp the backlog: a fast flick's speed is preserved as
                 * VELOCITY (below), never as a queue of stale clicks. */
                const int acc_max = 6 * (int)digitizer_scroll_divisor;
                if (scroll_acc_h > acc_max) scroll_acc_h = acc_max;
                if (scroll_acc_h < -acc_max) scroll_acc_h = -acc_max;
                if (scroll_acc_v > acc_max) scroll_acc_v = acc_max;
                if (scroll_acc_v < -acc_max) scroll_acc_v = -acc_max;

                /* Emit the travel the fingers earned this tick, scaled by
                 * the divisor, magnitude intact (the host-side pixel
                 * conversion honors it) - speed is linear in finger speed
                 * at every speed, remainder carries, no rate ceiling. */
                if (timer_elapsed32(scroll_click_t) >= digitizer_scroll_interval_ms) {
                    int sh = scroll_acc_h / (int)digitizer_scroll_divisor;
                    int sv = scroll_acc_v / (int)digitizer_scroll_divisor;
                    if (sh > 100) sh = 100;
                    if (sh < -100) sh = -100;
                    if (sv > 100) sv = 100;
                    if (sv < -100) sv = -100;
                    /* axis lock on the accumulated travel */
                    if (abs(scroll_acc_v) >= 2 * abs(scroll_acc_h)) sh = 0;
                    else if (abs(scroll_acc_h) >= 2 * abs(scroll_acc_v)) sv = 0;
                    if (sh || sv) {
                        scroll_clicked = true;
                        scroll_acc_h -= sh * (int)digitizer_scroll_divisor;
                        scroll_acc_v -= sv * (int)digitizer_scroll_divisor;
                        /* velocity in clicks/s x16, from clicks over the gap
                         * since the previous emission */
                        const uint32_t nowc = timer_read32();
                        uint32_t       gap  = nowc - scroll_click_t;
                        if (gap < digitizer_scroll_interval_ms) gap = digitizer_scroll_interval_ms;
                        if (gap > 1000) gap = 1000;
                        scroll_click_t = nowc;
                        coast_v16h     = (sh * 16000) / (int)gap;
                        coast_v16v     = (sv * 16000) / (int)gap;
                        /* the coast emits single clicks on a rhythm - cap
                         * its launch speed at ~60 clicks/s */
                        if (coast_v16h > 992) coast_v16h = 992;
                        if (coast_v16h < -992) coast_v16h = -992;
                        if (coast_v16v > 992) coast_v16v = 992;
                        if (coast_v16v < -992) coast_v16v = -992;
                        mouse_report.h = digitizer_natural_scroll ? -sh : sh;
                        mouse_report.v = digitizer_natural_scroll ? -sv : sv;
#ifdef MAXTOUCH_EVENT_TRACE
                        uprintf("SCRL div=%u sh=%d sv=%d\n", digitizer_scroll_divisor, sh, sv);
#endif
                    }
                }
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
                    digitizer_swipe_action(DIGITIZER_SWIPE_DIR_RIGHT);
                    state = Finished;
#ifdef MAXTOUCH_EVENT_TRACE
                    uprintf("SWIPE right\n");
#endif
                } else if (distance_x < -DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_y) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe left
                    digitizer_swipe_action(DIGITIZER_SWIPE_DIR_LEFT);
                    state = Finished;
#ifdef MAXTOUCH_EVENT_TRACE
                    uprintf("SWIPE left\n");
#endif
                } else if (distance_y > DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_x) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe down
                    digitizer_swipe_action(DIGITIZER_SWIPE_DIR_DOWN);
                    state = Finished;
#ifdef MAXTOUCH_EVENT_TRACE
                    uprintf("SWIPE down\n");
#endif
                } else if (distance_y < -DIGITIZER_MOUSE_SWIPE_DISTANCE && abs(distance_x) < DIGITIZER_MOUSE_SWIPE_THRESHOLD) {
                    // Swipe up
                    digitizer_swipe_action(DIGITIZER_SWIPE_DIR_UP);
                    state = Finished;
#ifdef MAXTOUCH_EVENT_TRACE
                    uprintf("SWIPE up\n");
#endif
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
    /* Kinetic scroll (fling): after liftoff the captured click rhythm
     * continues - single clicks whose gaps stretch as friction (Apple's
     * 0.998/ms, fixed-point per elapsed gap) bleeds the velocity, until
     * the rhythm falls under ~3 clicks/s. The tail literally ends line
     * by line. A new touch cancels it (see state None). */
    /* Momentum also carries UNDER a lone leftover finger: lifting one
     * finger of a scrolling pair must not kill the fling (Apple keeps
     * it). The shared rhythm clock arbitrates - a moving finger's own
     * clicks defer the coast automatically. */
    if ((state == None && contacts == 0) || (scroll_branch_live && contacts == 1)) {
        const int vmag = MAX(abs(coast_v16h), abs(coast_v16v));
        if (vmag >= 48) {
            scroll_coasting = true;
            if (timer_elapsed32(scroll_click_t) >= 16000u / (uint32_t)vmag) {
                const uint32_t nowc = timer_read32();
                uint32_t       g    = nowc - scroll_click_t;
                if (g > 1000) g = 1000;
                scroll_click_t = nowc;
                int k = 256 - ((int)g * 512) / 1000;
                if (k < 64) k = 64;
                const int sh = coast_v16h > 0 ? 1 : (coast_v16h < 0 ? -1 : 0);
                const int sv = coast_v16v > 0 ? 1 : (coast_v16v < 0 ? -1 : 0);
                coast_v16h   = (coast_v16h * k) / 256;
                coast_v16v   = (coast_v16v * k) / 256;
                mouse_report.h = digitizer_natural_scroll ? -sh : sh;
                mouse_report.v = digitizer_natural_scroll ? -sv : sv;
#ifdef MAXTOUCH_EVENT_TRACE
                if (sh || sv) uprintf("COAST sh=%d sv=%d\n", sh, sv);
#endif
            }
        } else if (state == None) {
            scroll_coasting = false;
            coast_v16h      = 0;
            coast_v16v      = 0;
            scroll_acc_h    = 0;
            scroll_acc_v    = 0;
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
