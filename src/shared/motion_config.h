// ============================================================
//  motion_config.h
//  Skycart Delivery Rack — Shared Hardware & Geometry Constants
//
//  Single source of truth for both stepper_test and
//  teeter_totter firmware builds.
// ============================================================
#pragma once

// ── Leadscrew + microstepping ────────────────────────────────
// 4-start screw, 2 mm pitch -> lead = 8.0 mm/rev = 0.3150 in/rev
// 1/8 microstepping: 1600 steps/rev
// -> 1600 / 0.3150 ~ 5080 steps/inch
static constexpr float STEPS_PER_INCH = 5080.0f;

// ── Stepper test — pins ──────────────────────────────────────
#define STEP_PIN    26
#define DIR_PIN     25
#define ENABLE_PIN  27

// ── Teeter-totter — pins ─────────────────────────────────────
#define STEP_PIN_L    26
#define DIR_PIN_L     25
#define ENABLE_PIN_L  27
#define STEP_PIN_R    18   // <- confirm with wiring
#define DIR_PIN_R     19
#define ENABLE_PIN_R  23

// ── Trapdoor servo — pins ────────────────────────────────────
#define SERVO_L_PIN   13   // <- confirm with wiring
#define SERVO_R_PIN   14   // <- confirm with wiring

// ── Motion tuning ────────────────────────────────────────────
// 24 V: 10,000 Hz ~ 0.98 in/s
// 12 V: 5,000 Hz ~ 0.49 in/s
static constexpr uint32_t TEST_SPEED_HZ    = 10000;
static constexpr uint32_t TEST_ACCEL       = 3250;
static constexpr uint32_t BALANCE_SPEED_HZ = 5000;
static constexpr uint32_t BALANCE_ACCEL    = 3250;

// ── Trapdoor servo tuning ────────────────────────────────────
static constexpr int SERVO_CLOSED_DEG      = 0;     // <- calibrate on bench
static constexpr int SERVO_OPEN_DEG        = 90;    // <- calibrate on bench
static constexpr unsigned long DROP_DELAY_MS = 500;  // time for box to fall through

// ── Rack geometry ────────────────────────────────────────────
static constexpr float RACK_LENGTH_IN   = 25.0f;
static constexpr float SUPPORT_POINT_IN = 12.5f;   // <- update from Nimbus v0.7D
static constexpr float RACK_MASS_KG     = 1.5f;    // <- weigh the rack
static constexpr float RACK_CG_IN       = RACK_LENGTH_IN / 2.0f;

// ── Box and pusher geometry ──────────────────────────────────
static constexpr int   NUM_BOXES        = 3;
static constexpr float BOX_WIDTH_IN     = 6.0f;
static constexpr float BOX_HALF_IN      = BOX_WIDTH_IN / 2.0f;
static constexpr float PUSHER_WIDTH_IN  = 2.0f;
static constexpr float HOLE_POS_IN      = 12.5f;

// ── Pusher travel limits ─────────────────────────────────────
// pL = inner face of left pusher (contacts leftmost box)
// pR = inner face of right pusher (contacts rightmost box)
static constexpr float PUSHER_L_MIN     = PUSHER_WIDTH_IN;                    // 2.0"
static constexpr float PUSHER_R_MAX     = RACK_LENGTH_IN - PUSHER_WIDTH_IN;   // 23.0"

// ── Pusher home positions ────────────────────────────────────
// Box 1 (rank 1, center) aligned over the drop hole at startup.
//   box1 CG = pL + 1.5 * BOX_WIDTH = HOLE_POS
//   pL      = HOLE_POS - 9
//   pR      = pL + 3 * BOX_WIDTH
static constexpr float PUSHER_L_HOME    = HOLE_POS_IN - 1.5f * BOX_WIDTH_IN;
static constexpr float PUSHER_R_HOME    = PUSHER_L_HOME + NUM_BOXES * BOX_WIDTH_IN;
