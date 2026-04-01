#pragma once

// ============================================================
//  motion_config.h
//  Shared hardware constants for all Skycart ESP32 firmware.
//  Update values here — changes apply to both firmwares.
// ============================================================

// ── Leadscrew + microstepping ────────────────────────────────
// 4-start screw, 2 mm pitch → lead = 8.0 mm/rev = 0.3150 in/rev
// 1/8 microstepping (confirmed by Simon): 1600 steps/rev
// → 1600 / 0.3150 ≈ 5080 steps/inch
static constexpr float STEPS_PER_INCH = 5080.0f;

// ── Stepper test — pins ──────────────────────────────────────
#define STEP_PIN    26
#define DIR_PIN     25
#define ENABLE_PIN  27

// ── Teeter-totter — pins ─────────────────────────────────────
#define STEP_PIN_L    26
#define DIR_PIN_L     25
#define ENABLE_PIN_L  27
#define STEP_PIN_R    18   // ← confirm with your wiring
#define DIR_PIN_R     19
#define ENABLE_PIN_R  23

// ── Motion tuning ────────────────────────────────────────────
// 24 V validated safe limit: 10,000 Hz ≈ 0.98 in/s
// Drop to 5,000 Hz for 12 V operation
static constexpr uint32_t TEST_SPEED_HZ    = 10000;
static constexpr uint32_t TEST_ACCEL       = 3250;
static constexpr uint32_t BALANCE_SPEED_HZ = 5000;
static constexpr uint32_t BALANCE_ACCEL    = 3250;

// ── Rack geometry ────────────────────────────────────────────
static constexpr float RACK_LENGTH_IN   = 25.0f;

// Where the drone holds the rack (inches from left edge).
// ← Update from Nimbus v0.7D Design.xlsx before flight.
static constexpr float SUPPORT_POINT_IN = 12.5f;

// Rack mass and CG — assume symmetric until physically measured.
static constexpr float RACK_MASS_KG     = 1.5f;    // ← weigh the rack
static constexpr float RACK_CG_IN       = RACK_LENGTH_IN / 2.0f;

// Carriage travel limits (inches from left edge)
static constexpr float LEFT_CARRIAGE_MIN  =  1.0f;
static constexpr float LEFT_CARRIAGE_MAX  = 10.0f;
static constexpr float RIGHT_CARRIAGE_MIN = 15.0f;
static constexpr float RIGHT_CARRIAGE_MAX = 24.0f;

// Carriage home positions at startup
static constexpr float LEFT_HOME_IN  =  5.0f;
static constexpr float RIGHT_HOME_IN = 20.0f;
