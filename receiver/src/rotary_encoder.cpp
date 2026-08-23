// Quadrature state machine and transition tables derived from Ben Buxton's
// Rotary library (github.com/buxtronix/arduino), Copyright 2011 Ben Buxton, GPL v3.
// This file remains under GPL v3 — see README.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Arduino.h>
#pragma GCC diagnostic pop

#include "rotary_encoder.h"

// Emit flags carried in the state byte's high bits; low nibble is the table row.
constexpr uint8_t EMIT_CW = 0x10;
constexpr uint8_t EMIT_CCW = 0x20;
constexpr uint8_t EMIT_MASK = 0x30;
constexpr uint8_t ROW_MASK = 0x0F;

// Each row is a state; each column is the next (A,B) reading as an index
// (00, 01, 10, 11); the entry is the state to move to, optionally OR'd with an
// emit flag. Only the exact Gray sequence reaches an emitting entry; any invalid
// jump falls back to START, so bounce and EMI walk harmlessly between sub-states.

namespace fullstep {
    // Rest (and emit) at AB = 11: START ─01→ CW_BEGIN ─00→ CW_NEXT ─10→ CW_FINAL ─11→ START+emit
    constexpr uint8_t START = 0x0;
    constexpr uint8_t CW_FINAL = 0x1;
    constexpr uint8_t CW_BEGIN = 0x2;
    constexpr uint8_t CW_NEXT = 0x3;
    constexpr uint8_t CCW_BEGIN = 0x4;
    constexpr uint8_t CCW_FINAL = 0x5;
    constexpr uint8_t CCW_NEXT = 0x6;

    constexpr uint8_t TRANSITIONS[7][4] = {
        // 00            01          10          11
        {START,        CW_BEGIN,   CCW_BEGIN,  START},            // START
        {CW_NEXT,      START,      CW_FINAL,   START | EMIT_CW},  // CW_FINAL
        {CW_NEXT,      CW_BEGIN,   START,      START},            // CW_BEGIN
        {CW_NEXT,      CW_BEGIN,   CW_FINAL,   START},            // CW_NEXT
        {CCW_NEXT,     START,      CCW_BEGIN,  START},            // CCW_BEGIN
        {CCW_NEXT,     CCW_FINAL,  START,      START | EMIT_CCW}, // CCW_FINAL
        {CCW_NEXT,     CCW_FINAL,  CCW_BEGIN,  START},            // CCW_NEXT
    };
}

namespace halfstep {
    // Rest (and emit) at both AB = 11 and AB = 00. Suffix = which rest the
    // motion departed from: CW from 11 goes 11→01→00 (emit), CW from 00 goes
    // 00→10→11 (emit).
    constexpr uint8_t START_11 = 0x0;      // the 11 rest (row 0, like fullstep::START)
    constexpr uint8_t CCW_BEGIN_11 = 0x1;
    constexpr uint8_t CW_BEGIN_11 = 0x2;
    constexpr uint8_t START_00 = 0x3;      // the 00 rest
    constexpr uint8_t CW_BEGIN_00 = 0x4;
    constexpr uint8_t CCW_BEGIN_00 = 0x5;

    constexpr uint8_t TRANSITIONS[6][4] = {
        // 00                      01             10             11
        {START_00,               CW_BEGIN_11,   CCW_BEGIN_11,  START_11},            // START_11
        {START_00 | EMIT_CCW,    START_11,      CCW_BEGIN_11,  START_11},            // CCW_BEGIN_11
        {START_00 | EMIT_CW,     CW_BEGIN_11,   START_11,      START_11},            // CW_BEGIN_11
        {START_00,               CCW_BEGIN_00,  CW_BEGIN_00,   START_11},            // START_00
        {START_00,               START_00,      CW_BEGIN_00,   START_11 | EMIT_CW},  // CW_BEGIN_00
        {START_00,               CCW_BEGIN_00,  START_00,      START_11 | EMIT_CCW}, // CCW_BEGIN_00
    };
}

void RotaryEncoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    tableState = fullstep::START; // 0x0 in both tables; self-syncs from any shaft position
}

uint8_t RotaryEncoder::samplePins() const {
    const uint8_t a = digitalRead(pinA) == HIGH ? 1u : 0u;
    const uint8_t b = digitalRead(pinB) == HIGH ? 1u : 0u;
    return static_cast<uint8_t>((a << 1u) | b);
}

int8_t RotaryEncoder::sampleStep() {
    const uint8_t (*transitions)[4] = stepMode == EncoderStepMode::FULL_STEP
        ? fullstep::TRANSITIONS
        : halfstep::TRANSITIONS;

    tableState = transitions[tableState & ROW_MASK][samplePins()];

    const uint8_t emit = tableState & EMIT_MASK;
    if (emit == EMIT_CW) {
        return 1;
    }

    if (emit == EMIT_CCW) {
        return -1;
    }

    return 0;
}
