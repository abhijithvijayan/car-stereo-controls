/*
 *  rotary_encoder — state-table quadrature decoder for a detented rotary encoder.
 *
 *  Every transition of the 2-bit (A,B) Gray code advances a state machine that only emits when a full
 *  step's worth of codes arrives IN ORDER. Contact bounce merely flips between
 *  adjacent substates until it settles, so debounce is inherent — no timers, and
 *  therefore no speed penalty on fast rotation. An invalid transition (e.g. 01 straight to 10) resets to start
 *  a missed sample can lose a step but can never invent one in the wrong direction.
 *
 *  Derived from Ben Buxton's Rotary library (Copyright 2011 Ben Buxton, GPL v3);
 *  this file remains under GPL v3 — see README.
 */

#pragma once

#include <stdint.h>

// With the encoder's common pin on GND and inputs pulled up, a detent rest reads
// AB = 11. FULL_STEP emits once per full 4-transition cycle (one event per click for encoders that rest only at 11).
// HALF_STEP emits at both 11 and 00, for encoders that click twice per electrical cycle
enum class EncoderStepMode : uint8_t {
    FULL_STEP,
    HALF_STEP,
};

class RotaryEncoder {
    public:
        RotaryEncoder(const uint8_t a, const uint8_t b, const EncoderStepMode mode)
            : pinA(a), pinB(b), stepMode(mode) {}

        // Configures both pins INPUT_PULLUP. Call once from setup(). The state
        // machine starts at START and self-synchronizes from whatever position
        // the shaft booted in — it cannot emit until a full valid sequence occurs.
        void begin();

        // Sample once per loop pass. Returns +1 or -1 when a detent completes
        // (sign is wiring-dependent — verify on the bench), 0 otherwise.
        int8_t sampleStep();

    private:
        uint8_t samplePins() const;

        const uint8_t pinA;
        const uint8_t pinB;
        const EncoderStepMode stepMode;

        uint8_t tableState = 0; // low nibble: table row; bits 4-5: emit flags
};
