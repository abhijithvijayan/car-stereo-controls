#pragma once
#include <stdint.h>

namespace swc {

    enum ButtonEvent : uint8_t {
        PRESS = 0,
        HOLD = 1, // repeat tick while held (VOL±/SRC± only)
        RELEASE = 2,
        FAULT = 3, // ladder short / out-of-range reading
    };

    constexpr uint8_t BUTTON_NONE = 0xFF; // IDLE state

    // company ID 0xFFFF (little-endian)
    constexpr uint8_t COMPANY_ID_LO = 0xFF;
    constexpr uint8_t COMPANY_ID_HI = 0xFF;
    constexpr uint8_t MAGIC = 0x42; // identifies our packets
    constexpr uint8_t VERSION = 0x02; // bump if layout changes (v2: +battery byte, +FAULT event)
    constexpr uint8_t PACKET_LENGTH = 8;

    enum PacketIndex : uint8_t {
        BUTTON_DATA = 4, // Index of button code in bleManufacturerData buffer
        EVENT_DATA = 5, // Index of event in bleManufacturerData buffer
        SEQUENCE_NUMBER = 6, // Index of sequence number in bleManufacturerData buffer
        BATTERY_LEVEL = 7 // cell voltage in BATTERY_UNIT_MILLIVOLTS units, or BATTERY_NOT_SAMPLED
    };

    // One byte spans 0–5.1V at 20mV resolution (e.g. 150 = 3.00V)
    constexpr uint16_t BATTERY_UNIT_MILLIVOLTS = 20;
    // Sentinel above any reachable reading (ADC full scale = 3.6V = 180): a real
    // measurement can never produce it, unlike 0, which a failed conversion could.
    constexpr uint8_t BATTERY_NOT_SAMPLED = 0xFF;

    // Guard the hand-maintained coupling: every PacketIndex must fall within the buffer.
    static_assert(PACKET_LENGTH > PacketIndex::BATTERY_LEVEL,
                  "PACKET_LENGTH must cover all PacketIndex fields");

    enum ButtonIndex : uint8_t {
        VOICE = 0,
        SRC_PUSH,
        SRC_UP,
        SRC_DOWN,
        VOL_UP,
        MUTE,
        VOL_DOWN,
        ANSWER,
        CALL_END
    };

    // One row per physical button on the resistor ladder.
    // (index 0..8) carried in the BLE packet. `repeats` = auto-repeat while held (VOL±/SRC±).
    struct Button {
        ButtonIndex id;
        const char* name;
        float resistance; // ohms, for ADC classification
        bool repeats; // whether this button can be repeated by holding on
    };

    constexpr Button BUTTONS[] = {
        { VOICE, "VOICE",     360,        false },
        { SRC_PUSH, "SRC_PUSH",  870,        false },
        { SRC_UP, "SRC_UP",    1550,       true  },
        { SRC_DOWN, "SRC_DOWN",  2460,       true  },
        { VOL_UP, "VOL_UP",    3760,       true  },
        { MUTE, "MUTE",      5260,       false },
        { VOL_DOWN, "VOL_DOWN",  7260,       true  },
        { ANSWER, "ANSWER",    10860,      false },
        { CALL_END, "CALL_END",  16460,      false },
    };

    constexpr uint8_t BUTTON_COUNT = sizeof(BUTTONS) / sizeof(BUTTONS[0]);

    enum Mux : uint8_t {
        MUX_NONE,
        MUX_1,
        MUX_2
    };

    struct OutputCode {
        ButtonIndex id;
        const char *name;
        Mux mux;
        uint8_t channel; // Y0...Y7
    };

    // all 9 codes safe on one voltage axis, min gap 214 mV on the measured Pioneer divider (3.335 V / ~650 Ω).
    constexpr OutputCode OUTPUT_CODES[] = {
        { VOICE, "VOICE (3.3k)", MUX_2, 0 },
        { SRC_PUSH, "SRC_PUSH (220Ω)", MUX_1, 2 },
        { SRC_UP, "SRC_UP (330Ω)", MUX_1, 3 },
        { SRC_DOWN, "SRC_DOWN (470Ω)", MUX_1, 4 },
        { VOL_UP, "VOL_UP (100Ω)", MUX_1, 0 },
        { MUTE, "MUTE (6.8k)", MUX_1, 5 },
        { VOL_DOWN, "VOL_DOWN (2k)", MUX_1, 1 },
        { ANSWER, "ANSWER (1k)", MUX_2, 1 },
        { CALL_END, "CALL_END (680Ω)", MUX_2, 2 },
    };

    static_assert(BUTTON_COUNT == sizeof(OUTPUT_CODES) / sizeof(OUTPUT_CODES[0]), "BUTTONS AND OUTPUT MAPPING SHOULD MATCH");

    constexpr bool rowsMatchIndices(const uint8_t index = 0) {
        return index == BUTTON_COUNT ||
            (BUTTONS[index].id == index && OUTPUT_CODES[index].id == index && rowsMatchIndices(static_cast<uint8_t>(index + 1u)));
    }

    static_assert(rowsMatchIndices(), "TABLE ORDER MUST MATCH ButtonIndex");

}