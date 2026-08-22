/*
 *  Wheel unit — XIAO nRF52840 BLE broadcaster.
 */

// Framework headers aren't -Wconversion clean; silence their warnings so the flag
// (build_src_flags in platformio.ini) only polices our own code below.
#pragma GCC diagnostic push                         // save current warning settings
#pragma GCC diagnostic ignored "-Wconversion"       // turn THIS warning off
#pragma GCC diagnostic ignored "-Wsign-conversion"  // turn THIS warning off
#include <Arduino.h>                                // code compiled here uses the relaxed setting
#include <Adafruit_TinyUSB.h>
#include <Adafruit_SPIFlash.h>
#include <bluefruit.h>
#pragma GCC diagnostic pop                          // restore what was saved at step 1

#include <swc_protocol.h>
#include "wake_on_press.h"

using namespace swc;

// Auto-repeat interval for held VOL±/SRC± buttons
constexpr uint32_t HOLD_REPEAT_INTERVAL = 200;

// Idle time since the last broadcast before suspending cpu
constexpr uint32_t IDLE_BEFORE_SLEEP = 1000;

uint8_t bleManufacturerData[PACKET_LENGTH] =
    {
    COMPANY_ID_LO, COMPANY_ID_HI, // company ID 0xFFFF (little-endian)
    MAGIC,
    VERSION,
    BUTTON_NONE, // button code: 0-8, or 0xFF = idle/none
    ButtonEvent::RELEASE, // event: start in the released/idle state
    0x00, // sequence: +1 on every broadcast, for receiver dedupe
    BATTERY_NOT_SAMPLED, // battery: cell voltage in 20mV units, refreshed at boot and on each wake
};

constexpr int PIN_DRIVE = D1;
constexpr int PIN_SENSE = A0; // on the XIAO nRF52840, D0 and A0 are literally the same physical pin
constexpr float DIVIDE_NETWORK_RESISTOR_VALUE = 5100.0; // 5.1K ohm (R2)
constexpr uint32_t NODE_FILTER_CAPACITOR_NANOFARADS = 10; // 10 nF (C1)

// The node settles exponentially through R2 into C1 (τ = R2 × C1) once D1 starts
// driving. Wait 6τ before reading: residual error e⁻⁶ ≈ 0.25% ≈ 10 ADC counts, well below the band spacing.
// Ω × nF / 1000 = µs
constexpr uint32_t NODE_SETTLE_MICROS =
    (6u * static_cast<uint32_t>(DIVIDE_NETWORK_RESISTOR_VALUE) * NODE_FILTER_CAPACITOR_NANOFARADS + 999u) / 1000u;

/**
 *  When button is pressed and drive D1 HIGH, the node sits in a voltage divider:
 *
*  D1 (VDD) ──[ 5100 Ω ]──┬── node (A0 reads here)
*                         │
*                      [ r Ω ]   (the pressed button)
*                         │
*                        GND
*
*  Voltage-divider rule: the lower resistor gets a share of the supply equal to its fraction of the total resistance
*
*  V_node = VDD × r / (r + 5100)
*  Now convert volts to ADC counts. The ADC gives counts = 4095 × (V_node / V_ref), and because we chose AR_VDD4, V_ref = VDD:
*  counts = 4095 × V_node / VDD = 4095 × [VDD × r/(r+5100)] / VDD = 4095 × r / (r + 5100)
 */
uint16_t resistanceToCounts(const float resistance) {
    return static_cast<uint16_t>(lroundf(4095.0f * resistance / (resistance + DIVIDE_NETWORK_RESISTOR_VALUE)));
}

uint16_t buttonThresholds[BUTTON_COUNT];

/**
 *  Compute the midpoint between this button and the next
 *
 *  Note: 4095 (idle/open is the next "level" above CALL_END)
 */
void computeButtonThresholds() {
    uint16_t counts[BUTTON_COUNT];
    for (int index = 0; index < BUTTON_COUNT; index += 1) {
        counts[index] = resistanceToCounts(BUTTONS[index].resistance);
    }

    for (int index = 0; index < BUTTON_COUNT; index += 1) {
        if (index == BUTTON_COUNT - 1) {
            constexpr int idle = 4095;
            buttonThresholds[index] = static_cast<uint16_t>((counts[index] + idle) / 2);
        } else {
            buttonThresholds[index] = static_cast<uint16_t>((counts[index] + counts[index + 1]) / 2);
        }
    }
}

int identifyButton(const uint16_t readCount) {
    if (readCount < (resistanceToCounts(BUTTONS[0].resistance) / 2)) {
        return -2; // Fault / Short
    }

    // Find the band the value falls in
    for (int index = 0; index < BUTTON_COUNT; index += 1) {
        if (readCount < buttonThresholds[index]) {
            return index;
        }
    }

    return -1; // IDLE
}

uint16_t sampleNode() {
    pinMode(PIN_DRIVE, OUTPUT);
    digitalWrite(PIN_DRIVE, HIGH);
    delayMicroseconds(NODE_SETTLE_MICROS);

    const auto first = static_cast<uint16_t>(analogRead(PIN_SENSE));
    const auto second = static_cast<uint16_t>(analogRead(PIN_SENSE));
    const auto third = static_cast<uint16_t>(analogRead(PIN_SENSE));

    // We need to take median of the readings to avoid any spikes polluting the output if we were to do average
    // median = a + b + c − max(a,b,c) − min(a,b,c)

    pinMode(PIN_DRIVE, INPUT); // release the pin

    return static_cast<uint16_t>(first + second + third - max(first, max(second, third)) - min(first, min(second, third)));
}

void configureAdvertising() {
    #ifdef DEBUG_BUILD
        Bluefruit.setName("SWC");
    #endif

    // Non-connectable, non-scannable: transmit and sleep between intervals — no RX window.
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

    #ifdef DEBUG_BUILD
        Bluefruit.Advertising.addName();
    #endif

    Bluefruit.Advertising.addManufacturerData(bleManufacturerData, sizeof(bleManufacturerData));
}

// Cell voltage in BATTERY_UNIT_MILLIVOLTS units. The battery IS VDD (cell feeds
// the 3V3 pin directly), so the chip measures its own supply. The ratiometric AR_VDD4
// used for button sampling would read VDD-vs-VDD = full scale always, so the reading
// swaps to the internal 0.6V reference (AR_INTERNAL, 3.6V full scale) and back.
uint8_t batteryLevel = BATTERY_NOT_SAMPLED;

void sampleBatteryLevel() {
    analogReference(AR_INTERNAL); // 0.6V reference, 1/6 gain: full scale = 3.6V
    const uint32_t counts = analogReadVDD();
    analogReference(AR_VDD4); // restore the ratiometric button reference
    const uint32_t millivolts = counts * 3600u / 4095u;
    batteryLevel = static_cast<uint8_t>(millivolts / BATTERY_UNIT_MILLIVOLTS);
}

uint32_t lastBroadcastAt = 0;

// Publish a new event to the live advert. We can't just mutate the buffer — Bluefruit copies
// it, and the SoftDevice won't reconfigure a running advert — so stop -> clear -> rebuild ->
// start. Bumping seq is what lets the receiver tell a fresh event from a re-broadcast.
void broadcastEvent(const uint8_t button, const ButtonEvent event) {
    bleManufacturerData[PacketIndex::BUTTON_DATA] = button;
    bleManufacturerData[PacketIndex::EVENT_DATA] = static_cast<uint8_t>(event);
    bleManufacturerData[PacketIndex::SEQUENCE_NUMBER]++; // intentional 255 -> 0 wrap
    bleManufacturerData[PacketIndex::BATTERY_LEVEL] = batteryLevel;

    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    configureAdvertising();
    Bluefruit.Advertising.start(0); // Advertise forever

    lastBroadcastAt = millis();
}

void sleepUntilPress() {
    // Wake-on-press: R1 (1M) holds the node HIGH at rest; any button pulls it LOW
    // through the ladder. PIN_DRIVE watches the node (an input carries no current
    // through R2, so no drop — it sees the node exactly). Armed only while sleeping:
    // sampleNode() drives PIN_DRIVE every awake pass.
    wake::arm();

    // Never sleep with a button already down (node low): no falling level would follow
    if (digitalRead(PIN_DRIVE) == HIGH) {
        #ifdef DEBUG_BUILD
            Serial.println("[sleep] idle timeout — advertising stopped, suspending");
            // println only queues into the CDC FIFO; TinyUSB transmits at 64 queued
            // bytes or on flush. Without this, the line stays in RAM across the
            // suspend and only reaches the terminal stapled to the wake message.
            Serial.flush();
        #endif

        Bluefruit.Advertising.stop();
        // The XIAO's LEDs are active-LOW but this variant defines LED_STATE_ON=1,
        // so the library's "LED off" write inside Advertising.stop() actually lights
        // it solid for the whole sleep. Overwrite with the true off level.
        digitalWrite(LED_BLUE, HIGH);
        wake::waitForPress(); // sleeps HERE until the wake interrupt notifies
        sampleBatteryLevel(); // at-rest voltage: radio still off, cell unloaded
        Bluefruit.Advertising.start(0);

        #ifdef DEBUG_BUILD
            Serial.println("[sleep] woke on press");
            Serial.printf("[battery] %umV\n", static_cast<unsigned>(batteryLevel) * BATTERY_UNIT_MILLIVOLTS);
        #endif
    } else {
        #ifdef DEBUG_BUILD
            Serial.println("[sleep] aborted — press arrived during entry");
        #endif
    }

    wake::disarm(); // before loop() resumes sampling (which drives PIN_DRIVE)
}

void setup() {
    #ifdef DEBUG_BUILD
        Serial.begin(115200);
    #endif

    pinMode(PIN_DRIVE, INPUT);
    pinMode(PIN_SENSE, INPUT);

    wake::configure(PIN_DRIVE); // setup() runs on the loop task — the task configure() captures to wake

    analogReadResolution(12); // generate values between 0...4095
    // Sets the ADC's reference voltage to AR_VDD4 = reference is VDD/4. This makes the measurement proportional to VDD itself.
    // analogRead essentially computes: counts = full_scale × (V_input / V_reference)
    // Divider is powered from VDD: D1 drives HIGH to 3V3 (= VDD), so the node voltage is VDD × R/(R+5100).
    // And now the ADC reference is also VDD. counts = 4095 × V_node / VDD = 4095 × (VDD × R/(R+5100)) / VDD = 4095 × R/(R+5100) (VDD cancels out)
    // The battery voltage disappears from the equation. A freshly-fitted 3.0 V cell and an old 2.5 V cell produce the same count for the same button
    analogReference(AR_VDD4);

    computeButtonThresholds();
    sampleBatteryLevel();

    if (!Bluefruit.begin()) {
        #ifdef DEBUG_BUILD
            Serial.println("[error] Bluefruit initialization failed");
        #endif
    }

    // nRF52840 can feed its internals through either an internal LDO or a buck converter
    // This switches to the buck, roughly halving the current of every radio burst
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    Bluefruit.setTxPower(4);
    #ifndef DEBUG_BUILD
        Bluefruit.autoConnLed(false); // silence the advertising LED in production
    #endif
    configureAdvertising();
    // BLE advertising intervals are in units of 0.625 ms (fixed by the Bluetooth spec).
    // Fast = 32 × 0.625 ms = 20 ms, Slow = 244 × 0.625 ms = 152.5 ms.
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30); // Advertise at the fast rate for 30s, then automatically drop to the slow rate
    Bluefruit.Advertising.start(0); // Advertise forever

    // Deep power down 2MB QSPI flash chip as it is not used. It drops the chip to ~0.1µA.
    Adafruit_FlashTransport_QSPI flashTransport;
    flashTransport.begin();
    flashTransport.runCommand(0xB9); // deep power-down
    flashTransport.end();

    #ifdef DEBUG_BUILD
        Serial.println("[boot] wheel unit ready — advertising");
        Serial.printf("[battery] %umV\n", static_cast<unsigned>(batteryLevel) * BATTERY_UNIT_MILLIVOLTS);
    #endif
}

void loop() {
    static int lastButtonIndex = -1;
    const uint16_t reading = sampleNode();
    const int currentButtonIndex = identifyButton(reading);

    // button state changed since the last poll -> emit a one-off PRESS or RELEASE.
    if (lastButtonIndex != currentButtonIndex) {
        lastButtonIndex = currentButtonIndex;

        if (currentButtonIndex >= 0) {
            broadcastEvent(static_cast<uint8_t>(currentButtonIndex), ButtonEvent::PRESS);

            #ifdef DEBUG_BUILD
                Serial.print("[button] reading: ");
                Serial.print(reading);
                Serial.print(" pressed: ");
                Serial.println(BUTTONS[currentButtonIndex].name);
            #endif
        } else if (currentButtonIndex == -1) {
            broadcastEvent(BUTTON_NONE, ButtonEvent::RELEASE);

            #ifdef DEBUG_BUILD
                Serial.println("[button] idle");
            #endif
        } else {
            // FAULT still makes the dash let go immediately (it may be repeating a PRESS):
            // the receiver treats an unknown button as release. The distinct event lets it
            // also surface the wiring fault to the user.
            broadcastEvent(BUTTON_NONE, ButtonEvent::FAULT);

            #ifdef DEBUG_BUILD
                Serial.println("[button] fault — broadcasting fault event");
            #endif
        }
    } else {
        // No change = button still held. Repeatable buttons (VOL±/SRC±) re-emit HOLD every
        // HOLD_REPEAT_INTERVAL so the stereo keeps ramping/seeking; one-shot buttons do nothing.
        if (currentButtonIndex >= 0 && BUTTONS[currentButtonIndex].repeats && lastBroadcastAt > 0 && millis() - lastBroadcastAt > HOLD_REPEAT_INTERVAL) {
            broadcastEvent(static_cast<uint8_t>(currentButtonIndex), ButtonEvent::HOLD);

            #ifdef DEBUG_BUILD
                Serial.println("[button] repeat");
            #endif
        }
    }

    // Poll interval (~20 Hz button sampling).
    delay(50);

    // When on idle, after the IDLE_BEFORE_SLEEP since last broadcast has elapsed, put chip to sleep
    if (currentButtonIndex == -1 && millis() - lastBroadcastAt > IDLE_BEFORE_SLEEP) {
        sleepUntilPress();
    }
}
