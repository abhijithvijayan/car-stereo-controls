/*
 *  Dash unit — ESP32-C3 BLE scanner + SWC output stage.
 */

// Framework headers aren't -Wconversion clean; silence their warnings so the flag
// (build_src_flags in platformio.ini) only polices our own code below.
#pragma GCC diagnostic push                         // save current warning settings
#pragma GCC diagnostic ignored "-Wconversion"       // turn THIS warning off
#pragma GCC diagnostic ignored "-Wsign-conversion"  // turn THIS warning off
#include <Arduino.h>                                // code compiled here uses the relaxed setting
#include <NimBLEDevice.h>
#pragma GCC diagnostic pop                          // restore what was saved at step 1

#include <swc_protocol.h>
#include <rotary_encoder.h>

using namespace swc;

// cooperative yield per loop pass (keeps BLE/idle/watchdog alive)
constexpr uint32_t LOOP_YIELD_MS = 5;

constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;

constexpr uint8_t PIN_MUX_S0 = 0;
constexpr uint8_t PIN_MUX_S1 = 1;
constexpr uint8_t PIN_MUX_S2 = 3;
constexpr uint8_t PIN_MUX_1_ENABLE = 4;
constexpr uint8_t PIN_MUX_2_ENABLE = 5;

constexpr uint8_t PIN_ENCODER_A = 9;
constexpr uint8_t PIN_ENCODER_B = 8;
constexpr uint8_t PIN_ENCODER_PUSH = 10;

// push contact must be edge-free this long before a press-down edge counts
constexpr uint32_t PUSH_QUIET_MS = 30;

RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, EncoderStepMode::FULL_STEP);

volatile uint8_t pendingWheelButton = BUTTON_NONE;
volatile uint8_t pendingWheelEvent = RELEASE;
volatile bool hasPendingWheelEvent = false;
volatile uint32_t lastWheelHeardAt = 0; // heartbeat for the timeout failsafe

enum OutputState : uint8_t {
    OUTPUT_IDLE,
    OUTPUT_ACTIVE
};

OutputState outputState = OUTPUT_IDLE;
uint8_t currentActiveButton = BUTTON_NONE;

class BLEScanCallback : public NimBLEScanCallbacks {
    public: void onResult(const NimBLEAdvertisedDevice *device) override {
        const std::string manufacturerData = device->getManufacturerData();
        if (manufacturerData.length() != PACKET_LENGTH) {
            return;
        }

        const auto *packet = reinterpret_cast<const uint8_t *>(manufacturerData.data());
        if (packet[0] != COMPANY_ID_LO || packet[1] != COMPANY_ID_HI || packet[2] != MAGIC || packet[3] != VERSION) {
            return;
        }

        const uint32_t now = millis();
        const uint32_t sinceLast = now - lastWheelHeardAt;
        lastWheelHeardAt = now;

        const uint8_t button = packet[PacketIndex::BUTTON_DATA];
        const uint8_t event = packet[PacketIndex::EVENT_DATA];
        const uint8_t sequence = packet[PacketIndex::SEQUENCE_NUMBER];

        static bool initialized = false;
        // The wheel re-broadcasts the same frame until the next event; act once per sequence.
        static uint8_t lastSequence = 0;
        const bool isNewSession = !initialized || sinceLast > HEARTBEAT_TIMEOUT_MS;
        if (!isNewSession && sequence == lastSequence) {
            return;
        }

        initialized = true;
        lastSequence = sequence;

        pendingWheelButton = button;
        pendingWheelEvent = event;
        hasPendingWheelEvent = true;

        #ifdef DEBUG_BUILD
            const char *eventName =
                event == PRESS   ? "PRESS"   :
                event == HOLD    ? "HOLD"    :
                event == RELEASE ? "RELEASE" :
                event == FAULT   ? "FAULT"   : "?";
            const uint8_t batteryByte = packet[PacketIndex::BATTERY_LEVEL];
            if (batteryByte == BATTERY_NOT_SAMPLED) {
                Serial.printf("[%s] RSSI %d button=%u event=%s seq=%u batt=n/a\n", device->getAddress().toString().c_str(), device->getRSSI(), button, eventName, sequence);
            } else {
                Serial.printf("[%s] RSSI %d button=%u event=%s seq=%u batt=%umV\n", device->getAddress().toString().c_str(), device->getRSSI(), button, eventName, sequence,
                              static_cast<unsigned>(batteryByte) * BATTERY_UNIT_MILLIVOLTS);
            }
        #endif
    }
};

void setAddress(const uint8_t channel) {
    digitalWrite(PIN_MUX_S0, (channel & 0x01u) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S1, (channel & 0x02u) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S2, (channel & 0x04u) ? HIGH : LOW);
}

void releaseCodes() { // Ē active-low → HIGH = off
    digitalWrite(PIN_MUX_1_ENABLE, HIGH);
    digitalWrite(PIN_MUX_2_ENABLE, HIGH);
}

void activateKeyCode(const uint8_t button) {
    const OutputCode &code = OUTPUT_CODES[button];
    releaseCodes();
    setAddress(code.channel);
    digitalWrite(code.mux == MUX_1 ? PIN_MUX_1_ENABLE : PIN_MUX_2_ENABLE, LOW);
}

void setup() {
    // The 74HC4051's enable pin is Ē — active-low (the bar over the E means "true when low")
    // Ē = LOW → mux is on, Ē = HIGH → mux is off
    // pinMode MUST come first: this core discards digitalWrite() on an unconfigured pin,
    // so a preceding write would be a no-op and the pin would come up driving LOW.
    // R10/R11 hold Ē high until here; the µs LOW blip between the two calls is far
    // below what the stereo's SWC sensing can register.
    pinMode(PIN_MUX_1_ENABLE, OUTPUT);
    digitalWrite(PIN_MUX_1_ENABLE, HIGH);

    pinMode(PIN_MUX_2_ENABLE, OUTPUT);
    digitalWrite(PIN_MUX_2_ENABLE, HIGH);

    pinMode(PIN_MUX_S0, OUTPUT);
    digitalWrite(PIN_MUX_S0, LOW);
    pinMode(PIN_MUX_S1, OUTPUT);
    digitalWrite(PIN_MUX_S1, LOW);
    pinMode(PIN_MUX_S2, OUTPUT);
    digitalWrite(PIN_MUX_S2, LOW);

    // the encoder's common pin is wired to GND, so its contacts pull lines down; the internal ~45 k pull-ups give the idle-HIGH level
    encoder.begin();
    pinMode(PIN_ENCODER_PUSH, INPUT_PULLUP);

    #ifdef DEBUG_BUILD
        Serial.begin(115200);
    #endif

    NimBLEDevice::init("");
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new BLEScanCallback());
    scan->setActiveScan(false); // We only get advertisements passively
    scan->setInterval(100); // 100ms scan cycle
    scan->setWindow(100); // window == interval -> radio listening 100% of the time
    scan->setDuplicateFilter(false); // deliver EVERY advertisement, even from known devices
    scan->start(0, false); // 0 = scan forever
}

constexpr uint8_t TAP_QUEUE_SIZE = 10;
constexpr uint32_t TAP_HOLD_MS = 50;
constexpr uint32_t TAP_SPACING_MS = 50;

uint8_t tapQueue[TAP_QUEUE_SIZE];

uint8_t tapHead = 0;
uint8_t tapCount = 0;

void enqueueTap(const uint8_t button) {
    if (tapCount >= TAP_QUEUE_SIZE) {
        #ifdef DEBUG_BUILD
            Serial.printf("[queue] FULL — dropped %s\n", OUTPUT_CODES[button].name);
        #endif

        return;
    }

    tapQueue[(tapHead + tapCount) % TAP_QUEUE_SIZE] = button;
    tapCount += 1;

    #ifdef DEBUG_BUILD
        Serial.printf("[queue] +%s (depth %u)\n", OUTPUT_CODES[button].name, tapCount);
    #endif
}

void pollEncoder(const uint32_t now) {
    const int8_t step = encoder.sampleStep();
    if (step > 0) {
        enqueueTap(VOL_UP);
    } else if (step < 0) {
        enqueueTap(VOL_DOWN);
    }

    static bool previous = digitalRead(PIN_ENCODER_PUSH) == LOW; // poll first on boot to get the actual state in case user is pressing on boot
    static uint32_t lastPressAt = 0;

    const bool current = digitalRead(PIN_ENCODER_PUSH) == LOW;  // pull-up: LOW = pressed
    if (current != previous) {
        // mute if the line was quiet beforehand
        const bool quietLongEnough = now - lastPressAt >= PUSH_QUIET_MS;
        if (current && quietLongEnough) {
            enqueueTap(MUTE);
        }

        #ifdef DEBUG_BUILD
            if (current && !quietLongEnough) {
                Serial.printf("[enc] push down-edge rejected — line moved %ums ago\n", now - lastPressAt);
            }
        #endif

        previous = current;
        lastPressAt = now;
    }
}

void processButtonsPress(const uint32_t now) {
    switch (outputState) {
        case OUTPUT_IDLE: {
            if (hasPendingWheelEvent) {
                hasPendingWheelEvent = false;

                // Take a snapshot to prevent volatile data getting overwritten by the NimBLE task mid processing
                const uint8_t nextButton = pendingWheelButton;
                const uint8_t nextEvent = pendingWheelEvent;

                if ((nextEvent == PRESS || nextEvent == HOLD) && nextButton < BUTTON_COUNT) {
                    activateKeyCode(nextButton);
                    currentActiveButton = nextButton;
                    outputState = OUTPUT_ACTIVE;

                    #ifdef DEBUG_BUILD
                        Serial.printf("[mux] assert %s\n", OUTPUT_CODES[nextButton].name);
                    #endif
                }
            }

            break;
        }

        case OUTPUT_ACTIVE: {
            if (hasPendingWheelEvent) {
                hasPendingWheelEvent = false;

                // Take a snapshot to prevent volatile data getting overwritten by the NimBLE task mid processing
                const uint8_t nextButton = pendingWheelButton;
                const uint8_t nextEvent = pendingWheelEvent;

                if (nextEvent == RELEASE || nextEvent == FAULT || nextButton >= BUTTON_COUNT) {
                    releaseCodes();
                    currentActiveButton = BUTTON_NONE;
                    outputState = OUTPUT_IDLE;

                    #ifdef DEBUG_BUILD
                        Serial.printf("[mux] release (%s)\n", nextEvent == FAULT ? "wheel FAULT" : "wheel RELEASE");
                    #endif
                } else if (nextButton != currentActiveButton) {
                    activateKeyCode(nextButton);
                    currentActiveButton = nextButton;

                    #ifdef DEBUG_BUILD
                        Serial.printf("[mux] switch to %s\n", OUTPUT_CODES[nextButton].name);
                    #endif
                }
            } else if (now - lastWheelHeardAt > HEARTBEAT_TIMEOUT_MS) {
                releaseCodes();
                currentActiveButton = BUTTON_NONE;
                outputState = OUTPUT_IDLE;

                #ifdef DEBUG_BUILD
                    Serial.printf("[mux] release (heartbeat TIMEOUT after %ums)\n", HEARTBEAT_TIMEOUT_MS);
                #endif
            }

            break;
        }

        default: {
            break;
        }
    }
}

void loop() {
    const uint32_t now = millis();

    processButtonsPress(now);
    pollEncoder(now);
    delay(LOOP_YIELD_MS); // yield to the BLE + idle tasks (single core)
}
