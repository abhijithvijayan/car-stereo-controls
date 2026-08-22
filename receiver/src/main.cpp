/*
 * Dash unit — ESP32-C3 BLE scanner + SWC output stage.
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

using namespace swc;

// cooperative yield per loop pass (keeps BLE/idle/watchdog alive)
constexpr uint32_t LOOP_YIELD_MS = 5;

constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;

constexpr uint8_t PIN_MUX_S0 = 0;
constexpr uint8_t PIN_MUX_S1 = 1;
constexpr uint8_t PIN_MUX_S2 = 3;
constexpr uint8_t PIN_MUX_1_ENABLE = 4;
constexpr uint8_t PIN_MUX_2_ENABLE = 5;
constexpr uint8_t PIN_ENCODER_A = 8;
constexpr uint8_t PIN_ENCODER_B = 10;
constexpr uint8_t PIN_ENCODER_PUSH = 9;

volatile uint8_t pendingButton = BUTTON_NONE;
volatile uint8_t pendingEvent = RELEASE;
volatile bool hasPendingAction = false;
volatile uint32_t lastHeardAt = 0; // heartbeat for the timeout failsafe

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
        const uint32_t sinceLast = now - lastHeardAt;
        lastHeardAt = now;

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

        pendingButton = button;
        pendingEvent = event;
        hasPendingAction = true;

        #ifdef DEBUG_BUILD
            const char *eventName =
                event == PRESS   ? "PRESS"   :
                event == HOLD    ? "HOLD"    :
                event == RELEASE ? "RELEASE" : "?";
            Serial.printf("[%s] RSSI %d button=%u event=%s seq=%u\n", device->getAddress().toString().c_str(), device->getRSSI(), button, eventName, sequence);
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
    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);
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

void processButtonsPress(const uint32_t now) {
    switch (outputState) {
        case OUTPUT_IDLE: {
            if (hasPendingAction) {
                hasPendingAction = false;

                // Take a snapshot to prevent volatile data getting overwritten by the NimBLE task mid processing
                const uint8_t nextButton = pendingButton;
                const uint8_t nextEvent = pendingEvent;

                if (nextEvent != RELEASE && nextButton < BUTTON_COUNT) {
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
            if (hasPendingAction) {
                hasPendingAction = false;

                // Take a snapshot to prevent volatile data getting overwritten by the NimBLE task mid processing
                const uint8_t nextButton = pendingButton;
                const uint8_t nextEvent = pendingEvent;

                if (nextEvent == RELEASE || nextButton >= BUTTON_COUNT) {
                    releaseCodes();
                    currentActiveButton = BUTTON_NONE;
                    outputState = OUTPUT_IDLE;

                    #ifdef DEBUG_BUILD
                        Serial.printf("[mux] release (wheel RELEASE)\n");
                    #endif
                } else if (nextButton != currentActiveButton) {
                    activateKeyCode(nextButton);
                    currentActiveButton = nextButton;

                    #ifdef DEBUG_BUILD
                        Serial.printf("[mux] switch to %s\n", OUTPUT_CODES[nextButton].name);
                    #endif
                }
            } else if (now - lastHeardAt > HEARTBEAT_TIMEOUT_MS) {
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
    delay(LOOP_YIELD_MS); // yield to the BLE + idle tasks (single core)
}
