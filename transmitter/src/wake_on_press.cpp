#include "wake_on_press.h"

// Framework headers aren't -Wconversion clean; silence their warnings so the flag
// (build_src_flags in platformio.ini) only polices code below.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Arduino.h>
#pragma GCC diagnostic pop

namespace wake {

    constexpr uint32_t WAKE_PPI_CHANNEL = 10; // SoftDevice reserves 17–31

    // input, buffer connected, no pull, SENSE watching for LOW (the pressed level)
    constexpr uint32_t PIN_CNF_INPUT_SENSE_LOW =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

    // same, SENSE off — the awake state (matches what pinMode(INPUT) writes)
    constexpr uint32_t PIN_CNF_INPUT_PLAIN =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);

    static TaskHandle_t sleepingTask = nullptr;
    static NRF_GPIO_Type *wakePort = nullptr;
    static uint32_t wakePinRaw = 0;

    void configure(const int wakePin) {
        sleepingTask = xTaskGetCurrentTaskHandle();
        wakePinRaw = g_ADigitalPinMap[wakePin];
        wakePort = nrf_gpio_pin_port_decode(&wakePinRaw);
        NRF_PPI->CH[WAKE_PPI_CHANNEL].EEP = reinterpret_cast<uint32_t>(&NRF_GPIOTE->EVENTS_PORT);
        NRF_PPI->CH[WAKE_PPI_CHANNEL].TEP = reinterpret_cast<uint32_t>(&NRF_EGU3->TASKS_TRIGGER[0]);
        NRF_EGU3->INTENSET = EGU_INTENSET_TRIGGERED0_Msk;
        NVIC_SetPriority(SWI3_EGU3_IRQn, 6); // app-allowed priority under the SoftDevice; ≥2 so FromISR calls are legal
        NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
        NVIC_EnableIRQ(SWI3_EGU3_IRQn);
    }

    void arm() {
        ulTaskNotifyTake(pdTRUE, 0); // discard a stale notification from an aborted entry
        NRF_GPIOTE->EVENTS_PORT = 0;
        wakePort->PIN_CNF[wakePinRaw] = PIN_CNF_INPUT_SENSE_LOW;
        NRF_PPI->CHENSET = 1u << WAKE_PPI_CHANNEL;
    }

    void waitForPress() {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    void disarm() {
        NRF_PPI->CHENCLR = 1u << WAKE_PPI_CHANNEL;
        wakePort->PIN_CNF[wakePinRaw] = PIN_CNF_INPUT_PLAIN;
    }

}

// The wake interrupt: EGU3's vector, claimed by nothing else in the core or the
// SoftDevice. extern "C" so the symbol matches the vector table entry.
extern "C" void SWI3_EGU3_IRQHandler(void) {
    NRF_EGU3->EVENTS_TRIGGERED[0] = 0; // acknowledge, or the interrupt re-fires
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(wake::sleepingTask, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
