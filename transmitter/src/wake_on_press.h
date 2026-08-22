#pragma once

/*
 *  Low-power wake-on-press for the sleep path.
 *
 *  The Arduino core's only wake API is attachInterrupt(), which uses a GPIOTE IN
 *  channel and keeps the 64MHz clock running through the whole sleep (~14µA).
 *  The nRF52's GPIO SENSE detector is the low-power alternative (~1µA), but the
 *  core exposes no API for it — and its PORT event lands on the GPIOTE interrupt
 *  vector, which the core's handler owns and never clears (enabling that interrupt
 *  would storm). This module is the missing abstraction: the PORT event is routed
 *  in hardware (PPI → EGU3) to an interrupt vector nothing else claims, which
 *  wakes the sleeping task with a FreeRTOS notification.
 *
 *  SENSE is level-based and notifications latch, so a press landing anywhere in
 *  sleep-entry makes waitForPress() return immediately — no lost-wake window.
 *
 *  Usage:
 *      setup():            wake::configure(PIN);        // once, on the loop task
 *      before sleeping:    wake::arm();
 *      to sleep:           wake::waitForPress();        // blocks; chip idles here
 *      after waking:       wake::disarm();              // before sampling resumes
 */

namespace wake {

    // One-time hardware wiring (PPI channel, EGU3 interrupt) and capture of the
    // calling task as the one to wake. Call once from setup(), on the loop task.
    // The wake stays inert until arm().
    void configure(int wakePin);

    // Arm the SENSE tripwire on the configured pin (watches for the pressed LOW
    // level) and connect its event to the wake interrupt. Also discards any stale
    // notification from a previously aborted sleep entry.
    void arm();

    // Block until a press (returns immediately if one already latched since arm()).
    // With the caller blocked, the scheduler idles the CPU — this is the sleep.
    void waitForPress();

    // Tear down: disconnect the event route and stop SENSE watching the pin.
    // Must be called before the awake loop resumes driving the pin.
    void disarm();

}
