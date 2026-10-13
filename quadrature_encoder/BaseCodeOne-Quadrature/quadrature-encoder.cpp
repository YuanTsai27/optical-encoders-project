#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "quadrature-encoder.h"

// Quadrature transition table is indexed by (previous_state << 2) | current_state,
// where state = (A << 1) | B.
// A leading B (00 -> 10 -> 11 -> 01 -> 00) counts +1 per edge, which is taken as CW.
// Entries of 0 on the diagonal mean no change. The value 2 marks a transition
// where both channels changed at once, which cannot happen in valid quadrature
// and means an edge was missed between samples.

static const int8_t MISSED_EDGE = 2;
static const int8_t TRANSITION_TABLE[16] = {
    0,           -1,          1,           MISSED_EDGE,
    1,           0,           MISSED_EDGE, -1,
    -1,          MISSED_EDGE, 0,           1,
    MISSED_EDGE, 1,           -1,          0};

// Initialising the quadrature encoder class with the pin numbers, thresholds, edges per revolution, and window time in milliseconds.
QuadratureEncoder::QuadratureEncoder(uint8_t pin_a_, uint8_t pin_b_, int threshold_high_, int threshold_low_,
                                     uint16_t edges_per_rev_, unsigned long window_ms_){
                                        pin_a = pin_a_;
                                        pin_b = pin_b_;
                                        threshold_high = threshold_high_;
                                        threshold_low = threshold_low_;
                                        edges_per_rev = edges_per_rev_;
                                        window_ms = window_ms_;
                                        level_a = 0;
                                        level_b = 0;
                                        last_state = 0;
                                        window_edge_count = 0;
                                        window_start_ms = 0;
                                        rpm = 0.0f;
                                        direction = DIR_STOPPED;
                                        reading_ready = false;
                                        invalid_transitions = 0;
                                        raw_a = 0;
                                        raw_b = 0;
                                        dpe = 360 / edges_per_rev_;
                                     }


void QuadratureEncoder::begin() {
    raw_a = analogRead(pin_a);
    raw_b = analogRead(pin_b);

    // No previous level exists yet, so split at the midpoint of the two thresholds.
    int midpoint = (threshold_high + threshold_low) / 2;
    level_a = (raw_a > midpoint) ? 1 : 0;
    level_b = (raw_b > midpoint) ? 1 : 0;
    last_state = (level_a << 1) | level_b;

    window_edge_count = 0;
    window_start_ms = millis();
}

uint8_t QuadratureEncoder::to_logic(int raw, uint8_t previous_level) const {
    // Hysteresis: the level only changes once the signal has fully crossed the
    // far threshold, so noise around a single threshold cannot create extra edges.
    if (raw >= threshold_high) {
        return 1;
    }
    if (raw <= threshold_low) {
        return 0;
    }
    return previous_level;
}

void QuadratureEncoder::update() {
    // Signal reading
    raw_a = analogRead(pin_a);
    raw_b = analogRead(pin_b);
    level_a = to_logic(raw_a, level_a);
    level_b = to_logic(raw_b, level_b);
    uint8_t state = (level_a << 1) | level_b;

    // Edge and direction detection
    if (state != last_state) {
        int8_t step = TRANSITION_TABLE[(last_state << 2) | state];
        if (step == MISSED_EDGE) {
            invalid_transitions++;
        } else {
            window_edge_count += step;
        }
        last_state = state;
    }

    // Speed calculation at the end of each window
    unsigned long now = millis();
    unsigned long elapsed = now - window_start_ms;
    if (elapsed >= window_ms) {
        // Use the measured elapsed time, not the nominal window, so a late
        // update does not scale the result.

        float dps = dpe * ((float)window_edge_count / (float)elapsed) * 1000.0f; // Calculate degrees per second based on edges counted and elapsed time (1000 ms in a second).
        rpm = dps * (60.0f / 360.0f); // Convert degrees per second to revolutions per minute (60 seconds in a minute, 360 degrees in a revolution).

        if (window_edge_count > 0) {
            direction = DIR_CW;
        } else if (window_edge_count < 0) {
            direction = DIR_CCW;
        } else {
            direction = DIR_STOPPED;
        }

        window_edge_count = 0;
        window_start_ms = now;
        reading_ready = true;
    }
}

bool QuadratureEncoder::new_reading_available() {
    if (reading_ready) {
        reading_ready = false;
        return true;
    }
    return false;
}