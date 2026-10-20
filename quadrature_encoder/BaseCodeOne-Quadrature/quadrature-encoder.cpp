#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "quadrature-encoder.h"

// quadrature transition table is indexed by (previous_state << 2) | current_state,
// where state = (A << 1) | B.
// e.g when A leads B (00 -> 10 -> 11 -> 01 -> 00) counts +1 per edge, which is taken as CW.
// 0 = no change, 1 = CW, -1 = CCW, 2 = missed edge (where both channels changed at once)

static const int8_t MISSED_EDGE = 2;
static const int8_t TRANSITION_TABLE[16] = {
    0, -1, 1, MISSED_EDGE,
    1, 0, MISSED_EDGE, -1,
    -1, MISSED_EDGE, 0, 1,
    MISSED_EDGE, 1, -1, 0};

// initialising the quadrature encoder class with the pin numbers, thresholds, edges per revolution, and window time in milliseconds.
QuadratureEncoder::QuadratureEncoder(uint8_t pin_a_, uint8_t pin_b_, int threshold_high_, int threshold_low_,
                                     uint16_t edges_per_rev_, unsigned long window_ms_)
{
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
    invalid_transitions = 0;
    raw_a = 0;
    raw_b = 0;

    // calculate the fixed number: degrees per edge based on the number of edges per revolution
    dpe = 360.0f / (float)edges_per_rev_;
}

// start-up function 
void QuadratureEncoder::begin()
{
    raw_a = analogRead(pin_a);
    raw_b = analogRead(pin_b);

    int midpoint = (threshold_high + threshold_low) / 2;

    // no previous level exists yet, so make the initial levels based on midpoint of the thresholds
    level_a = (raw_a > midpoint) ? 1 : 0;
    level_b = (raw_b > midpoint) ? 1 : 0;
    last_state = (level_a << 1) | level_b;

    window_edge_count = 0;
    window_start_ms = millis();
}

// reset function at the start of each measurement window
void QuadratureEncoder::reset()
{
    window_edge_count = 0;
    window_start_ms = millis();
}

// get logic level based on analog reading 
uint8_t QuadratureEncoder::to_logic(int raw, uint8_t previous_level) const
{
    // hysteresis function to eliminate noise around a single threshold value
    if (raw >= threshold_high)
    {
        return 1;
    }
    if (raw <= threshold_low)
    {
        return 0;
    }

    // if within hysteresis range, return the previous level
    return previous_level;
}

// function called every loop iteraction to sample both channels, decode any transition, and recompute RPM at the end of each window
void QuadratureEncoder::update()
{
    // sample both channels 
    raw_a = analogRead(pin_a);
    raw_b = analogRead(pin_b);

    // convert the raw readings to logic levels with hysteresis
    level_a = to_logic(raw_a, level_a);
    level_b = to_logic(raw_b, level_b);
    uint8_t state = (level_a << 1) | level_b;

    // decode the transition via edge/direction detection and update count 
    if (state != last_state)
    {
        int8_t step = TRANSITION_TABLE[(last_state << 2) | state];
        if (step == MISSED_EDGE)
        {
            invalid_transitions++;
        }
        else
        {
            window_edge_count += step;
        }
        last_state = state;
    }
}

// calculate the signed RPM over the time since the start of the measurement window
// also updates direction and restarts the measurement window
float QuadratureEncoder::calculate_rpm()
{
    unsigned long now = millis();
    unsigned long elapsed = now - window_start_ms;
    if (elapsed > 0)
    {
        // calculate degrees per second based on edges counted and elapsed time (1000 ms in a second).
        float dps = dpe * ((float)window_edge_count / (float)elapsed) * 1000.0f; 
        // convert degrees per second to revolutions per minute (60 seconds in a minute, 360 degrees in a revolution).
        rpm = dps * (60.0f / 360.0f);                                
    }

    // determine the direction based on the sign of edge count
    if (window_edge_count > 0)
    {
        direction = DIR_CW;
    }
    else if (window_edge_count < 0)
    {
        direction = DIR_CCW;
    }
    else
    {
        direction = DIR_STOPPED;
    }

    // reset the edge count and window start time for the next measurement window
    window_edge_count = 0;
    window_start_ms = now;
    return rpm;
}
