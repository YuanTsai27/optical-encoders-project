// updated

#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

// Optical quadrature encoder read from two analog LED/phototransistor channels.
// Disk: outer track of the 6-track Gray code disk, 32 segments (16 black, 16 clear).
// Decoding: both edges on both channels (4x), giving 64 edges per revolution,
//   i.e. 360 / 64 = 5.625 degrees per edge.
// Speed: RPM = (edges / edges_per_rev) * (60000 / window_ms)

// Signals are polled with analogRead(), so update() must be called as often as
// possible. At 360 RPM there are 384 edges/s (one every ~2.6 ms), so the main
// loop must never block for longer than ~1 ms.

enum Direction
{
    DIR_STOPPED,
    DIR_CW,
    DIR_CCW
};

class QuadratureEncoder
{
public:
    QuadratureEncoder(uint8_t pin_a, uint8_t pin_b, int threshold_high, int threshold_low,
                      uint16_t edges_per_rev, unsigned long window_ms);

    // Latch the initial channel state. Call once in setup().
    void begin();

    // Reset the edge count and window start time. Call to restart speed measurement.
    void reset();

    // Sample both channels, decode any transition, and recompute RPM at the end
    // of each window. Call every loop iteration.
    void update();

    // Signed RPM over the time since the last call: positive CW, negative CCW.
    // Also updates direction and restarts the measurement window.
    float calculate_rpm();

    // Direction from the last completed window.
    Direction get_direction() const { return direction; }

    // True once per completed window, then cleared. Use it to print results.
    //bool new_reading_available();

    // Transitions where both channels changed between samples, which means an
    // edge was missed because sampling was too slow.
    uint32_t get_invalid_transitions() const { return invalid_transitions; }

    // Raw ADC values from the latest sample, for threshold calibration.
    int get_raw_a() const { return raw_a; }
    int get_raw_b() const { return raw_b; }

private:
    // Convert an analog reading to a logic level with hysteresis.
    uint8_t to_logic(int raw, uint8_t previous_level) const;

    uint8_t pin_a;
    uint8_t pin_b;
    int threshold_high; // rising input must exceed this to read HIGH
    int threshold_low;  // falling input must drop below this to read LOW
    uint16_t edges_per_rev;
    unsigned long window_ms;

    uint8_t level_a;
    uint8_t level_b;
    uint8_t last_state; // (A << 1) | B

    int32_t window_edge_count;
    unsigned long window_start_ms;

    float rpm;
    float dpe;
    Direction direction;
    bool reading_ready;
    uint32_t invalid_transitions;

    int raw_a;
    int raw_b;
};