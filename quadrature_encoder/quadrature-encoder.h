// updated

#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

enum Direction
{
    DIR_STOPPED,
    DIR_CW,
    DIR_CCW
};

// QuadratureEncoder class definition
class QuadratureEncoder
{
public:
    QuadratureEncoder(uint8_t pin_a, uint8_t pin_b, int threshold_high, int threshold_low,
                      uint16_t edges_per_rev, unsigned long window_ms);

    // to initialise channel state, called once in setup()
    void begin();

    // reset function at the start of each measurement window
    void reset();

    // function called every loop iteraction to sample both channels, decode any transition, and recompute RPM at the end of each window
    void update();

    // to return signed RPM over the time since the last call: positive = CW, negative = CCW.
    // also restarts the measurement window.
    float calculate_rpm();

    // getter for direction from the last completed window.
    Direction get_direction() const { return direction; }

    // getter for no. invalid transitions (debugging analysis)
    uint32_t get_invalid_transitions() const { return invalid_transitions; }

    // getter for raw ADC values from the latest sample for viewing (debugging analysis)
    int get_raw_a() const { return raw_a; }
    int get_raw_b() const { return raw_b; }

    // getter for Logic levels (0 or 1), for viewing the square wave (debugging analysis)
    uint8_t get_level_a() const { return level_a; }
    uint8_t get_level_b() const { return level_b; }

private:
    // convert an analog reading to a logic level with hysteresis.
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
    uint32_t invalid_transitions;

    int raw_a;
    int raw_b;
};
