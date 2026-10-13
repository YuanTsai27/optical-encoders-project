// Standalone test for the optical quadrature encoder.
// Kept separate from BaseCodeOne for now. Spin the shaft and watch the Serial Monitor.

#include "quadrature-encoder.h"

// ---------------- Configuration ----------------

// PLACEHOLDERS: set to the analog pins the two phototransistor outputs are wired to.
const uint8_t PIN_CHANNEL_A = A4;
const uint8_t PIN_CHANNEL_B = A5;

// ADC thresholds (0 to 1023). Output is active high: light through = high reading.
// Measured: about 900 with light passing, about 100 when blocked (midpoint 500).
// The gap between the two thresholds is the hysteresis band, centred on the midpoint.
const int THRESHOLD_HIGH = 550;
const int THRESHOLD_LOW = 450;

// 32 segments on the outer track (16 black, 16 clear) = 32 edges per channel.
// Two channels in quadrature = 64 edges per revolution = 5.625 degrees per edge.
const uint16_t EDGES_PER_REV = 64;

// Speed is computed once per window.
const unsigned long WINDOW_MS = 1000;

// Set to true to print raw ADC values instead of speed, for checking thresholds.
const bool CALIBRATION_MODE = false;
const unsigned long CALIBRATION_PRINT_MS = 50;

// -----------------------------------------------

QuadratureEncoder encoder(PIN_CHANNEL_A, PIN_CHANNEL_B, THRESHOLD_HIGH, THRESHOLD_LOW,
                          EDGES_PER_REV, WINDOW_MS);

unsigned long last_calibration_print_ms = 0;

const char *direction_to_text(Direction direction) {
    switch (direction) {
    case DIR_CW:
        return "CW";
    case DIR_CCW:
        return "CCW";
    default:
        return "STOPPED";
    }
}

void setup() {
    Serial.begin(250000);
    encoder.begin();
    Serial.println(F("Optical quadrature encoder test"));
}

void loop() {
    encoder.update();
    analogWrite(6, 500);

    if (CALIBRATION_MODE) {
        unsigned long now = millis();
        if (now - last_calibration_print_ms >= CALIBRATION_PRINT_MS) {
            last_calibration_print_ms = now;
            // Two columns, readable in the Serial Plotter as well.
            Serial.print(encoder.get_raw_a());
            Serial.print('\t');
            Serial.println(encoder.get_raw_b());
        }
        return;
    }

    if (encoder.new_reading_available()) {
        Serial.print(F("RPM from optical quadrature encoder: "));
        Serial.println(encoder.get_rpm(), 2);
        Serial.print(F("direction read by sensor: "));
        Serial.println(direction_to_text(encoder.get_direction()));
        Serial.print(F("missed edges so far: "));
        Serial.println(encoder.get_invalid_transitions());
        Serial.println();
    }
}
