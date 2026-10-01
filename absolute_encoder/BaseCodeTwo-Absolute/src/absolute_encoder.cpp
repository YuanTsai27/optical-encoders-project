#include "../include/Absolute_encoder.h"

// TO DO:

//1. Add function that returns reading from 1 sensor.
bool read_sensor(int pin, int threshold) {
  int sensor_value = analogRead(pin);
  return sensor_value > threshold;
}

//2. Add function that reads all sensors and returns gray code in a byte
byte read_all_sensors() {
  byte gray_code = 0;
  for (int i = 0; i < 5; i++) { // Assuming 5 sensors connected to pins 0-4 (change to appropriate pin numbers)
    if (read_sensor(i, 1000)) { // Assuming threshold is 1000 for analogRead
      gray_code |= (1 << i);
    }
  }
  return gray_code;
}

//3. Add a function that converts gray code to binary code and returns the result as a byte.
byte convert_gray_to_binary(byte gray_code) {
    byte binary_code = 0;
    bool prev_bit = (gray_code>>4) & 1;
    binary_code |= (prev_bit<<4);
    for (int i=3 ; 0<=i ; i--) {
        bool current_bit = prev_bit ^ ((gray_code>>i) & 1);
        binary_code |= (current_bit<<i);
        prev_bit = current_bit;
        
    }
    return binary_code;
}

//4. Add a function that converts binary code to degrees.
float convert_binary_to_degrees(byte binary_code) {
    return binary_code * (360.0 / 32.0);
}


//5. Save starting angle, and calculate difference compared to current angle
//to calculate rotation. negative sign means counterclockwise rotation, positive sign means clockwise rotation.
float calculate_rotation(float starting_angle, float current_angle) {
    float rotation = current_angle - starting_angle;
    return rotation;
}

//6. Find the mean of 8 out of 10 best readings.