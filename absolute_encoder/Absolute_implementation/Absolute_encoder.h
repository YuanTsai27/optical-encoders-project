#pragma once
#include <Arduino.h>

bool read_sensor(int pin, int threshold); 

byte read_all_sensors(const int pins[5], const int thresholds[5]);

byte convert_gray_to_binary(byte gray_code);

float convert_binary_to_degrees(byte binary_code);

float calculate_angular_displacement(float initial_unwrapped_angle, float current_unwrapped_angle);

float unwrap_angle(float raw_angle, float previous_raw_angle, float &cumulative_offset);

const char* get_direction(float displacement);

float calculate_mean_of_best_readings(float differences[], int count);