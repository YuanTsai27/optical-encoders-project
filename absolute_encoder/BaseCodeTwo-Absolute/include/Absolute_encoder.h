#pragma once
#include <Arduino.h>

bool read_sensor(int pin, int threshold); 

byte read_all_sensors();

byte convert_gray_to_binary(byte gray_code);

float convert_binary_to_degrees(byte binary_code);

void calculate_rotationAndDirection(float starting_angle, float current_angle);