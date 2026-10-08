
// TO DO:

//1. Add function that returns reading from 1 sensor.
bool read_sensor(int pin, int threshold) {
  int sensor_value = analogRead(pin);
  return sensor_value > threshold;
}

//2. Add function that reads all sensors and returns gray code in a byte
byte read_all_sensors(const int pins[5], const int thresholds[5]) {
  byte gray_code = 0;
  for (int i = 0; i < 5; i++) {
    if (read_sensor(pins[i], thresholds[i])) {
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

float calculate_angular_displacement(float initial_unwrapped_angle, float current_unwrapped_angle) {
  return current_unwrapped_angle - initial_unwrapped_angle;
}

// in case of angle wrapping, we need to unwrap the angle to get the correct displacement. This function takes the raw angle, previous raw angle, and a reference to cumulative offset to keep track of the total rotation.
float unwrap_angle(float raw_angle, float previous_raw_angle, float &cumulative_offset) {
  float delta = raw_angle - previous_raw_angle;

  if (delta > 180.0) {
    cumulative_offset -= 360.0;
  } else if (delta < -180.0) {
    cumulative_offset += 360.0;
  }

  return raw_angle + cumulative_offset;
}


// This function determines the direction of rotation based on the displacement. It returns "CW" for clockwise, "CCW" for counterclockwise, and "--" for no movement.
const char* get_direction(float displacement) {
  if (displacement > 0) return "CW";
  if (displacement < 0) return "CCW";
  return "--";
}


//6. Find the mean of 8 out of 10 best readings.