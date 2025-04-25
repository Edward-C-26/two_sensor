// Pin Settings
const int CS_PIN_A = 20; // left 0
const int TRIGGER_PIN = 4;
const int CS_PIN_B = 21; // right 10
const int WATER_PIN = 5;

// Calibration or Test variables
int left_sensor_sum = 0;
int right_sensor_sum = 0;

// y is forward and backward
// x is left and right

// right y 1203103331000 slow 43.7cm
// right y 1110013113020 fast 43.7cm

float left_y = 17074896; // units moved for 30cm
float right_y = -11697208; // units moved for 30cm

float magic_number = 1000;