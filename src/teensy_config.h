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

const float SENSOR_ANGLE_OFFSET = 18.5; // Each sensor is 18.5° from the forward axis (37° total between them)
const float angle_left_rad = -SENSOR_ANGLE_OFFSET * PI / 180.0;  // Left sensor at -18.5°
const float angle_right_rad = SENSOR_ANGLE_OFFSET * PI / 180.0;  // Right sensor at +18.5°

const float BALL_RADIUS = 101.56; // Radius of the ball in mm (adjust to your hardware)
const float SENSOR_VERTICAL_OFFSET = 25; // 25mm = 2.5cm below equator
const float SENSOR_ANGLE = atan2(SENSOR_VERTICAL_OFFSET, BALL_RADIUS); // Tilt angle (radians)

float magic_number = 1000;