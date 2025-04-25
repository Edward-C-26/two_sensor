#include <Arduino.h>
#include <Bounce2.h>  //todo
#include <CircularBuffer.h>
#include <DigitalIO.h>
#include <SPI.h>
#include <teensy_config.h>

#define _USE_MATH_DEFINES
#include <math.h>
// #include <iostream>

// Include ADNS Library for ADNS-9800 Sensor
#include "ADNS9800/adns.h"

const bool TRIGGER_ACTIVE_STATE = HIGH;

// Global Constants 
const int Baud_rate = 115200;
const float ballCircumferenceCm = 63.82; 
const float unitsPerRotationA = 1000000.1232/(3*2);  //average of three complete rotations
const float unitsPerRotationB =  773091.0366/(3*2); //average of three complete rotations
const float sensorA_unit2Cm = ballCircumferenceCm/unitsPerRotationA;
const float sensorB_unit2Cm = ballCircumferenceCm/unitsPerRotationB;
const float unitsPer2PiRotationL = 24645112/1.2; // Number of sensor units per full rotation for left sensor
const float unitsPer2PiRotationR = 9687052/75;
const float pi = 3.14159; 
const float sensorAngleRadians = 78 * 2 * M_PI/360;
const float direction_offset = (-152 * 2 * pi / 360);
const float ballRadiusCm = ballCircumferenceCm/2/M_PI;
const float MAX_SINGLE_DISPLACEMENT = 10.0f; 
const float MAX_THETA = M_PI;

const float unitToCM_left = 30.0 / left_y; 
const float unitToCM_right = 30.0 / right_y; 

// Pre-Compute semi-synchronous sample rates for navigation sensors and camera
const int NAVSENSOR_FPS = 1000;

// =============================================================================
// Timing & Trigger-Output Settings and Implementation
// =============================================================================
// // Use zero-jitter & cross-platform Frequency-Timer-2 library for main clock
// #include <FrequencyTimer2.h>

// Use Interval Timer for Triggering
#include <IntervalTimer.h>

// Use ElapsedMillis for time-keeping
#include <elapsedMillis.h>


// Embedded Template Library Timer
// #include <timer.h>

// =============================================================================
// Enumeration and Type Definitions
// =============================================================================
// Data-descriptor type (string or char, variable or fixed-width)
typedef String sensor_name_t;
typedef String field_name_t;

// Message Frame Format
typedef struct {
  uint32_t length;
  enum FrameType : uint8_t { DATA, HEADER, SETTINGS };
  FrameType type;
  uint8_t flags;
  int8_t id;
} message_frame_t;  // todo

// Define data structure for a sample from a single sensor
typedef struct {
  char id = 'C';
  displacement_t p;  // todo: use generic point_t or vec2
} labeled_sample_t;

typedef struct {
  time_t timestamp;
  labeled_sample_t center;
} sensor_sample_t;

// Delimiter & Precision for Conversion to String
const unit_specification_t units = {Unit::Distance::MICROMETER,
                                    Unit::Time::MICROSECOND};  
constexpr char delimiter = ',';
constexpr unsigned char decimalPlaces = 8;

// Sensor and Field Nafes
const sensor_name_t sensorNames[] = {"center"};
const field_name_t fieldNames[] = {"dx", "dy", "dt"};
const String flatFieldNames[] = {
    sensorNames[0] + '_' + fieldNames[0], sensorNames[0] + '_' + fieldNames[1],
    sensorNames[0] + '_' + fieldNames[2]};

// =============================================================================
// Task Declarations
// =============================================================================

// Task: INITIALIZE
static inline bool initialize();

// Task: IDLE
static inline void beginAcquisition();
static inline void beginDataFrame();
static inline void endDataFrame();
static inline void endAcquisition();
void getRandomFrames(int samplingInterval,int jitterRange[], int nreps);
// Task: TRIGGERED_ACQUISITION
// (capture/acquire/read-loop)
static void captureDisplacement();

// Task: DATA_TRANSFER
static void sendHeader();
static void sendData(sensor_sample_t sampleA, String sensorNameA, sensor_sample_t sampleB, String sensorNameB);


// aggregate sums
static float sensorA_x_pos = 0;
static float sensorA_y_pos = 0;
static float sensorB_x_pos = 0;
static float sensorB_y_pos = 0;
static float x_pos = 0;
static float y_pos = 0;

// #endif




