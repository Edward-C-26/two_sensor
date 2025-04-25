#include <Arduino.h>
#include "main_config.h"
#include <cmath>

const String fileVersion = __TIMESTAMP__;

// Create Sensor Objects with Specified Slave-Select Pins
ADNS adnsA(CS_PIN_A);
ADNS adnsB(CS_PIN_B);

// Capture Task (on interrupt)
IntervalTimer captureTimer;

// Counter and Timestamp Generator
elapsedMillis millisSinceAcquisitionStart;
elapsedMicros microsSinceFrameStart;
// volatile time_t currentSampleTimestamp;
volatile time_t currentFrameTimestamp;
volatile time_t currentFrameDuration;
volatile uint32_t currentFrameCount;

volatile bool isRunning = false;
volatile uint32_t nreps = 0;


String count;
// char input2[20]; no longer needed if not using virmen

// =============================================================================
//   SETUP & LOOP
// =============================================================================
void setup() {
  initialize();
  beginAcquisition();
}


void loop() {
  // Time couner with nreps
  // while(nreps>=currentFrameCount) {
  // }
  // endAcquisition();
}




// =============================================================================
//   TASKS: INITIALIZE
// =============================================================================
inline static bool initialize() {
  // Begin Serial //115200//
  Serial.begin(Baud_rate);
  delay(100);

  Serial.println("Waiting for start message...");
  while (true) { // polling start message
    if (Serial.available() > 0) {
      String message = Serial.readStringUntil('\n'); // Read until newline
      message.trim(); // Remove any extra whitespace
      if (message == "start") { // Wait for the message "start"
        Serial.println("Start message received. Beginning program...");
        break; // Exit the loop and continue with the program
      }
    }
  }

  // Begin Sensor A
  adnsA.begin();

  // Begin Sensor B
  adnsB.begin();

  // Reset displacement values for both sensors
  adnsA.triggerSampleCapture();
  adnsA.readDisplacement(units); // Clear any residual displacement data

  adnsB.triggerSampleCapture();
  adnsB.readDisplacement(units); // Clear any residual displacement data
  
  fastPinMode(TRIGGER_PIN, OUTPUT);
  fastDigitalWrite(TRIGGER_PIN, LOW);
  delay(1);
  return true;
};

static inline void beginAcquisition() {
  //Parse input
  float trial_length_minutes_int = 10000;
  float sampling_interval_ms_int = 50.00;


  nreps = floor(trial_length_minutes_int*60.0*1000.0/sampling_interval_ms_int);
  Serial.println("Sampling time interval in ms: " + (String)((sampling_interval_ms_int)));

  sendHeader();

  // Trigger start using class methods in ADNS library
  adnsA.triggerAcquisitionStart();
  adnsA.triggerSampleCapture();

  adnsB.triggerAcquisitionStart();
  adnsB.triggerSampleCapture();
  // Change State
  isRunning = true;

  // Reset Elapsed Time Counter
  millisSinceAcquisitionStart = 0;

  currentFrameTimestamp = millisSinceAcquisitionStart;
  
  currentFrameCount = 0;

  fastDigitalWrite(TRIGGER_PIN,HIGH);

  captureTimer.begin(captureDisplacement, 1000*sampling_interval_ms_int);

  
}

static inline void endAcquisition() {
  // End IntervalTimer
  fastDigitalWrite(TRIGGER_PIN, LOW);
  // Trigger start using class methods in ADNS library
  adnsA.triggerAcquisitionStop();
  adnsB.triggerAcquisitionStop();
  // Change state
  isRunning = false;
  captureTimer.end();
}

// =============================================================================
// TASKS: TRIGGERED_ACQUISITION
// =============================================================================
void captureDisplacement() {
  fastDigitalWrite(TRIGGER_PIN,LOW);
  // Initialize container for combined & stamped sample
  sensor_sample_t currentSample_A;
  currentSample_A.timestamp = currentFrameTimestamp; 
  sensor_sample_t currentSample_B;
  currentSample_B.timestamp = currentFrameTimestamp;

  currentFrameCount += 1;
  currentFrameTimestamp = millisSinceAcquisitionStart;

  // Trigger capture from each sensor
  fastDigitalWrite(CS_PIN_A, LOW);
  delay(5);
  adnsA.triggerSampleCapture();
  currentSample_A.center = {'C', adnsA.readDisplacement(units)};
  delay(5);
  fastDigitalWrite(CS_PIN_A, HIGH);

  fastDigitalWrite(CS_PIN_B, LOW);
  delay(5);
  adnsB.triggerSampleCapture();
  displacement_t displacementB = adnsB.readDisplacement(units);
  currentSample_B.center = {'C', displacementB};
  delay(5);
  fastDigitalWrite(CS_PIN_B, HIGH);

  
  sendData(currentSample_A, "Sensor A", currentSample_B, "Sensor B");

  currentFrameTimestamp = millisSinceAcquisitionStart;
  fastDigitalWrite(TRIGGER_PIN, HIGH);
  delay(1);
  fastDigitalWrite(TRIGGER_PIN, LOW);

}

// =============================================================================
// TASKS: DATA_TRANSFER
// =============================================================================
// doesn't need to be changed
void sendHeader() {
  const String dunit = getAbbreviation(units.distance);
  const String tunit = getAbbreviation(units.time);
  Serial.flush();
  Serial.print(String(
      String("timestamp [ms]") + delimiter + flatFieldNames[0] + " [" + dunit +
      "]" + delimiter + flatFieldNames[1] + " [" + dunit + "]" + delimiter +
      flatFieldNames[2] + " [" + tunit + "]" +delimiter + " waterPin " "\n"));
  left_sensor_sum = 0;
  right_sensor_sum = 0; 
}

 
void sendData(sensor_sample_t sampleA, String sensorNameA, sensor_sample_t sampleB, String sensorNameB) {
  // Convert to String class
  const String timestamp = String(sampleB.timestamp);
  // left_sensor_sum += sampleA.center.p.dx;
  // right_sensor_sum += sampleB.center.p.dx;
  float left_x_calibrated = sampleA.center.p.dx * unitToCM_left;
  float left_y_calibrated = sampleA.center.p.dy * unitToCM_left;
  float right_x_calibrated = sampleB.center.p.dx * unitToCM_right;
  float right_y_calibrated = sampleB.center.p.dy * unitToCM_right;
  left_x_calibrated = abs(left_x_calibrated) < 0.0001 ? 0 : left_x_calibrated;
  left_y_calibrated = abs(left_y_calibrated) < 0.0001 ? 0 : left_y_calibrated;
  right_x_calibrated = abs(right_x_calibrated) < 0.0001 ? 0 : right_x_calibrated;
  right_y_calibrated = abs(right_y_calibrated) < 0.0001 ? 0 : right_y_calibrated;
  const String dxL = String(left_x_calibrated, decimalPlaces);
  const String dyL = String(left_y_calibrated, decimalPlaces);
  const String dtL = String(sampleA.center.p.dt, decimalPlaces);
  const String dxR = String(right_x_calibrated, decimalPlaces);
  const String dyR = String(right_y_calibrated, decimalPlaces);
  const String dtR = String(sampleB.center.p.dt, decimalPlaces);
  const String waterPinVal = 0;
  const String endline = String("\n");

  float movement_X =  (left_x_calibrated + right_x_calibrated * cos(37/360 * 2 * pi) -  left_y_calibrated * sin(37/360 * 2 * pi))/2 * magic_number;
  float movement_Y =  (left_y_calibrated + right_x_calibrated * cos(37/360 * 2 * pi) +  left_y_calibrated * sin(37/360 * 2 * pi))/2 * magic_number;
  float rel_direction = atan2(movement_Y, movement_X) * 180 / PI;
  float theta = atan2(movement_Y, movement_X);

  // // Print ASCII Strings
  // Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter + dtL + delimiter + 
  //   dxR + delimiter + dyR + delimiter + dtR  + delimiter + 
  //   movement_X + delimiter + movement_Y + delimiter + rel_direction + delimiter + 
  //   rel_direction + delimiter + String(rel_direction) + delimiter + 
  //   String(currentFrameCount) + endline);
  Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter + dtL + delimiter + 
    dxR + delimiter + dyR + delimiter + dtR  + delimiter + 
    String(movement_X,7) + delimiter + String(movement_Y,7) + delimiter + String(theta, 7) + delimiter + 
    0 + delimiter + String(rel_direction, 7) + delimiter + 
    String(currentFrameCount) + endline);

}

