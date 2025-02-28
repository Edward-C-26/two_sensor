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
char input2[20];
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

void sendHeader() {
  const String dunit = getAbbreviation(units.distance);
  const String tunit = getAbbreviation(units.time);
  // Serial.flush();
  sensorA_x_pos = 0;
  sensorA_y_pos = 0;
  sensorB_x_pos = 0;
  sensorB_y_pos = 0;
  x_pos = 0;
  y_pos = 0;
  Serial.print(String(
      String("timestamp [ms]") + delimiter + flatFieldNames[0] + " [" + dunit +
      "]" + delimiter + flatFieldNames[1] + " [" + dunit + "]" + delimiter +
      flatFieldNames[2] + " [" + tunit + "]" +delimiter + " waterPin " "\n"));
}

 
void sendData(sensor_sample_t sampleA, String sensorNameA, sensor_sample_t sampleB, String sensorNameB) {
  // Convert to String class
  const String timestamp = String(sampleB.timestamp);
  sampleA.center.p.dx = abs(sampleA.center.p.dx) < 1000 ? 0 : sampleA.center.p.dx;
  sampleA.center.p.dy = abs(sampleA.center.p.dy) < 1000 ? 0 : sampleA.center.p.dy;
  sampleB.center.p.dx = abs(sampleB.center.p.dx) < 1000 ? 0 : sampleB.center.p.dx;
  sampleB.center.p.dy = abs(sampleB.center.p.dy) < 1000 ? 0 : sampleB.center.p.dy;
  const String dxL = String(sampleA.center.p.dx, decimalPlaces);
  const String dyL = String(sampleA.center.p.dy, decimalPlaces);
  const String dtL = String(sampleA.center.p.dt, decimalPlaces);
  const String dxR = String(sampleB.center.p.dx, decimalPlaces);
  const String dyR = String(sampleB.center.p.dy, decimalPlaces);
  const String dtR = String(sampleB.center.p.dt, decimalPlaces);
  const String waterPinVal = 0;
  const String endline = String("\n");
  // Serial.availableForWrite
  //getMovement data
  // Virmen Scale
  float scale = 128/23; //virmen distance/experimental distance
  //scale of maze taken into account, measurement in mm
  float unitsPerRotationL = 1963651.453/(3*2*2.54);  //average of three complete rotations
  float unitsPerRotationR =  1963651.233/(3*2*2.54); //average of three complete rotations
  float ballCircumferenceIn = 25.125; //measured in lab
  float ballCircumferenceCm = ballCircumferenceIn*2.54; //definition
  float ballRadiusCm  = ballCircumferenceCm/(2*M_PI); //definition


  float sensorAngleDegrees = 78; //measured in lab
  float sensorAngleRadians = sensorAngleDegrees*2*M_PI/360; //definition

  float cmPerUnitL = ballCircumferenceCm/unitsPerRotationL;
  float cmPerUnitR = ballCircumferenceCm/unitsPerRotationR;

  float dlx = sampleA.center.p.dx*cmPerUnitL; //convert measurements to units of cm
  float drx = sampleB.center.p.dx*cmPerUnitR; //convert measurements to units of cm
  float dry = sampleA.center.p.dy*cmPerUnitR; //convert measurements to units of cm
  float dly = sampleA.center.p.dy*cmPerUnitR;

  float dThetaL = (sampleA.center.p.dx)*2*M_PI/unitsPer2PiRotationL;
  float dThetaR = (sampleB.center.p.dx)*2*M_PI/unitsPer2PiRotationR;

  float dTheta = (dThetaL + dThetaR)/2 ;
  float sgn = (dTheta > 0) - (dTheta < 0);
  const float pi = 3.14159;  // Use a constant for pi


  float dyT = dry;

  float dxT = (dly-dry*cos(sensorAngleRadians))/cos(M_PI/2-sensorAngleRadians);
  float sgn2 = (dyT > 0) - (dyT < 0);
  float distance = sqrt(dxT*dxT + dyT*dyT);

  // Validate displacement values
  if (distance > MAX_SINGLE_DISPLACEMENT) {
      // Scale down the values proportionally if they exceed threshold
      float scale = MAX_SINGLE_DISPLACEMENT / distance;
      dxT *= scale;
      dyT *= scale;
      distance = MAX_SINGLE_DISPLACEMENT;
  }

  float theta = atan2((dxT), dyT);
  theta = sgn * std::min(static_cast<float>(std::exp(1.4 * std::pow(std::fabs(theta), 1.2))) - 1, pi);

  // Clamp theta to reasonable range
  theta = std::max(std::min(theta, MAX_THETA), -MAX_THETA);

  // compute the angle relative to the dx axis
  float rel_direction = atan2(dyT,dxT);
  rel_direction = rel_direction+(-152*2*M_PI/360);

  // Validate final displacement calculations
  dyT = 4.60975609756/4*sin(rel_direction)*distance*6.0;
  dxT = 4.04115037444/4*cos(rel_direction)*distance*6.0;

  const String dxTriangle = String(dxT, decimalPlaces);
  const String dyTriangle = String(dyT, decimalPlaces);

  const String dThetaTest = String(theta,decimalPlaces);
  int varile=0;
  for(int i=0;i<10;i++){
    varile+=fastDigitalRead(WATER_PIN);
    delay(2);
  }
  String variable = String(varile);

  // x_pos += sampleB.center.p.dx;
  // y_pos += sampleB.center.p.dy;
  x_pos += sampleB.center.p.dx;
  y_pos += sampleB.center.p.dy;
  

  // Print ASCII Strings
  Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter + dtL + delimiter + 
                                       dxR + delimiter + dyR + delimiter + dtR  + delimiter + 
                                       dxTriangle + delimiter + dyTriangle + delimiter + dTheta + delimiter + 
                                       variable + delimiter + String(rel_direction) + delimiter + 
                                       String(currentFrameCount) + endline);
  // Serial.print(String(x_pos) + delimiter + String(y_pos) + endline);
  // Serial.print((dxL) + delimiter + (dyL) + endline);


}

