#include <Wire.h>             // I2C Library
#include <SPI.h>              // SPI Library
#include <RF24.h>             // Radio Library
// #include <nRF24L01.h>         // Radio Library
// #include <SFE_BMP180.h>       // Barometer Library
// #include <MPU6050_tockn.h>    // IMU Library
// #include <QMC5883LCompass.h>  // Magnetometer Library
// #include <AsyncSonarLib.h>    // Sonar Library

// Settings
int16_t interval = 50;    // Measurement cycle period (Milliseconds)
float sonarThres = 2.0;   // Threshold at which Sonar is used for Altitude Measurement (meters)

// IMU Variables
int16_t accelX, accelY, accelZ; // Accelerations (LSB Raw)
int16_t rateX, rateY, rateZ;    // Rotation Rates (LSB Raw)
// Magnetometer Variables
int16_t magX, magY, magZ;       // Magnetic field strengths (LSB Raw)
// Barometer Variables
int32_t pressure;               // Barometric pressure (Pascals)
// Sonar Variables
int16_t distance;               // Distance (Millimeters)


// This is the structure used to transmit data between the Arduinos
struct Data {
  // Raw Calculated Values
  float rawHead;        // Heading calculated from Magnetometer alone
  float rawAlti;        // Altitude calculated from Barometer alone
  float rawDesc;        // Descent rate calculated from altitude derivation
  // Filtered / Fused Values
  float filHead;        // Heading from complemetary filter (Magnetometer + Gyroscope)
  float filAlti;        // Altitude from Kalman Filter / Switching Logic (Barometer/Sonar + Accelerometer)
  float filDesc;        // Descent rate from Kalman Filter (Barometer/Sonar + Accelerometer)
  // Time
  uint32_t time;        // time past program start in Milliseconds
  uint16_t dt;          // time between cycles (Milliseconds)
  // Flags
  bool sonarOn;         // Is sonar being used for altitude calculation
}; Data data;


//////////// Serial (for debugging) ////////////

void initSerial(){
  Serial.begin(115200);
  Serial.println("Serial OK.");
}

void printAccel(){
  // Accelerometer
  Serial.print(accelX);   Serial.print('\t');
  Serial.print(accelY);   Serial.print('\t');
  Serial.print(accelZ);   Serial.print('\t');
}

void printGyro(){
  //Gyroscope
  Serial.print(rateX);    Serial.print('\t');
  Serial.print(rateY);    Serial.print('\t');
  Serial.print(rateZ);    Serial.print('\t');
}

void printMag(){
  // Magnetometer
  Serial.print(magX);     Serial.print('\t');
  Serial.print(magY);     Serial.print('\t');
  Serial.print(magZ);     Serial.print('\t');
}

void printBaro(){
  // Barometer
  Serial.print(pressure); Serial.print('\t');
}

void printSonar(){
  // Sonar
  Serial.print(distance); Serial.print('\t');
}

void printData(){
  Serial.print(data.rawHead); Serial.print('\t');
  Serial.print(data.rawAlti); Serial.print('\t');
  Serial.print(data.rawDesc); Serial.print('\t');
  Serial.print(data.filHead); Serial.print('\t');
  Serial.print(data.filAlti); Serial.print('\t');
  Serial.print(data.filDesc); Serial.print('\t');
  Serial.print(data.dt);      Serial.print('\t');
  Serial.print(data.time);    Serial.print('\t');
  Serial.print(data.sonarOn); Serial.print('\t');
}

void printDataHeader(){
  Serial.println();
  Serial.println("-------------------------------Data-------------------------------------");
  Serial.println("rawHead\trawAlti\trawDesc\tfilHead\tfilAlti\tfilDesc\tdTime\ttime\tsonarOn");
}

//////////// I2C (Wire) ////////////

// Initialize I2C Communication
void initI2C(){
  Wire.begin();
  Serial.println("I2C OK.");
}

//////////// IMU (MPU6050) ////////////

#define MPU_ADDR 0x68 // Standard I2C address for MPU6050
float gBiasX = 0, gBiasY = 0, gBiasZ = 0;
float aBiasX = 0, aBiasY = 0, aBiasZ = 0;
float Roll = 0;
float Pitch = 0;
float trueYawRate = 0; // Earth-frame vertical rotation rate
float trueAccelZ = 0;  // Earth-frame vertical acceleration (Gravity removed)
float tiltAlpha = 0.96; // Trust gyro 96%, Accel 4%

// Initialize IMU Module
void initIMU(){
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1 register
  Wire.write(0);    // Set to 0 (wakes up the sensor)
  Wire.endTransmission(true);

  // 2. Set Gyro Range to ±250°/s (Register 0x1B, Bits 3 and 4 to 00)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00); 
  Wire.endTransmission(true);

  // 3. Set Accel Range to ±2g (Register 0x1C, Bits 3 and 4 to 00)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00); 
  Wire.endTransmission(true);

  Serial.println("IMU INITIALIZED.");
}

void readIMU(){
  // --- 1. Read Raw & Apply Static Biases (LSB) ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start reading from the first Accel register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 registers total
  // Read 6 bytes for Accelerometer (X, Y, Z) and remove bias
  float rawAX = (int16_t)(Wire.read() << 8 | Wire.read()) - aBiasX;
  float rawAY = (int16_t)(Wire.read() << 8 | Wire.read()) - aBiasY;
  float rawAZ = (int16_t)(Wire.read() << 8 | Wire.read()) - aBiasZ;
  // Skip 2 bytes for Temperature (0x41, 0x42)
  Wire.read(); Wire.read();
  // Read 6 bytes for Gyroscope (X, Y, Z) and remove bias
  float rawGX = (int16_t)(Wire.read() << 8 | Wire.read()) - gBiasX;
  float rawGY = (int16_t)(Wire.read() << 8 | Wire.read()) - gBiasY;
  float rawGZ = (int16_t)(Wire.read() << 8 | Wire.read()) - gBiasZ;

  // --- 2. Calculate Orientation (Roll and Pitch in Radiants) ---
  // With Complementary filter (gyro and accelerometer)
  float dtSec = data.dt / 1000.0;
  float rawRoll = atan2(rawAY, rawAZ);
  float rawPitch = atan2(-rawAX, sqrt(rawAY * rawAY + rawAZ * rawAZ));
  // Gyro Rate units are Deg/Sec. Convert to Radians/Sec for the filter.
  float gyroRateX_rad = (rawGX / 131.0) * (PI / 180.0);
  float gyroRateY_rad = (rawGY / 131.0) * (PI / 180.0);
  // Integrate gyro and combine with accelerometer
  Roll = tiltAlpha * (Roll + gyroRateX_rad * dtSec) + (1.0 - tiltAlpha) * rawRoll;
  Pitch = tiltAlpha * (Pitch + gyroRateY_rad * dtSec) + (1.0 - tiltAlpha) * rawPitch;

  // --- 3. Accelerometer Earth-Frame Compensation (LSB) ---
  // These become Linear Acceleration (Gravity Removed) in LSB (1G = 16384)
  accelX = rawAX + (16384.0 * sin(Pitch));
  accelY = rawAY - (16384.0 * sin(Roll) * cos(Pitch));
  accelZ = rawAZ - (16384.0 * cos(Roll) * cos(Pitch));

  // --- 4. Gyroscope Earth-Frame Compensation (LSB) ---
  // These become compensated rotation rates in LSB
  // Uses the Euler transformation matrix logic
  rateX = rawGX + (rawGY * sin(Roll) * tan(Pitch)) + (rawGZ * cos(Roll) * tan(Pitch));
  rateY = (rawGY * cos(Roll)) - (rawGZ * sin(Roll));
  rateZ = (rawGY * sin(Roll) / cos(Pitch)) + (rawGZ * cos(Roll) / cos(Pitch));

  // --- 5. Unit Conversion for Filters ONLY ---
  // trueAccelZ (m/s^2) for the Kalman Filter
  trueAccelZ = (float)accelZ / 16384.0 * 9.81;
  // trueYawRate (deg/s) for the Complementary Filter
  trueYawRate = ((float)rateZ / 131.0) * 0.6;
}

// Calibrate IMU
void calibrateIMU() {
  Serial.println("IMU CALIBRATION: Keep the module perfectly level and still...");
  long sGX = 0, sGY = 0, sGZ = 0;
  long sAX = 0, sAY = 0, sAZ = 0;
  const int samples = 1000;

  // Discard first few readings to let sensor settle
  for(int i=0; i<100; i++) { readIMU_Raw(); delay(2); }

  for(int i = 0; i < samples; i++) {
    readIMU_Raw(); // A helper function to just get raw numbers
    sGX += rateX; sGY += rateY; sGZ += rateZ;
    sAX += accelX; sAY += accelY; sAZ += accelZ;
    delay(2);
  }

  // Average the noise out
  gBiasX = (float)sGX / samples;
  gBiasY = (float)sGY / samples;
  gBiasZ = (float)sGZ / samples;

  // For Accel X and Y, the "Bias" is the entire reading (forcing it to 0)
  aBiasX = (float)sAX / samples;
  aBiasY = (float)sAY / samples;

  // For Z, the "Bias" is the difference from 1G (16384)
  // If we read 17000, bias is +616. 
  aBiasZ = ((float)sAZ / samples) - 16384.0;

  Serial.println("IMU Calibrated.");
}

// Helper to keep code clean (used only inside calibration)
void readIMU_Raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accelX = (int16_t)(Wire.read() << 8 | Wire.read());
  accelY = (int16_t)(Wire.read() << 8 | Wire.read());
  accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); 
  rateX = (int16_t)(Wire.read() << 8 | Wire.read());
  rateY = (int16_t)(Wire.read() << 8 | Wire.read());
  rateZ = (int16_t)(Wire.read() << 8 | Wire.read());
}

//////////// Barometer (MPU180) ////////////

#define BMP_ADDR 0x77

// BMP180 Calibration Coefficients
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
int32_t b5; 
int32_t baseline;

// Initialize Barometer Module
void initBaro() {
  // Read 22 bytes of calibration data from EEPROM
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xAA);
  Wire.endTransmission();
  Wire.requestFrom(BMP_ADDR, 22);

  ac1 = Wire.read() << 8 | Wire.read();
  ac2 = Wire.read() << 8 | Wire.read();
  ac3 = Wire.read() << 8 | Wire.read();
  ac4 = Wire.read() << 8 | Wire.read();
  ac5 = Wire.read() << 8 | Wire.read();
  ac6 = Wire.read() << 8 | Wire.read();
  b1  = Wire.read() << 8 | Wire.read();
  b2  = Wire.read() << 8 | Wire.read();
  mb  = Wire.read() << 8 | Wire.read();
  mc  = Wire.read() << 8 | Wire.read();
  md  = Wire.read() << 8 | Wire.read();

  Serial.print("Barometer Initialized.");
}

int32_t getTruePressure() {
  // 1. Get Uncompensated Temperature (UT)
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF4); 
  Wire.write(0x2E); 
  Wire.endTransmission();
  delay(5); // Wait at least 4.5ms
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP_ADDR, 2);
  int32_t ut = Wire.read() << 8 | Wire.read();

  // 2. Get Uncompensated Pressure (UP)
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34 + (0 << 6)); // oss = 0 (ultra low power)
  Wire.endTransmission();
  delay(5); 
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP_ADDR, 3);
  int32_t up = ((int32_t)Wire.read() << 16 | (int32_t)Wire.read() << 8 | (int32_t)Wire.read()) >> (8 - 0);

  // 3. Temperature Compensation
  int32_t x1 = (ut - ac6) * ac5 >> 15;
  int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
  b5 = x1 + x2;
  
  // 4. Pressure Compensation
  int32_t b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((( (int32_t)ac1 * 4 + x3) << 0) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (uint32_t)ac4 * (uint32_t)(x3 + 32768) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (50000 >> 0);
  int32_t p = (b7 < 0x80000000) ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return p + ((x1 + x2 + 3791) >> 4); // Returns Pressure in Pa
}

void readBaro() {
  // Read pressure from sensor
  pressure = getTruePressure();  // Raw Pa
}

void calibrateBaro() {
  Serial.println("BARO CALIBRATION: Recording ground pressure...");
  long total = 0;
  for(int i = 0; i < 100; i++) {
    total += getTruePressure();
    delay(10);
  }
  baseline = total / 100;
  Serial.print("Barometer Calibrated. Baseline set: "); Serial.println(baseline);
}

//////////// Magnetometer (QMC5883L) ////////////

#define QMC_ADDR 0x0D
// Hard-Iron Offsets
float mOffX = 0;
float mOffY = 0;
float mOffZ = 0;
// Soft-Iron Scaling
float mScaleX = 1;
float mScaleY = 1;
float mScaleZ = 1; 

// Initialize Magnetometer Module
void initMag() {  
  // Set/Reset Period (Required for QMC5883L)
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission();

  // Control Register 1: 
  // 0x1D = Continuous Mode, 200Hz, 8G Scale, 512 Over Sampling
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D); 
  Wire.endTransmission();

  // FORCE the filter to start at the truth
  // Avoid large initial error that takes long to be corrected
  readMag();
  calcHeading();
  data.filHead = data.rawHead;
  
  Serial.println("Magnetometer OK.");
}

void readMag() {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00); // Start reading from Data Output X LSB
  Wire.endTransmission(false);
  
  Wire.requestFrom(QMC_ADDR, 6); // Request X, Y, Z (2 bytes each)
  
  if (Wire.available() >= 6) {
    // QMC5883L is Little-Endian (LSB first, then MSB)
    magX = (int16_t)(Wire.read() | Wire.read() << 8);
    magY = (int16_t)(Wire.read() | Wire.read() << 8);
    magZ = (int16_t)(Wire.read() | Wire.read() << 8);
  }
}

void calibrateMag() {
  Serial.println("MAG CALIBRATION: Rotate the module in a Figure-8 pattern for 40 seconds...");
  
  int16_t xM = 32767, xX = -32768;
  int16_t yM = 32767, yX = -32768;
  int16_t zM = 32767, zX = -32768;
  
  uint32_t start = millis();
  while(millis() - start < 40000) {
    readMag(); // Uses your existing readMag() function
    if(magX < xM) xM = magX; if(magX > xX) xX = magX;
    if(magY < yM) yM = magY; if(magY > yX) yX = magY;
    if(magZ < zM) zM = magZ; if(magZ > zX) zX = magZ;
    delay(10);
  }

  // Calculate Hard-Iron Offsets
  float offX = (xX + xM) / 2.0;
  float offY = (yX + yM) / 2.0;
  float offZ = (zX + zM) / 2.0;
  
  // Calculate Soft-Iron Scaling
  float dX = (xX - xM) / 2.0;
  float dY = (yX - yM) / 2.0;
  float dZ = (zX - zM) / 2.0;
  float avgD = (dX + dY + dZ) / 3.0;

  // Average noise out
  float scX = avgD / dX;
  float scY = avgD / dY;
  float scZ = avgD / dZ;

  // Print results to hard-code in the program
  Serial.print("float mOffX = "); Serial.print(offX); Serial.println(";");
  Serial.print("float mOffY = "); Serial.print(offY); Serial.println(";");
  Serial.print("float mOffZ = "); Serial.print(offZ); Serial.println(";");
  Serial.print("float mScaleX = "); Serial.print(scX); Serial.println(";");
  Serial.print("float mScaleY = "); Serial.print(scY); Serial.println(";");
  Serial.print("float mScaleZ = "); Serial.print(scZ); Serial.println(";");
  Serial.println("-------------------------------------------------------");
}

//////////// Sonar ////////////

#define PIN_ECHO A0
#define PIN_TRIG A1
float soundSpeedCorrection;
const uint16_t sonarTimeout = 18000; // 18ms timeout (~3 meters)

// Initialize Ultrasonic Distance Sensor
void initSonar() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  Serial.println("Sonar OK.");
}

void readSonar() {
  // Clear the trigger
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse to trigger the sensor
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Measure the duration of the Echo pulse in microseconds
  int16_t duration = pulseIn(PIN_ECHO, HIGH, sonarTimeout); 

  // Distance (mm) = duration (us) * calibrated speed (mm/us)
  distance = duration * soundSpeedCorrection;
}

void calibrateSonar() {
  Serial.println("SONAR CALIBRATION: Compensating Speed of Sound for Temperature...");
  // BMP180 b5 variable holds temperature info
  float tempC = ((b5 + 8) >> 4) / 10.0;
  // Speed of sound = 331.3 + 0.606 * temp
  // Convert to mm/us and divide by 2 for round-trip
  soundSpeedCorrection = (331.3 + (0.606 * tempC)) / 2000.0;
  Serial.print("Sonar Calibrated. Temperature: "); Serial.print(tempC); Serial.println("C");
}

//////////// Radio Frequency Module ////////////

RF24 radio(7, 8); // CE (Chip Enable), CSN (Chip Select Not)
const byte address[6] = "00001";

// Initialize Radio Transreceiver
void initRadio(){
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  Serial.println("Radio OK.");
}

void sendRadio(){
  // Pack data in data structure
  data.time = millis();

  // Transmit the entire data structure
  bool report = radio.write(&data, sizeof(data));
  
  // Report transmission result
  if (report) {
    // Serial.println("Telemetry successful.");
  } else {
    Serial.println("Telemetry failed.");
  }
}

//////////// Calculations ////////////

// Calculates raw heading in degrees (Magnetometer)
void calcHeading() {
  // 1. Apply Calibration Offsets and Scaling
  // (Hardcoded from a calibrateMag() run!)
  float calMagX = (magX - mOffX) * mScaleX;
  float calMagY = (magY - mOffY) * mScaleY;
  float calMagZ = (magZ - mOffZ) * mScaleZ;
  // Calculate horizontal components based on the drone's current Roll and Pitch.
  // These formulas "rotate" the mag vector back to the horizon
  float magX_comp = calMagX * cos(Pitch) + calMagZ * sin(Pitch);
  float magY_comp = calMagX * sin(Roll) * sin(Pitch) + calMagY * cos(Roll) - calMagZ * sin(Roll) * cos(Pitch);
  // Calculate Heading in Radians, then convert to Degrees
  data.rawHead = atan2(magY_comp, magX_comp) * 180.0 / PI;
  // Normalize heading to 0-360 degrees
  if (data.rawHead < 0) data.rawHead += 360;
}

// Calculates raw Altitude in meters (Barometer)
void calcAltitude(){
  // International barometric formula
  data.rawAlti = 44330.0 * (1.0 - pow((float)pressure / (float)baseline, 0.1903));
}

// Calculates raw Descent rate in m/s (Barometer)
void calcDescentRate() {
  static float lastRawAltitude = 0;
  float dtSec = data.dt / 1000.0;

  if (dtSec > 0) {
    // (Current - Last) / (dt in seconds)
    // Positive = Climbing, Negative = Descending
    data.rawDesc = (data.rawAlti - lastRawAltitude) / dtSec;
  }
  
  lastRawAltitude = data.rawAlti;
}

//////////// Filtering ////////////

// --- Kalman Filter (Altitude + Descent Rate) ---
// --- Tuning ---
float q_alt = 0.0001;    // Process Noise Covariance (Low = high trust in prediction, because of noisy accelerometer)
float r_baro = 10.00;    // Baro Measurement Noise Covariance (Higher = less trust, smoother)
float r_sonar = 0.01;    // Sonar Measurement Noise Covariance (Lower = high trust, very accurate)

// --- State ---
float kalman_p = 1.0;    // Estimation Error
float kalman_g = 0.0;    // Resulting Kalman Gain
float current_r = 0.50;  // This will switch between r_baro and r_sonar

// --- Complementary Filter (Heading) ---
// --- Tuning ---
float headingAlpha = 0.95; // Trust in gyro (Higher = slower drift correction, less noise)

void updateFilters() {
  // --- 0. PREPARE TIME  ---
  float dtSec = data.dt / 1000.0; 
  
  // --- 1. COMPLEMENTARY FILTER (Filter Heading using Magnetometer and Gyroscope) ---
  // Calculate the difference between the raw sensor and our current heading
  float headingDiff = data.rawHead - data.filHead;
  // Adjust the difference so it always takes the "short way" around the circle
  if (headingDiff > 180) headingDiff -= 360;
  if (headingDiff < -180) headingDiff += 360;
  // Calculate Filtered Heading
  data.filHead += (headingAlpha * (-trueYawRate * dtSec)) + ((1.0 - headingAlpha) * headingDiff);
  // Normalize heading to 0-360
  if (data.filHead < 0) data.filHead += 360;
  if (data.filHead >= 360) data.filHead -= 360;

  // --- 2. ALTITUDE SWITCHING LOGIC (Barometer or Sonar) ---
  float measuredAlt;
  float sonarMeters = (float)distance / 1000.0;
  if (sonarMeters > 0.02 && sonarMeters < sonarThres) {
    measuredAlt =  sonarMeters;   // Get Altitude from sonar (in meters)
    current_r = r_sonar;          // Trust the sonar heavily
    data.sonarOn = true;
  } else {
    measuredAlt = data.rawAlti;   // Get Altitude from Barometer (in meters)
    current_r = r_baro;           // Trust the barometer less
    data.sonarOn = false;
  }

  // --- 3. KALMAN PREDICTION (Using Accelerometer) ---
  // PREDICT ALTITUDE: Derivation of previous speed to predict new position
  data.filAlti = data.filAlti + data.filDesc * dtSec;
  // PREDICT Z VELOCITY: Add a damping factor to prevent runaway drift
  data.filDesc = (data.filDesc + trueAccelZ * dtSec) * 0.90;

  // --- 4. KALMAN CORRECTION (Using Barometer or Sonar) ---
  // Increase uncertainty during prediction
  kalman_p = kalman_p + q_alt;
  // Calculate trust (Gain)
  kalman_g = kalman_p / (kalman_p + current_r);
  // CORRECT ALTITUDE
  // Calculate the Difference between prediction and reality
  float altitudeError = measuredAlt - data.filAlti;
  // We use the Kalman Gain to decide how much of the sensor's "truth" to accept
  data.filAlti += kalman_g * altitudeError;
  // Simple "Drift Clamp": If we are very close to zero and not moving much, 
  // nudge the altitude back to the sensor reading faster.
  if (abs(data.filDesc) < 0.05) {
    data.filAlti = (data.filAlti * 0.999) + (measuredAlt * 0.001);
  }
  // CORRECT Z VELOCITY
  // This is the "Feedback Loop" that stops the altitude from drifting.
  // We scale the correction by dtSec to keep units consistent.
  data.filDesc += (kalman_g * altitudeError / dtSec);
  // Decrease uncertainty after correction
  kalman_p = (1.0 - kalman_g) * kalman_p;
}

//////////// Main Methods ////////////

void setup() {
  // Initialize Communications
  initSerial();
  initI2C();
  initRadio();
  // Initialize Sensors
  initIMU();
  initBaro();
  initMag();
  initSonar();
  // Calibrate Sensors
  calibrateIMU();
  calibrateBaro();
  // calibrateMag();
  calibrateSonar();
  // Print data header to Serial (debug)
  printDataHeader();
}


void loop() {
  // Update the time
  static int32_t previousTime = 0;
  int32_t currentTime = millis();
  if (currentTime - previousTime >= interval) {
    data.dt = currentTime - previousTime;
    previousTime = currentTime;

    // Read Sensors
    readIMU();
    readBaro();
    readMag();
    readSonar();

    // Calculate Values
    calcHeading();
    calcAltitude();
    calcDescentRate();

    // Filter Values
    updateFilters();
    
    // Print data to Serial (debug)
    printData();

    // Send Data to Ground Station
    sendRadio();
  }
}
