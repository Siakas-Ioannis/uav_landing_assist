#include <Servo.h>            // Servomotor Library
#include <SPI.h>              // SPI Library
#include <RF24.h>             // Radio Library
// #include <nRF24L01.h>         // Radio Library


// LED Pins
// 1-5 is the first array, 6-10 is the second array
#define PIN_LED_1 A0
#define PIN_LED_2 A1
#define PIN_LED_3 A2
#define PIN_LED_4 A3
#define PIN_LED_5 A4
#define PIN_LED_6 2
#define PIN_LED_7 3
#define PIN_LED_8 9
#define PIN_LED_9 A5
#define PIN_LED_10 A6
// Buzzer Pin
#define PIN_BUZZER  4
// Servo Pins
#define PIN_SERVO_1 5
#define PIN_SERVO_2 6

// Settings
float targetHeading = 180.0;    // The direction you want to face
float targetDescent = 0.0;      // Target m/s (Negative = descending)
float altiThresMin = 0.05;      // Altitude (m) where the buzzer goes solid
float altiThresMax = 0.5;       // Altitude (m) where the buzzer starts beeping
uint16_t beepMin = 50;          // Minimum beeping interval
uint16_t beepMax = 500;         // Maximum beeping interval
uint16_t beepLow = 250;         // Low frequency beeping
uint16_t beepHigh = 500;        // High frequency beeping
float headSensi = 0.5;          // How many degrees the servo turns per 1 deg error
float descSensi = 30.0;         // How many degrees the servo turns per 1 m/s error

// This is the structure used to transmit data between the Arduinos
struct Data {
  // Raw Calculated Values
  float rawHead;        // Heading calculated from Magnetometer alone
  float rawAlti;        // Altitude calculated from Barometer alone
  float rawDesc;        // Descent rate calculated from altitude derivation
  // Filtered / Fused Values
  float filHead;        // Heading from complemetary filter (Magnetometer + Gyroscope)
  float filAlti;        // Altitude from Kalman Filter / Switching Logic (Barometer/Sonar + Accelerometer)
  float filDesc;        // Descent rate from Kalman Filter / Switching Logic (Barometer/Sonar + Accelerometer)
  // Time
  uint32_t time;        // time past program start in Milliseconds
  uint16_t dt;          // delta Time in Milliseconds
  // Flags
  bool sonarOn;         // Is sonar being used for altitude calculation
}; Data data;



//////////// Serial (for debugging) ////////////

void initSerial(){
  Serial.begin(115200);

  Serial.println("Serial OK.");
}

void printData(){
  Serial.print(data.rawHead); Serial.print(',');
  Serial.print(data.filHead); Serial.print(',');
  Serial.print(data.rawAlti); Serial.print(',');
  Serial.print(data.filAlti); Serial.print(',');
  Serial.print(data.rawDesc); Serial.print(',');
  Serial.print(data.filDesc); Serial.print(',');
  Serial.print(data.dt);      Serial.print(',');
  // Serial.print(data.time);    Serial.print(',');
  Serial.print(data.sonarOn);
  Serial.println();
}

void printDataHeader(){
  Serial.println();
  Serial.println("---------------------------Data-----------------------------------");
  Serial.println("rawHead,filHead,rawAlti,filAlti,rawDesc,filDesc,dTime,sonarOn"); // rawHead,filHead,rawAlti,filAlti,rawDesc,filDesc,dTime,time,sonarOn
}

//////////// RF Module ////////////

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void initRadio(){
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);  // Set minimum signal strength
  radio.startListening();

  Serial.println("Radio OK.");
}

void receiveRadio(){ 
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    //printData();
    
    controlHeading();
    controlDescent();
    controlAltitude();
  }
}

//////////// Servomotor ////////////

Servo servo1;
Servo servo2;

void initServo(){
  servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2);
  servo1.write(0);
  servo2.write(0);
  delay(500);
  servo1.write(180);
  servo2.write(180);
  delay(500);
  servo1.write(90);
  servo2.write(90);
  Serial.println("Servo OK.");
}

//////////// LED Array ////////////

byte led[] = {
  PIN_LED_1, 
  PIN_LED_2, 
  PIN_LED_3, 
  PIN_LED_4, 
  PIN_LED_5, 
  PIN_LED_6, 
  PIN_LED_7, 
  PIN_LED_8, 
  PIN_LED_9, 
  PIN_LED_10,
  }; 

void initLED(){
  for(int i = 0; i<10; i++){
    pinMode(led[i], OUTPUT);
    digitalWrite(led[i], 1);
  }

  Serial.println("LEDs OK.");
}

// Configures the LED Array 1
void setLED1(int ledIndex){
  // Turn on the specified LED, while turning off the rest
  for(int i = 1; i<=5; i++){ 
    if(i == ledIndex){ digitalWrite(led[i-1], HIGH); }
    else{              digitalWrite(led[i-1], LOW); }
  }
}

// Configures the LED Array 2
void setLED2(int ledIndex){
  // Turn on the specified LED, while turning off the rest
  for(int i = 1; i<=5; i++){ 
    if(i == ledIndex){ digitalWrite(led[i+4], HIGH); }
    else{              digitalWrite(led[i+4], LOW); }
  }
}

//////////// Buzzer ////////////

// Global control variables
unsigned long buzzPrevMillis = 0;
bool buzzOn = false;

// Buzzer Settings
int buzzFreq = 0;   // frequency(hz) = 0 (Off), >0 (On)
int buzzDur = 0;    // duration(ms) = 0 (Continuous), >0 (Intermittent)

// Initializes the Buzzer
void initBuzz(){
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.println("Buzzer OK.");
}

// Creates the set buzzer behavior
void updateBuzz() {
  // STATE 1: OFF (frequency = 0)
  if (buzzFreq <= 0) {
    noTone(PIN_BUZZER);
    buzzOn = false;
    return;
  }

  // STATE 2: CONTINUOUS (duration = 0)
  if (buzzDur <= 0) {
    tone(PIN_BUZZER, buzzFreq);
    buzzOn = true;
    return;
  }

  // STATE 3: INTERMITTENT
  unsigned long buzzMillis = millis();
  if (buzzMillis - buzzPrevMillis >= buzzDur) {
    buzzPrevMillis = buzzMillis;
    buzzOn = !buzzOn;

    if (buzzOn) {
      tone(PIN_BUZZER, buzzFreq); // Start beep
    } else {
      noTone(PIN_BUZZER); // End beep, start silence phase
    }
  }
}

// Configures the buzzer
void setBuzz(int freq, int dur) {
  buzzFreq = freq;
  buzzDur = dur;
}

//////////// Control Actuators ////////////

void controlHeading() {
  // 1. Calculate Error with Wrap-around
  float error = data.filHead - targetHeading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // 2. Map error to Servo angle
  int angle = constrain(90 + error*headSensi, 0, 180);
  
  // 3. Map angle to LED Array
  int ledIndex = map(constrain(angle, 0, 180), 0, 180, 1, 6);

  // 4. Apply to Servo1 and LED Array 1
  servo1.write(180 - angle);Serial.print(180 - angle);
  setLED1(ledIndex);Serial.print(ledIndex);
  Serial.println();
}

void controlDescent() {
  // 1. Calculate Error
  float error = data.filDesc - targetDescent;

  // 2. Map error to Servo angle
  int angle = constrain(90 + error*descSensi, 0, 180);

  // 3. Map angle to LED Array
  int ledIndex = map(constrain(angle, 0, 180), 0, 180, 1, 6);

  // 4. Apply to Servo2 and LED Array 2
  servo2.write(180 - angle);
  setLED2(ledIndex);
}

void controlAltitude() {
  // 1. If sonar isn't in range, keep it quiet
  if (!data.sonarOn) {
    setBuzz(0, 0); 
    return;
  }

  // 2. Continuous Tone if below threshold
  if (data.filAlti <= altiThresMin) {
    setBuzz(beepHigh, 0); // High pitch, 0 duration = solid
  }
  // 3. Proportional Beeping
  else if (data.filAlti <= altiThresMax) {
    // Map altitude to beep interval. As altitude decreases, duration decreases (beeps faster).
    int interval = map(data.filAlti*100, altiThresMin*100, altiThresMax*100, beepMin, beepMax);
    setBuzz(beepLow, interval);
  }
  // 4. Out of sonar range but sonar "active" (edge case)
  else {
    setBuzz(0, 0);
  }
}

//////////// Main Methods ////////////

void setup() {
  initSerial();
  initRadio();
  initServo();
  initLED();
  initBuzz();
  printDataHeader();
}

void loop() {
  receiveRadio();
  updateBuzz();
}
