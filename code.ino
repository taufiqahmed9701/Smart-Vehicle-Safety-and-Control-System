/*********** BLYNK CONFIG ***********/
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID   "TMPL3jGjKZjNe"
#define BLYNK_TEMPLATE_NAME "MINI PROJECT"
#define BLYNK_AUTH_TOKEN    "a90uWkkT2bS-VelLv1E0glPdHdOsmSog"

/*********** LIBRARIES ***********/
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "HX711.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU9250_asukiaaa.h>

/*********** WIFI ***********/
char ssid[] = "Realme9701";
char pass[] = "Taufiq9701";

/*********** PIN DEFINITIONS ***********/
#define DOUT 19
#define CLK  18
#define POT_PIN 34
#define HALL_PIN 23
#define BUZZER_PIN 4
#define RELAY_PIN 5
#define BATTERY_PIN 35
#define AIRBAG_PIN 13

#define ENA_PIN 25
#define IN1_PIN 27
#define IN2_PIN 14
#define ENB_PIN 26
#define IN3_PIN 33
#define IN4_PIN 32

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 1000
#define PWM_RES 8

#define WEIGHT_LIMIT 500.0
#define PULSES_PER_REV 2
#define RPM_INTERVAL_MS 500

/*********** OBJECTS ***********/
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 20, 4);
BlynkTimer timer;
MPU9250_asukiaaa mySensor;

/*********** CONTROL VARIABLES ***********/
float counts_per_kg = 97333.0;
long baseline_raw = 0;

volatile unsigned long pulseCount = 0;
unsigned long lastRPMTime = 0;
float rpm = 0;

int driveMode = 1;
int speedLimit = 200;
int targetRPM = 0;
bool emergencyStop = false;

/*********** PID VARIABLES ***********/
float kP = 0.4;
float kI = 0.02;
float kD = 0.1;

float pidIntegral  = 0;
float pidPrevError = 0;
float pidOutput    = 0;
float currentPWM   = 0;

const float DT = 0.5;

/*********** POT VARIABLES ***********/
int stableTarget  = 0;
int lastStablePot = 0;
int lockedPot     = 0;
bool potLocked    = false;

/*********** LCD PAGE ***********/
unsigned long lastPageChange = 0;
int currentPage = 0;

/*********** AIRBAG VARIABLES ***********/
float accelBuffer[3]      = {0, 0, 0};
int   bufferIndex         = 0;
int   crashCounter        = 0;
int   crashThresholdCount = 1;

bool  crashDetected   = false;
bool  airbagTriggered = false;

unsigned long airbagStartTime = 0;
unsigned long lastCrashTime   = 0;
unsigned long lastIMUTime     = 0;
unsigned long lastSerialTime  = 0;

const unsigned long airbagDuration = 10000;  // 10 sec airbag ON
const unsigned long crashCooldown  = 3000;   // 5 sec re-trigger guard
const unsigned long imuInterval    = 50;     // read IMU every 50ms
const unsigned long serialInterval = 1000;   // serial print every 1s

/*********** INTERRUPT ***********/
void IRAM_ATTR countPulse() {
pulseCount++;
}

/*********** BLYNK ***********/
BLYNK_WRITE(V1) { driveMode     = param.asInt(); }
BLYNK_WRITE(V3) { emergencyStop = param.asInt(); }

/*********** READ POT ***********/
int readStablePot() {
int potRaw = 0;
for (int i = 0; i < 8; i++) {
potRaw += analogRead(POT_PIN);
delayMicroseconds(200);
}
potRaw /= 8;

if (abs(potRaw - lastStablePot) > 80) {
lastStablePot = potRaw;
lockedPot     = potRaw;
potLocked     = true;
}

if (!potLocked) {
lockedPot = potRaw;
if (potRaw > 150) potLocked = true;
}

if (lockedPot < 150) {
stableTarget = 0;
} else {
stableTarget = map(lockedPot, 150, 4095, 0, 300);
}

return stableTarget;
}

/*********** AIRBAG CHECK — NON BLOCKING ***********/
void checkCrash() {

unsigned long now = millis();

// Throttle IMU reads to every 50ms — does not block main loop
if (now - lastIMUTime < imuInterval) return;
lastIMUTime = now;

mySensor.accelUpdate();

float ax = mySensor.accelX();
float ay = mySensor.accelY();
float az = mySensor.accelZ();

float totalAccel = sqrt(ax*ax + ay*ay + az*az) * 9.8;
static float prevAccel = 0;
float jerk = abs(totalAccel - prevAccel);
prevAccel = totalAccel;

// Moving average
float avgAccel = totalAccel;   // faster response (no delay)

// Roll & Pitch
float roll  = atan2(ay, az) * 180.0 / PI;
float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

// Impact detection
if (avgAccel > 12) crashCounter++;
else               crashCounter = 0;

bool impactCrash = (crashCounter >= crashThresholdCount);
bool rollover    = (abs(roll) > 30 || abs(pitch) > 30);
bool sudden      = (jerk > 6);   // new condition

// ----- SERIAL DEBUG (every 1 sec) -----
if (now - lastSerialTime >= serialInterval) {
Serial.print("Accel:"); Serial.print(avgAccel);
Serial.print(" | Roll:"); Serial.print(roll);
Serial.print(" | Pitch:"); Serial.print(pitch);
Serial.print(" | Airbag:");
Serial.println(airbagTriggered ? "ACTIVE" : "STANDBY");
lastSerialTime = now;
}

// ----- CRASH TRIGGER -----
if ((impactCrash || rollover || sudden) &&
!airbagTriggered &&
(now - lastCrashTime > crashCooldown)) {

Serial.println("CRASH DETECTED — AIRBAG DEPLOYED");  

crashDetected   = true;  
airbagTriggered = true;  
airbagStartTime = now;  
lastCrashTime   = now;  

// Always deploy airbag + buzzer  
digitalWrite(AIRBAG_PIN, HIGH);  
digitalWrite(BUZZER_PIN, HIGH);  

// Cut motor ONLY if vehicle was moving  
if (rpm > 20) {  
  currentPWM   = 0;  
  pidIntegral  = 0;  
  pidPrevError = 0;  
  ledcWrite(PWM_CHANNEL_A, 0);  
  ledcWrite(PWM_CHANNEL_B, 0);  
  digitalWrite(RELAY_PIN, HIGH);  
  Serial.println("MOTOR CUT — was moving");  
} else {  
  Serial.println("MOTOR WAS ALREADY STOPPED");  
}  

// Blynk crash alert  
Blynk.logEvent("crash_alert", "ACCIDENT DETECTED - Airbag Deployed!");  

// Override LCD with crash screen  
lcd.clear();  
lcd.setCursor(0, 0); lcd.print("!!! ACCIDENT !!!");  
lcd.setCursor(0, 1); lcd.print("AIRBAG ACTIVE   ");  
lcd.setCursor(0, 2);  
if (rpm > 20)        lcd.print("MOTOR STOPPED   ");  
else                 lcd.print("MOTOR WAS OFF   ");  
lcd.setCursor(0, 3); lcd.print("SEEK HELP NOW   ");

}

// ----- AIRBAG AUTO RESET -----
if (airbagTriggered && (now - airbagStartTime >= airbagDuration)) {

digitalWrite(AIRBAG_PIN, LOW);  
digitalWrite(BUZZER_PIN, LOW);  

// Restore relay only if emergency stop is not active  
if (!emergencyStop) {  
  digitalWrite(RELAY_PIN, LOW);  
}  

airbagTriggered = false;  
crashDetected   = false;  
crashCounter    = 0;  

Serial.println("Airbag Reset — System Normal");  

lcd.clear();  
lcd.setCursor(0, 0); lcd.print("System Normal   ");  
lastPageChange = millis();  // reset page timer cleanly

}
}

/*********** CONTROL LOOP ***********/
void controlLoop() {

// Crash takes full priority — skip motor control
if (airbagTriggered) {
return;
}

// ----- RPM -----
unsigned long now     = millis();
unsigned long elapsed = now - lastRPMTime;
if (elapsed == 0) return;

noInterrupts();
unsigned long count = pulseCount;
pulseCount = 0;
interrupts();

float newRPM = (count * 60000.0) / ((float)elapsed * PULSES_PER_REV);
rpm = (rpm * 0.5) + (newRPM * 0.5);
lastRPMTime = now;

// ----- LOAD -----
long raw       = scale.read_average(10);
long delta     = raw - baseline_raw;
float weight_g = abs(delta) * 1000.0 / counts_per_kg;
if (weight_g < 5) weight_g = 0;

// ----- DRIVE MODE SPEED LIMIT -----
if      (driveMode == 0) speedLimit = 120;
else if (driveMode == 1) speedLimit = 200;
else                     speedLimit = 300;

// ----- POT → TARGET -----
int requestedRPM = readStablePot();
targetRPM = min(requestedRPM, speedLimit);

// ----- SAFETY CUTOFF -----
if (emergencyStop || weight_g > WEIGHT_LIMIT) {
currentPWM   = 0;
pidIntegral  = 0;
pidPrevError = 0;
ledcWrite(PWM_CHANNEL_A, 0);
ledcWrite(PWM_CHANNEL_B, 0);
digitalWrite(RELAY_PIN, HIGH);
digitalWrite(BUZZER_PIN, HIGH);
updateLCD(weight_g);
sendBlynk(weight_g);
return;
} else {
digitalWrite(RELAY_PIN, LOW);
digitalWrite(BUZZER_PIN, LOW);
}

// ----- PID -----
if (targetRPM == 0) {
currentPWM   = 0;
pidIntegral  = 0;
pidPrevError = 0;
}
else if (targetRPM > 20 && rpm < 10) {
currentPWM  = 130;
pidIntegral = 0;
}
else {
float error      = (float)targetRPM - rpm;
pidIntegral     += error * DT;
pidIntegral      = constrain(pidIntegral, -100.0, 100.0);
float derivative = (error - pidPrevError) / DT;
pidOutput        = (kP * error) + (kI * pidIntegral) + (kD * derivative);
currentPWM      += pidOutput;
currentPWM       = constrain(currentPWM, 0.0, 255.0);
pidPrevError     = error;

// If RPM overshoots mode limit, pull PWM down instantly  
if (rpm > (float)speedLimit + 5.0) {  
  float overSpeed = rpm - (float)speedLimit;  
  currentPWM -= overSpeed * 0.8;  
  pidIntegral *= 0.5;  
  currentPWM = constrain(currentPWM, 0.0, 255.0);  
}

}

// ----- APPLY PWM -----
ledcWrite(PWM_CHANNEL_A, (int)currentPWM);
ledcWrite(PWM_CHANNEL_B, (int)currentPWM);

updateLCD(weight_g);
sendBlynk(weight_g);
}

/*********** LCD ***********/
void updateLCD(float weight_g) {

// Crash screen locks LCD — do not overwrite
if (airbagTriggered) return;

if (millis() - lastPageChange > 5000) {
currentPage = !currentPage;
lcd.clear();
lastPageChange = millis();
}

if (currentPage == 0) {
lcd.setCursor(0, 0);
lcd.print("Wt:");
lcd.print(weight_g, 1);
lcd.print("g     ");

lcd.setCursor(0, 1);  
lcd.print("RPM:");  
lcd.print((int)rpm);  
lcd.print(" T:");  
lcd.print(targetRPM);  
lcd.print("    ");  

lcd.setCursor(0, 2);  
if      (driveMode == 0) lcd.print("Mode: ECO     ");  
else if (driveMode == 1) lcd.print("Mode: NORMAL  ");  
else                     lcd.print("Mode: SPORT   ");  

lcd.setCursor(0, 3);  
if (emergencyStop)                lcd.print("EMERGENCY STOP ");  
else if (weight_g > WEIGHT_LIMIT) lcd.print("OVERLOAD       ");  
else {  
  if (targetRPM == speedLimit && targetRPM > 0) {  
    lcd.print("SPEED LIMITED  ");  
  } else {  
    lcd.print("PWM:");  
    lcd.print((int)currentPWM);  
    lcd.print(" Lim:");  
    lcd.print(speedLimit);  
    lcd.print("  ");  
  }  
}

}
else {
int rawADC = analogRead(BATTERY_PIN);
float voltage        = (rawADC / 4095.0) * 3.3;
float batteryVoltage = voltage * 5.35;

lcd.setCursor(0, 0);  
lcd.print("BATTERY:");  
lcd.print(batteryVoltage, 2);  
lcd.print("V   ");  

lcd.setCursor(0, 1);  
lcd.print("STATUS:");  
if      (batteryVoltage > 12.2) lcd.print("FULL    ");  
else if (batteryVoltage > 11.7) lcd.print("GOOD    ");  
else if (batteryVoltage > 11.2) lcd.print("LOW     ");  
else                            lcd.print("CRITICAL");  

lcd.setCursor(0, 2);  
lcd.print("Crash:  NORMAL  ");  

lcd.setCursor(0, 3);  
lcd.print("Airbag: STANDBY ");

}
}

/*********** BLYNK SEND ***********/
void sendBlynk(float weight_g) {
int rawADC = analogRead(BATTERY_PIN);
float batteryVoltage = ((rawADC / 4095.0) * 3.3) * 5.35;
Blynk.virtualWrite(V0, weight_g);
Blynk.virtualWrite(V2, rpm);
Blynk.virtualWrite(V4, batteryVoltage);
}

/*********** SETUP ***********/
void setup() {
Serial.begin(115200);

// I2C for MPU9250 + LCD
Wire.begin();
mySensor.setWire(&Wire);
mySensor.beginAccel();

pinMode(RELAY_PIN, OUTPUT);
digitalWrite(RELAY_PIN, HIGH);
delay(200);
digitalWrite(RELAY_PIN, LOW);

pinMode(BUZZER_PIN, OUTPUT);
digitalWrite(BUZZER_PIN, LOW);

pinMode(AIRBAG_PIN, OUTPUT);
digitalWrite(AIRBAG_PIN, LOW);

scale.begin(DOUT, CLK);

lcd.init();
lcd.backlight();
lcd.setCursor(0, 0);
lcd.print("Initializing... ");
lcd.setCursor(0, 1);
lcd.print("Safety System ON");
delay(5000);
baseline_raw = scale.read_average(50);
lcd.clear();

pinMode(IN1_PIN, OUTPUT);
pinMode(IN2_PIN, OUTPUT);
pinMode(IN3_PIN, OUTPUT);
pinMode(IN4_PIN, OUTPUT);
digitalWrite(IN1_PIN, HIGH);
digitalWrite(IN2_PIN, LOW);
digitalWrite(IN3_PIN, HIGH);
digitalWrite(IN4_PIN, LOW);

ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
ledcAttachPin(ENA_PIN, PWM_CHANNEL_A);
ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
ledcAttachPin(ENB_PIN, PWM_CHANNEL_B);

analogSetPinAttenuation(POT_PIN, ADC_11db);
analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

pinMode(HALL_PIN, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(HALL_PIN), countPulse, FALLING);

lastStablePot = analogRead(POT_PIN);
lockedPot     = lastStablePot;
lastRPMTime   = millis();

Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
timer.setInterval(RPM_INTERVAL_MS, controlLoop);
}

/*********** LOOP ***********/
void loop() {
Blynk.run();
timer.run();
checkCrash();   // non-blocking — internally throttled to 50ms
}
