/******************************************************************************
 * CubeSat Reaction Wheel — Secondary (Command & Control) Board Code
 * [Integrated Version with Bluetooth + UART Protocol]
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include "ArduinoBLE.h"

// ---------------------------------------------------------------------------
// 1. Pin Definitions and Constants
// ---------------------------------------------------------------------------
const int LED_PIN = LED_BUILTIN;
const unsigned long TELEMETRY_INTERVAL_MS = 1000;
const float SAMPLE_TIME_S = 0.01f;
const unsigned long LED_BLINK_INTERVAL_MS = 50;
bool newBLEcommand = false;
String incoming;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CONTROL_ENABLE true
#define TARGET_ROLL 0.0f
#define MAX_CONTROL_TORQUE 100.0f

const float ATTITUDE_KP = 50.0f;
const float ATTITUDE_KI = 5.0f;
const float ATTITUDE_KD = 10.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;

// BLE Components
BLEDevice peripheral;
BLECharacteristic targetChar;

// ---------------------------------------------------------------------------
// UART Protocol Definitions
// ---------------------------------------------------------------------------
#define PACKET_START_BYTE    0xAA
#define PACKET_END_BYTE      0x55
#define CMD_TYPE_SPEED       0x01
#define CMD_TYPE_TORQUE      0x02
#define CMD_TYPE_STATUS_REQ  0x03

// ---------------------------------------------------------------------------
// 2. Global Variables
// ---------------------------------------------------------------------------
struct Orientation {
  float roll;
  float pitch;
  float yaw;
};

SparkFun_ISM330DHCX imu;

Orientation currentOrientation = {0.0f, 0.0f, 0.0f};
Orientation targetOrientation = {TARGET_ROLL, 0.0f, 0.0f};

unsigned long lastTelemetryMillis = 0;
unsigned long lastControlMillis = 0;
unsigned long lastLedMillis = 0;
unsigned long previousMicros = 0;

bool ledState = false;
float gyroRates[3] = {0.0f, 0.0f, 0.0f};
float previousRoll = 0.0f;
float previousPitch = 0.0f;

float rollError = 0.0f;
float rollErrorIntegral = 0.0f;
float rollErrorPrevious = 0.0f;
float commandedTorque = 0.0f;

bool manualCommandActive = false;
float manualCommandTorque = 0.0f;
float manualCommandRPM = 0.0f;

// ---------------------------------------------------------------------------
// Primary Board Integration
// ---------------------------------------------------------------------------
#define PRIMARY_BAUD 115200
HardwareSerial &primarySerial = Serial1;

int16_t currentWheelRPM = 0;
float currentWheelTorque = 0.0f;
uint8_t primaryStatus = 0;
uint8_t primaryFaults = 0;

// ---------------------------------------------------------------------------
// 3. Forward Declarations
// ---------------------------------------------------------------------------
void initIMU();
void readIMU();
void calculateOrientation(float dt);
void updateAttitudeControl();
void handleSerialCommands();
void sendLocalTelemetry();
void sendBinaryCommand(uint8_t cmdType, int16_t value);
void processPrimaryPacket();
void updateLedBlink();

// ---------------------------------------------------------------------------
// 4. setup()
// ---------------------------------------------------------------------------
void setup() {
  pinMode(8, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Reaction Wheel C&C Board (Integrated) ===");
  
  // BLE Initialization
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  BLE.scan();

  // UART Initialization
  primarySerial.begin(PRIMARY_BAUD);
  
  // IMU Initialization
  Wire.begin();
  initIMU();
  
  targetOrientation.roll = TARGET_ROLL;
  previousMicros = micros();
  
  pinMode(LED_PIN, OUTPUT);
  Serial.println("System Initialized");
}

// ---------------------------------------------------------------------------
// 5. loop()
// ---------------------------------------------------------------------------
void loop() {
  // BLE Handling
  if (!peripheral) {
    BLEDevice discoveredDevice = BLE.available();
    if (discoveredDevice) {
      Serial.print("Found device: ");
      Serial.println(discoveredDevice.address());

      if (discoveredDevice.advertisedServiceUuid() == SERVICE_UUID) {
        Serial.println("Target peripheral found. Connecting...");
        BLE.stopScan();

        if (discoveredDevice.connect()) {
          Serial.println("Connected to peripheral");
          if (discoveredDevice.discoverAttributes()) {
            targetChar = discoveredDevice.characteristic(CHARACTERISTIC_UUID);
            if (targetChar && targetChar.canSubscribe()) {
              if (targetChar.subscribe()) {
                Serial.println("Subscribed to notifications");
                digitalWrite(8, HIGH);
                peripheral = discoveredDevice;
              }
            }
          }
        }
      }
    }
  }

  // Process BLE Notifications
  if (peripheral && peripheral.connected() && targetChar && targetChar.valueUpdated()) {
    int len = targetChar.valueLength();
    const uint8_t* data = targetChar.value();
    incoming = "";
    newBLEcommand = true;
    for (int i = 0; i < len; i++) {
      incoming += (char)data[i];
    }
  }

  // Handle BLE Disconnections
  if (peripheral && !peripheral.connected()) {
    Serial.println("Peripheral disconnected. Restarting scan...");
    digitalWrite(8, LOW);
    peripheral = BLEDevice();
    BLE.scan();
  }

  // Process New BLE Commands
  if (newBLEcommand) {
    Serial.print("New BLE Command: ");
    Serial.println(incoming);
    newBLEcommand = false;
    // Add BLE command processing here if needed
  }

  // Main Control Loop
  handleSerialCommands();
  readIMU();
  
  float dt = (micros() - previousMicros) / 1000000.0f;
  previousMicros = micros();
  calculateOrientation(dt);
  
  if (millis() - lastControlMillis >= (SAMPLE_TIME_S * 1000)) {
    lastControlMillis = millis();
    updateAttitudeControl();
  }
  
  if (primarySerial.available() >= 9) {
    processPrimaryPacket();
  }
  
  updateLedBlink();
  
  if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMillis = millis();
    sendLocalTelemetry();
  }
}

// ---------------------------------------------------------------------------
// 6. IMU Functions
// ---------------------------------------------------------------------------
void initIMU() {
  if (!imu.begin()) {
    Serial.println("IMU Init Failed!");
    while(1);
  }
  imu.setAccelDataRate(4);
  imu.setGyroDataRate(4);
  imu.setGyroFullScale(4);
  imu.setAccelFullScale(2);
}

void readIMU() {
  sfe_ism_data_t accelData, gyroData;
  imu.getAccel(&accelData);
  imu.getGyro(&gyroData);
  
  gyroRates[0] = (float)gyroData.xData / 131.0f;
  gyroRates[1] = (float)gyroData.yData / 131.0f;
  gyroRates[2] = (float)gyroData.zData / 131.0f;
  
  float accelRoll = atan2((float)accelData.yData, (float)accelData.zData) * 57.2958f;
  float accelPitch = atan2(-(float)accelData.xData, 
                     sqrt((float)(accelData.yData*accelData.yData + 
                                   accelData.zData*accelData.zData))) * 57.2958f;
  
  currentOrientation.roll = accelRoll;
  currentOrientation.pitch = accelPitch;
}

void calculateOrientation(float dt) {
  if (dt <= 0.0f || dt > 0.1f) dt = SAMPLE_TIME_S;
  
  float gyroRoll = previousRoll + gyroRates[0] * dt;
  float gyroPitch = previousPitch + gyroRates[1] * dt;
  
  currentOrientation.roll = COMPLEMENTARY_ALPHA * gyroRoll + 
                          (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.roll;
  currentOrientation.pitch = COMPLEMENTARY_ALPHA * gyroPitch + 
                           (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.pitch;
  
  previousRoll = currentOrientation.roll;
  previousPitch = currentOrientation.pitch;
  
  currentOrientation.yaw += gyroRates[2] * dt;
  currentOrientation.yaw = fmod(currentOrientation.yaw, 360.0f);
}

// ---------------------------------------------------------------------------
// 7. Control & Communication (Modified Section)
// ---------------------------------------------------------------------------
void updateAttitudeControl() {
  if (manualCommandActive) {
    sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(manualCommandTorque * 100));
    return;
  }
  
  if (!CONTROL_ENABLE) return;
  
  rollError = targetOrientation.roll - currentOrientation.roll;
  rollErrorIntegral += rollError * SAMPLE_TIME_S;
  rollErrorIntegral = constrain(rollErrorIntegral, -100.0f, 100.0f);
  
  float rollErrorDerivative = (rollError - rollErrorPrevious) / SAMPLE_TIME_S;
  rollErrorPrevious = rollError;
  
  float pidOutput = ATTITUDE_KP * rollError + 
                   ATTITUDE_KI * rollErrorIntegral + 
                   ATTITUDE_KD * rollErrorDerivative;
  pidOutput = constrain(pidOutput, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
  
  sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(pidOutput * 100));
}

// MODIFIED: 5-byte command packets
void sendBinaryCommand(uint8_t cmdType, int16_t value) {
  uint8_t packet[5];
  packet[0] = PACKET_START_BYTE;
  packet[1] = cmdType;
  packet[2] = (uint8_t)(value & 0xFF);
  packet[3] = (uint8_t)(value >> 8);
  packet[4] = packet[0] ^ packet[1] ^ packet[2] ^ packet[3];
  
  primarySerial.write(packet, sizeof(packet));
}

void processPrimaryPacket() {
  uint8_t packet[9];
  size_t n = primarySerial.readBytes(packet, 9);
  
  if (n != 9) {
    // Discard remaining bytes if packet is incomplete.
    while (primarySerial.available()) {
      primarySerial.read();
    }
    return;
  }
  
  if (packet[0] != PACKET_START_BYTE || packet[8] != PACKET_END_BYTE) {
    return;
  }
  
  uint8_t calcChecksum = 0;
  for (int i = 0; i < 7; i++) {
    calcChecksum ^= packet[i];
  }
  
  if (calcChecksum == packet[7]) {
    currentWheelRPM = packet[1] | (packet[2] << 8);
    currentWheelTorque = (float)(packet[3] | (packet[4] << 8)) / 100.0f;
    primaryStatus = packet[5];
    primaryFaults = packet[6];
  }
}

// ---------------------------------------------------------------------------
// 8. Command Handling & Telemetry
// ---------------------------------------------------------------------------
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String cmdString = Serial.readStringUntil('\n');
    cmdString.trim();
    
    if (cmdString.equalsIgnoreCase("GET")) {
      sendLocalTelemetry();
    }
    else if (cmdString.equalsIgnoreCase("AUTO")) {
      manualCommandActive = false;
    }
    else if (cmdString.startsWith("TARGET ")) {
      targetOrientation.roll = cmdString.substring(7).toFloat();
      manualCommandActive = false;
    }
    else if (cmdString.startsWith("T ")) {
      manualCommandActive = true;
      manualCommandTorque = cmdString.substring(2).toFloat();
      sendBinaryCommand(CMD_TYPE_TORQUE, (int16_t)(manualCommandTorque * 100));
    }
    else if (cmdString.startsWith("S ")) {
      manualCommandActive = true;
      manualCommandRPM = cmdString.substring(2).toFloat();
      sendBinaryCommand(CMD_TYPE_SPEED, (int16_t)manualCommandRPM);
    }
  }
}

void sendLocalTelemetry() {
  Serial.print("Roll: ");
  Serial.print(currentOrientation.roll, 1);
  Serial.print("° | Target: ");
  Serial.print(targetOrientation.roll, 1);
  Serial.print("° | Error: ");
  Serial.print(rollError, 1);
  Serial.print("° | Torque: ");
  Serial.print(currentWheelTorque, 1);
  Serial.print("mNm | RPM: ");
  Serial.println(currentWheelRPM);
  
}

// ---------------------------------------------------------------------------
// 9. Utility Functions
// ---------------------------------------------------------------------------
void updateLedBlink() {
  if (millis() - lastLedMillis >= LED_BLINK_INTERVAL_MS) {
    lastLedMillis = millis();
    digitalWrite(LED_PIN, (ledState = !ledState) ? HIGH : LOW);
  }
}
