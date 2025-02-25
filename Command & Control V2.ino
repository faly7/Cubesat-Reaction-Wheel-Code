/******************************************************************************
 * CubeSat Reaction Wheel — Secondary (Command & Control) Board Code
 *
 * Microcontroller: Teensy 4.1 (Integrated Bluetooth)
 * Sensor:         Inertial Measurement Unit (MPU6050)
 * Communication:  Bluetooth (via built-in Serial interface) for ground commands/telemetry,
 *                 and UART (Serial1) for interfacing with the primary board.
 * Author:         Fahd Aly (faly7@gatech.edu)
 *
 * This code initializes an IMU to monitor the CubeSat's orientation (roll, pitch, yaw),
 * processes any user commands received over Bluetooth, and transmits orientation telemetry.
 * It also calculates the needed reaction wheel speeds based on orientation error.
 ******************************************************************************/

 #include <Arduino.h>
 #include <Wire.h>           // For I2C communication with the IMU
 #include <MPU6050.h>        // For the MPU6050 sensor
 
 // ---------------------------------------------------------------------------
 // 1. Pin Definitions and Constants
 // ---------------------------------------------------------------------------
 
 const int LED_PIN = LED_BUILTIN;  // Built-in LED for status indication
 
 // I2C address for the MPU6050 (default is 0x68)
 #define MPU6050_ADDRESS 0x68
 
 // Telemetry interval (milliseconds)
 const unsigned long TELEMETRY_INTERVAL_MS = 1000; // 1 second telemetry update
 
 // Sample rate for sensor fusion and control loop
 const float SAMPLE_TIME_S = 0.01f; // 10ms sample time (100Hz)
 
 // LED blink timing (non-blocking)
 const unsigned long LED_BLINK_INTERVAL_MS = 50;
 
 // Attitude control constants
 #define CONTROL_ENABLE true  // Set to false to disable attitude control
 #define TARGET_ROLL 0.0f     // Target orientation (level in roll axis)
 #define MAX_CONTROL_RPM 5000 // Maximum RPM that can be commanded for attitude control
 
 // PID constants for attitude control
 const float ATTITUDE_KP = 50.0f;  // Proportional gain (RPM per degree of error)
 const float ATTITUDE_KI = 5.0f;   // Integral gain
 const float ATTITUDE_KD = 10.0f;  // Derivative gain
 
 // Sensor fusion constants
 const float COMPLEMENTARY_ALPHA = 0.98f; // Weight factor for gyro vs. accel (higher = more gyro)
 
 // ---------------------------------------------------------------------------
 // 2. Global Variables
 // ---------------------------------------------------------------------------
 
 // For the IMU, we're using the MPU6050 as specified in the project proposal.
 MPU6050 imu;
 
 // Structure to hold orientation data (roll, pitch, yaw in degrees)
 struct Orientation {
   float roll;
   float pitch;
   float yaw;
 };
 
 // Current and target orientation
 Orientation currentOrientation = {0.0f, 0.0f, 0.0f};
 Orientation targetOrientation = {0.0f, 0.0f, 0.0f}; // Default to level
 
 // Timing variables
 unsigned long lastTelemetryMillis = 0;
 unsigned long lastControlMillis = 0;
 unsigned long lastLedMillis = 0;
 unsigned long previousMicros = 0;
 
 // LED state (for non-blocking blink)
 bool ledState = false;
 
 // Sensor fusion variables
 float gyroRates[3] = {0.0f, 0.0f, 0.0f}; // Roll, pitch, yaw rates in deg/sec
 float previousRoll = 0.0f;
 float previousPitch = 0.0f;
 
 // Attitude control variables
 float rollError = 0.0f;
 float rollErrorIntegral = 0.0f;
 float rollErrorPrevious = 0.0f;
 float commandedWheelRPM = 0.0f;
 
 // Manual command override (when user sends direct commands)
 bool manualCommandActive = false;
 float manualCommandRPM = 0.0f;
 
 // ---------------------------------------------------------------------------
 // 2.1 Integration with Primary Microcontroller
 // ---------------------------------------------------------------------------
 // We use Serial1 to communicate with the primary reaction wheel board.
 // Fahd - Ensure that the wiring on the Teensy 4.1 board assigns the correct TX/RX pins for Serial1.
 #define PRIMARY_BAUD 115200
 HardwareSerial &primarySerial = Serial1;
 
 // ---------------------------------------------------------------------------
 // 3. Forward Declarations
 // ---------------------------------------------------------------------------
 
 void initIMU();                           // Initialize the IMU sensor
 void readIMU();                           // Read and update orientation data from the IMU
 void calculateOrientation(float dt);      // Calculate orientation using sensor fusion
 void updateAttitudeControl();             // Update the attitude control loop
 void handleSerialCommands();              // Process incoming Bluetooth commands
 void sendTelemetry();                     // Transmit orientation telemetry
 void forwardToPrimary(String cmd);        // Forward commands to the primary board via Serial1
 void updateLedBlink();                    // Non-blocking LED blink
 
 // ---------------------------------------------------------------------------
 // 4. setup()
 // ---------------------------------------------------------------------------
 void setup()
 {
   // Initialize built-in LED for status indication
   pinMode(LED_PIN, OUTPUT);
   
   // Initialize Bluetooth Serial communication for commands and telemetry
   Serial.begin(115200);
   // Wait a bit for serial connection if needed
   delay(500);
   
   // Initialize Serial1 for communication with the primary microcontroller
   primarySerial.begin(PRIMARY_BAUD);
   
   // Initialize I2C communication for the IMU
   Wire.begin();
 
   // Initialize the IMU sensor
   initIMU();
 
   // Set target orientation to level
   targetOrientation.roll = TARGET_ROLL;
   targetOrientation.pitch = 0.0f;
   targetOrientation.yaw = 0.0f;
 
   // Initialize timing
   previousMicros = micros();
   
   Serial.println("=== Reaction Wheel Command & Control Board (Secondary) ===");
   Serial.println("IMU Initialization Complete. Awaiting Bluetooth commands and telemetry...");
 }
 
 // ---------------------------------------------------------------------------
 // 5. loop()
 // ---------------------------------------------------------------------------
 void loop()
 {
   // Calculate elapsed time since last cycle
   unsigned long currentMicros = micros();
   float deltaTime = (currentMicros - previousMicros) / 1000000.0f;
   previousMicros = currentMicros;
   
   // Process any incoming Bluetooth commands from the Serial interface
   handleSerialCommands();
   
   // Read data from the IMU and update orientation with sensor fusion
   readIMU();
   calculateOrientation(deltaTime);
   
   // Update attitude control every 10ms (100Hz)
   if (millis() - lastControlMillis >= (SAMPLE_TIME_S * 1000)) {
     lastControlMillis = millis();
     updateAttitudeControl();
   }
   
   // Transmit telemetry data at defined intervals
   if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
     lastTelemetryMillis = millis();
     sendTelemetry();
   }
   
   // Check for data from the primary microcontroller
   if (primarySerial.available() > 0) {
     String primaryMsg = primarySerial.readStringUntil('\n');
     primaryMsg.trim();
     if (primaryMsg.length() > 0) {
       Serial.print("[PRIMARY] ");
       Serial.println(primaryMsg);
     }
   }
   
   // Non-blocking LED blink
   updateLedBlink();
 }
 
 // ---------------------------------------------------------------------------
 // 6. Implementation
 // ---------------------------------------------------------------------------
 
 /** 6.1 Initialize the IMU Sensor */
 void initIMU()
 {
   Serial.println("Initializing MPU6050...");
   imu.initialize();
   
   // Verify connection to the MPU6050
   if (imu.testConnection()) {
     Serial.println("MPU6050 connection successful.");
     
     // Configure the MPU6050 for appropriate sensitivity
     imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250 °/s
     imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
     
     // Optional: Set digital low-pass filter
     imu.setDLPFMode(MPU6050_DLPF_BW_20);  // 20Hz low-pass filter
   } else {
     Serial.println("MPU6050 connection failed. Check wiring!");
   }
 }
 
 /** 6.2 Read IMU Raw Data */
 void readIMU()
 {
   // Variables to store raw accelerometer and gyroscope data
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
   
   // Read raw sensor data from the MPU6050
   imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
   // Convert raw gyro values to degrees per second
   // Sensitivity depends on the range set with setFullScaleGyroRange
   // For ±250 °/s: 131 LSB/°/s
   gyroRates[0] = (float)gx / 131.0f; // Roll rate
   gyroRates[1] = (float)gy / 131.0f; // Pitch rate
   gyroRates[2] = (float)gz / 131.0f; // Yaw rate
   
   // Calculate accelerometer angles - these will be combined with gyro data in calculateOrientation()
   float accelRoll = atan2((float)ay, (float)az) * 57.2958f; // Convert radians to degrees
   float accelPitch = atan2(-(float)ax, sqrt((float)(ay * ay + az * az))) * 57.2958f;
   
   // Store in temporary variables (will be fused with gyro data in calculateOrientation)
   currentOrientation.roll = accelRoll;
   currentOrientation.pitch = accelPitch;
   // Yaw is not reliable with just accelerometer data, so we'll integrate it from gyro only
 }
 
 /** 6.3 Calculate Orientation using Sensor Fusion */
 void calculateOrientation(float dt)
 {
   // Prevent division by zero or unrealistic dt values
   if (dt <= 0.0f || dt > 0.1f) {
     dt = SAMPLE_TIME_S; // Use default sample time if dt is unrealistic
   }
   
   // Complementary filter for roll and pitch
   // 1. Predict orientation using gyro integration
   float gyroRoll = previousRoll + gyroRates[0] * dt;
   float gyroPitch = previousPitch + gyroRates[1] * dt;
   
   // 2. Correct using accelerometer data (already stored in currentOrientation)
   float filteredRoll = COMPLEMENTARY_ALPHA * gyroRoll + (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.roll;
   float filteredPitch = COMPLEMENTARY_ALPHA * gyroPitch + (1.0f - COMPLEMENTARY_ALPHA) * currentOrientation.pitch;
   
   // 3. Store results
   previousRoll = filteredRoll;
   previousPitch = filteredPitch;
   currentOrientation.roll = filteredRoll;
   currentOrientation.pitch = filteredPitch;
   
   // For yaw, I'm integrating gyro directly (drift will occur without magnetometer)
   currentOrientation.yaw += gyroRates[2] * dt;
   
   // Normalize yaw to 0-360 degrees
   while (currentOrientation.yaw < 0) currentOrientation.yaw += 360.0f;
   while (currentOrientation.yaw >= 360.0f) currentOrientation.yaw -= 360.0f;
 }
 
 /** 6.4 Update Attitude Control Loop */
 void updateAttitudeControl()
 {
   // Skip control update if manual command is active
   if (manualCommandActive) {
     // Forward manual command directly to primary board
     String speedCmd = "S " + String(manualCommandRPM, 0);
     forwardToPrimary(speedCmd);
     return;
   }
   
   // Skip if attitude control is disabled
   if (!CONTROL_ENABLE) {
     return;
   }
   
   // Calculate error for roll axis (single-axis control for our specific reaction wheel)
   rollError = targetOrientation.roll - currentOrientation.roll;
   
   // Integrate error for I term
   rollErrorIntegral += rollError * SAMPLE_TIME_S;
   
   // Anti-windup: limit integral term
   const float MAX_INTEGRAL = 100.0f;
   if (rollErrorIntegral > MAX_INTEGRAL) rollErrorIntegral = MAX_INTEGRAL;
   if (rollErrorIntegral < -MAX_INTEGRAL) rollErrorIntegral = -MAX_INTEGRAL;
   
   // Calculate derivative (change in error)
   float rollErrorDerivative = (rollError - rollErrorPrevious) / SAMPLE_TIME_S;
   rollErrorPrevious = rollError;
   
   // PID control law
   commandedWheelRPM = ATTITUDE_KP * rollError + 
                        ATTITUDE_KI * rollErrorIntegral + 
                        ATTITUDE_KD * rollErrorDerivative;
   
   // Limit commanded RPM
   if (commandedWheelRPM > MAX_CONTROL_RPM) commandedWheelRPM = MAX_CONTROL_RPM;
   if (commandedWheelRPM < -MAX_CONTROL_RPM) commandedWheelRPM = -MAX_CONTROL_RPM;
   
   // Create speed command
   String speedCmd = "S " + String(commandedWheelRPM, 0);
   
   // Forward command to primary board
   forwardToPrimary(speedCmd);
 }
 
 /** 6.5 Handle Incoming Bluetooth Commands */
 void handleSerialCommands()
 {
   if (Serial.available() > 0)
   {
     String cmdString = Serial.readStringUntil('\n');
     cmdString.trim();
     if (cmdString.length() < 1) return;
     
     // Process known commands:
     if (cmdString.equalsIgnoreCase("GET")) {
       // Force immediate telemetry update
       sendTelemetry();
       Serial.println("[CMD] Telemetry sent on demand.");
     } 
     else if (cmdString.equalsIgnoreCase("AUTO")) {
       // Switch to automatic attitude control
       manualCommandActive = false;
       Serial.println("[CMD] Switched to automatic attitude control.");
     }
     else if (cmdString.startsWith("TARGET ")) {
       // Set target roll angle for attitude control
       // Format: "TARGET 10.5" for +10.5 degrees roll
       float newTarget = cmdString.substring(7).toFloat();
       targetOrientation.roll = newTarget;
       Serial.print("[CMD] New target roll set to: ");
       Serial.println(newTarget, 2);
       
       // Ensure we're in automatic mode
       manualCommandActive = false;
     }
     else if (cmdString.startsWith("S") || cmdString.startsWith("s")) {
       // Direct speed command - override attitude control
       manualCommandActive = true;
       manualCommandRPM = cmdString.substring(2).toFloat();
       
       // Forward to primary directly
       forwardToPrimary(cmdString);
       Serial.print("[CMD] Manual speed command: ");
       Serial.println(manualCommandRPM, 0);
     }
     else if (cmdString.startsWith("T") || cmdString.startsWith("t") ||
             cmdString.equalsIgnoreCase("CLR")) {
       // Other commands forwarded directly to primary
       forwardToPrimary(cmdString);
       Serial.print("[CMD] Command forwarded to primary: ");
       Serial.println(cmdString);
     }
     else {
       Serial.println("[CMD] Unrecognized command!");
     }
   }
 }
 
 /** 6.6 Transmit Orientation Telemetry */
 void sendTelemetry()
 {
   // First line: Orientation data
   Serial.print("ORIENTATION | Roll: ");
   Serial.print(currentOrientation.roll, 2);
   Serial.print(" | Pitch: ");
   Serial.print(currentOrientation.pitch, 2);
   Serial.print(" | Yaw: ");
   Serial.println(currentOrientation.yaw, 2);
   
   // Second line: Control data
   Serial.print("CONTROL | Target: ");
   Serial.print(targetOrientation.roll, 2);
   Serial.print(" | Error: ");
   Serial.print(rollError, 2);
   Serial.print(" | Mode: ");
   Serial.print(manualCommandActive ? "MANUAL" : "AUTO");
   Serial.print(" | Wheel RPM: ");
   Serial.println(manualCommandActive ? manualCommandRPM : commandedWheelRPM, 0);
 }
 
 /** 6.7 Forward Command to Primary Board */
 void forwardToPrimary(String cmd)
 {
   // Forward the command string over Serial1 (with newline termination)
   primarySerial.println(cmd);
 }
 
 /** 6.8 Non-blocking LED Blink */
 void updateLedBlink()
 {
   if (millis() - lastLedMillis >= LED_BLINK_INTERVAL_MS) {
     lastLedMillis = millis();
     ledState = !ledState;
     digitalWrite(LED_PIN, ledState ? HIGH : LOW);
   }
 }