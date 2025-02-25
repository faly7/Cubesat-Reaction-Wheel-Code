/******************************************************************************
 * CubeSat Reaction Wheel — Primary Board Code
 *
 * Microcontroller: ATSAMD21 (Arduino-compatible) with Integrated Bluetooth
 * Motor Driver:   TI MCT8316 
 * Motor:          Maxon ECX Flat
 * Author:         Fahd Aly (faly7@gatech.edu)
 *
 * This code reads command inputs (RPM or torque) from the integrated Bluetooth,
 * uses the Hall interrupts to measure current wheel speed, runs a PID controller,
 * and drives the motor driver.
 ******************************************************************************/

 #include <Arduino.h>
 #include <SPI.h>      // For MCT8316 config via SPI
 
 // ---------------------------------------------------------------------------
 // 1. Pin Definitions and Constants
 // ---------------------------------------------------------------------------
 
 // Motor driver pins
 const int MCT8316_ENABLE_PIN    = 2;   // Enable pin for the driver
 const int MCT8316_DIRECTION_PIN = 3;   // Direction control pin
 const int MCT8316_PWM_PIN       = 5;   // PWM input pin
 const int MCT8316_CS_PIN        = 4;   // Chip Select for SPI
 
 // Hall sensor input pins
 const int HALL_A_PIN            = 6;
 const int HALL_B_PIN            = 7;
 const int HALL_C_PIN            = 8;
 
 // Fault pin for MCT8316
 const int MCT8316_nFAULT_PIN    = 10;  // Fault monitoring pin
 
 // Communication baud rates
 #define BT_BAUD    115200  // Integrated Bluetooth baud rate
 #define UART_BAUD  115200  // UART to secondary board baud rate
 #define DEBUG_BAUD 115200  // USB Serial debugging baud rate
 
 // Command definitions
 enum CommandMode {
   SPEED_MODE,
   TORQUE_MODE
 };
 
 #define MAX_SPEED_CMD   12000  // Maximum RPM command
 #define MAX_TORQUE_CMD  1000   // Arbitrary torque range
 #define MAX_PWM         255    // 8-bit PWM duty cycle
 
 // ---------------------------------------------------------------------------
 // 2. Global Variables
 // ---------------------------------------------------------------------------
 CommandMode currentMode   = SPEED_MODE;
 float       targetCommand = 0.0f;      // Target RPM or torque
 
 // -- Hall & Speed Measurement --
 volatile float currentSpeedRPM = 0.0f;
 volatile uint8_t hallState     = 0;
 volatile uint32_t lastHallTime = 0;
 
 // PID Controller variables
 float Kp = 0.5f;                // Proportional gain
 float Ki = 0.1f;                // Integral gain  
 float Kd = 0.05f;               // Derivative gain
 
 float errorIntegral      = 0.0f;   // Accumulated integral error
 float lastError          = 0.0f;   // Last error (for derivative)
 float filteredDerivative = 0.0f;   // Low-pass filtered derivative
 float controlOutput      = 0.0f;   // Final PID output (0-255 PWM)
 
 const float dt = 0.01f;  // Control loop period (10 ms -> 100 Hz)
 
 // Telemetry interval (milliseconds)
 unsigned long lastTelemetryMillis = 0;
 const unsigned long TELEMETRY_INTERVAL_MS = 2500; // 2.5 seconds
 
 // Fault handling
 bool motorFaultActive = false;
 bool allowReverse = true;  // Allow negative speeds (reverse)
 
 // Hardware serial for communication with Secondary board
 HardwareSerial &secondarySerial = Serial1;
 
 // ---------------------------------------------------------------------------
 // 3. Forward Declarations
 // ---------------------------------------------------------------------------
 void initSystemClock();          // System clock init (if needed)
 void initPins();                 // Initialize GPIO pins
 void initMotorDriver();          // SPI setup and MCT8316 configuration
 void initBoostConverter();       // Power system init (TBD)
 
 void handleBTCommands();         // Process incoming Bluetooth commands
 void handleSecondaryCommands();  // Process commands from secondary board
 void updatePID();                // Run PID controller update
 void updateMotorDriver(float cmd);   // Update PWM output to motor driver
 
 float clampValue(float val, float minVal, float maxVal); // Utility: constrain value
 
 // SPI helper for MCT8316 config
 void writeRegister16(uint8_t addr6, uint8_t data8, bool readNotWrite);
 uint16_t readRegister16(uint8_t addr6);
 
 // Hall sensor ISR
 void hallISR();
 
 // Fault handling functions
 void checkMCT8316Fault();
 void clearFault();
 
 // ---------------------------------------------------------------------------
 // 4. setup()
 // ---------------------------------------------------------------------------
 void setup() {
   // Initialize USB Serial for debugging
   Serial.begin(DEBUG_BAUD);
   delay(500);  // Give time for serial to connect
 
   // Initialize UART connection to secondary board
   secondarySerial.begin(UART_BAUD);
 
   // Initialize system components
   initSystemClock();
   initPins();
   initMotorDriver();
   initBoostConverter();
 
   Serial.println("=== Reaction Wheel Control Board (Primary) ===");
   Serial.println("SAMD21 Init Complete.");
 
   // Send initial status to secondary board
   secondarySerial.println("INIT_COMPLETE");
 }
 
 // ---------------------------------------------------------------------------
 // 5. loop()
 // ---------------------------------------------------------------------------
 void loop() {
   // 5.1 Process incoming Bluetooth commands (directly from Serial)
   handleBTCommands();
   
   // 5.2 Process commands from secondary board (via UART)
   handleSecondaryCommands();
 
   // 5.3 Check for motor driver fault
   checkMCT8316Fault();
   if (motorFaultActive) {
     // On fault, ensure PWM remains at 0
     analogWrite(MCT8316_PWM_PIN, 0);
   } else {
     // 5.4 Run PID update
     updatePID();
     // 5.5 Update motor driver PWM output
     updateMotorDriver(controlOutput);
   }
 
   // 5.6 Telemetry: send status periodically
   if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
     lastTelemetryMillis = millis();
     
     // Send telemetry to secondary board
     secondarySerial.print("MODE:");
     secondarySerial.print((currentMode == SPEED_MODE) ? "Speed" : "Torque");
     secondarySerial.print(",CMD:");
     secondarySerial.print(targetCommand);
     secondarySerial.print(",RPM:");
     secondarySerial.print(currentSpeedRPM);
     secondarySerial.print(",OUT:");
     secondarySerial.print(controlOutput);
     secondarySerial.print(",FLT:");
     secondarySerial.println(motorFaultActive ? "1" : "0");
 
     // Also print to USB Serial for debugging
     Serial.print("Mode: ");
     Serial.print((currentMode == SPEED_MODE) ? "Speed" : "Torque");
     Serial.print(" | TargetCmd: ");
     Serial.print(targetCommand);
     Serial.print(" | CurrentSpeed(RPM): ");
     Serial.print(currentSpeedRPM);
     Serial.print(" | ControlOut: ");
     Serial.print(controlOutput);
     Serial.print(" | Fault: ");
     Serial.println(motorFaultActive ? "YES" : "NO");
   }
 
   // 5.7 Loop delay (~10ms for 100Hz control loop)
   delay(10);
 }
 
 // ---------------------------------------------------------------------------
 // 6. Implementation
 // ---------------------------------------------------------------------------
 
 /** 6.1 System Clock Initialization */
 void initSystemClock() {
   // The SAMD MCU typically initializes the clock automatically.
 }
 
 /** 6.2 Pin Initialization */
 void initPins() {
   pinMode(MCT8316_ENABLE_PIN, OUTPUT);
   pinMode(MCT8316_DIRECTION_PIN, OUTPUT);
   pinMode(MCT8316_PWM_PIN, OUTPUT);
 
   pinMode(MCT8316_CS_PIN, OUTPUT);
   digitalWrite(MCT8316_CS_PIN, HIGH);
 
   // Hall sensor pins as inputs with pull-ups
   pinMode(HALL_A_PIN, INPUT_PULLUP);
   pinMode(HALL_B_PIN, INPUT_PULLUP);
   pinMode(HALL_C_PIN, INPUT_PULLUP);
 
   // Set default states
   digitalWrite(MCT8316_ENABLE_PIN, LOW);  // disable driver initially
   digitalWrite(MCT8316_DIRECTION_PIN, LOW);
   analogWrite(MCT8316_PWM_PIN, 0);
 
   // Attach interrupts for Hall sensors
   attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
   attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);
   attachInterrupt(digitalPinToInterrupt(HALL_C_PIN), hallISR, CHANGE);
 
   // nFAULT pin setup (with pull-up if needed)
   pinMode(MCT8316_nFAULT_PIN, INPUT_PULLUP);
 }
 
 /** 6.3 Motor Driver Initialization via SPI */
 void initMotorDriver() {
   SPI.begin();
   SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
 
   // Enable driver
   digitalWrite(MCT8316_ENABLE_PIN, HIGH);
   // Write configuration registers (per datasheet)
   writeRegister16(0x08, 0x02, false);
   writeRegister16(0x06, 0x10, false);
 
   SPI.endTransaction();
   Serial.println("MCT8316 driver configured via SPI.");
 }
 
 /** 6.4 Boost Converter Initialization */
 void initBoostConverter() {
   // TBD: Initialize any boost converter or power management circuitry.
 }
 
 /** 6.5 Hall Sensor ISR: Compute RPM */
 void hallISR() {
   uint32_t now = micros();
   uint8_t newState = (digitalRead(HALL_A_PIN) << 2) |
                      (digitalRead(HALL_B_PIN) << 1) |
                      (digitalRead(HALL_C_PIN));
 
   if (newState != hallState) {
     float timeDiff = (now - lastHallTime) * 1e-6f; // in seconds
     if (timeDiff > 1e-6f) {  // Prevent division by zero
       // 6 transitions equal 1 revolution
       float revPerSec = 1.0f / (6.0f * timeDiff);
       currentSpeedRPM = revPerSec * 60.0f;
       
       // Adjust sign based on direction pin
       if (digitalRead(MCT8316_DIRECTION_PIN) == HIGH) {
         currentSpeedRPM = -currentSpeedRPM;
       }
     }
     lastHallTime = now;
     hallState = newState;
   }
 }
 
 /** 6.6 Handle Incoming Bluetooth Commands (from Serial) */
 void handleBTCommands() {
   // Check if data is available on the Bluetooth serial port
   if (Serial.available() > 0) {
     String cmdString = Serial.readStringUntil('\n');
     cmdString.trim();
     if (cmdString.length() < 1) return;
 
     processCommand(cmdString, true); // true = reply to Bluetooth
   }
 }
 
 /** 6.7 Handle Commands from Secondary Board */
 void handleSecondaryCommands() {
   // Check if data is available from secondary board
   if (secondarySerial.available() > 0) {
     String cmdString = secondarySerial.readStringUntil('\n');
     cmdString.trim();
     if (cmdString.length() < 1) return;
 
     processCommand(cmdString, false); // false = don't reply (secondary already knows)
   }
 }
 
 /** 6.8 Process Command String */
 void processCommand(String cmdString, bool replyToBT) {
   // If the command is "CLR", attempt to clear fault
   if (cmdString.equalsIgnoreCase("CLR")) {
     clearFault();
     if (replyToBT) {
       Serial.println("[CMD] Attempted CLEAR of latched faults.");
     }
     return;
   }
 
   // Expected command format: "S 3000" for speed or "T 100" for torque
   if (cmdString.length() < 2) return;  // Minimal valid command length
 
   char modeChar = cmdString.charAt(0);
   float value   = cmdString.substring(2).toFloat();
 
   if (modeChar == 'S' || modeChar == 's') {
     currentMode = SPEED_MODE;
     // For bidirectional control, set direction pin accordingly
     if (value < 0 && allowReverse) {
       digitalWrite(MCT8316_DIRECTION_PIN, HIGH);
       value = -value;
     } else {
       digitalWrite(MCT8316_DIRECTION_PIN, LOW);
     }
     targetCommand = constrain(value, 0, (float)MAX_SPEED_CMD);
     
     if (replyToBT) {
       Serial.print("[CMD] Speed Mode => target RPM = ");
       Serial.println(targetCommand);
     }
   }
   else if (modeChar == 'T' || modeChar == 't') {
     currentMode = TORQUE_MODE;
     if (value < 0 && allowReverse) {
       digitalWrite(MCT8316_DIRECTION_PIN, HIGH);
       value = -value;
     } else {
       digitalWrite(MCT8316_DIRECTION_PIN, LOW);
     }
     targetCommand = constrain(value, 0, (float)MAX_TORQUE_CMD);
     
     if (replyToBT) {
       Serial.print("[CMD] Torque Mode => target Torque = ");
       Serial.println(targetCommand);
     }
   }
   else if (replyToBT) {
     Serial.println("[CMD] Unrecognized command!");
   }
 }
 
 /** 6.9 PID Controller Update (with anti-windup and derivative filtering) */
 void updatePID() {
   if (currentMode == SPEED_MODE) {
     float error = targetCommand - currentSpeedRPM;
     errorIntegral += error * dt;
     float errorDerivative = (error - lastError) / dt;
     lastError = error;
 
     // Low-pass filter the derivative term
     float alpha = 0.2f;
     filteredDerivative = alpha * errorDerivative + (1.0f - alpha) * filteredDerivative;
 
     float rawOutput = (Kp * error) + (Ki * errorIntegral) + (Kd * filteredDerivative);
 
     // Anti-windup: if output saturates, back out the recently added integral term
     if (rawOutput > MAX_PWM || rawOutput < 0) {
       errorIntegral -= error * dt;
     }
 
     controlOutput = clampValue(rawOutput, 0.0f, (float)MAX_PWM);
   }
   else {
     // Torque mode: simple mapping from targetCommand to PWM range
     controlOutput = map((int)targetCommand, 0, MAX_TORQUE_CMD, 0, MAX_PWM);
   }
 }
 
 /** 6.10 Update the Motor Driver (PWM output) */
 void updateMotorDriver(float cmd) {
   analogWrite(MCT8316_PWM_PIN, (int)cmd);
 }
 
 /** 6.11 Utility: Clamp Value */
 float clampValue(float val, float minVal, float maxVal) {
   if (val < minVal) return minVal;
   if (val > maxVal) return maxVal;
   return val;
 }
 
 // ---------------------------------------------------------------------------
 // SPI Communication Helpers for MCT8316 (16-bit frame with parity)
 // ---------------------------------------------------------------------------
 void writeRegister16(uint8_t addr6, uint8_t data8, bool readNotWrite) {
   // Build 16-bit frame: [R/W (1 bit) | address (6 bits) | parity (1 bit) | data (8 bits)]
   uint16_t frame = 0;
   frame |= ((readNotWrite ? 1 : 0) << 15);
   frame |= ((addr6 & 0x3F) << 9);
 
   // Temporarily combine to calculate parity
   uint16_t tempNoParity = frame | (data8 & 0xFF);
   uint8_t bitCount = 0;
   for (int i = 0; i < 16; i++) {
     if (tempNoParity & (1 << i)) bitCount++;
   }
   uint8_t parityBit = (bitCount % 2 == 0) ? 0 : 1;
   frame |= (parityBit << 8);
   frame |= (data8 & 0x00FF);
 
   digitalWrite(MCT8316_CS_PIN, LOW);
   SPI.transfer16(frame);
   digitalWrite(MCT8316_CS_PIN, HIGH);
 }
 
 uint16_t readRegister16(uint8_t addr6) {
   uint16_t frame = 0;
   frame |= (1 << 15); // Set R/W=1 for read
   frame |= ((addr6 & 0x3F) << 9);
   uint16_t tempNoParity = frame;
   uint8_t bitCount = 0;
   for (int i = 0; i < 16; i++) {
     if (tempNoParity & (1 << i)) bitCount++;
   }
   uint8_t parityBit = (bitCount % 2 == 0) ? 0 : 1;
   frame |= (parityBit << 8);
 
   digitalWrite(MCT8316_CS_PIN, LOW);
   uint16_t resp = SPI.transfer16(frame);
   digitalWrite(MCT8316_CS_PIN, HIGH);
 
   return resp;  // Returned frame: [status (8 bits) | regData (8 bits)]
 }
 
 // ---------------------------------------------------------------------------
 // Fault Handling
 // ---------------------------------------------------------------------------
 void checkMCT8316Fault() {
   if (digitalRead(MCT8316_nFAULT_PIN) == LOW) {
     Serial.println("[FAULT] MCT8316 nFAULT is LOW => driver fault active.");
     secondarySerial.println("FAULT:DRIVER_FAULT");
     motorFaultActive = true;
   }
 }
 
 void clearFault() {
   // Clear fault according to the datasheet: set CLR_FLT bit in Control_Register_2A (offset 0x04)
   writeRegister16(0x04, 0x01, false);
   digitalWrite(MCT8316_ENABLE_PIN, HIGH);
   motorFaultActive = false;
   Serial.println("[CMD] Fault cleared.");
   secondarySerial.println("FAULT:CLEARED");
 }