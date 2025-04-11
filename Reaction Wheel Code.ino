// MCT8316Z_Motor_Controller.ino
// Using MotorDriver class for the MCT8316Z digital hall effect motor controller
// Enhanced with bidirectional torque control capabilities
// Added UART communication with Command & Control board

#include <Adafruit_ZeroTimer.h>
#include "MotorDriver.h"

//==============================================================================
// 1. SYSTEM CONSTANTS AND EXPERIMENT SETTINGS
//==============================================================================

#define N 6000                    // Number of samples in experiment
#define T 0.08                    // Sample period (seconds)
#define THRESHOLD 24000           // Timer threshold (12MHz clock)
#define KI 0                      // Integral gain
#define KP 1                      // Proportional gain
#define DEFAULT_MOTOR_INERTIA 5.1e-7 // Default motor inertia in kg*m^2

// UART communication with Command & Control board
#define CC_UART Serial1           // Using hardware Serial1 for communication
#define CC_UART_BAUD 115200       // Baud rate
#define TELEMETRY_INTERVAL 50     // Send telemetry every 50ms (20Hz)

// Protocol definitions
#define PACKET_START_BYTE    0xAA  // Start of packet marker
#define PACKET_END_BYTE      0x55  // End of packet marker
#define CMD_TYPE_SPEED       0x01  // Command type for speed control
#define CMD_TYPE_TORQUE      0x02  // Command type for torque control
#define CMD_TYPE_STATUS_REQ  0x03  // Request status update

//==============================================================================
// 2. PIN DEFINITIONS
//==============================================================================

// MCT8316Z control pins
const int MCT8316_nSLEEP_PIN = 4;     // nSLEEP pin
const int MCT8316_DRVOFF_PIN = 3;     // DRVOFF pin 
const int MCT8316_PWM_PIN = 0;        // PWM output from Arduino to MCT8316Z
const int MCT8316_ILIM_PIN = A0;      // Current limit pin

// MCT8316Z monitoring pins
const int MCT8316_nFAULT_PIN = 5;     // Fault monitoring pin
const int MCT8316_FGOUT_PIN = 6;      // FG output for speed feedback
const int MCT8316_ISENSE_PIN = A2;    // Current sense pin
const int MCT8316_VINSENSE_PIN = A3;  // Input voltage sense pin
const int MCT8316_24VSENSE_PIN = A1;  // 24V rail sense pin

// Hardware configuration pins
const int MCT8316_CS_PIN = 7;         // Direction control

const int MCT8316Z_BRAKE_PIN = 2;     // Brake control pin

//==============================================================================
// 3. GLOBAL VARIABLES
//==============================================================================

// Create motor driver instance
MotorDriver motor(MCT8316_nSLEEP_PIN, MCT8316_DRVOFF_PIN, MCT8316_PWM_PIN, 
                 MCT8316_FGOUT_PIN, MCT8316_nFAULT_PIN,
                 MCT8316_ILIM_PIN, MCT8316_CS_PIN, MCT8316Z_BRAKE_PIN);

// Constants for motor control
#define MOTOR_MAX_SPEED 512

// Timer and experiment variables
Adafruit_ZeroTimer zt4 = Adafruit_ZeroTimer(4);
volatile int16_t rpmLog[N];           // Log of RPM values for experiment
volatile int16_t cmdLog[N];           // Log of command values for experiment
volatile bool running = false;         // Experiment running flag
int sampleNum = 0;                     // Current sample number
float integral = 0;                    // Integral term for PID control

// UART communication variables
unsigned long lastTelemetryTime = 0;   // Last time telemetry was sent

//==============================================================================
// 4. INTERRUPT HANDLERS
//==============================================================================

void TC4_Handler() {
  Adafruit_ZeroTimer::timerHandler(4);
}

void Timer4Callback0() {
  if (running) {
    logData();
    updateController();
    sampleNum++;
    if (sampleNum >= N) {
      motor.stop();
      resetExperiment();
      transmitData();
      running = false;
    }
  }
}

void fgoutInterrupt() {
  motor.processFGOUTpulse();
}

void processCommandPacket();
void sendTelemetry(bool immediate = false);
//==============================================================================
// 5. SETUP FUNCTION
//==============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to connect (for native USB ports)
  while (!Serial);
  
  Serial.println("\n===== MCT8316Z Digital Hall Effect Motor Controller - Enhanced Version =====");
  
  // Initialize UART communication with Command & Control board
  CC_UART.begin(CC_UART_BAUD);
  
  // Initialize motor driver
  motor.begin();
  
  // Set default motor inertia for torque calculations
  motor.setMotorInertia(DEFAULT_MOTOR_INERTIA);
  
  // Configure timer interrupt for experiment sampling
  zt4.configure(TC_CLOCK_PRESCALER_DIV16,   // prescaler
                TC_COUNTER_SIZE_16BIT,      // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_FREQ  // match style
                );

  zt4.setCompare(0, THRESHOLD);  // 1 match, channel 0
  zt4.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer4Callback0);
  zt4.enable(true);
  
  // Attach interrupt for FGOUT pin if available
  if (digitalPinToInterrupt(MCT8316_FGOUT_PIN) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(MCT8316_FGOUT_PIN), fgoutInterrupt, FALLING);
    Serial.println("FGOUT interrupt attached");
  } else {
    Serial.println("FGOUT using polling mode");
  }
  
  // Print menu
  printMenu();
  
  Serial.println("UART communication with Command & Control board initialized");
}

//==============================================================================
// 6. MAIN LOOP
//==============================================================================
bool on = false;
void loop() {
  // Process incoming commands from Command & Control board
  processCommandPacket();
  
  // Send telemetry at regular intervals
  sendTelemetry();
  
  // Check for faults
  if (motor.checkFaults()) {
    if (!running) {
      Serial.println("\n*** FAULT DETECTED");
      Serial.print(micros()%10000000/10000.0);
      Serial.println("ms");
    }
  }
  
  // Process serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
  
  // Update controller
  motor.updatePID();
  delayMicroseconds(1000);
}

//==============================================================================
// 7. EXPERIMENT FUNCTIONS
//==============================================================================

void logData() {
  rpmLog[sampleNum] = motor.getCurrentRPM();
}

void transmitData() {
  for (int i = 0; i < N; i++) {
    Serial.write((uint8_t)(cmdLog[i]));
    Serial.write((uint8_t)(cmdLog[i] >> 8));
    Serial.write((uint8_t)(rpmLog[i]));
    Serial.write((uint8_t)(rpmLog[i] >> 8));
  }
}

void resetExperiment() {
  sampleNum = 0;
  integral = 0;
}

void updateController() {
  if (motor.isTorqueMode()) {
    cmdLog[sampleNum] = motor.setTargetTorque(cmdLog[sampleNum]);
  } else {
    cmdLog[sampleNum] = motor.setTargetRPM(cmdLog[sampleNum]);
  }
}

void getCMD() {
  int i = 0;
  Serial.println("Getting command values:");
  while (i < N) {
    while (!Serial.available());
    cmdLog[i] = Serial.parseInt();
    i++;
  }
  while (Serial.available()) {
    Serial.read();
  }
}

//==============================================================================
// 8. COMMAND PROCESSING
//==============================================================================

void processCommand(char cmd) {
  switch (cmd) {
    case '1': // Start motor
      if (!running) Serial.println("Starting motor...");
      motor.start();
      break;
      
    case '0': // Stop motor
      if (!running) Serial.println("Stopping motor...");
      motor.stop();
      break;
      
    case '+': // Increase speed
      motor.adjustSpeed(10);
      if (!running) {
        Serial.print("Speed increased to: ");
        Serial.println(motor.getCurrentSpeed());
      }
      break;
      
    case 'z': // Print all registers (debug)
      motor.printAllRegisters();
      break;
      
    case '-': // Decrease speed
      motor.adjustSpeed(-10);
      if (!running) {
        Serial.print("Speed decreased to: ");
        Serial.println(motor.getCurrentSpeed());
      }
      break;
      
    case 'f': // Set to full speed
      motor.setSpeed(MOTOR_MAX_SPEED);
      if (!running) Serial.println("Set to full speed");
      break;
      
    case 'h': // Set to half speed
      motor.setSpeed(MOTOR_MAX_SPEED / 2);
      if (!running) Serial.println("Set to half speed");
      break;
      
    case 'd': // Toggle direction
      motor.toggleDirection();
      if (!running) {
        Serial.print("Direction set to: ");
        Serial.println(motor.getDirection() ? "CCW" : "CW");
      }
      break;
      
    case 'c': // Clear faults
      if (!running) Serial.println("Clearing faults...");
      motor.clearFaults();
      break;
      
    case 'b': // Brake motor
      if (!running) Serial.println("Engaging brake...");
      motor.brake();
      break;
      
    case 'r': // Release brake
      if (!running) Serial.println("Releasing brake...");
      motor.releaseBrake();
      break;
    
    case 's': // Print status
      printStatus();
      break;
      
    case 'm': // Print menu
      printMenu();
      break;
      
    case '?': // Print configuration
      printConfiguration();
      break;
      
    case 'p': // Set PID parameters
      {
        Serial.println("Enter Kp value:");
        while (!Serial.available());
        float kp = Serial.parseFloat();
        
        Serial.println("Enter Ki value:");
        while (!Serial.available());
        float ki = Serial.parseFloat();
        
        Serial.println("Enter Kd value:");
        while (!Serial.available());
        float kd = Serial.parseFloat();
        
        motor.setPIDParameters(kp, ki, kd);
        Serial.println("PID parameters updated");
      }
      break;
      
    case 't': // Set target RPM for PID control
      {
        Serial.println("Enter target RPM:");
        while (!Serial.available());
        float targetRpm = Serial.parseFloat();
        
        motor.setTargetRPM(targetRpm);
        motor.enablePID(true);
        Serial.print("Target RPM set to: ");
        Serial.println(targetRpm);
      }
      break;
      
    case 'q': // Toggle PID control
      {
        Serial.println("Enable PID? (1=yes, 0=no):");
        while (!Serial.available());
        int enable = Serial.parseInt();
        
        motor.enablePID(enable == 1);
        Serial.println(enable == 1 ? "PID enabled" : "PID disabled");
      }
      break;
      
    case 'x': // Run experiment
      Serial.println("Experiment started.....");
      getCMD();
      if (motor.isTorqueMode()) {
        Serial.println("Running in TORQUE control mode");
      } else {
        Serial.println("Running in SPEED control mode");
      }
      running = true;
      motor.setSpeed(cmdLog[0]);
      motor.start();
      break;
      
    // New commands for torque control
    case 'M': // Toggle control mode (Speed vs Torque)
      {
        Serial.println("Select control mode: (1=Speed, 2=Torque):");
        while (!Serial.available());
        int mode = Serial.parseInt();
        
        if (mode == 1) {
          motor.setTorqueMode(false);
          Serial.println("Speed control mode enabled");
        } else if (mode == 2) {
          motor.setTorqueMode(true);
          Serial.println("Torque control mode enabled");
        } else {
          Serial.println("Invalid selection - no change made");
        }
      }
      break;
      
    case 'o': // Set target torque
      {
        Serial.println("Enter target torque (mNm, positive/negative for direction):");
        while (!Serial.available());
        float targetTorque = Serial.parseFloat();
        
        motor.setTargetTorque(targetTorque);
        motor.enablePID(true);
        Serial.print("Target torque set to: ");
        Serial.print(targetTorque);
        Serial.println(" mNm");
        
        if (targetTorque > 0) {
          Serial.println("Positive torque: Motor will accelerate in forward direction");
        } else if (targetTorque < 0) {
          Serial.println("Negative torque: Motor will accelerate in reverse direction");
        } else {
          Serial.println("Zero torque: Motor will maintain current speed");
        }
      }
      break;
      
    case 'I': // Set motor inertia
      {
        Serial.println("Enter motor inertia (kg*m^2, scientific notation OK):");
        while (!Serial.available());
        float inertia = Serial.parseFloat();
        
        motor.setMotorInertia(inertia);
        Serial.print("Motor inertia set to: ");
        Serial.print(inertia, 7);
        Serial.println(" kg*m^2");
      }
      break;
  }
  
  // Flush any remaining characters
  while (Serial.available()) {
    Serial.read();
  }
}

//==============================================================================
// 9. UTILITY FUNCTIONS
//==============================================================================

void printStatus() {
  Serial.println("\n----- Motor Status -----");
  Serial.print("Motor Running: ");
  Serial.println(motor.isRunning() ? "YES" : "NO");
  
  Serial.print("Control Mode: ");
  Serial.println(motor.isTorqueMode() ? "TORQUE" : "SPEED");

  Serial.print("Torque target: ");
  Serial.println(motor.getTargetTorque());

  
  Serial.print("Current Speed: ");
  Serial.print(motor.getCurrentRPM(), 1);
  Serial.println(" RPM");
  
  Serial.print("Raw Speed: ");
  Serial.print(motor.getRawRPM(), 1);
  Serial.println(" RPM");
  
  if (motor.isTorqueMode()) {
    Serial.print("Target Torque: ");
    Serial.print(motor.getTargetTorque(), 2);
    Serial.println(" mNm");
    
    Serial.print("Current Torque (est): ");
    Serial.print(motor.getCurrentTorque(), 2);
    Serial.println(" mNm");
    
    Serial.print("Target RPM: ");
    Serial.println(motor.getTargetRPM(), 1);
  } else {
    Serial.print("Target RPM: ");
    Serial.println(motor.getTargetRPM(), 1);
  }
  
  Serial.print("PWM Setting: ");
  Serial.print(motor.getCurrentSpeed());
  Serial.print(" (");
  Serial.print((float)abs(motor.getCurrentSpeed()) / MOTOR_MAX_SPEED * 100.0, 1);
  Serial.println("%)");
  
  Serial.print("Direction: ");
  Serial.println(motor.getDirection() ? "CCW" : "CW");
  
  Serial.print("Acceleration: ");
  Serial.print(motor.getFilteredAcceleration());
  Serial.println(" RPM/s");
  
  Serial.print("Fault Status: ");
  Serial.println(motor.checkFaults() ? "FAULT DETECTED" : "Normal");
  Serial.print("Current Limit: ");
  Serial.println(motor.getCurrentLimit());
  
  // Read analog inputs directly to maintain compatibility with original code
  Serial.print("24V Rail: ");
  Serial.println(analogRead(MCT8316_24VSENSE_PIN)/41.0);
  Serial.print("VIN: ");
  Serial.println(analogRead(MCT8316_VINSENSE_PIN)/28.4);
  Serial.print("Current: ");
  Serial.println(analogRead(MCT8316_ISENSE_PIN));
  
  Serial.println("------------------------");
}

void printConfiguration() {
  Serial.println("\n----- Driver Configuration -----");
  Serial.print("Control Mode: ");
  Serial.println(motor.isTorqueMode() ? "TORQUE" : "SPEED");
  
  Serial.print("Motor Inertia: ");
  Serial.print(motor.getMotorInertia(), 7);
  Serial.println(" kg*m^2");
  
  // Add more configuration details here if needed
  Serial.println("------------------------");
}

void printMenu() {
  Serial.println("\n===== Command Menu =====");
  Serial.println("1 - Start motor");
  Serial.println("0 - Stop motor");
  Serial.println("+ - Increase speed");
  Serial.println("- - Decrease speed");
  Serial.println("f - Full speed");
  Serial.println("h - Half speed");
  Serial.println("d - Toggle direction (stop first)");
  Serial.println("b - Brake motor");
  Serial.println("r - Release brake");
  Serial.println("c - Clear faults");
  Serial.println("s - Print status");
  Serial.println("? - Print configuration");
  Serial.println("p - Set PID parameters");
  Serial.println("t - Set target RPM for PID control");
  Serial.println("q - Toggle PID control");
  Serial.println("x - Run experiment");
  Serial.println("m - Show this menu");
  Serial.println("\n-- Torque Control --");
  Serial.println("M - Toggle control mode (Speed/Torque)");
  Serial.println("o - Set target torque (pos/neg)");
  Serial.println("I - Set motor inertia");
  Serial.println("=======================");
}

//==============================================================================
// 10. UART COMMUNICATION FUNCTIONS
//==============================================================================

void processCommandPacket() {
  // Check if data is available
  if (CC_UART.available() >= 5) { // Minimum packet size: start + type + value (2 bytes) + checksum
    
    // Look for start byte
    if (CC_UART.peek() != PACKET_START_BYTE) {
      CC_UART.read(); // Discard byte if not start byte
      return;
    }
    
    // Buffer for incoming packet (max 8 bytes)
    uint8_t packet[8];
    packet[0] = CC_UART.read(); // Start byte
    
    // Read command type
    packet[1] = CC_UART.read(); // Command type
    
    // Read value (2 bytes, little-endian)
    packet[2] = CC_UART.read(); // LSB
    packet[3] = CC_UART.read(); // MSB
    
    // Read checksum
    packet[4] = CC_UART.read();
    
    // Verify checksum (simple XOR of all previous bytes)
    uint8_t calculatedChecksum = packet[0] ^ packet[1] ^ packet[2] ^ packet[3];
    if (calculatedChecksum != packet[4]) {
      Serial.println("Command checksum error");
      return;
    }
    
    // Process command based on type
    uint16_t value = packet[2] | (packet[3] << 8); // Combine LSB and MSB
    
    switch (packet[1]) {
      case CMD_TYPE_SPEED:
        // Convert from signed integer to RPM
        motor.setTargetRPM((int16_t)value);
        motor.setTorqueMode(false);
        if (!running) Serial.println("Received Speed command");
        break;
        
      case CMD_TYPE_TORQUE:
        // Convert from fixed-point to floating-point (value is torque * 100)
        motor.setTargetTorque((int16_t)value / 100.0f);
        motor.setTorqueMode(true);
        if (!running) Serial.println("Received Torque command");
        break;
        
      case CMD_TYPE_STATUS_REQ:
        // Send telemetry immediately
        sendTelemetry(true);
        break;
        
      default:
        Serial.println("Unknown command type");
        break;
    }
  }
}

void sendTelemetry(bool immediate) {
  unsigned long currentTime = millis();
  
  // Send telemetry at the defined interval or immediately if requested
  if (immediate || (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL)) {
    lastTelemetryTime = currentTime;
    
    // Prepare telemetry packet
    uint8_t packet[9]; // Start + RPM (2 bytes) + Torque (2 bytes) + Status + Faults + Checksum + End
    
    packet[0] = PACKET_START_BYTE;
    
    // Current RPM (2 bytes, signed)
    int16_t rpm = motor.getCurrentRPM();
    packet[1] = rpm & 0xFF;         // LSB
    packet[2] = (rpm >> 8) & 0xFF;  // MSB
    
    // Current Torque (2 bytes, fixed-point: torque * 100)
    int16_t torque = motor.getCurrentTorque() * 100.0f;
    packet[3] = torque & 0xFF;        // LSB
    packet[4] = (torque >> 8) & 0xFF; // MSB
    
    // Status byte
    packet[5] = 0;
    if (motor.isRunning()) packet[5] |= 0x01;    // Bit 0: Running
    if (motor.isTorqueMode()) packet[5] |= 0x02; // Bit 1: Torque mode
    if (motor.getDirection()) packet[5] |= 0x04; // Bit 2: Direction (1=CCW)
    
    // Fault byte
    packet[6] = motor.checkFaults() ? 0x01 : 0x00;
    
    // Calculate checksum (XOR of all previous bytes)
    packet[7] = 0;
    for (int i = 0; i < 7; i++) {
      packet[7] ^= packet[i];
    }
    
    // End byte
    packet[8] = PACKET_END_BYTE;
    
    // Send the packet
    CC_UART.write(packet, 9);
  }
}