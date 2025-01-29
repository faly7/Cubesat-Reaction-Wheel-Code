/******************************************************************************
 * CubeSat Reaction Wheel — Primary Board Firmware
 *
 * Microcontroller: ATSAMD21
 * Motor Driver:  TI MCT8316 
 * Motor:         Maxon ECX Flat
 * Author:        Fahd Aly (faly7@gatech.edu)
 * This code reads command inputs (RPM or torque) from UART, uses the Hall interrupts
 * to measure current wheel speed, runs a PID controller, and drives the motor driver.
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>  // For MCT8316 config via SPI

// ---------------------------------------------------------------------------
// 1. Pin Definitions and Constants
// ---------------------------------------------------------------------------

// PLACE HOLDER PINS! I'll them once the hardware arrives!
const int MCT8316_ENABLE_PIN    = 2;   // Enable pin for the driver
const int MCT8316_DIRECTION_PIN = 3;   // Direction enable pin
const int MCT8316_PWM_PIN       = 5;   // PWM input pin
const int MCT8316_CS_PIN        = 4;   // Chip Select pin

// Hall sensor inputs (GPIO pins), I'll change these pins as well to the correct ones:
const int HALL_A_PIN            = 6;
const int HALL_B_PIN            = 7;
const int HALL_C_PIN            = 8;

// Serial port used for command/telemetry
#define SERIAL_BAUD  115200

enum CommandMode
{
  SPEED_MODE,
  TORQUE_MODE
};

// I may change the torque range, it's just a placeholder for now once i get more details.
#define MAX_SPEED_CMD   12000  // Max RPM command
#define MAX_TORQUE_CMD  1000   // Arbitrary torque range
#define MAX_PWM         255    // 8-bit duty cycle on SAMD21

// ---------------------------------------------------------------------------
// 2. Global Variables
// ---------------------------------------------------------------------------

CommandMode currentMode   = SPEED_MODE;
float       targetCommand = 0.0f;      // Could be target RPM or torque

// -- Hall & Speed Measurement --
volatile float currentSpeedRPM = 0.0f;
volatile uint8_t hallState     = 0;
volatile uint32_t lastHallTime = 0;

/* PID Controller */
float Kp = 0.5f;                // Proportional gain
float Ki = 0.1f;                // Integral gain  
float Kd = 0.05f;               // Derivative gain

float errorIntegral      = 0.0f;   // Accumulated integral error
float lastError          = 0.0f;   // Previous error for derivative calculation
float filteredDerivative = 0.0f;   // Low-pass filtered derivative term
float controlOutput      = 0.0f;   // Final PID output (0-255 PWM value)

const float dt = 0.01f;  // Control loop period - 10 ms cycle -> 100 Hz (I may also change this but probably once the hardware arrives and I can physically test it)

// Telemetry intervals
unsigned long lastTelemetryMillis = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 2500; // Print status every 2.5 s

// ---------------------------------------------------------------------------
// --- FAULT HANDLING ---
// ---------------------------------------------------------------------------
// Pin to read nFAULT from MCT8316Z
const int MCT8316_nFAULT_PIN = 10;  // <--- Adjust this to our actual GPIO pin we use

// Track if a major fault has latched
bool motorFaultActive = false;

// This is optional: allow negative speeds => reverse direction
bool allowReverse = true;

// ---------------------------------------------------------------------------
// 3. Forward Declarations
// ---------------------------------------------------------------------------

/* Core System Functions */
void initSystemClock();          // Configure MCU clock (if needed)
void initPins();                 // Initialize GPIO pin directions
void initMotorDriver();          // SPI setup and MCT8316 configuration
void initBoostConverter();       // Power system initialization (TBD)

/* Control Functions */  
void handleSerialCommands();         // Process incoming UART commands
void updatePID();                    // Run PID control algorithm
void updateMotorDriver(float cmd);   // Update PWM output to motor driver

/* Utility Functions */
float clampValue(float val, float minVal, float maxVal); // Constrain values

// SPI helper for MCT8316 config
void writeRegister16(uint8_t addr6, uint8_t data8, bool readNotWrite);
uint16_t readRegister16(uint8_t addr6);

// Hall sensor ISR
void hallISR();

// Function to check nFAULT and latch motorFaultActive
void checkMCT8316Fault();

// Function to clear latched fault by writing CLR_FLT
void clearFault();

// ---------------------------------------------------------------------------
// 4. setup()
// ---------------------------------------------------------------------------
void setup()
{
  initSystemClock();
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { ; }  // Wait for USB serial

  initPins();
  initMotorDriver();
  initBoostConverter();

  delay(500);
  Serial.println("=== Reaction Wheel Control Board (Primary) ===");
  Serial.println("SAMD21 Init Complete. Waiting for commands...");
}

// ---------------------------------------------------------------------------
// 5. loop()
// ---------------------------------------------------------------------------

void loop()
{
  // 5.1 Check for commands over serial
  handleSerialCommands();

  // Check if driver has signaled a fault
  checkMCT8316Fault();
  // If a fault is active, we can zero the output:
  if (motorFaultActive)
  {
    // Keep PWM at 0 to avoid further damage
    analogWrite(MCT8316_PWM_PIN, 0);
  }
  else
  {
    // 5.2 PID update
    updatePID();

    // 5.3 Update motor driver output
    updateMotorDriver(controlOutput);
  }

  // 5.4 Print telemetry
  if (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMillis = millis();
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

  // 5.5 Wait ~10ms (100 Hz)
  delay(10);
}

// ---------------------------------------------------------------------------
// 6. Implementation
// ---------------------------------------------------------------------------

/** 6.1 System Clock Init */
void initSystemClock()
{
  // I am pretty sure the SAMD MCU already sets this up, I'll double check this later
}

/** 6.2 Pin Setup */
void initPins()
{
  pinMode(MCT8316_ENABLE_PIN, OUTPUT);
  pinMode(MCT8316_DIRECTION_PIN, OUTPUT);
  pinMode(MCT8316_PWM_PIN, OUTPUT);

  pinMode(MCT8316_CS_PIN, OUTPUT);
  digitalWrite(MCT8316_CS_PIN, HIGH);

  // Hall sensors as inputs w/ pullups
  pinMode(HALL_A_PIN, INPUT_PULLUP);
  pinMode(HALL_B_PIN, INPUT_PULLUP);
  pinMode(HALL_C_PIN, INPUT_PULLUP);

  // Default states
  digitalWrite(MCT8316_ENABLE_PIN, LOW);   // disable driver initially
  digitalWrite(MCT8316_DIRECTION_PIN, LOW);
  analogWrite(MCT8316_PWM_PIN, 0);

  // Attach interrupts for Hall sensors
  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_C_PIN), hallISR, CHANGE);

  // Make nFAULT pin an input (possibly with pull-up? Fahd - double check if a pull up is needed).
  pinMode(MCT8316_nFAULT_PIN, INPUT_PULLUP);
}

/** 6.3 Motor Driver Init */
void initMotorDriver()
{
  SPI.begin();
  // SPI with up to 5MHz, Mode0
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  // Enable driver
  digitalWrite(MCT8316_ENABLE_PIN, HIGH);
  // write => W=0, address=8, parity=??, data=0x02
  writeRegister16(0x08, 0x02, false);
  writeRegister16(0x06, 0x10, false);

  SPI.endTransaction();

  Serial.println("MCT8316 driver configured via SPI.");
}

/** 6.5 Hall Sensor ISR
    Measures time between transitions to compute RPM.
    6 Hall transitions per mechanical revolution for a 3-phase BLDC.
**/
void hallISR()
{
  uint32_t now = micros();
  uint8_t newState = (digitalRead(HALL_A_PIN) << 2)
                   | (digitalRead(HALL_B_PIN) << 1)
                   | (digitalRead(HALL_C_PIN));

  if (newState != hallState)
  {
    float timeDiff = (now - lastHallTime) * 1e-6; // seconds
    if (timeDiff > 0.000001f)  // prevent div-by-zero
    {
      // 6 transitions = 1 rev
      float revPerSec = 1.0f / (6.0f * timeDiff);
      currentSpeedRPM = revPerSec * 60.0f;
    }
    lastHallTime = now;
    hallState = newState;
  }
}

/** 6.6 Handle UART Commands */
void handleSerialCommands()
{
  if (Serial.available() > 0)
  {
    String cmdString = Serial.readStringUntil('\n');
    cmdString.trim();
    if (cmdString.length() < 1) return;

    // ADDED: If user enters "CLR", we call clearFault()
    if (cmdString.equalsIgnoreCase("CLR"))
    {
      clearFault();
      Serial.println("[CMD] Attempted CLEAR of latched faults.");
      return;
    }

    if (cmdString.length() < 2) return; // we need at least 2 chars for "S 3000"

    // Format: "S ±3000" => speed mode, target ±3000 RPM
    //         "T ±100"  => torque mode, target ±100
    char modeChar = cmdString.charAt(0);
    float value   = cmdString.substring(2).toFloat();

    if (modeChar == 'S' || modeChar == 's')
    {
      currentMode   = SPEED_MODE;

      // --- BIDIRECTIONAL CONTROL ---
      if (value < 0 && allowReverse)
      {
        digitalWrite(MCT8316_DIRECTION_PIN, HIGH);
        value = -value;
      }
      else
      {
        digitalWrite(MCT8316_DIRECTION_PIN, LOW);
      }

      targetCommand = constrain(value, 0, (float)MAX_SPEED_CMD);
      Serial.print("[CMD] Speed Mode => target RPM = ");
      Serial.println(targetCommand);
    }
    else if (modeChar == 'T' || modeChar == 't')
    {
      currentMode   = TORQUE_MODE;

      if (value < 0 && allowReverse)
      {
        digitalWrite(MCT8316_DIRECTION_PIN, HIGH);
        value = -value;
      }
      else
      {
        digitalWrite(MCT8316_DIRECTION_PIN, LOW);
      }

      targetCommand = constrain(value, 0, (float)MAX_TORQUE_CMD);
      Serial.print("[CMD] Torque Mode => target Torque = ");
      Serial.println(targetCommand);
    }
    else
    {
      Serial.println("[CMD] Unrecognized command!");
    }
  }
}

/** 6.7 PID Controller Update (with anti-windup + derivative filter) */
void updatePID()
{
  // If Speed Mode => do PID on (target RPM - currentSpeedRPM).
  // Else => direct map for torque.

  if (currentMode == SPEED_MODE)
  {
    float error = targetCommand - currentSpeedRPM;

    // Integrator
    errorIntegral += error * dt;

    // Derivative
    float errorDerivative = (error - lastError) / dt;
    lastError = error;

    // Low-pass filter for derivative
    float alpha = 0.2f;
    filteredDerivative = alpha * errorDerivative + (1.0f - alpha) * filteredDerivative;

    // Raw PID
    float rawOutput = (Kp * error) + (Ki * errorIntegral) + (Kd * filteredDerivative);

    // Anti-windup: If saturating, push back on integral
    if ((rawOutput > MAX_PWM) || (rawOutput < 0))
    {
      // If at the limit, subtract the newly added integral
      errorIntegral -= error * dt;
    }

    // Clamp final output to 0–255
    controlOutput = clampValue(rawOutput, 0.0f, (float)MAX_PWM);
  }
  else
  {
    // Torque mode => interpret targetCommand as an arbitrary torque request
    // simplest approach: map to 0–255
    controlOutput = map((int)targetCommand, 0, MAX_TORQUE_CMD, 0, MAX_PWM);
  }
}

/** 6.8 Update the Motor Driver (PWM + DIR) */
void updateMotorDriver(float cmd)
{
  analogWrite(MCT8316_PWM_PIN, (int)cmd);
}

/** 6.9 clampValue utility */
float clampValue(float val, float minVal, float maxVal)
{
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

// ---------------------------------------------------------------------------
// 16-bit writes & reads, including parity bit
// ---------------------------------------------------------------------------

// SPI Helper: writeRegister() for MCT8316
void writeRegister16(uint8_t addr6, uint8_t data8, bool readNotWrite)
{
  // This is all according to the datasheet btw 

  // Build 16-bit frame as [ R/W(1b) | address(6b) | parity(1b) | data(8b) ]
  // R/W = 1 => read, R/W=0 => write
  uint16_t frame = 0;
  // bit15 => R/W
  frame |= ((readNotWrite ? 1 : 0) << 15);
  // bits14..9 => address
  frame |= ((addr6 & 0x3F) << 9);

  // temporarily combine to calculate parity
  uint16_t tempNoParity = frame | (data8 & 0xFF);

  // count bits in tempNoParity to determine parity
  uint8_t bitCount = 0;
  for (int i=0; i<16; i++)
  {
    if (tempNoParity & (1 << i)) bitCount++;
  }
  // even parity => if bitCount is odd, set parity bit
  uint8_t parityBit = (bitCount % 2 == 0) ? 0 : 1;

  // set bit8 => parity
  frame |= (parityBit << 8);

  // bits7..0 => data
  frame |= (data8 & 0x00FF);

  digitalWrite(MCT8316_CS_PIN, LOW);
  SPI.transfer16(frame);
  digitalWrite(MCT8316_CS_PIN, HIGH);
}

uint16_t readRegister16(uint8_t addr6)
{
  // For read => R/W=1
  // Build [1|address(6)|parity(1)|dummy(8)]
  uint16_t frame = 0;
  frame |= (1 << 15); // set R/W=1
  frame |= ((addr6 & 0x3F) << 9);

  // compute parity
  uint16_t tempNoParity = frame;
  uint8_t bitCount = 0;
  for (int i=0; i<16; i++)
  {
    if (tempNoParity & (1 << i)) bitCount++;
  }
  uint8_t parityBit = (bitCount % 2 == 0) ? 0 : 1;
  frame |= (parityBit << 8);

  digitalWrite(MCT8316_CS_PIN, LOW);
  uint16_t resp = SPI.transfer16(frame);
  digitalWrite(MCT8316_CS_PIN, HIGH);

  // The returned 16 bits from MCT8316Z: [status(8) | regData(8)]
  return resp;
}

// ---------------------------------------------------------------------------
//  Fault handling
// ---------------------------------------------------------------------------
void checkMCT8316Fault()
{
  if (digitalRead(MCT8316_nFAULT_PIN) == LOW)
  {
    Serial.println("[FAULT] MCT8316 nFAULT is LOW => driver fault active.");
    motorFaultActive = true;
  }
}

// Function to attempt clearing any slatched faults
void clearFault()
{
  // According to the datasheet for MCT8316, to clear latched bits, set CLR_FLT=1 in Control_Register_2A (offset=4h).
  // data8 => 0x01 sets bit0 => CLR_FLT
  writeRegister16(0x04, 0x01, false);

  // re-enable driver command
  digitalWrite(MCT8316_ENABLE_PIN, HIGH);

  // Software flag reset
  motorFaultActive = false;
}
