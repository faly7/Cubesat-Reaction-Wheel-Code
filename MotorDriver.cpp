// MotorDriver.cpp
#include "MotorDriver.h"

//==============================================================================
// Constructor and Initialization
//==============================================================================

MotorDriver::MotorDriver(uint8_t nSleepPin, uint8_t drvoffPin, uint8_t pwmPin, uint8_t fgoutPin,
                         uint8_t nFaultPin, uint8_t ilimPin, uint8_t csPin, uint8_t brakePin) {
  // Store pin assignments
  _nSleepPin = nSleepPin;
  _drvoffPin = drvoffPin;
  _pwmPin = pwmPin;
  _fgoutPin = fgoutPin;
  _nFaultPin = nFaultPin;
  _ilimPin = ilimPin;
  _csPin = csPin;
  _brakePin = brakePin;

  // SPI Setup
  _spiSpeed = 1000000;

  // Initialize state variables
  _motorRunning = false;
  _currentSpeed = 0;
  _currentAcc = 0;
  _currentDirection = false;  // CW
  _maxSpeed = 511;
  _minSpeed = -511;

  // Initialize speed measurement variables
  _lastFgPulseTime = 0;
  _fgPulsePeriod = 0;
  _pulseCount = 0;
  _currentRPM = 0;
  _rawRPM = 0;
  _lastValidPulsePeriod = 0;
  _pulseTimeout = 300000;  // 300ms
  _maxPulseDeviation = 0.1;
  _pulsesPerRevolution = 4;
  _motorPolePairs = 4;
  _lastDirChange = 0;
  _lastDir = false;

  // Initialize PID variables
  _kp = 1;
  _ki = 1;
  _kd = 0.01;
  _targetRPM = 0;
  _targetTorque = 0;
  _integral = 0;
  _lastError = 0;
  _pidEnabled = false;
  _lastPIDUpdate = 0;
  _torqueMode = true;

  // Filter variables initialization
  _filtersInitialized = false;
  _historyIndex = 0;
  _notchQ = 10;
  _lowPassQ = 0.75;
  _lowPassCutoff = .05;
  _lowPassAlpha = 0.05;
  _freqMultiplier = 1.0;
  _secondaryFreqMultiplier = 2.0;


  for (int i = 0; i < 5; i++) {
    _lastFilteredValues[i] = 0;
  }
}

void MotorDriver::begin() {
  // Initialize pins
  pinMode(_nSleepPin, OUTPUT);
  pinMode(_drvoffPin, OUTPUT);
  pinMode(_nFaultPin, INPUT_PULLUP);
  pinMode(_fgoutPin, INPUT_PULLUP);
  pinMode(_ilimPin, OUTPUT);
  pinMode(_csPin, OUTPUT);
  pinMode(_brakePin, OUTPUT);


  // Set initial pin states
  digitalWrite(_nSleepPin, HIGH);  // Enable the driver
  digitalWrite(_drvoffPin, LOW);   // Enable the driver output
  digitalWrite(_csPin, HIGH);      // Set CW direction
  digitalWrite(_brakePin, LOW);    // Disable Brake

  // Set current limit to mid-range
  analogWrite(_ilimPin, 127);

  // Initialize SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);  // Using SPI Mode 1 based on diagnostics
  SPI.setBitOrder(MSBFIRST);

  // Reset the driver
  resetDriver();
  initializeDriver();
  setupFilters();
  setupPWM();
  setupBrakePWM();
}

void MotorDriver::printAllRegisters() {
  Serial.println("\n----- All Register Values -----");

  for (int reg = 0; reg <= 0x0C; reg++) {
    SpiResponse resp = readRegisterFull(reg);

    Serial.print("REG 0x");
    if (reg < 16) Serial.print("0");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    if (resp.data < 16) Serial.print("0");
    Serial.print(resp.data, HEX);
    Serial.print(" (Status: 0x");
    if (resp.status < 16) Serial.print("0");
    Serial.print(resp.status, HEX);
    Serial.println(")");
  }

  Serial.println("------------------------------");
}


void MotorDriver::setupPWM(uint32_t frequency) {
  // This method should implement the detailed PWM setup from your original code
  // for the Arduino MKR Zero / SAMD21

  // Step 1: Configure GCLK4 for 48MHz
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Step 2: Connect GCLK4 to TCC0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Step 3: Enable pin multiplexing for the PWM pin
  PORT->Group[g_APinDescription[_pwmPin].ulPort].PINCFG[g_APinDescription[_pwmPin].ulPin].bit.PMUXEN = 1;

  // Determine if pin is odd or even
  uint8_t pin_num = g_APinDescription[_pwmPin].ulPin;
  if (pin_num % 2 == 0) {
    // Even pin - PMUXE
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXE_Msk;
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXE_F;
  } else {
    // Odd pin - PMUXO
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXO_Msk;
    PORT->Group[g_APinDescription[_pwmPin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXO_F;
  }

  // Step 4: Disable TCC0 before configuration
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;

  // Step 5: Configure waveform generation
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;  // Normal PWM mode
  while (TCC0->SYNCBUSY.bit.WAVE)
    ;

  // Step 6: Configure output matrix to connect CC0 to WO4
  TCC0->WEXCTRL.reg = TCC_WEXCTRL_OTMX(2);  // Output matrix setting

  // Step 7: Set the period value (frequency)
  // Calculate PER value based on frequency
  uint32_t per_value = 512;  // Default value


  TCC0->PER.reg = per_value;
  while (TCC0->SYNCBUSY.bit.PER)
    ;

  // Step 8: Set initial duty cycle
  TCC0->CC[0].reg = 0;  // Start with 0% duty cycle
  while (TCC0->SYNCBUSY.bit.CC0)
    ;

  // Step 9: Enable TCC0 with prescaler
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;
}

void MotorDriver::setupFilters(float notchQ, float lowPassAlpha,
                               float freqMultiplier, float secondaryFreqMultiplier) {
  // Store filter parameters
  _notchQ = notchQ;
  _lowPassAlpha = lowPassAlpha;
  _freqMultiplier = freqMultiplier;
  _secondaryFreqMultiplier = secondaryFreqMultiplier;

  // Create the filters
  _primaryNotchFilter = new Biquad(bq_type_notch, 0.1, _notchQ, 0);
  _secondaryNotchFilter = new Biquad(bq_type_notch, 0.2, _notchQ, 0);
  _lowPassFilter = new Biquad(bq_type_lowpass, _lowPassCutoff, _lowPassQ, 0);

  // Initialize history buffers
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    _rpmHistory[i] = 0;
  }

  for (int i = 0; i < 5; i++) {
    _lastFilteredValues[i] = 0;
  }

  _filtersInitialized = true;
}

//==============================================================================
// Core Motor Control Methods
//==============================================================================

void MotorDriver::start() {
  // Check for faults before starting
  if (!digitalRead(_nFaultPin)) {
    return;  // Can't start if there's a fault
  }

  // If currently at zero speed, set to a minimum starting speed
  if (_currentSpeed == 0) {
    _currentSpeed = 50;  // Minimum starting speed
  }

  // Reset speed measurement
  _lastFgPulseTime = 0;
  if(_torqueMode){_targetRPM = _currentRPM;}
  
  // Apply PWM signal
  TCC0->CCB[0].reg = _currentSpeed;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;

  // Make sure DRVOFF is LOW (motor enabled)
  digitalWrite(_drvoffPin, LOW);
  

  _motorRunning = true;
}

void MotorDriver::stop() {
  // Set PWM to zero
  TCC0->CCB[0].reg = 0;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;
  _motorRunning = false;
}

float MotorDriver::getRawRPM(){
    return _rawRPM;
}

// Add these method implementations to MotorDriver.cpp

bool MotorDriver::isTorqueMode() const {
  return _torqueMode;
}

void MotorDriver::setTorqueMode(bool enable) {
  _torqueMode = enable;
}

float MotorDriver::getTargetTorque() const {
  return _targetTorque;
}

float MotorDriver::getTargetRPM() const {
  return _targetRPM;
}

float MotorDriver::getCurrentTorque() const {
  // Calculate torque from acceleration using T = I * α
  // Convert RPM/s to rad/s² (RPM/s * 2π/60)
  float accelerationRadPerSec2 = _currentAcc * (2.0 * PI / 60.0);
  return _motorInertia * accelerationRadPerSec2;
}

void MotorDriver::setMotorInertia(float inertia) {
  _motorInertia = inertia;
}

float MotorDriver::getMotorInertia() const {
  return _motorInertia;
}

void MotorDriver::setSpeed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, _minSpeed, _maxSpeed);
  _currentSpeed = speed;

  if (_motorRunning) {
    // Apply the new speed via PWM
    if(speed > 0 and _currentRPM > -200){
      if(_currentDirection){
        setDirection(false);
      }
      //Serial.println("1");
      TCC1->CCB[0].reg = 0;
      while (TCC1->SYNCBUSY.bit.CC0);
      TCC0->CCB[0].reg = _currentSpeed;
      while (TCC0->SYNCBUSY.bit.CC0)
      ;

    }
    else if(speed < 0 and _currentRPM < 200){
      //Serial.println("2");
      if(!_currentDirection){
        setDirection(true);
      }
      TCC1->CCB[0].reg = 0;
      while (TCC1->SYNCBUSY.bit.CC0);
      TCC0->CCB[0].reg = -_currentSpeed;
      while (TCC0->SYNCBUSY.bit.CC0)
      ;
    }
      else{
        //Serial.println("3");
        TCC1->CCB[0].reg = abs(_currentSpeed);
        while (TCC1->SYNCBUSY.bit.CC0);

        TCC0->CCB[0].reg = 0;
        while (TCC0->SYNCBUSY.bit.CC0)
        ;
      }
    
  }
}
// void MotorDriver::setSpeed(int speed) {
//   // Constrain speed to valid range
//   speed = constrain(speed, _minSpeed, _maxSpeed);
//   _currentSpeed = speed;

//   if (_motorRunning) {
//     // Apply the new speed via PWM
//     if(speed > 0){
//       TCC1->CCB[0].reg = 0;
//       while (TCC1->SYNCBUSY.bit.CC0);
//       TCC0->CCB[0].reg = _currentSpeed;
//       while (TCC0->SYNCBUSY.bit.CC0)
//       ;
//     }
//     else{
//       TCC1->CCB[0].reg = -speed;
//       while (TCC1->SYNCBUSY.bit.CC0);

//       TCC0->CCB[0].reg = 0;
//       while (TCC0->SYNCBUSY.bit.CC0)
//       ;
//       //Serial.println(-speed/2);
//     }
    
//   }
// }

void MotorDriver::setupBrakePWM() {
  // Step 1: Enable pin multiplexing for the brake pin
  PORT->Group[g_APinDescription[_brakePin].ulPort].PINCFG[g_APinDescription[_brakePin].ulPin].bit.PMUXEN = 1;
  
  // Determine if pin is odd or even
  uint8_t pin_num = g_APinDescription[_brakePin].ulPin;
  if (pin_num % 2 == 0) {
    // Even pin - PMUXE
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXE_Msk;
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXE_E;
  } else {
    // Odd pin - PMUXO
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg &= ~PORT_PMUX_PMUXO_Msk;
    PORT->Group[g_APinDescription[_brakePin].ulPort].PMUX[pin_num >> 1].reg |= PORT_PMUX_PMUXO_E;
  }

  // Connect GCLK4 to TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  // Disable TCC1 before configuration
  TCC1->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE);
  
  // Configure waveform generation
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;  // Normal PWM mode
  while (TCC1->SYNCBUSY.bit.WAVE);
  
  // Use fixed period value of 512
  TCC1->PER.reg = 512;
  while (TCC1->SYNCBUSY.bit.PER);
  
  // Set initial duty cycle to 0
  TCC1->CC[0].reg = 0;
  while (TCC1->SYNCBUSY.bit.CC0);
  
  // Enable TCC1 with no prescaler for highest frequency
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE);
}
void MotorDriver::adjustSpeed(int increment) {
  int newSpeed = _currentSpeed + increment;
  setSpeed(newSpeed);
}

void MotorDriver::setDirection(bool directionCCW) {
  _currentDirection = directionCCW;
  writeRegister(CONTROL_REG_7, _currentDirection ? 0x01 : 0x00);
  //Serial.println("dirswap");
}

void MotorDriver::toggleDirection() {
  if (!_motorRunning) {
    _currentDirection = !_currentDirection;
    writeRegister(CONTROL_REG_7, _currentDirection ? 0x01 : 0x00);
  }
}

void MotorDriver::brake() {
  // First stop the PWM
  TCC0->CCB[0].reg = 0;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;
  _motorRunning = false;

  TCC1->CCB[0].reg = 511;
  while (TCC1->SYNCBUSY.bit.CC0);

}

void MotorDriver::releaseBrake() {
  TCC1->CCB[0].reg = 0;
  while (TCC1->SYNCBUSY.bit.CC0);
}

//==============================================================================
// Status Methods
//==============================================================================

bool MotorDriver::isRunning() const {
  return _motorRunning;
}

float MotorDriver::getCurrentRPM() const {
  return _currentRPM;
}

int MotorDriver::getCurrentSpeed() const {
  return _currentSpeed;
}

bool MotorDriver::getDirection() const {
  return _currentDirection;
}

bool MotorDriver::checkFaults() {
  // Check fault pin
  bool faultPinState = digitalRead(_nFaultPin);

  // Return true if fault detected
  return !faultPinState;
}

// Clear faults
void MotorDriver::clearFaults() {
  Serial.println("Clearing faults...");

  // Set the CLR_FLT bit in CONTROL_REG_2A
  // Preserve PWM_MODE bits and other settings
  uint8_t currentValue = readRegisterFull(CONTROL_REG_2A).data;
  writeRegister(CONTROL_REG_2A, currentValue | 0x01);

  // Wait for fault clear to process
  delay(5);

  // Check if faults were cleared
  SpiResponse icStatus = readRegisterFull(IC_STATUS_REG);
  if (!(icStatus.data & 0x01) && digitalRead(_nFaultPin)) {
    Serial.println("All faults cleared");
  } else {
    Serial.println("Some faults still present - check status");
  }
}


//==============================================================================
// Configuration Methods
//==============================================================================



void MotorDriver::setCurrentLimit(uint8_t limit) {
  analogWrite(_ilimPin, limit);
}

float MotorDriver::getCurrentLimit() {
  return analogRead(_ilimPin);
}

//==============================================================================
// PID Control Methods
//==============================================================================

void MotorDriver::setPIDParameters(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

float MotorDriver::setTargetRPM(float targetRPM) {
  _targetRPM = targetRPM;
  //return _currentAcc;
  return _currentSpeed;
}

float MotorDriver::setTargetTorque(float targetTorque) {
  _targetTorque = targetTorque;
  //return _rawRPM;
  return _currentSpeed;
}

void MotorDriver::enablePID(bool enable) {
  if (enable && !_pidEnabled) {
    // Reset PID variables when enabling
    _integral = 0;
    _lastError = 0;
    _lastPIDUpdate = micros();
  }
  _pidEnabled = enable;
}
// In MotorDriver.cpp, add these implementations:

// Savitzky-Golay filter implementation
float MotorDriver::savitzkyGolayFilter(float newValue) {
  // Add new value to circular buffer
  _sgBuffer[_sgBufferIndex] = newValue;
  _sgBufferIndex = (_sgBufferIndex + 1) % SG_WINDOW_SIZE;
  
  // Check if buffer is filled
  if (_sgBufferIndex == 0) {
    _sgBufferFilled = true;
  }
  
  // If buffer isn't filled yet, return input value
  if (!_sgBufferFilled) {
    return newValue;
  }
  
  // Apply 7-point quadratic Savitzky-Golay filter
  // These coefficients are pre-calculated for a quadratic fit
  static const float coeffs[] = {-2, 3, 6, 7, 6, 3, -2};
  static const float norm = 21.0;  // Normalization factor
  
  float result = 0;
  for (int i = 0; i < SG_WINDOW_SIZE; i++) {
    int idx = (_sgBufferIndex + i) % SG_WINDOW_SIZE;
    result += coeffs[i] * _sgBuffer[idx];
  }
  
  return result / norm;
}

// Calculate derivative using Savitzky-Golay
float MotorDriver::savitzkyGolayDerivative() {
  // If buffer isn't filled yet, use simple difference
  if (!_sgBufferFilled) {
    return _currentAcc;
  }
  
  // Apply 7-point Savitzky-Golay first derivative coefficients
  // These coefficients are pre-calculated for first derivative
  static const float coeffs[] = {-3, -2, -1, 0, 1, 2, 3};
  static const float norm = 28.0;  // Normalization factor
  
  float derivative = 0;
  for (int i = 0; i < SG_WINDOW_SIZE; i++) {
    int idx = (_sgBufferIndex + i) % SG_WINDOW_SIZE;
    derivative += coeffs[i] * _sgBuffer[idx];
  }
  
  // Scale by effective sampling rate (samples per second)
  unsigned long currentTime = micros();
  static unsigned long lastTime = currentTime;
  float deltaT = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  
  // Assuming consistent sampling, convert to RPM/s
  float samplesPerSecond = 1.0 / deltaT;
  return (derivative / norm) * samplesPerSecond;
}

// Public method to get filtered acceleration
float MotorDriver::getFilteredAcceleration() {
  return savitzkyGolayDerivative();
}
void MotorDriver::updatePID() {
  unsigned long currentTime = micros();
  float deltaT = (currentTime - _lastPIDUpdate) / 1000000.0;  // Convert to seconds
  _lastRPM = _currentRPM;
  _currentRPM = applyFilters(_rawRPM);
  _currentAcc = (_currentRPM-_lastRPM) / deltaT;

  
  if (!_motorRunning) {
    return;
  }
  
  // Calculate time delta
  _lastPIDUpdate = currentTime;

  if(_torqueMode && deltaT < 0.01){
    if(abs(_targetRPM + _targetTorque*deltaT) < 15000){
       _targetRPM = _targetRPM + _targetTorque*deltaT;
    }
  }

  // Calculate error
  float error = _targetRPM - _currentRPM;

  // Calculate PID terms
  _integral += error * deltaT;

  // Prevent integral windup
  if (_ki * _integral > _maxSpeed) _integral = _maxSpeed;
  if (_ki * _integral < _minSpeed) _integral = _minSpeed;

  float derivative = deltaT > 0 ? (error - _lastError) / deltaT : 0;
  _lastError = error;

  // Calculate control output
  float output = _kp * error + _ki * _integral + _kd * derivative;

  // Convert output to PWM value and apply
  int pwmValue = constrain(output, _minSpeed, _maxSpeed);
  if(abs(_targetRPM) < 200 && abs(_currentRPM) < 200) pwmValue = 0;
  setSpeed(pwmValue);
  //setSpeed(200);

}

//==============================================================================
// Speed Measurement and Filtering
//==============================================================================

void MotorDriver::processFGOUTpulse() {
  unsigned long currentTime = micros();
  unsigned long currentPeriod = 0;

  if (_lastFgPulseTime > 0) {
    currentPeriod = currentTime - _lastFgPulseTime;
    if(isValidPulsePeriod(currentPeriod)){
      _rawRPM = (60.0 * 1000000.0) / (currentPeriod * _pulsesPerRevolution);
      if (_rawRPM > 30000){
        _rawRPM = 0;
      }
      if(_currentDirection != _lastDir && _lastDirChange - currentTime > 10000){
        _lastDir = _currentDirection;
        _lastDirChange = currentTime;
        //Serial.println("dirchange");
      }
      if(_lastDir & _currentRPM < 210){
        _rawRPM = -_rawRPM;
      }
    }

    
    // Apply the filtering pipeline
    //_currentRPM = applyFilters(rawRpm);

    // Remember this period for next comparison
    _lastValidPulsePeriod = currentPeriod;
  }

  // Always update the last pulse time for timeout detection
  _lastFgPulseTime = currentTime;
  _pulseCount++;
}

bool MotorDriver::isValidPulsePeriod(unsigned long period) {
  // Skip validation for the first pulse
  if (_lastValidPulsePeriod == 0) {
    return true;
  }
  if (abs(_currentRPM) < 500){
    return true;
  }

  // Check if pulse period is too short (possible double-count)
  if (period < _lastValidPulsePeriod * (1.0 - _maxPulseDeviation)) {
    return false;
  }

  // Check if pulse period is too long (possible missed pulse)
  if (period > _lastValidPulsePeriod * (1.0 + _maxPulseDeviation)) {
    // Special case: if it's almost exactly 2x, likely a missed pulse
    return false;
  }

  return true;
}

void MotorDriver::checkPulseTimeout() {
  // Only check when motor is supposed to be running
  if (_motorRunning) {
    unsigned long currentTime = micros();
    // If no pulse received for too long
    if ((currentTime - _lastFgPulseTime) > _pulseTimeout) {
      // Adjust timeout based on current speed (longer timeout at lower speeds)
      _pulseTimeout = max(100000UL, (unsigned long)(60000000.0 / (_currentRPM * _pulsesPerRevolution) * 3));

      // Optional: Handle the timeout condition (e.g., emergency stop, reset driver)
    }
  }
}

void MotorDriver::updateNotchFilters(float rpm) {
  // Only update if RPM has changed significantly
  if (abs(rpm - _lastFilterUpdateRPM) > 100) {
    // Calculate noise frequency based on RPM
    float rotationFrequency = rpm / 60.0;  // Revolutions per second

    // For this motor with 4 pole pairs and 3 hall sensors,
    // we expect noise at rotation frequency and electrical frequency
    float primaryNoiseFreq = rotationFrequency * _freqMultiplier;
    float secondaryNoiseFreq = rotationFrequency * _secondaryFreqMultiplier;
    float effectiveSampleRate = 1000 * 2;

    // Normalize frequencies (between 0-0.5)
    float primaryNormFreq = primaryNoiseFreq / (effectiveSampleRate / 2);
    float secondaryNormFreq = secondaryNoiseFreq / (effectiveSampleRate / 2);

    // Constrain to valid range
    primaryNormFreq = constrain(primaryNormFreq, 0.05, 0.45);
    secondaryNormFreq = constrain(secondaryNormFreq, 0.05, 0.45);
    // Serial.println("Normalized freqs:");
    // Serial.println(primaryNormFreq);
    // Serial.println(secondaryNormFreq);
    // Update filter parameters
    _primaryNotchFilter->setFc(primaryNormFreq);
    _secondaryNotchFilter->setFc(secondaryNormFreq);

    _lastFilterUpdateRPM = rpm;
  }
}
// void MotorDriver::updateNotchFilters(float rpm) {
//   // Only update if RPM has changed significantly
//   if (abs(rpm - _lastFilterUpdateRPM) > 100) {
//     // Calculate noise frequency based on RPM
//     float rotationFrequency = rpm / 60.0;  // Revolutions per second
    
//     // For this motor with 4 pole pairs and 3 hall sensors,
//     // we expect noise at rotation frequency and electrical frequency
//     float primaryNoiseFreq = rotationFrequency * _freqMultiplier;
//     float secondaryNoiseFreq = rotationFrequency * _secondaryFreqMultiplier;
    
//     // Calculate effective sample rate based on hall pulse timing
//     float hallPulseFreq = rotationFrequency * _pulsesPerRevolution;
//     float effectiveSampleRate = hallPulseFreq * 2;  // Conservative estimate
    
//     // Normalize frequencies (between 0-0.5)
//     float primaryNormFreq = primaryNoiseFreq / (effectiveSampleRate/2);
//     float secondaryNormFreq = secondaryNoiseFreq / (effectiveSampleRate/2);
    
//     // Constrain to valid range
//     primaryNormFreq = constrain(primaryNormFreq, 0.05, 0.45);
//     secondaryNormFreq = constrain(secondaryNormFreq, 0.05, 0.45);
    
//     // Update filter parameters
//     _primaryNotchFilter->setFc(primaryNormFreq);
//     _secondaryNotchFilter->setFc(secondaryNormFreq);
    
//     _lastFilterUpdateRPM = rpm;
//   }
// }


float MotorDriver::movingAverage(float newValue) {
  // Add new value to history
  _rpmHistory[_historyIndex] = newValue;
  _historyIndex = (_historyIndex + 1) % MOVING_AVG_SIZE;

  // Calculate average
  float sum = 0;
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    sum += _rpmHistory[i];
  }
  return sum / MOVING_AVG_SIZE;
}

float MotorDriver::medianFilter(float newValue) {
  // Shift values
  for (int i = 4; i > 0; i--) {
    _lastFilteredValues[i] = _lastFilteredValues[i - 1];
  }
  _lastFilteredValues[0] = newValue;

  // Copy to temporary array for sorting
  float temp[5];
  for (int i = 0; i < 5; i++) {
    temp[i] = _lastFilteredValues[i];
  }

  // Simple bubble sort
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (temp[j] > temp[j + 1]) {
        float tmp = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = tmp;
      }
    }
  }

  // Return middle value
  return temp[2];
}

float MotorDriver::applyFilters(float rawRPM) {
  // Update notch filter parameters if needed
  updateNotchFilters(_currentRPM > 0 ? _currentRPM : rawRPM);

  // Apply multi-stage filtering pipeline
  float stage1 = _primaryNotchFilter->process(rawRPM);
  float stage2 = _secondaryNotchFilter->process(stage1);
  float stage3 = movingAverage(stage2);
  float stage4 = _lowPassFilter->process(stage3);

  // Apply additional exponential smoothing
  static float lastSmoothed = 0;
  float stage5 = lastSmoothed * (1 - _lowPassAlpha) + stage4 * _lowPassAlpha;
  lastSmoothed = stage5;

  // Apply median filter
  //return medianFilter(stage5);
  return stage4;
}

void MotorDriver::resetDriver() {
  // Brief pulse on nSLEEP pin for reset
  digitalWrite(_nSleepPin, LOW);
  delayMicroseconds(10);  // 10μs reset pulse
  digitalWrite(_nSleepPin, HIGH);
  delay(10);  // Give time for reset to complete

  // Initialize motor state
  _motorRunning = false;
  _currentSpeed = 0;
  _currentDirection = false;  // CW

  // Reset speed measurement variables
  _pulseCount = 0;
  _lastFgPulseTime = 0;
  _currentRPM = 0;
  _lastValidPulsePeriod = 0;
}

// Advanced callback methods
void MotorDriver::attachFGCallback(void (*callback)(void)) {
  // This would attach a callback for FGOUT pulses if needed
  // Not implemented in this version
}

void MotorDriver::attachFaultCallback(void (*callback)(void)) {
  // This would attach a callback for fault detection if needed
  // Not implemented in this version
}

// Initialize motor driver with optimal settings
bool MotorDriver::initializeDriver() {
  Serial.println("Initializing MCT8316Z...");

  // Unlock registers
  if (!unlockRegisters()) {
    Serial.println("Failed to unlock registers!");
    return false;
  }

  // Clear any existing faults
  clearFaults();


  // Configure slew rate for clean switching, async PWM
  if (!writeRegister(CONTROL_REG_2A, 0x0A)) {
    Serial.println("Failed to set slew rate!");
    return false;
  }

  // Enable Active Synchronous Rectification (ASR)
  // ASR improves efficiency
  // if (!writeRegister(CONTROL_REG_5, 0x02)) {
  //   Serial.println("Failed to configure rectification mode!");
  //   return false;
  // }

  // Configure OCP (overcurrent protection) for auto-retry
  // OCP_MODE = 0x01 (auto-retry)
  // OCP_LVL = 0x00 (16A limit)
  // OCP_RETRY = 0x00 (5ms retry)
  // OCP_DEG = 0x01 (0.6μs deglitch)
  if (!writeRegister(CONTROL_REG_4, 0x13)) {
    Serial.println("Failed to configure overcurrent protection!");
    return false;
  }

  // Configure Motor Lock detection
  // MTR_LOCK_MODE = 0x01 (auto-retry)
  // MTR_LOCK_TDET = 0x01 (500ms detection time)
  if (!writeRegister(CONTROL_REG_8, 0x45)) {
    Serial.println("Failed to configure motor lock detection!");
    return false;
  }



  // Configure phase advance for better performance
  if (!writeRegister(CONTROL_REG_9, 0x04)) {
    Serial.println("Failed to set phase advance!");
    return false;
  }

  // Configure Buck regulator with 5V output
  writeRegister(CONTROL_REG_6, 0x02);

  return true;

  // setup other stuff
  //writeRegister(CONTROL_REG_3, 0x01); // Current amp gain
}

// Unlock registers
bool MotorDriver::unlockRegisters() {
  Serial.print("Unlocking registers... ");

  // Write the unlock code (0x03) to CONTROL_REG_1
  if (!writeRegister(CONTROL_REG_1, 0x03)) {
    Serial.println("Failed to write to CONTROL_REG_1!");
    return false;
  }

  // Verify unlock was successful
  SpiResponse resp = readRegisterFull(CONTROL_REG_1);
  if ((resp.data & 0x07) == 0x03) {
    Serial.println("Success!");
    return true;
  } else {
    Serial.print("Failed! Register value: 0x");
    Serial.println(resp.data, HEX);
    return false;
  }
}

// Write to a register via SPI
bool MotorDriver::writeRegister(uint8_t reg, uint8_t value) {
  // Format: 0 (write) + 6-bit address + parity + 8-bit data
  uint16_t command = ((reg & 0x3F) << 9) | (value & 0xFF);

  // Calculate and set parity bit
  uint8_t parity = calculateParity(command);
  command |= (parity << 8);

  uint8_t highByte = (command >> 8) & 0xFF;
  uint8_t lowByte = command & 0xFF;

  // SPI transaction
  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_csPin, LOW);
  delayMicroseconds(5);

  SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE1));
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);

  // Verify write (optional but recommended for reliability)
  SpiResponse resp = readRegisterFull(reg);
  return (resp.data == value);
}

// Read a register via SPI and return both status and data bytes
SpiResponse MotorDriver::readRegisterFull(uint8_t reg) {
  // Format: 1 (read) + 6-bit address + parity + 8 zeros
  uint16_t command = (1 << 15) | ((reg & 0x3F) << 9);

  // Calculate and set parity bit
  uint8_t parity = calculateParity(command);
  command |= (parity << 8);

  uint8_t highByte = (command >> 8) & 0xFF;
  uint8_t lowByte = command & 0xFF;

  SpiResponse resp;

  // SPI transaction
  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_csPin, LOW);
  delayMicroseconds(5);

  SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE1));
  resp.status = SPI.transfer(highByte);
  resp.data = SPI.transfer(lowByte);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
  delayMicroseconds(5);

  return resp;
}

// Calculate even parity for SPI command
uint8_t MotorDriver::calculateParity(uint16_t command) {
  uint8_t parity = 0;

  for (int i = 0; i < 16; i++) {
    if (i != 8) {  // Skip bit 8 (parity bit position)
      parity ^= (command >> i) & 0x01;
    }
  }

  return parity;
}
