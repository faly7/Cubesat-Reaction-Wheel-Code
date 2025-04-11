// MotorDriver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "Biquad.h"
#include <SPI.h>

// Register addresses
#define IC_STATUS_REG     0x00  // IC Status Register
#define STATUS_REG_1      0x01  // Status Register 1
#define STATUS_REG_2      0x02  // Status Register 2
#define CONTROL_REG_1     0x03  // Control Register 1 (Register Unlock/Lock)
#define CONTROL_REG_2A    0x04  // Control Register 2A (Clear Faults, PWM Mode, Slew Rate)
#define CONTROL_REG_3     0x05  // Control Register 3 (OVP settings, PWM duty)
#define CONTROL_REG_4     0x06  // Control Register 4 (OCP settings, DRV_OFF)
#define CONTROL_REG_5     0x07  // Control Register 5 (CSA gain, Active Rectification)
#define CONTROL_REG_6     0x08  // Control Register 6 (Buck settings)
#define CONTROL_REG_7     0x09  // Control Register 7 (Direction, Brake, Coast modes)
#define CONTROL_REG_8     0x0A  // Control Register 8 (Motor Lock, FGOUT)
#define CONTROL_REG_9     0x0B  // Control Register 9 (Phase Advance)
#define CONTROL_REG_10    0x0C  // Control Register 10 (Driver Delay Compensation)


#define MOVING_AVG_SIZE 12  // Size of moving average window

// Structure to hold SPI response
struct SpiResponse {
  uint8_t status;  // First byte (status byte)
  uint8_t data;    // Second byte (data)
};


class MotorDriver {
public:
  // Constructor & initialization
  MotorDriver(uint8_t nSleepPin, uint8_t drvoffPin, uint8_t pwmPin, uint8_t fgoutPin, 
              uint8_t nFaultPin, uint8_t ilimPin, uint8_t csPin, uint8_t brakePin);
  
  // Initialization methods
  void begin();
  void setupPWM(uint32_t frequency = 20000);
  void setupFilters(float notchQ = 1.2, float lowPassAlpha = 0.05, 
                   float freqMultiplier = 1.0, float secondaryFreqMultiplier = 2.0);
  
  // Core motor control methods
  void start();
  void stop();
  void setSpeed(int speed);
  void adjustSpeed(int increment);
  void setDirection(bool directionCCW);
  void toggleDirection();
  void brake();
  void releaseBrake();
  
  // Status methods
  bool isRunning() const;
  float getCurrentRPM() const;
  int getCurrentSpeed() const;
  bool getDirection() const;
  bool checkFaults();
  void clearFaults();
  float getCurrentLimit();
  void printAllRegisters();
  float getRawRPM();
  float getFilteredAcceleration();
  
  // State observer methods
  float getEstimatedRPM();
  float getEstimatedAcceleration();
  float getEstimatedTorque();
  void setObserverGains(float rpmGain, float accGain);
  
  // PID control methods
  void setPIDParameters(float kp, float ki, float kd);
  float setTargetRPM(float targetRPM);
  float setTargetTorque(float targetTorque);
  void enablePID(bool enable);
  void updatePID();
  
  // Configuration methods
  void setSlowSlew(bool slowSlew);
  void setPhaseAdvance(bool enableAdvance);
  void setDigitalHallMode(bool digitalHall);
  void setCurrentLimit(uint8_t limit);
  bool unlockRegisters();
  bool initializeDriver();
  void setupBrakePWM();

  // Torque control methods
  bool isTorqueMode() const;
  void setTorqueMode(bool enable);
  float getTargetTorque() const;
  float getTargetRPM() const;
  float getCurrentTorque() const;
  void setMotorInertia(float inertia);
  float getMotorInertia() const;
  
  
  // Advanced methods
  void attachFGCallback(void (*callback)(void));
  void attachFaultCallback(void (*callback)(void));
  void processFGOUTpulse();  // Made public so it can be called from ISR
  
private:
  // Pin assignments
  uint8_t _nSleepPin, _drvoffPin, _pwmPin, _fgoutPin;
  uint8_t _nFaultPin, _ilimPin, _csPin, _brakePin;

  // SPI Speed
  uint32_t _spiSpeed;
  
  // Motor state
  bool _motorRunning;
  int _currentSpeed;
  bool _currentDirection;
  int _maxSpeed;
  int _minSpeed;
  int _rawRPM;
  
  // Savitzky-Golay filter variables
  static const int SG_WINDOW_SIZE = 7;  // Must be odd number
  float _sgBuffer[SG_WINDOW_SIZE];
  int _sgBufferIndex = 0;
  bool _sgBufferFilled = false;
  
  // Savitzky-Golay filter methods
  float savitzkyGolayFilter(float newValue);
  float savitzkyGolayDerivative();

  // Speed measurement variables
  volatile unsigned long _lastFgPulseTime;
  volatile unsigned long _fgPulsePeriod;
  volatile uint16_t _pulseCount;
  volatile float _currentRPM;
  volatile float _currentAcc;
  volatile float _lastRPM;
  unsigned long _lastValidPulsePeriod;
  unsigned long _pulseTimeout;
  float _maxPulseDeviation;
  int _pulsesPerRevolution;
  int _motorPolePairs;
  unsigned long _lastDirChange;
  bool _lastDir;
  // Filter variables and objects
  Biquad *_primaryNotchFilter;
  Biquad *_secondaryNotchFilter;
  Biquad *_lowPassFilter;
  float _notchQ;
  float _lowPassQ;
  float _lowPassCutoff;
  float _lowPassAlpha;
  float _freqMultiplier;
  float _secondaryFreqMultiplier;
  float _rpmHistory[MOVING_AVG_SIZE];  // History buffer for moving average
  int _historyIndex;
  float _lastFilteredValues[5];
  bool _filtersInitialized;
  float _lastFilterUpdateRPM;
  
  // PID control variables
  float _kp, _ki, _kd;
  float _targetRPM;
  float _targetTorque;
  bool _torqueMode;
  float _integral;
  float _lastError;
  bool _pidEnabled;
  unsigned long _lastPIDUpdate;

  // Filter params
  float _lambda_e;
  float _ae;
  float _be;
  float _ce;
  float _de;
  
  // State observer variables
  float _motorInertia;       // J = 5.1 gcm² = 5.1E-7 kg·m²
  float _motorDamping;       // B estimated from speed/torque gradient
  float _motorTorqueConstant; // Kt = 11.8 mNm/A for 12V model
  float _motorResistance;    // Terminal resistance from datasheet
  
  float _estimatedRPM;
  float _estimatedAcceleration;
  float _estimatedTorque;
  float _observerGainRPM;     // Observer gain for RPM correction
  float _observerGainAcc;     // Observer gain for acceleration correction
  float _lastEstimatedRPM;
  
  // Advanced acceleration filtering
  static const int ACC_WINDOW_SIZE = 15;
  float _accelHistory[ACC_WINDOW_SIZE];
  int _accelHistoryIndex;
  Biquad* _accelLowPass1;     // First stage LP filter
  Biquad* _accelLowPass2;     // Second stage LP filter
  
  // Observer and filtering methods
  void initializeStateObserver();
  void updateStateObserver(float measuredRPM, float deltaT);
  float getTorqueFromPWM(int pwmValue);
  float getFilteredAcceleration(float rawAcceleration);
  
  // Private methods
  void resetDriver();
  void checkPulseTimeout();
  void updateNotchFilters(float rpm);
  float movingAverage(float newValue);
  float medianFilter(float newValue);
  float applyFilters(float rawRPM);
  bool isValidPulsePeriod(unsigned long period);
  bool writeRegister(uint8_t reg, uint8_t value);
  SpiResponse readRegisterFull(uint8_t reg);
  uint8_t calculateParity(uint16_t command);
};

#endif // MOTOR_DRIVER_H