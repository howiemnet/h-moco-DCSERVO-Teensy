// ------------------------------------------
//
//     ControlLoop.cpp
//
//    (C) 2015 h
//
//    Implements a PID control for use with
//    a DC servo. Originally based on the
//    Arduino library, re-written to add
//    feedforward support, and to use the
//    standard PID calculation (rather than
//    parallel): the proportional term acts
//    as a multiplier for the integral and
//    derivative terms as well as the error.
//
// ------------------------------------------

#include "ControlLoop.h"

ControlLoop controller;
extern void reportBoundsError();

void loopWrapper() {
  controller.theLoop();
}


// ------------------------------------------------------
//
//    INIT
//
// ------------------------------------------------------

ControlLoop::ControlLoop() {

}

void ControlLoop::init() {

  _encoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B);
  _motor = new Motor();
  pinMode(HOME_SENSOR_GROUND, OUTPUT);
  pinMode(HOME_SENSOR_SIGNAL, INPUT);
  pinMode(HOME_SENSOR_POWER, OUTPUT);
  digitalWrite( HOME_SENSOR_GROUND, LOW);
  digitalWrite( HOME_SENSOR_POWER, HIGH);

  // message types:
  setupData.parameters._header[1] = 2;
  liveData.liveValues._header[1] = 1;
  liveData.liveValues._header[0] = 1;

  setupData.parameters._PTerm = PRESET_P_TERM;
  setupData.parameters._ITerm = PRESET_I_TERM;
  setupData.parameters._DTerm = PRESET_D_TERM;
  setupData.parameters._ITermLimit = PRESET_I_TERM_LIMIT;
  setupData.parameters._velocityFeedForward = PRESET_VFF_TERM;
  setupData.parameters._accelerationFeedForward = PRESET_AFF_TERM;
  setupData.parameters._stiction = PRESET_STICTION;
  setupData.parameters._stictionVelocity = PRESET_STICTION_VELOCITY;
  setupData.parameters._backlash = PRESET_BACKLASH;
  setupData.parameters._backlashFactor = PRESET_BACKLASH_FACTOR;
  setupData.parameters._deadband = PRESET_DEADBAND;
  setupData.parameters._PWMLimit = PRESET_PWM_LIMIT;
  setupData.parameters._hardLimitMinimum = HARD_LIMIT_MINIMUM;
  setupData.parameters._hardLimitMaximum = HARD_LIMIT_MAXIMUM;
  setupData.parameters._header[2] = 0;


  _safetyLimitHit = false;

  _PIDReset();
  // _loopTimer.begin(loopWrapper, 1000000 / PID_LOOP_FREQUENCY);
  // _loopTimer.priority(128); // 0 is highest, most things run at 128
  _motor->enableMotor();
}



void ControlLoop::enablePIDControl() {
  _PIDReset();
  _motor->enableMotor();
}

void ControlLoop::disablePIDControl() {
  _motor->disableMotor();
}

// ------------------------------------------------------
//
//    Input filtering
//
//    Uses simple averaging over the last
//    INPUT_BUFFER_LENGTH samples (as defined
//    in Config.h)
//
// ------------------------------------------------------


float ControlLoop::_getFilteredCurrentPosition() {

  _unfilteredReading = (float) _encoder->read();

  /*if (DISABLE_MOTOR_ON_HARD_LIMIT) {
    if ((latestReading > setupData.parameters._hardLimitMaximum) || (latestReading < setupData.parameters._hardLimitMinimum)) {
      _motor->disableMotor();
      reportBoundsError();
    }
  }
  */


  _inputBufferPosition ++;
  if (_inputBufferPosition >= INPUT_BUFFER_LENGTH) {
    _inputBufferPosition = 0;
  }

  _inputBufferRunningTotal -= _inputBuffer[_inputBufferPosition];
  _inputBufferRunningTotal +=  _unfilteredReading;
  _inputBuffer[_inputBufferPosition] = _unfilteredReading;

  return _inputBufferRunningTotal / (float) (INPUT_BUFFER_LENGTH);

  //return (float) latestReading;
}





// ------------------------------------------------------
//
//    MAIN CALCULATION LOOP
//
// ------------------------------------------------------


void ControlLoop::theLoop(void) {

  digitalWrite(13, HIGH);

  // ------------------------------
  //
  //  get current position
  //
  // ------------------------------

  _lastUnfilteredReading = _unfilteredReading;
  _lastFilteredReading = _filteredReading;
  _filteredReading = _getFilteredCurrentPosition();

  _lastPosition = liveData.liveValues._currentPosition;
  liveData.liveValues._currentPosition = _filteredReading;


  // ------------------------------
  //
  //  calculate current velocity
  //  and acceleration
  //
  // ------------------------------


  float lastVelocity = liveData.liveValues._currentVelocity;
  float delta = _filteredReading - _lastFilteredReading;
  // Handle velocities below 4c/loop:
  
  liveData.liveValues._currentVelocity = delta; //* (float) PID_LOOP_FREQUENCY; // in units/sec
  
  //if (abs(liveData.liveValues._currentVelocity) < 2) {liveData.liveValues._currentVelocity = 0;}
  
  liveData.liveValues._currentAcceleration = liveData.liveValues._currentVelocity - lastVelocity;


  // ------------------------------
  //
  //  get next setpoint position
  //
  // ------------------------------


  _lastRequestedPosition = liveData.liveValues._requestedPosition;
  liveData.liveValues._requestedPosition = _nextRequestedPosition;


  // ------------------------------
  //
  //  calculate requested velocity
  //  and acceleration
  //
  // ------------------------------

  /*  THIS is now handled out in the 25fps getNextAnimatedSetpoint loop
    lastVelocity = _requestedVelocity;
    _requestedVelocity = (liveData.liveValues._requestedPosition - _lastRequestedPosition) * (float) PID_LOOP_FREQUENCY; // in units/sec
    _requestedAcceleration = _requestedVelocity - lastVelocity;
  */

  // ------------------------------
  //
  //  calculate current error
  //
  // ------------------------------


  liveData.liveValues._currentError = liveData.liveValues._currentPosition - liveData.liveValues._requestedPosition;


  // ------------------------------
  //
  //  PID calculation
  //
  // ------------------------------


  _currentITermTotal += (setupData.parameters._ITerm * liveData.liveValues._currentError);

  if (_currentITermTotal > setupData.parameters._ITermLimit) {
    _currentITermTotal = setupData.parameters._ITermLimit;
  }
  else if (_currentITermTotal < -setupData.parameters._ITermLimit) {
    _currentITermTotal = -setupData.parameters._ITermLimit;
  }

  liveData.liveValues._PIDOutput = setupData.parameters._PTerm * (liveData.liveValues._currentError + _currentITermTotal - (setupData.parameters._DTerm * liveData.liveValues._currentVelocity));


  // ------------------------------
  //
  //  Calculate velocity and
  //  acceleration feedforwards
  //
  // ------------------------------


  liveData.liveValues._velocityFeedForwardOutput = setupData.parameters._velocityFeedForward * _requestedVelocity;
  liveData.liveValues._accelerationFeedForwardOutput = setupData.parameters._accelerationFeedForward * _requestedAcceleration;


  // ------------------------------
  //
  //  Calculate friction feedforward
  //
  //  This just adds an offset for stiction
  //  at the moment; coulombic friction
  //  will hopefully be handled by the
  //  velocity feed forward (ha)
  //
  // ------------------------------


  if (abs(liveData.liveValues._currentVelocity) < setupData.parameters._stictionVelocity) {
    if (liveData.liveValues._currentError > 0.0) {
      liveData.liveValues._frictionFeedForwardOutput = setupData.parameters._stiction;
    } else if (liveData.liveValues._currentError < 0.0) {
      liveData.liveValues._frictionFeedForwardOutput = -(setupData.parameters._stiction);
    } else {
      liveData.liveValues._frictionFeedForwardOutput = 0.0;
    }
  } else {
    liveData.liveValues._frictionFeedForwardOutput = 0.0;
  }


  // ------------------------------
  //
  //  Total everything up and send
  //  to motor
  //
  // ------------------------------


  liveData.liveValues._outputPWM = liveData.liveValues._PIDOutput + liveData.liveValues._velocityFeedForwardOutput + liveData.liveValues._accelerationFeedForwardOutput + liveData.liveValues._frictionFeedForwardOutput;

  // ------------------------------
  //
  //  Limit the PWM
  //  and take dead band into account
  //
  // ------------------------------

  if (abs(liveData.liveValues._currentError) < setupData.parameters._deadband) {
    liveData.liveValues._outputPWM = 0.0;
  }

  liveData.liveValues._outputPWM /= 255;


  if (liveData.liveValues._outputPWM > setupData.parameters._PWMLimit) {
    liveData.liveValues._outputPWM = setupData.parameters._PWMLimit;
  } else if (liveData.liveValues._outputPWM < -setupData.parameters._PWMLimit) {
    liveData.liveValues._outputPWM = -setupData.parameters._PWMLimit;
  }


  // ------------------------------
  //
  //  Home switch stuff:
  //
  // ------------------------------

  if (digitalRead(HOME_SENSOR_SIGNAL) != _homeSensorState) {
    if (!_homeSensorState) {
      _homeSensorState = true;
      liveData.liveValues._homeSensorPos = liveData.liveValues._currentPosition;
      liveData.liveValues._homeSensorBand = liveData.liveValues._currentVelocity;

    } else {
      _homeSensorState = false;
    }
  }



  // ------------------------------
  //
  //  Send control value to the motor
  //
  // ------------------------------


  _motor->setPWM(liveData.liveValues._outputPWM);


  // ------------------------------
  //
  //  Get next setpoint. This is
  //  done at the end of the loop
  //  rather than the beginning
  //  as it's used to trigger the
  //  communications to the host.
  //  Best if that happens right
  //  after the PID update.
  //
  // ------------------------------

  _nextRequestedPosition = getNextSetpoint();

  digitalWrite(13, LOW);
}


// ------------------------------
//
//  Calibration utilities
//
// ------------------------------

long ControlLoop::getCurrentPositionRaw() {
  return _encoder->read();
}

void ControlLoop::setOutputPWMDirectly(float thePWM) {
  _motor->setPWM(thePWM);
}


void ControlLoop::_PIDReset()
{
  //_encoder->write(0);

  liveData.liveValues._currentPosition = _getFilteredCurrentPosition();
  liveData.liveValues._requestedPosition = liveData.liveValues._currentPosition;
  _lastRequestedPosition = liveData.liveValues._currentPosition;
  _nextRequestedPosition = liveData.liveValues._currentPosition;

  //_inputBufferRunningTotal = 0;
  //_inputBufferPosition = 0;

  liveData.liveValues._currentVelocity = _requestedVelocity = liveData.liveValues._currentAcceleration = _requestedAcceleration = 0;

  _currentITermTotal = 0;
  liveData.liveValues._PIDOutput = 0;

  liveData.liveValues._velocityFeedForwardOutput = 0;
  liveData.liveValues._accelerationFeedForwardOutput = 0;
  liveData.liveValues._frictionFeedForwardOutput = 0;

  liveData.liveValues._outputPWM = 0;

}


