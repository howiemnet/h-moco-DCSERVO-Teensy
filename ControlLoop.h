// --------------------------------------------
// 
//    TEENSY_PID control class
//    (c) h 2015
//
// -------------------------------------------


#include <Arduino.h>
#include <stdint.h>
#include "Config.h"
#include "Motor.h"
#include <Encoder.h>

#define MANUAL 0
#define AUTOMATIC 1

extern float getNextSetpoint();




class ControlLoop {
  
  private:

    // Hardware
    Encoder * _encoder;

  public:


    
    Motor * _motor;
    
    // Interrupt timer
    IntervalTimer _loopTimer;

    // live data for reporting
    struct LiveDataStruct {
      volatile byte           _header[4];
      volatile float _currentPosition;
      volatile float _requestedPosition; // aka PID setpoint
      volatile float _currentVelocity;
      volatile float _currentAcceleration;
      volatile float _currentError;
      volatile float _PIDOutput;
      volatile float _velocityFeedForwardOutput;
      volatile float _accelerationFeedForwardOutput;
      volatile float _frictionFeedForwardOutput;
      volatile float _outputPWM;
      float          _homeSensorPos;
      float          _homeSensorBand;
      volatile int32_t _debug;
      volatile int32_t _currentPlaybackFrame;
      int32_t        _timeStamp;
    };

    struct Parameters {

    // _header: bytes 0 and 1 are left for ID and messageType. 
    // byte 2: bit 0 is hardLimitEnabled flag
      
      byte           _header[4];

      float          _PTerm;
      float          _ITerm;
      float          _DTerm;
      float          _ITermLimit;
      float          _velocityFeedForward;
      float          _accelerationFeedForward;
      float          _stiction;
      float          _stictionVelocity;
      float          _backlash;
      float          _backlashFactor;
      float          _deadband;
      float          _PWMLimit;
      float           _hardLimitMinimum;
      float           _hardLimitMaximum;
      float          _futureUseQ;
    };
    
    union {
      LiveDataStruct  liveValues;
      byte            USBLiveDataBuffer[64];
    } liveData;

   union {
    Parameters parameters;
    byte        USBPersistentDataBuffer[64];
   } setupData;
    
    // position variables
    volatile float _lastPosition;
    volatile float _lastRequestedPosition;
    volatile float _nextRequestedPosition;


    // input buffer
    volatile float _inputBuffer[INPUT_BUFFER_LENGTH];
    volatile float _inputBufferRunningTotal;
    volatile uint8_t _inputBufferPosition;
    volatile float _lastUnfilteredReading;
    volatile float _unfilteredReading;
    volatile float _lastFilteredReading;
    volatile float _filteredReading;
 
    
    // velocity and acceleration

    volatile uint8_t _velCountsSeen;
    volatile uint8_t _velCountsHaveTaken;
    
    volatile uint8_t _reqVelCountsSeen;
    volatile uint8_t _reqVelCountsHaveTaken;
        
    volatile float _requestedVelocity;
    volatile float _requestedAcceleration;
    


    // PID calculation stuff
   volatile float _currentITermTotal;

    // home switch state
    bool _homeSensorState;
 
    
    // output
 
    // error recording
    volatile uint32_t _cumError;
    volatile uint32_t _largestError;
    volatile int32_t _errorSquared;
    volatile int32_t _errorCount;

    // safety limits
    bool _safetyLimitHit;

    float _getFilteredCurrentPosition();

    long getCurrentPositionRaw();
    void setOutputPWMDirectly(float thePWM);
  

    ControlLoop();
    void init();
    void enablePIDControl();
    void disablePIDControl();
    void _PIDReset();
    void theLoop(void);
      
 
} ;
