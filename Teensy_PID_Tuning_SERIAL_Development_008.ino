// --------------------------------------------
//
//    TEENSY_PID_B controller tester / calibrater etc
//    (c) h 2015
//
// -------------------------------------------
//
//   version 7: 26/10/2015
//      Un-scaled velocity (!)
//   version 6: 26/10/2015
//      Added home switch reporting
//   version 5: 23/10/2015
//      Moving from interrupt-driven to elapsedMicros
//

#define MODE_SERIAL
// (that is, not USB)

#include "Config.h"
//
//
//

#include <Encoder.h> // just so the other files can find it
#include "ControlLoop.h" // PID controller




extern ControlLoop controller;
elapsedMillis ticker;
elapsedMicros cycleTicker;
#define MICROS_PER_PID_CYCLE 500


// USB stuff
// number of PID updates per USB report
#define PID_TO_USB_UPDATE_RATIO 4
#define PID_TO_MANUAL_UPDATE_RATIO 80

uint32_t USBMessageCount;
uint16_t  USBUpdateRatioCounter;
uint16_t  messageUpdateRatio;
byte     StatusOutgoingBuffer[64];
byte     USBOutgoingBuffer[64];
byte     USBIncomingBuffer[64];
byte     USBResponse;
bool     USBDataStreamingEnabled = false;


// ------------------------------------------------------
//
//    SETUP
//
// ------------------------------------------------------


void setup() {
  controller.init();
  Serial.begin(1152000);
  pinMode (13, OUTPUT);

  digitalWrite(13, LOW);

  StatusOutgoingBuffer[0] = 0;
  StatusOutgoingBuffer[1] = 1;
  StatusOutgoingBuffer[2] = 1;
  StatusOutgoingBuffer[3] = THIS_INTERFACE_ID;

}


// ------------------------------------------------------
//
//    LOOP
//
//    Not a lot happens here; most things are checked
//    in the getNextSetpoint() func underneath
//
// ------------------------------------------------------


void loop() {
  if (cycleTicker >= MICROS_PER_PID_CYCLE) {
    cycleTicker -= MICROS_PER_PID_CYCLE;
    controller.theLoop();
  }

}

// ------------------------------------------------------
//
//    getNextSetpoint()
//
//    This handles quite a bit; it's triggered at the
//    end of each PID cycle, so it's the best time to
//    do any calculations, comms etc.
//
// ------------------------------------------------------

#define STATE_MOTOR_OFF 0
#define STATE_MOTOR_ON_IDLE 1
#define STATE_MOTOR_ON_PLAYING_FRAME 2
#define STATE_MOTOR_OFF_OUT_OF_BOUNDS 3
#define STATE_MOTOR_OFF_RAN_OUT_OF_FRAMES 4
#define STATE_DOING_STICTION_CALIBRATION 5
#define BUFFER_STATE_NEED_NEXT_FRAME 1
#define BUFFER_STATE_OK 0

volatile uint8_t state;
volatile uint8_t buffer_state;
long manualPosition;

float getNextSetpoint() {

  checkAndHandleInput();
  

  float returnValue = 0.0;
  switch (state) {
    case STATE_MOTOR_ON_PLAYING_FRAME:
      returnValue = getNextAnimatedSetpoint();
      break;
    case STATE_DOING_STICTION_CALIBRATION:
      calibrateStictionLoop();
      break;
    case STATE_MOTOR_ON_IDLE:
    default:
      returnValue = (float) manualPosition;

  }
  return returnValue;
}

volatile int32_t lastFramePosition;
volatile int32_t currentFramePosition;
volatile int32_t nextFramePosition;
float currentFrameVelocity;
float nextFrameVelocity;
float frameDelta;
volatile bool nextFrameReceived;
volatile uint16_t subFrame;
volatile uint32_t nextFrameNumber;


float getNextAnimatedSetpoint() {
  float returnValue = 0.0;
  subFrame++;
  if (subFrame >= PID_LOOPS_PER_FRAME) {
    subFrame = 0;
    controller.liveData.liveValues._currentPlaybackFrame++;
    if (buffer_state == BUFFER_STATE_OK) {
      lastFramePosition = currentFramePosition;
      currentFramePosition = nextFramePosition;
      frameDelta = currentFramePosition - lastFramePosition;
      buffer_state = BUFFER_STATE_NEED_NEXT_FRAME;

      sendBufferRequestPacket();

#ifndef MODE_SERIAL
      Serial.print("BUFFER_NEED_FRAMES at message ID ");
      Serial.print(USBMessageCount);
      Serial.print(" - just started playback of frame ");
      Serial.print(controller.liveData.liveValues._currentPlaybackFrame);

      Serial.println("-----");
#endif
      float oldVelocity = controller._requestedVelocity;
      controller._requestedVelocity = frameDelta;
      controller._requestedAcceleration = controller._requestedVelocity - oldVelocity;
      returnValue = (float) lastFramePosition;
    } else {
#ifndef MODE_SERIAL
      Serial.println("Ran out of frames...");
      Serial.print("... Ran out of frames...");
#endif
      controller._requestedVelocity = 0.0;
      controller._requestedAcceleration = 0.0;

      buffer_state = BUFFER_STATE_OK;
      //doUSBUpdate();
      if (DISABLE_MOTOR_ON_NO_FRAMES) {
        state = STATE_MOTOR_OFF_RAN_OUT_OF_FRAMES;
        
        controller.disablePIDControl();
      } else {
        state = STATE_MOTOR_OFF_RAN_OUT_OF_FRAMES;
      
        doUSBUpdate();
        state = STATE_MOTOR_ON_IDLE;
      }
      manualPosition = currentFramePosition;
      lastFramePosition = currentFramePosition;
      returnValue = currentFramePosition;
    }

  } else { // mid frame
    returnValue = (float) lastFramePosition + (( (float) frameDelta * (float) subFrame) / (float) PID_LOOPS_PER_FRAME);
  }



  return (float) returnValue;
}



void reportBoundsError() {
  state = STATE_MOTOR_OFF_OUT_OF_BOUNDS;
}






float currentPWM = 0.0;
float calibPWMIncrement = 0.02;
int cyclesPerCalibIncrement = 0;
int cyclesPerCalibIncrementReset = 100;
long sticCalibCurrentPosition = 0;


void startStictionCalibration() {
  // reset variables
  // turn off PID
  // wait for no movement
  //Serial.println("Starting stiction calibration...");
  currentPWM = 0;
  cyclesPerCalibIncrement = cyclesPerCalibIncrementReset;
  sticCalibCurrentPosition = controller.getCurrentPositionRaw();
  state = STATE_DOING_STICTION_CALIBRATION;


}

void stopStictionCalibration() {
  state = STATE_MOTOR_OFF;
  controller.setOutputPWMDirectly(0.0);
}

void calibrateStictionLoop() {
  // called every PID loop
  // check position
  // if it's changed, report back and stop!
  // if not,
  // decrement cycles per sticCalibIncrementDelay
  // if it's time,
  // increase PWM output
  if (sticCalibCurrentPosition != controller.getCurrentPositionRaw()) {
    // STOP
    // Serial.print("Motor moved at (*1000) ");
    // Serial.println(currentPWM * 1000);
    stopStictionCalibration();
  } else {
    cyclesPerCalibIncrement--;
    if (cyclesPerCalibIncrement == 0) {
      cyclesPerCalibIncrement = cyclesPerCalibIncrementReset;
      currentPWM += calibPWMIncrement;
      controller.setOutputPWMDirectly(currentPWM);
    }
  }

}





void checkAndHandleInput() {
#ifndef MODE_SERIAL

  do {
    USBResponse = RawHID.recv(&USBIncomingBuffer, 0);
    if (USBResponse > 0) {
      handleUSBInput();
    }
  } while (USBResponse > 0);


#else  // serial comms

  if (Serial.available() > 63) {
    for (int i = 0; i < 64; i++) {
      USBIncomingBuffer[i] = Serial.read();
    }
    handleUSBInput();
  }

#endif


  if (USBDataStreamingEnabled) {
    USBUpdateRatioCounter++;

    if (USBUpdateRatioCounter >= messageUpdateRatio) {
      USBUpdateRatioCounter = 0;
      doUSBUpdate();
    }
  }




}

void sendIdentificationPacket() {
  StatusOutgoingBuffer[0] = 0;
  StatusOutgoingBuffer[1] = 1;
  StatusOutgoingBuffer[2] = 1;
  StatusOutgoingBuffer[3] = THIS_INTERFACE_ID;

#ifndef MODE_SERIAL
  RawHID.send(&StatusOutgoingBuffer, 10);
#else
  for (int i = 0; i < 64; i++) {
    Serial.write(StatusOutgoingBuffer[i]);
  }
#endif
}

void sendBufferRequestPacket() {
  doUSBUpdate();

}

void handleUSBInput() {
  switch (USBIncomingBuffer[0]) {
    case 105: // 'i'
      // identify and reset!
      sendIdentificationPacket();
      USBDataStreamingEnabled = false;
      break;
    case 0: //
      //Serial.println("MSG: disablePID");
      commandDisablePID();
      messageUpdateRatio = PID_TO_MANUAL_UPDATE_RATIO ;
      USBDataStreamingEnabled = true;
      break;
    case 1:
      //Serial.println("MSG: enablePID");
      commandEnablePID();
      manualPosition = controller.liveData.liveValues._currentPosition;
      
      messageUpdateRatio = PID_TO_USB_UPDATE_RATIO;
      USBDataStreamingEnabled = false;
      break;
    case 2:
      receiveNextFrame();
      break;
    case 3:
      startAnimation();
      break;
    case 4:
      stopAnimation();
      break;
    case 5:
      goDirectlyToPosition();
      break;
    case 6:
      //  resetEncoder();
      break;
    case 7:
      getParameters();
      break;
    case 8:
      setParameters();
      break;
    case 9:
      //   Serial.println("MSG: enableStreaming");
      USBDataStreamingEnabled = true;
      break;
    case 10:
      //    Serial.print("MSG: disableStreaming - sent packets: ");
      //   Serial.println(USBMessageCount);
      USBDataStreamingEnabled = false;
      break;
    case 11:
      if (state == STATE_MOTOR_OFF) {
        startStictionCalibration();
      } else {
        //        Serial.println("Can't start calib - the motor's on");
      }

      break;
    case 12:
      if (state == STATE_DOING_STICTION_CALIBRATION) {
        stopStictionCalibration();
      }
      break;
      case 16:
      doUSBUpdate();
      break;
  }
}

void commandDisablePID() {
  controller.disablePIDControl();
  //controller.liveData.liveValues._requestedPosition = controller.liveData.liveValues._currentPosition;
  state = STATE_MOTOR_OFF;
}

void commandEnablePID() {
  //manualPosition = 0.0;
  controller.enablePIDControl();
  state = STATE_MOTOR_ON_IDLE;
}

void receiveNextFrame() {
  if (state == STATE_MOTOR_ON_IDLE) {
    startAnimation();
  }
  else {
    
      buffer_state = BUFFER_STATE_OK;
      nextFramePosition = getLongFromIncomingBufferAtPosition(1);
      
  }

}

void startAnimation() {
  controller.liveData.liveValues._currentPlaybackFrame = 1;
  //  Serial.println("start anim");
  if (state == STATE_MOTOR_ON_IDLE) {
    controller.liveData.liveValues._currentPlaybackFrame = 1;
    subFrame = 999;
    nextFrameNumber = 1;
    currentFramePosition = controller.liveData.liveValues._currentPosition;
    nextFramePosition = getLongFromIncomingBufferAtPosition(1);
    //controller.liveData.liveValues._currentPosition;
    //lastFramePosition = controller.liveData.liveValues._currentPosition;
    //controller.liveData.liveValues._bufferedUpToFrame = 1;//nextFramePosition = getLongFromIncomingBufferAtPosition(5);

    state = STATE_MOTOR_ON_PLAYING_FRAME;
    buffer_state = BUFFER_STATE_OK;
   // sendBufferRequestPacket();
    //        Serial.print("BUFFER_NEED_FRAMES (start of animation) at message ID ");
    //    Serial.println(USBMessageCount);
    //    Serial.println("-----");

  } else {
    //      Serial.println(".. can't, motor not idle");

  }

}

void stopAnimation() {
  //  Serial.println("stop anim");
  if (state == STATE_MOTOR_ON_PLAYING_FRAME) {
    manualPosition = controller.liveData.liveValues._requestedPosition;
    state = STATE_MOTOR_ON_IDLE;
    buffer_state = BUFFER_STATE_OK;
  }
}

void goDirectlyToPosition() {
  //  Serial.println("go to position");
  manualPosition = getLongFromIncomingBufferAtPosition(1);
}

void getParameters() {
  //  Serial.println("received a parameter request packet");
#ifndef MODE_SERIAL
  RawHID.send(&controller.setupData.USBPersistentDataBuffer, 10);
#else
  for (int i = 0; i < 64; i++) {
    Serial.write(controller.setupData.USBPersistentDataBuffer[i]);
  }
#endif
}
void setParameters() {
  //  Serial.println("received a set-parameters packet");
  memcpy(controller.setupData.USBPersistentDataBuffer, USBIncomingBuffer, 64);

}


void doUSBUpdate() {

  USBMessageCount ++;
  controller.liveData.liveValues._header[2] = (controller._homeSensorState) ? (state | 64) : state;
  
  controller.liveData.liveValues._header[3] = buffer_state;
  controller.liveData.liveValues._debug = USBMessageCount;


  controller.liveData.liveValues._timeStamp = millis();

#ifndef MODE_SERIAL
  RawHID.send(&controller.liveData.USBLiveDataBuffer, 1);
#else
  for (int i = 0; i < 64; i++) {
    Serial.write(controller.liveData.USBLiveDataBuffer[i]);
  }
#endif



}

float getFloatFromIncomingBufferAtPosition(uint8_t bufferPosition) {
  float myRetValue = 0.0;
  memcpy(&myRetValue, &USBIncomingBuffer[bufferPosition], 4);
  return myRetValue;
}

void putLongInOutgoingBufferAtPosition(long theValue, uint8_t bufferPosition) {
  USBOutgoingBuffer[bufferPosition]     = theValue & 0xFF;
  USBOutgoingBuffer[bufferPosition + 1] = (theValue >> 8)  & 0xFF;
  USBOutgoingBuffer[bufferPosition + 2] = (theValue >> 16) & 0xFF;
  USBOutgoingBuffer[bufferPosition + 3] = (theValue >> 24) & 0xFF;
}

int32_t getLongFromIncomingBufferAtPosition(uint8_t bufferPosition) {
  int32_t theVal = ((int32_t) USBIncomingBuffer[bufferPosition] & 0xFF);
  theVal |= (((int32_t) USBIncomingBuffer[bufferPosition + 1] & 0xFF)) << 8;
  theVal |= (((int32_t) USBIncomingBuffer[bufferPosition + 2] & 0xFF)) << 16;
  theVal |= (((int32_t) USBIncomingBuffer[bufferPosition + 3] & 0xFF)) << 24;
  return theVal;
}

