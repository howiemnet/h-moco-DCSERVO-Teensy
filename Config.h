// ------------------------------------------
//
//     Config.h
//
//    (C) 2015 h
//
//    Defines starting values for a number of
//    different motor configurations.
//
//  
//
// ------------------------------------------

#define PID_LOOP_FREQUENCY 2000
#define FRAME_RATE 25
#define PID_LOOPS_PER_FRAME (PID_LOOP_FREQUENCY / FRAME_RATE)



//#define TURNTABLE_MOTOR
#define PAN_MOTOR
//#define TILT_MOTOR
// #define SLIDER_MOTOR




// ------------------------------------------
//  Individual configuration set:
//
//  SLIDER MOTOR (24v)
//
// ------------------------------------------

#ifdef SLIDER_MOTOR

#define THIS_INTERFACE_ID 8

// encoder
#define ENCODER_CHANNEL_A 22
#define ENCODER_CHANNEL_B 23

// home sensor
#define HOME_SENSOR_GROUND 16
#define HOME_SENSOR_SIGNAL 15
#define HOME_SENSOR_POWER 17 


// motor driver
#define MOTOR_DRIVER_GROUND 1
#define MOTOR_DRIVER_ENABLE 2
#define MOTOR_DRIVER_LEFT_PWM 3
#define MOTOR_DRIVER_RIGHT_PWM 4

//
#define MOTOR_PWM_FREQUENCY 22050
#define MOTOR_PWM_RESOLUTION 8

// PID stuff
#define PRESET_P_TERM 0.2
#define PRESET_I_TERM 0.0
#define PRESET_D_TERM 0.0

#define PRESET_VFF_TERM 0.0
#define PRESET_AFF_TERM 0.0
#define PRESET_I_TERM_LIMIT 30
#define PRESET_STICTION 0.0
#define PRESET_STICTION_VELOCITY 0.0
#define PRESET_BACKLASH 0
#define PRESET_BACKLASH_FACTOR 0
#define PRESET_DEADBAND 10

#define PRESET_PWM_LIMIT 0.4

#define INPUT_BUFFER_LENGTH 2

// safety stuff
#define HARD_LIMIT_MINIMUM -5000
#define HARD_LIMIT_MAXIMUM 5000
#define DISABLE_MOTOR_ON_HARD_LIMIT false
#define DISABLE_MOTOR_ON_NO_FRAMES false


#endif






// ------------------------------------------
//  Individual configuration set:
//
//  TILT MOTOR (9v)
//
// ------------------------------------------

#ifdef TILT_MOTOR

#define THIS_INTERFACE_ID 3

// encoder
#define ENCODER_CHANNEL_A 22
#define ENCODER_CHANNEL_B 23

// home sensor
#define HOME_SENSOR_GROUND 16
#define HOME_SENSOR_SIGNAL 15
#define HOME_SENSOR_POWER 17 


// motor driver
#define MOTOR_DRIVER_GROUND 7
#define MOTOR_DRIVER_ENABLE 5
#define MOTOR_DRIVER_LEFT_PWM 4
#define MOTOR_DRIVER_RIGHT_PWM 6

//
#define MOTOR_PWM_FREQUENCY 22050
#define MOTOR_PWM_RESOLUTION 8

// PID stuff
#define PRESET_P_TERM 1.0
#define PRESET_I_TERM 0.0
#define PRESET_D_TERM -0.01

#define PRESET_VFF_TERM 0.0
#define PRESET_AFF_TERM 0.0
#define PRESET_I_TERM_LIMIT 30
#define PRESET_STICTION 0.0
#define PRESET_STICTION_VELOCITY 0.0
#define PRESET_BACKLASH 0
#define PRESET_BACKLASH_FACTOR 0
#define PRESET_DEADBAND 10

#define PRESET_PWM_LIMIT 0.3

#define INPUT_BUFFER_LENGTH 2

// safety stuff
#define HARD_LIMIT_MINIMUM -5000
#define HARD_LIMIT_MAXIMUM 5000
#define DISABLE_MOTOR_ON_HARD_LIMIT false
#define DISABLE_MOTOR_ON_NO_FRAMES false


#endif





// ------------------------------------------
//  Individual configuration set:
//
//  PAN MOTOR (9v)
//
// ------------------------------------------

#ifdef PAN_MOTOR

#define THIS_INTERFACE_ID 2

// encoder
#define ENCODER_CHANNEL_A 22
#define ENCODER_CHANNEL_B 23

// home sensor
#define HOME_SENSOR_GROUND 16
#define HOME_SENSOR_SIGNAL 15
#define HOME_SENSOR_POWER 17 


// motor driver
#define MOTOR_DRIVER_GROUND 7
#define MOTOR_DRIVER_ENABLE 5
#define MOTOR_DRIVER_LEFT_PWM 4
#define MOTOR_DRIVER_RIGHT_PWM 6

//
#define MOTOR_PWM_FREQUENCY 22050
#define MOTOR_PWM_RESOLUTION 8

// PID stuff
#define PRESET_P_TERM 0.35
#define PRESET_I_TERM 0.001
#define PRESET_D_TERM -64.0

#define PRESET_VFF_TERM 0.0
#define PRESET_AFF_TERM 0.0
#define PRESET_I_TERM_LIMIT 50
#define PRESET_STICTION 0.0
#define PRESET_STICTION_VELOCITY 2.0
#define PRESET_BACKLASH 0
#define PRESET_BACKLASH_FACTOR 0
#define PRESET_DEADBAND 5

#define PRESET_PWM_LIMIT 0.8

#define INPUT_BUFFER_LENGTH 10

// safety stuff
#define HARD_LIMIT_MINIMUM -5000
#define HARD_LIMIT_MAXIMUM 5000
#define DISABLE_MOTOR_ON_HARD_LIMIT false
#define DISABLE_MOTOR_ON_NO_FRAMES false


#endif




// ------------------------------------------
//  Individual configuration set:
//
//  TURNTABLE MOTOR (24V)
//
// ------------------------------------------

#ifdef TURNTABLE_MOTOR

#define THIS_INTERFACE_ID 7

// encoder
#define ENCODER_CHANNEL_A 22
#define ENCODER_CHANNEL_B 23

// home sensor
#define HOME_SENSOR_GROUND 16
#define HOME_SENSOR_SIGNAL 15
#define HOME_SENSOR_POWER 17 


// motor driver
#define MOTOR_DRIVER_GROUND 1
#define MOTOR_DRIVER_ENABLE 2
#define MOTOR_DRIVER_LEFT_PWM 3
#define MOTOR_DRIVER_RIGHT_PWM 4

#define MOTOR_PWM_FREQUENCY 22050
#define MOTOR_PWM_RESOLUTION 8

// PID stuff
#define PRESET_P_TERM 3.0
#define PRESET_I_TERM 0.0
#define PRESET_D_TERM -20.0

#define PRESET_VFF_TERM 0.0
#define PRESET_AFF_TERM 0.0
#define PRESET_I_TERM_LIMIT 5
#define PRESET_STICTION 0
#define PRESET_STICTION_VELOCITY 0.0
#define PRESET_BACKLASH 0
#define PRESET_BACKLASH_FACTOR 0
#define PRESET_DEADBAND 4.0

#define PRESET_PWM_LIMIT 0.4

#define INPUT_BUFFER_LENGTH 10

// safety stuff
#define HARD_LIMIT_MINIMUM -5000
#define HARD_LIMIT_MAXIMUM 5000
#define DISABLE_MOTOR_ON_HARD_LIMIT false
#define DISABLE_MOTOR_ON_NO_FRAMES false


#endif
