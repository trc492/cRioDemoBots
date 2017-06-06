#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the physical definitions of the robot.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

#define DEGREES_PER_RADIAN              (180.0/PI)

//
// CAN IDs for Jaguars.
//
#define CANID_LEFTFRONT_JAG             2       //green
#define CANID_LEFTREAR_JAG              3       //magenta
#define CANID_RIGHTFRONT_JAG            4       //orange
#define CANID_RIGHTREAR_JAG             5       //blue
#define CANID_FRONTSHOOTER_JAG          6       //yellow
#define CANID_REARSHOOTER_JAG           7       //white
#define CANID_PAN_JAG                   8       //gray
#define CANID_TILT_JAG                  9       //red

#define PWM_FEEDER_MOTOR                1       //orange

//
// Analog Channels.
//
#define AIN_GYRO                        1

//
// Digital Channels.
//
#define DIN_CMP_PRESSURE_SWITCH         3
#define DIN_FEEDER_SWITCH               2

//
// Relay Channels. 
//

#define RELAY_COMPRESSOR                1
#define RELAY_RINGLIGHT_POWER           2
#define RELAY_BLOWER_POWER              4

//
// Solenoid Channels.
//

// Solenoid module 3 - pistons only

//pushes the frisbee into the wheels to fire it
#define SOL_DEPLOYER_EXTEND             ((UINT32)1)
#define SOL_DEPLOYER_RETRACT            ((UINT32)2)

#define SOL_WHEELIE_EXTEND              ((UINT32)3)
#define SOL_WHEELIE_RETRACT             ((UINT32)4)

#define SOL_PANLIGHT_LEFT               ((UINT32)5)
#define SOL_PANLIGHT_RIGHT              ((UINT32)6)

#define SOL_TILTLIGHT_LOW               ((UINT32)7)
#define SOL_TILTLIGHT_HIGH              ((UINT32)8)

//
// Input Subsystems.
//
#define DEADBAND_INPUT_THRESHOLD        0.15
#define PAN_SLOWDOWN                    0.45
#define TILT_SLOWDOWN                   0.55
#define CAMERA_IP                       "10.4.92.11"
#define JSPORT_DRIVE_LEFT               1
#ifdef _USE_DUAL_JOYSTICKS
#define JSPORT_DRIVE_RIGHT              2
#endif
#define JSPORT_SHOOTER                  3

//
// DriveBase Subsystem.
//
#define WHEEL_DIAMETER                  6.0
#define DRIVE_ENCODER_PPR               360
#define DISTANCE_PER_REV                (WHEEL_DIAMETER*PI)

#define MOTOR_LEFT_FRONT_REVERSE        false
#define MOTOR_LEFT_REAR_REVERSE         false
#define MOTOR_RIGHT_FRONT_REVERSE       false
#define MOTOR_RIGHT_REAR_REVERSE        false

#define ENCODER_POLARITY_LEFT_FRONT     1.0
#define ENCODER_POLARITY_LEFT_REAR      1.0
#define ENCODER_POLARITY_RIGHT_FRONT    -1.0
#define ENCODER_POLARITY_RIGHT_REAR     -1.0

//
// Ziegler-Nichols method of tuning PID:
// (http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
//
//  P controller            - Kp = 0.500Ku
//  PI controller           - Kp = 0.450Ku, Ki = 1.2Kp/Tu
//  PID controller          - Kp = 0.600Ku, Ki = 2.0Kp/Tu,  Kd = 0.125KpTu
//  Pessen Integral Rule    - Kp = 0.700Ku, Ki = 2.5Kp/Tu,  Kd = 0.150KpTu
//  Some overshoot          - Kp = 0.333Ku, Ki = 1.0Kp/Tu,  Kd = 0.333KpTu
//  No overshoot            - Kp = 0.200Ku, Ki = 2.0Kp/Tu,  Kd = 0.333KpTu
//
#define DRIVE_KP                        0.5
#define DRIVE_KI                        0.0
#define DRIVE_KD                        0.0
#define DRIVE_KF                        0.0
#define DRIVE_TOLERANCE                 0.5
#define DRIVE_SETTLING                  200

//error in degrees
#define TURN_KP                         0.4     //0.017 //bigbot??  //0.05
#define TURN_KI                         0.0     //0.01  //??0.001
#define TURN_KD                         0.0     //0.03  //??0.5
#define TURN_KF                         0.0
#define TURN_TOLERANCE                  0.5
#define TURN_SETTLING                   200

#define DRIVE_RANGE_MIN                 -1.0
#define DRIVE_RANGE_MAX                 1.0
#define DRIVE_SLOWDOWN_X                1.0
#define DRIVE_SLOWDOWN_Y                0.4
#define DRIVE_SLOWDOWN_ROT              0.6

//
// Vision Target Subsystem.
//
#define TARGET_MODE_TOP                 1
#define TARGET_MODE_LEFT_SIDE           2
#define TARGET_MODE_RIGHT_SIDE          3
#define TARGET_MODE_CENTERMOST_TARGET   4   //only used during autonomous (to select any target)

#define SIDE_TARGET_DISTANCE_CONSTANT   12383.25          //h1*d1
#define TOP_TARGET_DISTANCE_CONSTANT    8469.5
#define LOW_TARGET_DISTANCE_CONSTANT    13000           //????
#define FOCAL_LENGTH                    392.61          //FocalLength=392.61
#define TARGET_KDH_LOW                  13659.1         //hl*d1=150.1*91
#define TARGET_KDH_MID                  939.6           //hm*d1=156.6*5
#define TARGET_KDH_HIGH                 -14406.6        //hh*d1=184.7*-78
#define TARGET_HEIGHT_LOW               (31.0    - CAMERA_HEIGHT)  //~8 in
#define TARGET_HEIGHT_SIDE              (99.125  - CAMERA_HEIGHT)  //~41 in
#define TARGET_HEIGHT_TOP               (110.125 - CAMERA_HEIGHT)  //~78 in
#define SCREEN_CENTER_X                 160
#define SCREEN_CENTER_Y                 120

#define MIN_TOP_TARGET_ASPECT_RATIO     2.35
#define MAX_TOP_TARGET_ASPECT_RATIO     3.60
#define MIN_SIDE_TARGET_ASPECT_RATIO    1.05
#define MAX_SIDE_TARGET_ASPECT_RATIO    2.40
#define MIN_LOW_TARGET_ASPECT_RATIO     0.50
#define MAX_LOW_TARGET_ASPECT_RATIO     0.95

#ifdef _USE_COMPETITION_ROBOT
  #define CAMERA_HEIGHT                 24.0
#else
  #define CAMERA_HEIGHT                 23.0    //???
#endif


#define MAX_ASPECT_RATIO_ANY_TARGET     (MAX_TOP_TARGET_ASPECT_RATIO)
#define MIN_ASPECT_RATIO_ANY_TARGET     (MIN_SIDE_TARGET_ASPECT_RATIO)

#ifdef _USE_COMPETITION_ROBOT
  #define GRAVITY_COMPENSATION_CONSTANT 35
#else
  #define GRAVITY_COMPENSATION_CONSTANT 35
#endif

//
//
// Shooter Subsystem.
//
#define SHOOTER_ENCODER_PPR             100
#define SHOOTER_SYNC_GROUP              ((UINT8)1)

#define SHOOTER_KP                      0.0001    //0.000042 for big robot (000005)
                                        //0.000016  6. April... the day we started graphing stuff
#define SHOOTER_KI                      0.0000001
#define SHOOTER_KD                      0.0   //0.05
#define SHOOTER_KF                      0.000285
#define SHOOTER_TOLERANCE               1.0
#define SHOOTER_SETTLING                300
#define TARGET_PANANGLE_TOLERANCE       1.5         //in degrees
#define TARGET_TILTANGLE_TOLERANCE      0.7         //in degrees
#define SHOOTER_SPEED_TOLERANCE         0.10        //percent
#define SHOOTER_INITIAL_OUTPUT          0.85
#define SHOOTER_FIRE_TIME               0.5         //seconds

#define MAX_VOLTAGE_LOW                 8.0
#define MAX_VOLTAGE_HIGH                12.0

#define DSSW_SHOOT                      9
#define DSSW_AIM_HIGHER                 10
#define DSSW_AIM_LOWER                  11
#define DSSW_REMEMBER_V_ANGLE           12
#define DSSW_SHOOTER_ON                 13
#define DSSW_AUTOMODE                   14
#define DSSW_SHOOTER_VOLTAGE            15
#define DSSW_BLOWER_ON                  16

#define V_ANGLE_ADJUSTMENT_INCREMENT    1.0         //degrees

//
// Feeder Subsystem
//
#define PWN_FEEDER_ULTRASONIC           2
#define FEEDER_ULTRASONIC_THRESHOLD     1.0

//
// Autonomous
//
#ifdef _USE_COMPETITION_ROBOT
  #define AUTO_SHOOTER_SPEED            50.0
#else
  #define AUTO_SHOOTER_SPEED            2300.0
#endif
#define AUTO_MAX_SHOOT_INTERVAL         2000
#define AUTO_MINIMUM_STARTUP_TIME       6000
#define BCD_AUTONOMOUS_BITS             0x00ff
#define BCD_LOW_DIGIT_CHANNEL           1
#define BCD_HIGH_DIGIT_CHANNEL          5

#define AUTO_STRATEGY_RIGHT_REAR        1       //- aim at the right side targets and lea\n down
#define AUTO_STRATEGY_LEFT_REAR         2       //- aim at the left side target and lean down
#define AUTO_STRATEGY_RIGHT_FRONT       3       //- aim at the right side targets (don't move)
#define AUTO_STRATEGY_LEFT_FRONT        4       //- aim at the left side target (don't move)
#define AUTO_STRATEGY_CENTER            5       //- aim at the center target (move forward)
#define AUTO_STRATEGY_BLIND             6       //- vision targeting failed... don't aim just shoot
#define AUTO_STRATEGY_NO_OP             7       //- don't do anything.  Something broke

//shooter controll modes
#define SHOOTER_MODE_MANUAL             1
#define SHOOTER_MODE_AUTO               2

//
//
// PanTilt Subsystem.
//
#ifdef _USE_COMPETITION_ROBOT
  #define PAN_DEGREE_PER_ENC_REVOLUTION 18.275
  #define PAN_KP                        0.05
  #define PAN_KI                        0.002
  #define PAN_KD                        0.05
  #define PAN_KF                        0.0
  #define PAN_ENCODER_PPR               360
#else
  #define PAN_DEGREE_PER_ENC_REVOLUTION 16.893
  #define PAN_KP                        0.06
  #define PAN_KI                        0.002
  #define PAN_KD                        0.05
  #define PAN_KF                        0.0
  #define PAN_ENCODER_PPR               360
#endif

#ifdef _USE_COMPETITION_ROBOT
  #define TILT_DOWN_KP                  0.09
  #define TILT_DOWN_KI                  0.00
  #define TILT_DOWN_KD                  0.004
  #define TILT_DOWN_KF                  0.0
 
  #define TILT_UP_KP                    0.06
  #define TILT_UP_KI                    0.0000
  #define TILT_UP_KD                    0.005
  #define TILT_UP_KF                    0.0
#else
  #define TILT_DOWN_KP                  0.065
  #define TILT_DOWN_KI                  0.0002
  #define TILT_DOWN_KD                  0.002
  #define TILT_DOWN_KF                  0.0

  #define TILT_UP_KP                    0.075
  #define TILT_UP_KI                    0.0002
  #define TILT_UP_KD                    0.001
  #define TILT_UP_KF                    0.0
#endif

#define TILT_ENCODER_PPR                360
//#define TILT_DEGREES_PER_CLICK          14.67575774
#ifdef _USE_COMPETITION_ROBOT
  #define TILT_ANGLE_AT_ZERO            38
  #define TILT_STRING_LENGTH_AT_ZERO    18.25
  #define TILT_SPOOL_CIRCUMFRENCE       ((0.84675 + 0.8705)*0.5*PI)
  #define TILT_SPOOL_RADIUS             12.75
  #define TILT_MAGIC_NUMBER             54.3
#else
  #define TILT_ANGLE_AT_ZERO            40
  #define TILT_STRING_LENGTH_AT_ZERO    17.50
  #define TILT_SPOOL_CIRCUMFRENCE       ((0.774 + 0.803)*0.5*PI)
  #define TILT_SPOOL_RADIUS             12.875
  #define TILT_MAGIC_NUMBER             52.6 
#endif

//
// Hanger Subsystem.
//
#define WHEELIE_EXTEND                  SolID(SOL_WHEELIE_EXTEND)
#define WHEELIE_RETRACT                 SolID(SOL_WHEELIE_RETRACT)

#endif  //ifndef _ROBOTINFO_H

