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

//
// CAN IDs for Jaguars.
//
#define CANID_LEFTFRONT_JAG             2       //orange
#define CANID_LEFTREAR_JAG              3       //yellow
#define CANID_RIGHTFRONT_JAG            4       //green
#define CANID_RIGHTREAR_JAG             5       //blue
#define CANID_SHOOTER1_JAG              6       //purple
#define CANID_SHOOTER2_JAG              7       //gray

//
// PWM Channels.
//
#define PWM_CONVEYOR_MOTOR              1       //white
#define PWM_ROLLER_MOTOR                2       //orange

//
// Analog Channels.
//
#define AIN_GYRO                        1

//
// Digital Channels.
//
#define DIN_CMP_PRESSURE_SWITCH         1

//
// Relay Channels.
//

#define RELAY_COMPRESSOR                1       //green
#define RELAY_RINGLIGHT_POWER           2       //yellow

//
// Solenoid Channels.
//
#define SOL_WHEELIE_UP                  ((UINT32)1)
#define SOL_WHEELIE_DOWN                ((UINT32)2)
#define SOL_BALLGATE_CLOSE              ((UINT32)3)
#define SOL_BALLGATE_OPEN               ((UINT32)4)

#define SOL_LEFT_LIGHT                  ((UINT32)5)
#define SOL_MIDDLE_LIGHT                ((UINT32)6)
#define SOL_RIGHT_LIGHT                 ((UINT32)7)
#define SOL_GREEN_LIGHT                 ((UINT32)8)


#define SOL_MODULE2                     ((UINT8)2)
#define SOL_WHITE_LIGHT                 ((UINT32)1)

//
// Input Subsystems.
//
#define DEADBAND_INPUT_THRESHOLD        0.15

#define CAMERA_IP                       "10.4.92.11"
#define JSPORT_DRIVE_LEFT               1
#ifdef _USE_DUAL_JOYSTICKS
#define JSPORT_DRIVE_RIGHT              2
#endif
#define JSPORT_SHOOTER                  3

//
// DriveBase Subsystem.
//
#ifdef _USE_NANOBOT
  #define WHEEL_DIAMETER                6
  #define DRIVE_ENCODER_PPR             250
#else
  #define WHEEL_DIAMETER                8
  #define DRIVE_ENCODER_PPR             360
#endif
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
#define DRIVE_TOLERANCE                 0.5
#define DRIVE_SETTLING                  200

//error in degrees
#define TURN_KP                         0.4     //0.017 //bigbot??  //0.05
#define TURN_KI                         0.0     //0.01  //??0.001
#define TURN_KD                         0.0     //0.03  //??0.5
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
#define TARGET_SELECTED                 -1
#define TARGET_TOP                      1
#define TARGET_BOTTOM                   2
#define TARGET_LEFT                     3
#define TARGET_RIGHT                    4

#ifdef _USE_NANOBOT
  #define CAMERA_HEIGHT                 21.5
#else
  #define CAMERA_HEIGHT                 30.8
#endif
#define TARGET_DISTANCE_CONSTANT        7205.0          //h1*d1
#define FOCAL_LENGTH                    392.61          //FocalLength=392.61
#define TARGET_KDH_LOW                  13659.1         //hl*d1=150.1*91
#define TARGET_KDH_MID                  939.6           //hm*d1=156.6*5
#define TARGET_KDH_HIGH                 -14406.6        //hh*d1=184.7*-78
#define TARGET_HEIGHT_LOW               (39.0 - CAMERA_HEIGHT)  //~8 in
#define TARGET_HEIGHT_MID               (72.0 - CAMERA_HEIGHT)  //~41 in
#define TARGET_HEIGHT_HIGH              (109.0 - CAMERA_HEIGHT) //~78 in
#define SCREEN_CENTER_X                 160
#define SCREEN_CENTER_Y                 120

#define MINIMUM_TARGET_ASPECT_RATIO     0.50
#define MAXIMUM_TARGET_ASPECT_RATIO     2.00

//
//
// Shooter Subsystem.
//
#define SHOOTER_ENCODER_PPR             360
#define SHOOTER_SYNC_GROUP              1

#define SHOOTER_KP                      0.000032    //0.000042 for big robot (000005)
                                        //0.000016  6. April... the day we started graphing stuff
#define SHOOTER_KI                      0.0
#define SHOOTER_KD                      0.000004   //0.05
#define SHOOTER_KF                      0.0
#define SHOOTER_TOLERANCE               1.0
#define SHOOTER_SETTLING                300

#define SHOOTER_ANGLE                   30.0        //??
#define SHOOTER_ANGLE_IN_RADIANS        (SHOOTER_ANGLE*PI/180.0)
#define G_INCHES_PER_SEC_SQUARE         (9.80665*39.370079)
#define SHOOTER_WHEEL_CIRCUMFERENCE     (8.0*PI)
#define TARGET_ANGLE_TOLERANCE          0.5         //in degrees
#define SHOOTER_SPEED_TOLERANCE         0.05        //percent
#define DEFAULT_ANGLE_ADJ               0.0
#define SHOOTER_ANGLE_DELTA             1.0         //in degrees
#define DEFAULT_SPEED_ADJ               0.99        //1.38
#define SHOOTER_SPEED_DELTA             0.01        //percent
#define SHOOTER_INITIAL_OUTPUT          0.85
#define AUTO_SHOOTER_SPEED              2650.0

//
// Wheelie Subsystem.
//
#define WHEELIE_DOWN                    SolID(SOL_WHEELIE_DOWN)
#define WHEELIE_UP                      SolID(SOL_WHEELIE_UP)

//
//
// Ballgate Subsystem.
//
#define BALLGATE_OPEN                   SolID(SOL_BALLGATE_OPEN)
#define BALLGATE_CLOSE                  SolID(SOL_BALLGATE_CLOSE)
#define BALLGATE_OPEN_PERIOD            0.4
#define BALLGATE_HOLD_TIME              0.5

//
// Pickup Subsystem.
//
#define CONVEYOR_SPEED                  -1.0
#define ROLLER_SPEED                    1.0

#endif  //ifndef _ROBOTINFO_H

