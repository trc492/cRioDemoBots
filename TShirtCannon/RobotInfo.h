#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

/**
 *  This module contains the definitions of all implementation constants of
 *  the robot.
 */

//
// CAN IDs.
//
#define CANID_LEFTFRONT_JAG             2       //Orange
#define CANID_LEFTREAR_JAG              3       //Yellow
#define CANID_RIGHTFRONT_JAG            4       //Green
#define CANID_RIGHTREAR_JAG             5       //Blue

//
// PWM channels.
//
#define PWM_TILT_MOTOR                  3

//
// Analog input channels.
//
#define AIN_GYRO                        1
#define AIN_PRESSURE_GAUGE              2

//
// Digital input channels.
//

//
// Solenoid channels - module 1.
//
#define SOL_LEFT_CANNON                 ((UINT32)1)
#define SOL_MID_CANNON                  ((UINT32)2)
#define SOL_RIGHT_CANNON                ((UINT32)3)

//
// Solenoid channels - module 2.
//
#define SOL_LEFT_RED_LED                ((UINT32)1)
#define SOL_LEFT_GREEN_LED              ((UINT32)2)
#define SOL_LEFT_BLUE_LED               ((UINT32)3)
#define SOL_RIGHT_RED_LED               ((UINT32)4)
#define SOL_RIGHT_GREEN_LED             ((UINT32)5)
#define SOL_RIGHT_BLUE_LED              ((UINT32)6)

//
// Input subsystem.
//
#define JOYSTICK_LEFT_DRIVE             1
#define JOYSTICK_RIGHT_DRIVE            2
#define JOYSTICK_OPERATOR               3

//
// Drive subsystem.
//
#define MOTOR_LEFT_FRONT_REVERSE        false
#define MOTOR_LEFT_REAR_REVERSE         false
#define MOTOR_RIGHT_FRONT_REVERSE       true
#define MOTOR_RIGHT_REAR_REVERSE        true

#define DRIVE_ENCODER_PPR               360

//
// RGBLights subsystem.
//
#define NUM_COLORS                      8

#define LED_LEFT_RED                    ((UINT8)SolID(SOL_LEFT_RED_LED))
#define LED_LEFT_GREEN                  ((UINT8)SolID(SOL_LEFT_GREEN_LED))
#define LED_LEFT_BLUE                   ((UINT8)SolID(SOL_LEFT_BLUE_LED))
#define LED_LEFT_ALL                    ((UINT8)(LED_LEFT_RED |             \
                                                 LED_LEFT_GREEN |           \
                                                 LED_LEFT_BLUE))
#define LED_RIGHT_RED                   ((UINT8)SolID(SOL_RIGHT_RED_LED))
#define LED_RIGHT_GREEN                 ((UINT8)SolID(SOL_RIGHT_GREEN_LED))
#define LED_RIGHT_BLUE                  ((UINT8)SolID(SOL_RIGHT_BLUE_LED))
#define LED_RIGHT_ALL                   ((UINT8)(LED_RIGHT_RED |            \
                                                 LED_RIGHT_GREEN |          \
                                                 LED_RIGHT_BLUE))

#define LED_COLOR_BLACK                 0
#define LED_COLOR_RED                   ((UINT8)(LED_LEFT_RED |             \
                                                 LED_RIGHT_RED))
#define LED_COLOR_GREEN                 ((UINT8)(LED_LEFT_GREEN |           \
                                                 LED_RIGHT_GREEN))
#define LED_COLOR_BLUE                  ((UINT8)(LED_LEFT_BLUE |            \
                                                 LED_RIGHT_BLUE))
#define LED_COLOR_YELLOW                ((UINT8)(LED_COLOR_RED |            \
                                                 LED_COLOR_GREEN))
#define LED_COLOR_MAGENTA               ((UINT8)(LED_COLOR_RED |            \
                                                 LED_COLOR_BLUE))
#define LED_COLOR_CYAN                  ((UINT8)(LED_COLOR_GREEN |          \
                                                 LED_COLOR_BLUE))
#define LED_COLOR_WHITE                 ((UINT8)(LED_COLOR_RED |            \
                                                 LED_COLOR_GREEN |          \
                                                 LED_COLOR_BLUE))
#define LEDLeftColors(d)                ((UINT8)((d) & LED_LEFT_ALL))
#define LEDRightColors(d)               ((UINT8)((d) & LED_RIGHT_ALL))

//
// Shooter subsystem.
//
#define TRIGGER_TIME                    0.035
#define LONG_TRIGGER_TIME               0.070

#endif
