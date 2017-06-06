//
// Input Subsystems.
//
#define JSPORT_DRIVE_LEFT               1
#define JSPORT_DRIVE_RIGHT              2
#define JSPORT_SHOOTER                  3
//
// CAN IDs for Jaguars.
//
#define CANID_LEFTFRONT_JAG             2
#define CANID_LEFTREAR_JAG              3
#define CANID_RIGHTFRONT_JAG            4
#define CANID_RIGHTREAR_JAG             5
//
// Relay channels.
//
#define RELAY_RINGLIGHT_POWER           1
//
// PWM channels.
//
#define PWM_PAN_SERVO                   1
#define PWM_TILT_SERVO                  2
#define PWM_NERFGUN_TRIGGER             3
//
// Digital I/O channels.
//
#define DIO_JAMDOOR_CLOSED              1
#define DIO_CLIP_PRESENT                2
//
// DriveBase subsystem.
//
#define DRIVE_ENCODER_PPR               360
#define MOTOR_LEFT_FRONT_REVERSE        false
#define MOTOR_LEFT_REAR_REVERSE         false
#define MOTOR_RIGHT_FRONT_REVERSE       true
#define MOTOR_RIGHT_REAR_REVERSE        true
//
// PanTilt subsystem.
//
#define PAN_KP                          0.012
#define PAN_KI                          0.0
#define PAN_KD                          0.0
#define PAN_KF                          0.0
#define TILT_KP                         0.012
#define TILT_KI                         0.0
#define TILT_KD                         0.0
#define TILT_KF                         0.0
#define PANANGLE_GEAR_RATIO             2.0
#define TILTANGLE_GEAR_RATIO            2.0
#define MAX_PAN_ANGLE                   90.0
#define MAX_TILT_ANGLE                  90.0
#define INIT_PAN_ANGLE                  (float)(45.0*PANANGLE_GEAR_RATIO)
#define INIT_TILT_ANGLE                 (float)(45.0*TILTANGLE_GEAR_RATIO)
#define MAX_PAN_STEPRATE                (float)(90.0*PANANGLE_GEAR_RATIO)
#define MAX_TILT_STEPRATE               (float)(90.0*TILTANGLE_GEAR_RATIO)
#define PANANGLE_LOW_LIMIT              (float)(0.0*PANANGLE_GEAR_RATIO)
#define PANANGLE_HIGH_LIMIT             (float)(90.0*PANANGLE_GEAR_RATIO)
#define TILTANGLE_LOW_LIMIT             (float)(5.0*TILTANGLE_GEAR_RATIO)
#define TILTANGLE_HIGH_LIMIT            (float)(45.0*TILTANGLE_GEAR_RATIO)
//
// VisionTarget subsystem.
//
#define CAMERA_IP                       "10.4.92.11"
#define TARGET_DISTANCE_SCALE           9900.0
#define SCREEN_CENTER_X                 160
#define SCREEN_CENTER_Y                 120
#define FOCAL_LENGTH                    392.61  //????
#define DART_VX0                        100.0       //inches/s
//
// Shooter subsystem.
//
#define TRIGGER_TIME                    0.2     //in seconds
#define NERFGUN_TRIGGER_ON              1.0     //Trigger motor power
#define NERFGUN_TRIGGER_OFF             0.0
