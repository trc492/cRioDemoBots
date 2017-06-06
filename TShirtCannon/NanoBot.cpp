#define PROGRAM_NAME            "TShirtCannon"

//#define _DBGTRACE_ENABLED
//#define TRACE_MODULES           (MOD_ALL)
//#define TRACE_LEVEL             HIFREQ
//#define MSG_LEVEL               VERBOSE

//
// Include libraries here.
//
#include "WPILib.h"
#include "RobotInfo.h"          //Robot configurations.
#include "..\frclib\TrcLib.h"

//
// Include subsystems here.
//

class NanoBot
    : public CoopMTRobot
    , public ButtonNotify
{
private:
    //
    // Input subsystem.
    //
    TrcJoystick         m_leftDriveStick;
    TrcJoystick         m_rightDriveStick;
    TrcJoystick         m_operatorStick;
    AnalogChannel       m_pressureGauge;
    //
    // Drive subsystem.
    //
    CanJag              m_leftFrontMotor;
    CanJag              m_leftRearMotor;
    CanJag              m_rightFrontMotor;
    CanJag              m_rightRearMotor;
    DriveBase           m_driveBase;
    //
    // Shooter subsystem.
    //
    TrcSol              m_leftCannon;
    TrcSol              m_midCannon;
    TrcSol              m_rightCannon;
    Victor              m_tiltMotor;
    bool                m_shooterEnabled;
    bool                m_shooterPowerHigh;
    //
    // RGBLights subsystem.
    //
    SolLight            m_rgbLeftLights;
    SolLight            m_rgbRightLights;
    UINT                m_colorIdx;
    bool                m_flashing;
    //
    // Miscellaneous
    //

public:
    /**
     *  Constructor.
     */
    NanoBot(void)
        : m_leftDriveStick(JOYSTICK_LEFT_DRIVE, this)
        , m_rightDriveStick(JOYSTICK_RIGHT_DRIVE, this)
        , m_operatorStick(JOYSTICK_OPERATOR, this)
        , m_pressureGauge(AIN_PRESSURE_GAUGE)
        , m_leftFrontMotor(CANID_LEFTFRONT_JAG)
        , m_leftRearMotor(CANID_LEFTREAR_JAG)
        , m_rightFrontMotor(CANID_RIGHTFRONT_JAG)
        , m_rightRearMotor(CANID_RIGHTREAR_JAG)
        , m_driveBase(&m_leftFrontMotor, &m_leftRearMotor,
                      &m_rightFrontMotor, &m_rightRearMotor)
        , m_leftCannon(SOL_LEFT_CANNON)
        , m_midCannon(SOL_MID_CANNON)
        , m_rightCannon(SOL_RIGHT_CANNON)
        , m_tiltMotor(PWM_TILT_MOTOR)
        , m_shooterEnabled(false)
        , m_shooterPowerHigh(false)
        , m_rgbLeftLights(SOL_LEFT_RED_LED,
                          SOL_LEFT_GREEN_LED,
                          SOL_LEFT_BLUE_LED,
                          2)
        , m_rgbRightLights(SOL_RIGHT_RED_LED,
                           SOL_RIGHT_GREEN_LED,
                           SOL_RIGHT_BLUE_LED,
                           2)
        , m_colorIdx(0)
        , m_flashing(false)
    {
        //
        // Initialize motor controllers.
        //
        m_leftFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_leftRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

        m_leftFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_leftRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);

        m_leftFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_leftRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);

        m_leftFrontMotor.SetSafetyEnabled(false);
        m_leftRearMotor.SetSafetyEnabled(false);
        m_rightFrontMotor.SetSafetyEnabled(false);
        m_rightRearMotor.SetSafetyEnabled(false);

        m_leftFrontMotor.SetExpiration(0.5);
        m_leftRearMotor.SetExpiration(0.5);
        m_rightFrontMotor.SetExpiration(0.5);
        m_rightRearMotor.SetExpiration(0.5);

        m_leftFrontMotor.EnableControl();
        m_leftRearMotor.EnableControl();
        m_rightFrontMotor.EnableControl();
        m_rightRearMotor.EnableControl();

        m_driveBase.SetInvertedMotor(RobotDrive::kFrontLeftMotor,
                                     MOTOR_LEFT_FRONT_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kRearLeftMotor,
                                     MOTOR_LEFT_REAR_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kFrontRightMotor,
                                     MOTOR_RIGHT_FRONT_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kRearRightMotor,
                                     MOTOR_RIGHT_REAR_REVERSE);
        m_driveBase.SetEncoderPolarities(MOTOR_LEFT_FRONT_REVERSE,
                                         MOTOR_LEFT_REAR_REVERSE,
                                         MOTOR_RIGHT_FRONT_REVERSE,
                                         MOTOR_RIGHT_REAR_REVERSE);

    }   //NanoBot

    /**
     *  Destructor.
     */
    ~NanoBot(void)
    {
    }   //~NanoBOt

    //
    // Functions for Disabled mode.
    //
    void DisabledStart(void)
    {
    }   //DisabledStart

    void DisabledStop(void)
    {
    }   //DisabledStop

    void DisabledPeriodic(void)
    {
    }   //DisabledPeriodic

    //
    // Functions for Autonomous mode.
    //
    void AutonomousStart(void)
    {
    }   //AutonomousStart

    void AutonomousStop(void)
    {
    }   //AutonomousStop

    void AutonomousPeriodic(void)
    {
    }   //AutonomousPeriodic

    //
    // Functions for TeleOp mode.
    //
    void TeleOpStart(void)
    {
    }   //TeleOpStart

    void TeleOpStop(void)
    {
    }   //TeleOpStop

    void TeleOpPeriodic(void)
    {
        //
        // Read pressure gauge.
        //
        LCDPrintf((LCD_LINE4, "PSI=%5.1f",
                   (m_pressureGauge.GetVoltage() - 0.5)*25.0));
        //
        // DriveBase operation.
        //
        float x = m_leftDriveStick.GetXWithDeadband(true);
        float y = m_rightDriveStick.GetYWithDeadband(true);
        float rot = m_rightDriveStick.GetZWithDeadband(true);
        LCDPrintf((LCD_LINE5, "x=%5.1f,y=%5.1f", x, y));
        LCDPrintf((LCD_LINE6, "rot=%5.1f", rot));

        m_driveBase.MecanumDrive_Cartesian(x, y, rot);

        //
        // Shooter tilter
        //
        float tiltPower = m_operatorStick.GetYWithDeadband(true);
        LCDPrintf((LCD_LINE4, "tiltPower=%5.1f", tiltPower));
        m_tiltMotor.Set(tiltPower);
    }   //TeleOpPeriodic
 
    void UpdateLEDLights(void)
    {
        static const UINT8 s_LEDColors[NUM_COLORS] =
        {
            LED_COLOR_BLACK,
            LED_COLOR_RED,
            LED_COLOR_GREEN,
            LED_COLOR_YELLOW,
            LED_COLOR_BLUE,
            LED_COLOR_MAGENTA,
            LED_COLOR_CYAN,
            LED_COLOR_WHITE
        };

        if (m_flashing)
        {
            m_rgbLeftLights.Set(0.5, 0.5,
                                LEDLeftColors(s_LEDColors[m_colorIdx]));
            m_rgbRightLights.Set(0.5, 0.5,
                                 LEDRightColors(s_LEDColors[m_colorIdx]));
        }
        else
        {
            m_rgbLeftLights.Set(true, LEDLeftColors(s_LEDColors[m_colorIdx]));
            m_rgbRightLights.Set(true, LEDRightColors(s_LEDColors[m_colorIdx]));
        }
    }   //UpdateLEDLights

    void NotifyButton(UINT32 port, UINT16 maskButton, bool fPressed)
    {
        if (port == JOYSTICK_LEFT_DRIVE)
        {
            switch (maskButton)
            {
                case Logitech_Trigger:
                    break;
            }
        }
        else if (port == JOYSTICK_RIGHT_DRIVE)
        {
            switch (maskButton)
            {
                case Logitech_Trigger:
                    break;
            }
        }
        else if (port == JOYSTICK_OPERATOR)
        {
            switch (maskButton)
            {
                case Logitech_Trigger:
                    break;

                case Logitech_Btn2:
                    break;

                case Logitech_Btn3:
                    if(fPressed && m_shooterEnabled)
                    {
                        m_midCannon.Set(SolID(SOL_MID_CANNON),
                                        m_shooterPowerHigh?
                                            LONG_TRIGGER_TIME:
                                            TRIGGER_TIME);
                    }
                    break;

                case Logitech_Btn4:
                    if(fPressed && m_shooterEnabled)
                    {
                        m_leftCannon.Set(SolID(SOL_LEFT_CANNON),
                                         m_shooterPowerHigh?
                                            LONG_TRIGGER_TIME:
                                            TRIGGER_TIME);
                    }
                    break;
                    
                case Logitech_Btn5:
                    if(fPressed && m_shooterEnabled)
                    {
                        m_rightCannon.Set(SolID(SOL_RIGHT_CANNON),
                                          m_shooterPowerHigh?
                                            LONG_TRIGGER_TIME:
                                            TRIGGER_TIME);
                    }
                    break;

                case Logitech_Btn6:
                    m_shooterPowerHigh = fPressed;
                    break;

                case Logitech_Btn7:
                    m_shooterEnabled = fPressed;
                    break;

                case Logitech_Btn8:
                    break;

                case Logitech_Btn9:
                    break;

                case Logitech_Btn10:
                    //
                    // Change RGB lights color.
                    //
                    if (fPressed)
                    {
                        m_colorIdx++;
                        if (m_colorIdx >= NUM_COLORS)
                        {
                            m_colorIdx = 0;
                        }
                        UpdateLEDLights();
                    }
                    break;

                case Logitech_Btn11:
                    //
                    // Toggle LED flashing.
                    //
                    if (fPressed)
                    {
                        m_flashing = !m_flashing;
                        UpdateLEDLights();
                    }
                    break;
            }
        }
    }   //NotifyButton

    //
    // Functions for Test mode.
    //
    void TestStart(void)
    {
    }   //TestStart

    void TestStop(void)
    {
    }   //TestStop

    void TestPeriodic(void)
    {
    }   //TestPeriodic

};  //class NanoBot

//
// Specifies main robot object.
//
START_ROBOT_CLASS(NanoBot);
