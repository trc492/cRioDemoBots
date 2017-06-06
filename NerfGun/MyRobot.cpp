#include "WPILib.h"
#include "..\frclib\TrcLib.h"

//#define _USE_TANKDRIVE
#define _USE_2STICKDRIVE

#include "RobotInfo.h"
#include "DashboardDataFormat.h"
#include "DriveBase.h"
#include "PanTilt.h"
#include "VisionTarget.h"
#include "Shooter.h"

class MyRobot
    : public CoopMTRobot
    , public ButtonNotify
{
private:
    //
    // Input subsystem.
    //
    TrcJoystick     m_leftStick;
    TrcJoystick     m_rightStick;
    TrcJoystick     m_shooterStick;
    //
    // DriveBase subsystem.
    //
    CanJag          m_leftFrontMotor;
    CanJag          m_leftRearMotor;
    CanJag          m_rightFrontMotor;
    CanJag          m_rightRearMotor;
    DriveBase       m_driveBase;
    //
    // PanTilt subsystem.
    //
    PanTilt         m_panTilt;
    //
    // VisionTarget subsystem.
    //
    VisionTarget    m_visionTarget;
    bool            m_fAutoTargetOn;
    //
    // Shooter subsystem.
    //
    Shooter         m_shooter;
    bool            m_fSingleShotOn;

public:
    MyRobot(void)
        : m_leftStick(JSPORT_DRIVE_LEFT, this)
        , m_rightStick(JSPORT_DRIVE_RIGHT, this)
        , m_shooterStick(JSPORT_SHOOTER, this)
        , m_leftFrontMotor(CANID_LEFTFRONT_JAG)
        , m_leftRearMotor(CANID_LEFTREAR_JAG)
        , m_rightFrontMotor(CANID_RIGHTFRONT_JAG)
        , m_rightRearMotor(CANID_RIGHTREAR_JAG)
        , m_driveBase(&m_leftFrontMotor, &m_leftRearMotor,
                      &m_rightFrontMotor, &m_rightRearMotor)
        , m_panTilt()
        , m_visionTarget(&m_panTilt)
        , m_fAutoTargetOn(false)
        , m_shooter()
        , m_fSingleShotOn(false)
    {
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
    }   //MyRobot

    ~MyRobot(void)
    {
    }   //~MyRobot

    void AutonomousStart(void)
    {
    }   //AutonomousStart

    void AutonomousStop(void)
    {
    }   //AutonomousStop

    void AutonomousPeriodic(void)
    {
    }   //AutonomousPeriodic

    void TeleOpStart(void)
    {
    }   //TeleOpStart

    void TeleOpStop(void)
    {
    }   //TeleOpStop

    void TeleOpPeriodic(void)
    {
        //
        // DriveBase subsystem.
        //
#ifdef _USE_TANKDRIVE
        float leftX = m_leftStick.GetXWithDeadband(true);
        float leftY = m_leftStick.GetYWithDeadband(true);
        float rightX = m_rightStick.GetXWithDeadband(true);
        float rightY = m_rightStick.GetYWithDeadband(true);

        float x = ((leftX < 0.0) && (rightX < 0.0) ||
                   (leftX > 0.0) && (rightX > 0.0))?
                           (leftX + rightX)/2.0: 0.0;
        float y = (leftY + rightY)/2.0;
        float rotation = (rightY - leftY)/2.0;
#else
  #ifdef _USE_2STICKDRIVE
        float x = m_leftStick.GetXWithDeadband(true);
  #else
        float x = m_rightStick.GetXWithDeadband(true);
  #endif
        float y = m_rightStick.GetYWithDeadband(true);
        float rotation = m_rightStick.GetZWithDeadband(true);
#endif

        m_driveBase.MecanumDrive_Cartesian(x, y, rotation);
        //
        // PanTilt subsystem.
        //
        if (!m_fAutoTargetOn)
        {
            float panPower = m_shooterStick.GetXWithDeadband(true);
            float tiltPower = m_shooterStick.GetYWithDeadband(true);

            m_panTilt.SetPanTiltPower(panPower, tiltPower);
        }
        //
        // Display status.
        //
        LCDPrintf((LCD_LINE3, "AutoTarget: %s", m_fAutoTargetOn? "ON": "OFF"));
        LCDPrintf((LCD_LINE4, "SingleShot: %s", m_fSingleShotOn? "ON": "OFF"));
    }   //TeleOpPeriodic

    void NotifyButton(
        UINT32 port,
        UINT16 bitButton,
        bool   fPressed
        )
    {
        if (port == JSPORT_DRIVE_LEFT)
        {
            switch (bitButton)
            {
                case Logitech_Trigger:
                    break;
            }
        }
        else if (port == JSPORT_DRIVE_RIGHT)
        {
            switch (bitButton)
            {
                case Logitech_Trigger:
                    break;
            }
        }
        else if (port == JSPORT_SHOOTER)
        {
            switch (bitButton)
            {
                case Logitech_Trigger:
                    if (m_fSingleShotOn)
                    {
                        if (fPressed)
                        {
                            m_shooter.FireOneShot();
                        }
                    }
                    else
                    {
                        m_shooter.SetNerfGunTrigger(fPressed);
                    }
                    break;

                case Logitech_Btn8:
                    if (fPressed)
                    {
                        //
                        // Release pan tilt control.
                        //
                        m_panTilt.Stop();
                        m_fAutoTargetOn = !m_fAutoTargetOn;
                        m_visionTarget.SetAutoTarget(m_fAutoTargetOn);
                    }
                    break;

                case Logitech_Btn9:
                    if (fPressed)
                    {
                        m_fSingleShotOn = !m_fSingleShotOn;
                    }
                    break;
            }
        }
    }   //NotifyButton
};  //class MyRobot

START_ROBOT_CLASS(MyRobot);
