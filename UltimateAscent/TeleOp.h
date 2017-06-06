#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="teleop.h" />
///
/// <summary>
///     This main module contains the teleoperated mode code.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

/**
 * This function is called before starting the teleop mode.
 */
void
TrcRobot::TeleOpStart(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do other initializations for TeleOp mode here.
    //
    m_visionLights.Set(Relay::kOn);
    if (m_DSSwitches.GetChannelState(DSSW_BLOWER_ON))
    {
        m_blowerPower.Set(Relay::kOn);
    }
    else
    {
        m_blowerPower.Set(Relay::kOff);
    }
    m_DSSwitches.RegisterNotification(this, 0xff00);

#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif

    TExit();
}   //TeleOpStart

/**
 * This function is called before exiting the teleop mode.
 */
void
TrcRobot::TeleOpStop(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do clean up before exiting TeleOp mode here.
    //
    m_DSSwitches.UnregisterNotification(this);
    m_visionLights.Set(Relay::kOff);
    m_blowerPower.Set(Relay::kOff);
    
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Stop();
#endif

    TExit();
}   //TeleOpStop

/**
 * This function is called periodically at fixed intervals in teleop mode.
 */
void
TrcRobot::TeleOpPeriodic(
    void
    )
{

    TLevel(TASK);
    TEnter();

    if(m_robotMode == SHOOTER_MODE_AUTO)
    {
        LCDPrintf((LCD_LINE3, "AUTOMATIC"));
    }
    else
    {
        LCDPrintf((LCD_LINE3, "MANUAL TARGETING"));
    }

    if(m_shooterMaxVoltage == MAX_VOLTAGE_LOW)
    {
        LCDPrintf((LCD_LINE6, "low voltage mode(%f)", MAX_VOLTAGE_LOW));
    }
    else
    {
        LCDPrintf((LCD_LINE6, "HIGH VOLTAGE MODE (%f)", MAX_VOLTAGE_HIGH));
    }

    //
    // Process input subsystems here.
    // (e.g. Reading joysticks, buttons, switches and sensors etc and
    //       computing the actions).
    //
    //if using manual mode, the joystick will be controlling the SPEED of pan/tilt
    if (m_robotMode == SHOOTER_MODE_MANUAL)//shooer in manual mode
    {
        float panPower = m_shooterJoystick.GetXWithDeadband() * PAN_SLOWDOWN;
        float tiltPower = -m_shooterJoystick.GetYWithDeadband() * TILT_SLOWDOWN;

        if(m_slowPanTilt)
        {
            panPower *= 0.5;
            //panPower = panPower * 0.5;
            tiltPower /= 3.0;
            //tiltPower = tiltPower / 2.0;
        }

        m_panTilt.SetPanPower(panPower);
        m_panTilt.SetTiltPower(tiltPower);
    }
//    m_shooter.SetPower(((-m_shooterJoystick.GetThrottle() + 1.0) / 2.0) * m_shooterMaxVoltage);
    m_shooter.SetPower(m_shooterPower);
    //
#ifdef _USE_TANK_DRIVE
  #ifdef _USE_MECANUM
    float leftX = m_driveJoystickLeft.GetXWithDeadband();
  #endif
    float leftY = -m_driveJoystickLeft.GetYWithDeadband();
  #ifdef _USE_DUAL_JOYSTICKS
   #ifdef _USE_MECANUM
    float rightX = m_driveJoystickRight.GetXWithDeadband();
   #endif
    float rightY = -m_driveJoystickRight.GetYWithDeadband();
  #else
   #ifdef _USE_MECANUM
    float rightX = m_driveJoystickLeft.GetZWithDeadband();
   #endif
    float rightY = m_driveJoystickLeft.GetTwistWithDeadband();
  #endif

    //
    // We only do crabbing if both the left and right X axes are the same
    // sign. So to prevent accidental crabbing during tank drive, one would
    // either toe-in or toe-out the left and right joysticks while tank
    // driving.
    //
  #ifdef _USE_MECANUM
    float x = ((leftX < 0.0) && (rightX < 0.0) ||
               (leftX > 0.0) && (rightX > 0.0))? (leftX + rightX)/2.0: 0.0;
  #endif
    float y = (leftY + rightY)/2.0;
    float rotation = (leftY - rightY)/2.0;
#else   // Arcade Drive
    //
    // We only rotate when we are not crabbing. If we are crabbing, we kept
    // the same heading by ignoring rotation.
    //
  #ifdef _USE_MECANUM
   #ifdef _USE_DUAL_JOYSTICKS
    float x = m_driveJoystickRight.GetXWithDeadband();
   #else
    float x = m_driveJoystickLeft.GetZWithDeadband();
   #endif
  #endif
    float rotation = (x == 0.0)? m_driveJoystickLeft.GetXWithDeadband(): 0.0;
    float y = -m_driveJoystickLeft.GetYWithDeadband();
#endif

    //
    // Perform output functions here.
    // (e.g. Programming motors, pneumatics and solenoids etc)
    //
    if (m_driveEnabled)
    {
        if(m_slowDrive)
        {
            y *= DRIVE_SLOWDOWN_Y;
            rotation *= DRIVE_SLOWDOWN_ROT;
        }
#ifdef _USE_MECANUM
        m_driveBase.MecanumDrive_Cartesian(x, y, rotation);
#else
        m_driveBase.ArcadeDrive(y, rotation);
#endif
    }
    else
    {
        m_driveBase.ArcadeDrive(0.0, 0.0);
    }
#ifdef _DEBUG_TELEOP
    LCDPrintf((LCD_LINE2, "D=%5.2f,T=%5.2f", y, rotation));
  #ifdef _USE_MECANUM
    LCDPrintf((LCD_LINE3, "Crab=%5.2f", x));
  #endif
    LCDPrintf((LCD_LINE4, "LF=%5.2f,RF=%5.2f",
               m_leftFrontMotor.GetPosition(),
               m_rightFrontMotor.GetPosition()));
    LCDPrintf((LCD_LINE5, "LR=%5.2f,RR=%5.2f",
               m_leftRearMotor.GetPosition(),
               m_rightRearMotor.GetPosition()));
#endif

    TExit();
}   //TeleopPeriodic

/**
 * This function is called by the TrcJoystick object when a button event
 * occurred.
 *
 * @param port Specifies the joystick port.
 * @param maskButton Specifies the button mask.
 * @param fPressed If true, specifies the button is pressed, false if
 *        released.
 */
void
TrcRobot::NotifyButton(
    UINT32 port,
    UINT16 maskButton,
    bool   fPressed
    )
{
    TLevel(EVENT);
    TEnterMsg(("port=%d,Button=0x%x,fPressed=%d",
               port, maskButton, fPressed));

    if (port == JSPORT_DRIVE_LEFT)
    {
        switch (maskButton)
        {
        case Logitech_Btn2:
            if (fPressed)
            {
                m_driveBase.ResetPosition();
            }
            break;
        }
    }
#ifdef _USE_DUAL_JOYSTICKS
    else if (port == JSPORT_DRIVE_RIGHT)
    {
        switch (maskButton)
        {
        case Logitech_Trigger:
            m_slowDrive = fPressed;
            break;

        case Logitech_Btn2:	//next 3 for pid tuning
                m_robotMode = SHOOTER_MODE_AUTO;
                m_panTilt.SetPanTiltAngle(10.0, 10.0);
            break;

        case Logitech_Btn3:
                m_robotMode = SHOOTER_MODE_AUTO;
                m_panTilt.SetPanTiltAngle(-20.0, 4.0);
            break;

        case Logitech_Btn4:
                m_robotMode = SHOOTER_MODE_AUTO;
                m_panTilt.SetPanTiltAngle(-10.0, -4.0);
            break;

#ifdef _TUNE_TURN_PID
        case Logitech_Btn5:
            m_driveEnabled = !fPressed;
            if (fPressed)
            {
                m_driveBase.DriveSetTarget(0.0, 45.0);
            }
            else
            {
                m_driveBase.Stop();
            }
#endif
        }
    }
#endif  //_USE_DUAL_JOYSTICKS
    else if (port == JSPORT_SHOOTER)
    {
        switch (maskButton)
        {
        case Logitech_Btn2:
            //
            // Slow down pan and tilt control.
            //
            m_slowPanTilt = fPressed;
            break;

        case Logitech_Btn3:
            //
            // Select top goal as target.
            //
            if (fPressed)
            {
                m_visionTarget.SelectTarget(TARGET_MODE_TOP);
            }
            break;

        case Logitech_Btn4:
            //
            // Select left mid goal as target.
            //
            if (fPressed)
            {
                m_visionTarget.SelectTarget(TARGET_MODE_LEFT_SIDE);
            }
            break;

        case Logitech_Btn5:
            //
            // Select right mid goal as target.
            //
            if (fPressed)
            {
                m_visionTarget.SelectTarget(TARGET_MODE_RIGHT_SIDE);
            }
            break;

        case Logitech_Btn6:
            //
            // Deploy/undeploy hanger.
            //
            if (fPressed)
            {
                m_isHangerDeployed = !m_isHangerDeployed;
                if (m_isHangerDeployed)
                {
                    //launch hanger
                    //hanger mechanism unknown right now, so we'll deal with that later
                    m_hanger.Extend();
                    //this might have to be a toggle
                }
                else
                {
                    m_hanger.Retract();
                }
            }
            break;

        case Logitech_Btn7:
            //
            // Cram a frisbee.
            //
            if (fPressed)
            {
                m_feeder.Cram();
            }
            break;

        case Logitech_Btn10:
            //
            // Feed a frisbee.
            //
            if (fPressed)
            {
                m_feeder.Start();
            }
            break;

        case Logitech_Btn11:
            //
            // Reverse the feeder.
            //
            if (fPressed)
            {
                m_feeder.Reverse();
            }
            else
            {
                m_feeder.Quit();
            }
            break;
        }
    }

    TExit();
}	//NotifyButton

/**
 * This function is called to handle a Driver Station Enhanced IO Digital
 * Input event.
 *
 * @param channel Specifies the Enhanced IO Digital Input channel (0-15)
 *        that generated the event.
 * @param state Specifies the state of the digital input channel (0 or 1).
 */
void
TrcRobot::NotifyEIODin(
    UINT32 channel,
    UINT32 state
    )
{
    TLevel(EVENT);
    TEnterMsg(("channel=%d,state=%d", channel, state));

    switch (channel)
    {
    case DSSW_SHOOT:
        //
        // Launch a frisbee.
        //
        if (state == 1)
        {
            m_shooter.LaunchFrisbee();
            m_driveEnabled = false;
        }
        else
        {
            m_driveEnabled = true;
        }
        break;
    
    case DSSW_AIM_HIGHER:
        //
        // Tilt the shooter higher.
        //
        if (state == 1)
        {
            //takes whatever the adjustment factor already was and increments
            //it up
            double newAngle = m_shooter.GetAngleAdj() +
                              V_ANGLE_ADJUSTMENT_INCREMENT;
            m_shooter.SetAngleAdj(newAngle);
        }
        break;
    
    case DSSW_AIM_LOWER:
        //
        // Tilt the shooter lower.
        //
        if (state == 1)
        {
            //takes whatever the adjustment factor already was and increments
            //it down
            double newAngle = m_shooter.GetAngleAdj() -
                              V_ANGLE_ADJUSTMENT_INCREMENT;
            m_shooter.SetAngleAdj(newAngle);
        }
        break;
    
    case DSSW_REMEMBER_V_ANGLE: 
        //
        // Memorize the current camera angle as the tilt angle adjustment.
        //
        if (state == 1)
        {
            //sets the adjustment factor to what the camera reads as the
            //current offset
            double newAngle = m_shooter.GetVAngle();
            m_shooter.SetAngleAdj(newAngle);
        }
        break;

    case DSSW_SHOOTER_ON:
        //
        // Turn shooter ON/OFF.
        //
        if (state == 0)
        {
            m_shooterPower = m_shooterMaxVoltage;
        }
        else
        {
            m_shooterPower = 0;
        }
        break;

    case DSSW_AUTOMODE:
        //
        // Turn on/off auto target mode.
        //
        if (state == 0)
        {
            m_robotMode = SHOOTER_MODE_AUTO;
            m_shooter.SetContinuousMode(true);
            LCDPrintf((LCD_LINE3, "AUTOMATIC"));
        }
        else
        {
            m_robotMode = SHOOTER_MODE_MANUAL;
            m_shooter.SetContinuousMode(false);
            LCDPrintf((LCD_LINE3, "MANUAL TARGETING"));
        }
        break;

    case DSSW_SHOOTER_VOLTAGE:
        //
        // Set shooter voltage.
        //
        if (state == 0)
        {
            m_shooterMaxVoltage = MAX_VOLTAGE_HIGH;
            if(m_shooterPower != 0)
            {
                m_shooterPower = m_shooterMaxVoltage;
            }
            LCDPrintf((LCD_LINE6, "HIGH VOLTAGE MODE (%f)", MAX_VOLTAGE_HIGH));
        }
        else
        {
            m_shooterMaxVoltage = MAX_VOLTAGE_LOW;
            if(m_shooterPower != 0)
            {
                m_shooterPower = m_shooterMaxVoltage;
            }
            LCDPrintf((LCD_LINE6, "low voltage mode(%f)", MAX_VOLTAGE_LOW));
        }
        break;

    case DSSW_BLOWER_ON:
        //
        // Tunr on/off deflector blower.
        //
        if (state == 0)
        {
            m_blowerPower.Set(Relay::kOn);
            TInfo(("Blower motor on"));
        }
        else
        {
            m_blowerPower.Set(Relay::kOff);
            TInfo(("Blower motor off"));
        }
        break;
    }

    TExit();
}   //NotifyEIODin
