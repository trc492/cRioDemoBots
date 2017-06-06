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
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif
    m_shooter.SetContinuousMode(true);

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
	
    TLevel(HIFREQ);
    TEnter();

    //
    // Process input subsystems here.
    // (e.g. Reading joysticks, buttons, switches and sensors etc and
    //       computing the actions).
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

    if(m_conveyorControlEnabled)
    {
        m_pickup.SetConveyorSpeed(m_shooterJoystick.GetYWithDeadband());
    }

    if (m_hellModeEnabled)
    {
        //
        // This is to feed the shooter watchdog so it doesn't timeout.
        //
        m_shooter.SetPower(m_shooterPower);
    }

    //
    // send the dashboard data associated with the I/O ports
    //
    m_dashboardDataFormat.SendIOPortData();

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

    static bool fWheelieDown = false;
    static bool fBallgateClose = false;

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
        
        case Logitech_Btn3:
            m_slowDrive = fPressed;
	        break;

#ifdef _TUNE_SHOOTER_PID
        case Logitech_Btn5:
            if (fPressed)
            {
                m_shooter.SetPID(m_shooter.GetKp() - 0.000002,
                                 m_shooter.GetKi(),
                                 m_shooter.GetKd());
                LCDPrintf((LCD_LINE3, "Kp=%10.9f", m_shooter.GetKp()));
            }
            break;

        case Logitech_Btn6:
            if (fPressed)
            {
                m_shooter.SetPID(m_shooter.GetKp() + 0.000002,
                                 m_shooter.GetKi(),
                                 m_shooter.GetKd());
                LCDPrintf((LCD_LINE3, "Kp=%10.9f", m_shooter.GetKp()));
            }
            break;
#endif
        }
    }
#ifdef _USE_DUAL_JOYSTICKS
    else if (port == JSPORT_DRIVE_RIGHT)
    {
        switch (maskButton)
        {
        case Logitech_Trigger:
            m_driveEnabled = !fPressed;
            m_shooter.SetAlignEnabled(fPressed);
            break;
            
        case Logitech_Btn2:
            if (fPressed)
            {
                //intake
                m_pickup.SetRollerSpeed(ROLLER_SPEED);
                m_pickup.SetConveyorSpeed(CONVEYOR_SPEED);
            }
            else
            {
                //stop
                m_pickup.SetRollerSpeed(0.0);
                m_pickup.SetConveyorSpeed(0.0);
            }
            break;
        
        case Logitech_Btn3:
            if (fPressed)
            {
                //vomit
                m_pickup.SetConveyorSpeed(-CONVEYOR_SPEED / 2.0);
                m_pickup.SetRollerSpeed(-ROLLER_SPEED / 2.0);
            }
            else
            {
                //stop
                m_pickup.SetRollerSpeed(0.0);
                m_pickup.SetConveyorSpeed(0.0);
            }
            break;
            
        case Logitech_Btn4:
            if (fPressed)
            {
                fWheelieDown = !fWheelieDown;
                if (fWheelieDown)
                {
                    m_wheelie.Set(WHEELIE_DOWN, WHEELIE_UP);
                }
                else
                {
                    m_wheelie.Set(WHEELIE_UP, WHEELIE_DOWN);
                }
            }
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
        case Logitech_Trigger:
            if (fPressed)
            {
                //triger => Align Robot THEN shoot ball
                //align
                //start rolling converyor to deliver ball to shooter
                //shoot
                if (m_hellModeEnabled)
                {
                    m_shooter.StopPID();
                    m_shooterPower = -NORMALIZE(-m_shooterJoystick.GetTwist(),
                                                -1.0, 1.0, 0.2, 1.0); 
                    m_shooter.SetPower(m_shooterPower);
                    LCDPrintf((LCD_LINE2, "ShooterPower=%5.2f",
                               m_shooterPower));
                }
                else if (m_purgatoryModeEnabled)
                {
                    double shooterSpeed = NORMALIZE(
                                                -m_shooterJoystick.GetTwist(),
                                                -1.0, 1.0, 1000.0, 3000.0); 
                    m_shooter.SetSpeed(shooterSpeed);
                    LCDPrintf((LCD_LINE2, "ShooterSpeed=%5.2f", shooterSpeed));
                }
                else
                {
                    //m_shooter.TurnToTarget();
                    //really shouldn't do anything
                }
            }
            break;

        case Logitech_Btn2:
            if (fPressed)
            {
                fBallgateClose = !fBallgateClose;
                m_ballgate.Set(BALLGATE_OPEN, BALLGATE_OPEN_PERIOD,
                               BALLGATE_CLOSE, BALLGATE_HOLD_TIME);
            }
            break;

        case Logitech_Btn3:
            if (fPressed)
            {
                m_shooter.SelectTarget(TARGET_LEFT);
            }
            break;

        case Logitech_Btn4:
            if (fPressed)
            {
                m_shooter.SelectTarget(TARGET_BOTTOM);
            }
            break;

        case Logitech_Btn5:
            if (fPressed)
            {
                m_shooter.SelectTarget(TARGET_TOP);
            }
            break;

        case Logitech_Btn6:
            if (fPressed)
            {
                m_shooter.SelectTarget(TARGET_RIGHT);
            }
            break;

        case Logitech_Btn7:
            //hold down button 7 to enable angle adjustment
            if(fPressed)
            {
                m_angleAdjEnabled = true;
            }
            else
            {
                m_angleAdjEnabled = false;
            }
            break;

        case Logitech_Btn8:
            //toggle hell mode
            if (fPressed)
            {
                m_hellModeEnabled = !m_hellModeEnabled;
                m_purgatoryModeEnabled = false;
                if (m_hellModeEnabled)
                {
                    m_whiteLight.Set(0.5, 0.5, SolID(SOL_WHITE_LIGHT));
                    m_shooter.StopPID();
                    m_shooter.SetContinuousMode(false);
                }
                else
                {
                    m_whiteLight.Set(false, SolID(SOL_WHITE_LIGHT));
                    m_shooter.SetContinuousMode(true);
                }
            }
            break;

        case Logitech_Btn9:
            if(fPressed)
            {
                if (m_angleAdjEnabled)
                {
                    //change angle ro the right
                    m_shooter.AddAngleAdj(SHOOTER_ANGLE_DELTA);
                }
                else
                {
                    //increase shooterRpmAdj
                    m_shooter.AddSpeedAdj(SHOOTER_SPEED_DELTA);
                
                }
            }
            break;

        case Logitech_Btn10:    //toggle purgatory mode
            if (fPressed)
            {
                m_purgatoryModeEnabled = !m_purgatoryModeEnabled;
                m_hellModeEnabled = false;
                if (m_purgatoryModeEnabled)
                {
                    m_whiteLight.Set(true, SolID(SOL_WHITE_LIGHT));
                    m_shooter.StopPID();
                    m_shooter.SetContinuousMode(false);
                }
                else
                {
                    m_whiteLight.Set(false, SolID(SOL_WHITE_LIGHT));
                    m_shooter.SetContinuousMode(true);
                }
            }
            break;

        case Logitech_Btn11:
            if(fPressed)
            {
                if (m_angleAdjEnabled)
                {
                    //change angle to the left
                    m_shooter.AddAngleAdj(-SHOOTER_ANGLE_DELTA);
                    
                }
                else
                {
                    //decrease shooterRpmAdj
                    m_shooter.AddSpeedAdj(-SHOOTER_SPEED_DELTA);
                    
                }
            }
            break;

        case Logitech_Btn12:
            //hold down button 12 to control the conveyor
            m_conveyorControlEnabled = fPressed;
            if(!fPressed)
            {
                m_pickup.SetConveyorSpeed(0.0);
            }
            break;
        }
        
        if (m_hellModeEnabled)
        {
            LCDPrintf((LCD_LINE5, "HELL control mode"));
        }
        else if(m_purgatoryModeEnabled)
        {
            LCDPrintf((LCD_LINE5, "PURGATORY control mode"));
        }
        else    //neither hell nor purgatory mode
        {
            LCDPrintf((LCD_LINE5, "STANDARD control mode"));
        }
    }

    TExit();
}	//NotifyButton
