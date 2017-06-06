#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="auto.h" />
///
/// <summary>
///     This main module contains the autonomous mode code.
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
 * This function is called before starting the autonomous mode.
 */
void
TrcRobot::AutonomousStart(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do other initializations for Autonomous mode here.
    //
    m_leftFrontMotor.SetSafetyEnabled(false);
    m_leftRearMotor.SetSafetyEnabled(false);
    m_rightFrontMotor.SetSafetyEnabled(false);
    m_rightRearMotor.SetSafetyEnabled(false);
    m_driveBase.SetSafetyEnabled(false);
    
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif

    m_autoSM.Start();

    TExit();
}   //AutonomousStart

/**
 * This function is called before exiting the autonmous mode.
 */
void
TrcRobot::AutonomousStop(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Stop autonomous mode.
    //
    m_autoSM.Stop();

#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Stop();
#endif

    //
    // Do clean up before exiting Autonomous mode here.
    //
    m_shooter.SetAlignEnabled(false);
    m_driveBase.SetSafetyEnabled(false);

    TExit();
}   //AutonomousStop

/**
 * This function is called periodically at fixed intervals in autonomous
 * mode.
 */
void
TrcRobot::AutonomousPeriodic(
    void
    )
{
    TLevel(HIFREQ);
    TEnter();

    //
    // Since we are not moving, feed the watchdog so it doesn't bark.
    //
    m_driveBase.ArcadeDrive(0.0, 0.0);

    //
    // Process the state machine if it is ready.
    //
    if (m_autoSM.IsReady())
    {
        UINT32 currState = m_autoSM.GetCurrentState();

        LCDPrintf((LCD_LINE2, "AutoState=%d", currState - SMSTATE_STARTED));
        switch (currState)
        {
        case SMSTATE_STARTED:
            //
            // Start shooter motor.
            //
            m_shooter.SetSpeed(AUTO_SHOOTER_SPEED);
            m_timer.SetTimer(1.5, &m_shooterEvent);
            m_autoSM.WaitForSingleEvent(&m_shooterEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 1:
            //
            // Start spinning up and wait for the speed to stablize.
            //
            m_timer.SetTimer(4.0, &m_shooterEvent);
            m_autoSM.WaitForSingleEvent(&m_shooterEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 2:
            //
            // Wait for the shooter to be at speed precisely.
            //
            if (fabs(AUTO_SHOOTER_SPEED - m_shooter.GetSpeed()) <
                SHOOTER_SPEED_TOLERANCE*AUTO_SHOOTER_SPEED)
            {
                m_autoSM.SetCurrentState(currState + 1);
            }
            break;

        case SMSTATE_STARTED + 3:
            //
            // Start the roller to move the other ball.
            //
            m_pickup.SetRollerSpeed(ROLLER_SPEED / 2.0);
            m_pickup.SetConveyorSpeed(CONVEYOR_SPEED / 2.0);
            //
            // Open ballgate to shoot one ball and loop back.
            //
            m_ballgate.Set(BALLGATE_OPEN, BALLGATE_OPEN_PERIOD,
                           BALLGATE_CLOSE, BALLGATE_HOLD_TIME,
                           0, &m_shooterEvent);
            m_autoSM.WaitForSingleEvent(&m_shooterEvent, SMSTATE_STARTED + 1);
            break;
            
        default:
            //
            // Blow the smoke out of the end of your gun.
            //
            m_autoSM.Stop();
            break;
        }
    }

    //
    // send the dashbaord data associated with the I/O ports
    //
    m_dashboardDataFormat.SendIOPortData();

    TExit();
}   //AutonomousPeriodic
