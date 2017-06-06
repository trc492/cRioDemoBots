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

    m_visionLights.Set(Relay::kOn);

#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif

    m_autoSM.Start(SMSTATE_STARTED + 666);
    m_autoStartTime = GetMsecTime();

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

    m_visionLights.Set(Relay::kOff);

#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Stop();
#endif

    //
    // Do clean up before exiting Autonomous mode here.
    //
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

    int startingStep = 0;
    
    //
    // Process the state machine if it is ready.
    //
    if (m_autoSM.IsReady())
    {
        //
        //strategies:
        //
        UINT32 currState = m_autoSM.GetCurrentState();
        LCDPrintf((LCD_LINE2, "AutoState=%d", currState - SMSTATE_STARTED));
        
        //TInfo(("State: %d",  currState));
        //TInfo(("Speed: %10.5f", m_shooter.GetSpeed()))
        
        switch (currState)
        {
        //
        //1 - set the shooter to aim at the targets (top target)
        //    (don't feed another frisbee in)
        //2 - wait for shooter wheels to speed up (or time out after 5 seconds)
        //3 - Launch the frisbee
        //4 - Run the feeder
        //5 - cycle
        //
        case SMSTATE_STARTED + 666:
            
            m_autoStrategy = m_DSSwitches.GetBCDDigit(BCD_LOW_DIGIT_CHANNEL);
            switch (m_autoStrategy)
            {
            case AUTO_STRATEGY_CENTER:
                m_panTilt.SetTiltPower(TILT_SLOWDOWN);
                TInfo(("Selected auto strategy: Center!!!"));
                startingStep = SMSTATE_STARTED;
                break;
                
            case AUTO_STRATEGY_RIGHT_REAR:
            case AUTO_STRATEGY_LEFT_REAR:
                m_panTilt.SetTiltPower(TILT_SLOWDOWN);
            case AUTO_STRATEGY_RIGHT_FRONT:
            case AUTO_STRATEGY_LEFT_FRONT:
                TInfo(("Selected auto strategy: Either Left or Right"));
                startingStep = SMSTATE_STARTED + 1;
                break;
    
            case AUTO_STRATEGY_BLIND:
                TInfo(("Selected auto strategy: blind"));
                startingStep = SMSTATE_STARTED + 2;
                break;
    
            case AUTO_STRATEGY_NO_OP:
            default:  
                TInfo(("Selected auto strategy: none"))
                startingStep = 1000;
                break;
            }
            
            m_timer.SetTimer(0.5, &m_autoTimeoutEvent);
            m_autoSM.WaitForSingleEvent(&m_autoTimeoutEvent, startingStep);
            break;
        
        case SMSTATE_STARTED:
            //
            //move forward to get a view of the center target (dead reackoning)
            //set the robot to move forward (actual driving happens at end of
            // this function
            //
            m_autoDriveY = 0.60;
            m_autoDriveRot = 0.0;
            m_panTilt.SetTiltPower(0);

            m_autoShooterSpeed = MAX_VOLTAGE_HIGH;
            m_timer.SetTimer(1.0, &m_autoTimeoutEvent);
            m_autoSM.WaitForSingleEvent(&m_autoTimeoutEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 1:
            m_panTilt.SetTiltPower(0);
            //
            // AIM
            //
            switch (m_autoStrategy)
            {
            case AUTO_STRATEGY_RIGHT_FRONT:
            case AUTO_STRATEGY_RIGHT_REAR:
                m_visionTarget.SelectTarget(TARGET_MODE_CENTERMOST_TARGET);
                                        //TARGET_MODE_RIGHT_SIDE
                break;

            case AUTO_STRATEGY_LEFT_FRONT:
            case AUTO_STRATEGY_LEFT_REAR:
                m_visionTarget.SelectTarget(TARGET_MODE_LEFT_SIDE);
                break;

            case AUTO_STRATEGY_CENTER:
                m_visionTarget.SelectTarget(TARGET_MODE_CENTERMOST_TARGET);
                break;
            }
            m_autoShooterSpeed = MAX_VOLTAGE_HIGH;
            m_autoDriveY = 0.0;     //set the robot to stop (actual call happens at end of this function)
            m_autoDriveRot = 0.0;
            m_shooter.SetContinuousMode(true, &m_autoShooterEvent);
            m_autoSM.WaitForSingleEvent(&m_autoShooterEvent, currState + 1, 3000);
            break;

        case SMSTATE_STARTED + 2:// WAIT FOR WHEELS TO SPEED UP
            m_autoShooterSpeed = MAX_VOLTAGE_HIGH;
            if (GetMsecTime() >= m_autoStartTime + AUTO_MINIMUM_STARTUP_TIME &&
                    (m_shooter.GetSpeed() >= AUTO_SHOOTER_SPEED || 
                     GetMsecTime() >= m_autoPrevShootTime + AUTO_MAX_SHOOT_INTERVAL)
                )
            {
                m_autoPrevShootTime = GetMsecTime();
                m_autoSM.SetCurrentState(currState + 1);
            }
            break;

        case SMSTATE_STARTED + 3:// LAUNCH FRISBEE
            m_shooter.LaunchFrisbee (&m_autoLaunchEvent);
            m_autoSM.WaitForSingleEvent(&m_autoLaunchEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 4:// RUN FEEDER
            m_feeder.Start(&m_autoFeederEvent);
            m_autoSM.WaitForSingleEvent(&m_autoFeederEvent,
                                        currState + 1,
                                        2200);
            break;
        
        case SMSTATE_STARTED + 5:
            m_timer.SetTimer(0.75, &m_autoTimeoutEvent);
            m_autoSM.WaitForSingleEvent(&m_autoTimeoutEvent, currState - 3);
            break;

        default:
            m_autoSM.Stop();
            break;
        }
    }
    

    
    //constantly update drive motor output so we don't upset the watchdog
    m_driveBase.ArcadeDrive(m_autoDriveY, m_autoDriveRot);  //these vars are set by thate machine states
                                                            //they default to 0 and 0 (no movement)
    m_shooter.SetPower(m_autoShooterSpeed);

    TExit();
}   //AutonomousPeriodic
