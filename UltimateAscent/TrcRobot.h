#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcRobot.h" />
///
/// <summary>
///     This main module contains the definitions and implementation of the
///     TrcRobot class.
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
 * This class defines and implements the main robot object. It inherits the
 * CoopMTRobot object which is similar to the IterativeRobot class from the
 * WPI library. It also inherits the ButtonNotify interface so that it can
 * provide the notification callback for joystick button events.
 */
class TrcRobot:
    public CoopMTRobot,
    public ButtonNotify,        //providing the Joystick Button handler
    public EIODinNotify         //providing the Driver Station Button handler
{
private:
    DashboardDataFormat m_dashboardDataFormat;
    Compressor          m_compressor;
    //
    // Input Subsystem.
    //
    TrcJoystick         m_driveJoystickLeft;
#ifdef _USE_DUAL_JOYSTICKS
    TrcJoystick         m_driveJoystickRight;
#endif
    TrcJoystick         m_shooterJoystick;
    DigitalIn           m_digitalIn;
    DSEnhDin            m_DSSwitches;   //Driver station Enhanced Digital In
    //
    // DriveBase Subsystem.
    //
#ifdef _CANJAG_PERF
    PerfData            m_setMotorPerfData;
    PerfData            m_getPosPerfData;
    PerfData            m_getSpeedPerfData;
#endif
    CanJag              m_leftFrontMotor;
    CanJag              m_leftRearMotor;
    CanJag              m_rightFrontMotor;
    CanJag              m_rightRearMotor;
    DriveBase           m_driveBase;
    Event               m_driveEvent;
    //
    // VisionTarget Subsystem.
    //
    Relay               m_visionLights;
#ifdef _USE_VISION
    VisionTarget        m_visionTarget;
#endif
    //
    // Shooter Subsystem.
    //
    Shooter             m_shooter;
    PanTilt             m_panTilt;
    Feeder              m_feeder;
    bool                m_slowPanTilt;
    Event               m_shooterEvent;
    float               m_shooterMaxVoltage;
    float               m_shooterPower;
    //
    // Hanger Subsystem.
    //
    Hanger              m_hanger;
    bool                m_isHangerDeployed;
    //
    // Blower Subsystem.
    //
    Relay               m_blowerPower;
    //
    // Used by Autonomous.
    //
    StateMachine        m_autoSM;
    Event               m_autoShooterEvent;
    Event               m_autoFeederEvent;
    Event               m_autoLaunchEvent;
    Event               m_autoTimeoutEvent;
    UINT32              m_autoStartTime;
    UINT32              m_autoPrevShootTime;
    float               m_autoDriveY;
    float               m_autoDriveRot;
    float               m_autoShooterSpeed;
    UINT8               m_autoStrategy;
    
    //
    // Miscellaneous.
    //
    bool                m_driveEnabled;
    TrcTimer            m_timer;
    bool                m_slowDrive;
    int                 m_robotMode;

public:
    /**
     * Constructor for the TrcRobot class.
     * Create instances of all the components.
     */
    TrcRobot(
        void
        ): m_dashboardDataFormat()
         , m_compressor(DIN_CMP_PRESSURE_SWITCH, RELAY_COMPRESSOR)
         , m_driveJoystickLeft(JSPORT_DRIVE_LEFT, this)
#ifdef _USE_DUAL_JOYSTICKS
         , m_driveJoystickRight(JSPORT_DRIVE_RIGHT, this)
#endif
         , m_shooterJoystick(JSPORT_SHOOTER, this)
         , m_digitalIn()
         , m_DSSwitches()
#ifdef _CANJAG_PERF
         , m_setMotorPerfData()
         , m_getPosPerfData()
         , m_getSpeedPerfData()
#endif
         , m_leftFrontMotor(CANID_LEFTFRONT_JAG, CANJaguar::kPercentVbus)
         , m_leftRearMotor(CANID_LEFTREAR_JAG, CANJaguar::kPercentVbus)
         , m_rightFrontMotor(CANID_RIGHTFRONT_JAG, CANJaguar::kPercentVbus)
         , m_rightRearMotor(CANID_RIGHTREAR_JAG, CANJaguar::kPercentVbus)
         , m_driveBase(&m_leftFrontMotor, &m_leftRearMotor,
                       &m_rightFrontMotor, &m_rightRearMotor)
         , m_driveEvent()
         , m_visionLights(RELAY_RINGLIGHT_POWER, Relay::kForwardOnly)
#ifdef _USE_VISION
         , m_visionTarget(&m_dashboardDataFormat)
#endif
#ifdef _USE_VISION
         , m_shooter(&m_visionTarget, &m_panTilt, &m_dashboardDataFormat)
#else
         , m_shooter(&m_panTilt, &m_dashboardDataFormat)
#endif
         , m_panTilt()
         , m_feeder(&m_digitalIn)
         , m_slowPanTilt(false)
         , m_shooterEvent()
         , m_shooterMaxVoltage(MAX_VOLTAGE_LOW)
         , m_shooterPower(0.0)
         , m_hanger()
         , m_isHangerDeployed(false)
         , m_blowerPower(RELAY_BLOWER_POWER, Relay::kForwardOnly)
         , m_autoSM()
         , m_autoShooterEvent()
         , m_autoFeederEvent()
         , m_autoLaunchEvent()
         , m_autoTimeoutEvent()
         , m_autoStartTime(0)
         , m_autoPrevShootTime(0)
         , m_autoDriveY(0.0)
         , m_autoDriveRot(0.0)
         , m_autoShooterSpeed(0.0)
         , m_autoStrategy(AUTO_STRATEGY_CENTER)
         , m_driveEnabled(true)
         , m_timer()
         , m_slowDrive(false)
         , m_robotMode(SHOOTER_MODE_MANUAL)
    {
        TLevel(INIT);
        TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);
        TEnter();

        m_compressor.Start();
        //
        // Initialize motor controllers.
        //
#ifdef _CANJAG_PERF
        m_leftFrontMotor.SetPerfData(&m_setMotorPerfData,
                                     &m_getPosPerfData,
                                     &m_getSpeedPerfData);
        m_leftRearMotor.SetPerfData(&m_setMotorPerfData,
                                    &m_getPosPerfData,
                                    &m_getSpeedPerfData);
        m_rightFrontMotor.SetPerfData(&m_setMotorPerfData,
                                      &m_getPosPerfData,
                                      &m_getSpeedPerfData);
        m_rightRearMotor.SetPerfData(&m_setMotorPerfData,
                                     &m_getPosPerfData,
                                     &m_getSpeed PerfData);
#endif
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

#ifdef _CANJAG_PERF
        m_shooter.SetPerfData(&m_setMotorPerfData,
                              &m_getPosPerfData,
                              &m_getSpeedPerfData);
#endif
        //
        // Set task loop period to 100msec.
        //
        SetPeriod(0.1);

        TExit();
    }   //TrcRobot

    /**
     * Destructor for the TrcRobot class.
     * Destroy instances of components that were created in the constructor.
     */
    ~TrcRobot(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        m_compressor.Stop();

        TExit();
    }   //~TrcRobot

#ifdef _PERFDATA_LOOP
    /**
     * This function is called at the beginning of each robot loop.
     */
    void
    StartPerfDataLoop(
        void
        )
    {
        TLevel(FUNC);
        TEnter();

#ifdef _CANJAG_PERF
        m_setMotorPerfData.StartPerfPeriod();
        m_getPosPerfData.StartPerfPeriod();
        m_getSpeedPerfData.StartPerfPeriod();
#endif

        TExit();
        return;
    }   //StartPerfDataLoop

    /**
     * This function is called at the end of each robot loop.
     */
    void
    EndPerfDataLoop(
        void
        )
    {
        TLevel(FUNC);
        TEnter();

#ifdef _CANJAG_PERF
        m_setMotorPerfData.EndPerfPeriod();
        m_getPosPerfData.EndPerfPeriod();
        m_getSpeedPerfData.EndPerfPeriod();
        LCDPrintf((LCD_LINE1, "Set:%d<%d<%d",
                   m_setMotorPerfData.GetPerfMinTime(),
                   (m_setMotorPerfData.GetPerfTotalTime()/
                    m_setMotorPerfData.GetPerfCount()),
                   m_setMotorPerfData.GetPerfMaxTime()));
        LCDPrintf((LCD_LINE2, "Set:%d/%d",
                   m_setMotorPerfData.GetPerfCount(),
                   m_setMotorPerfData.GetPerfPeriodTime()));
        LCDPrintf((LCD_LINE3, "Pos:%d<%d<%d",
                   m_getPosPerfData.GetPerfMinTime(),
                   (m_getPosPerfData.GetPerfTotalTime()/
                    m_getPosPerfData.GetPerfCount()),
                   m_getPosPerfData.GetPerfMaxTime()));
        LCDPrintf((LCD_LINE4, "Pos:%d/%d",
                   m_getPosPerfData.GetPerfCount(),
                   m_getPosPerfData.GetPerfPeriodTime()));
        LCDPrintf((LCD_LINE5, "Speed:%d<%d<%d",
                   m_getSpeedPerfData.GetPerfMinTime(),
                   (m_getSpeedPerfData.GetPerfTotalTime()/
                    m_getSpeedPerfData.GetPerfCount()),
                   m_getSpeedPerfData.GetPerfMaxTime()));
        LCDPrintf((LCD_LINE6, "Speed:%d/%d",
                   m_getSpeedPerfData.GetPerfCount(),
                   m_getSpeedPerfData.GetPerfPeriodTime()));
#endif

        TExit();
        return;
    }   //EndJaguarPerfDataPeriod
#endif

    //
    // The following functions are in disabled.h
    //
    void
    DisabledStart(
        void
        );

    void
    DisabledStop(
        void
        );

    void
    DisabledPeriodic(
        void
        );

    //
    // The following functions are in auto.h
    //
    void
    AutonomousStart(
        void
        );

    void
    AutonomousStop(
        void
        );

    void
    AutonomousPeriodic(
        void
        );

    //
    // The following functions are in teleop.h
    //
    void
    TeleOpStart(
        void
        );

    void
    TeleOpStop(
        void
        );

    void
    TeleOpPeriodic(
        void
        );

    void
    NotifyButton(
        UINT32 port,
        UINT16 maskButton,
        bool   fPressed
        );

    void
    NotifyEIODin(
        UINT32 channel,
        UINT32 state
        );

};  //class TrcRobot

