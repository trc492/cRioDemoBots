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
class TrcRobot: public CoopMTRobot,
                public ButtonNotify     //providing the Button handler
{
private:
    DashboardDataFormat m_dashboardDataFormat;
    Compressor          m_compressor;
    SolLight            m_whiteLight;
    //
    // Input Subsystem.
    //
    TrcJoystick         m_driveJoystickLeft;
#ifdef _USE_DUAL_JOYSTICKS
    TrcJoystick         m_driveJoystickRight;
#endif
    TrcJoystick         m_shooterJoystick;
    DigitalIn           m_digitalIn;
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
    VisionTarget        m_visionTarget;
    //
    // Shoot Subsystem.
    //
    Shooter             m_shooter;
    Event               m_shooterEvent;
    //
    // Shooter ballgate Subsystem.
    //
    TrcSol              m_ballgate;
    //
    // Pickup Subsystem.
    //
    Pickup              m_pickup;
    //
    // Wheelie Subsystem.
    //
    TrcSol              m_wheelie;
    //
    // Used by Autonomous.
    //
    StateMachine        m_autoSM;

    //
    // Miscellaneous.
    //
    bool                m_driveEnabled;
    bool                m_conveyorControlEnabled;
    bool                m_angleAdjEnabled;
    bool                m_hellModeEnabled;  //manual set power with no encoders
    bool                m_purgatoryModeEnabled;//manual set speed with encoders
    double              m_shooterPower;
    TrcTimer            m_timer;
    bool                m_slowDrive;

public:
    /**
     * Constructor for the TrcRobot class.
     * Create instances of all the components.
     */
    TrcRobot(
        void
        ): m_dashboardDataFormat()
         , m_compressor(DIN_CMP_PRESSURE_SWITCH, RELAY_COMPRESSOR)
         , m_whiteLight(SOL_WHITE_LIGHT)
         , m_driveJoystickLeft(JSPORT_DRIVE_LEFT, this)
#ifdef _USE_DUAL_JOYSTICKS
         , m_driveJoystickRight(JSPORT_DRIVE_RIGHT, this)
#endif
         , m_shooterJoystick(JSPORT_SHOOTER, this)
         , m_digitalIn()
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
         , m_visionTarget(&m_dashboardDataFormat)
         , m_shooter(&m_visionTarget, &m_driveBase, &m_pickup, &m_ballgate)
         , m_shooterEvent()
         , m_ballgate(SOL_BALLGATE_CLOSE, SOL_BALLGATE_OPEN, SOL_MODULE2)
         , m_pickup()
         , m_wheelie(SOL_WHEELIE_DOWN, SOL_WHEELIE_UP)
         , m_autoSM()
         , m_driveEnabled(true)
         , m_angleAdjEnabled(false)
         , m_hellModeEnabled(false)
         , m_purgatoryModeEnabled(false)
         , m_shooterPower(0.0)
         , m_timer()
         , m_slowDrive(false)
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
                                     &m_getSpeedPerfData);
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

};  //class TrcRobot

