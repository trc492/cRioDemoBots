#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Shooter.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Shooter class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _SHOOTER_H
#define _SHOOTER_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_SHOOTER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Shooter"

#define VARID_PID               (VARID_DATAPTR + 1)
#define VARID_KP                (VARID_DATAPTR + 2)
#define VARID_KI                (VARID_DATAPTR + 3)
#define VARID_KD                (VARID_DATAPTR + 4)
#define VARID_KF                (VARID_DATAPTR + 5)

#define GUIDE_LIGHT_PAN_LEFT    SolID(SOL_PANLIGHT_LEFT)
#define GUIDE_LIGHT_PAN_RIGHT   SolID(SOL_PANLIGHT_RIGHT)
#define GUIDE_LIGHT_PAN_ALL     ((UINT8)(GUIDE_LIGHT_PAN_LEFT |     \
                                         GUIDE_LIGHT_PAN_RIGHT))

#define GUIDE_LIGHT_TILT_LOW    SolID(SOL_TILTLIGHT_LOW)
#define GUIDE_LIGHT_TILT_HIGH   SolID(SOL_TILTLIGHT_HIGH)
#define GUIDE_LIGHT_TILT_ALL    ((UINT8)(GUIDE_LIGHT_TILT_LOW |     \
                                         GUIDE_LIGHT_TILT_HIGH))

#define SPEED_LIGHT_ALL         ((UINT8)(GUIDE_LIGHT_PAN_ALL |      \
                                         GUIDE_LIGHT_TILT_ALL))

/**
 * This module defines and implements the Shooter subsytem. The Shooter
 * subsystem consists of two shooter motors and the pan and tilt motors.
 */

class Shooter: public CoopTask,
               public PIDInput,
               public CmdHandler
{
private:
    //
    // Dependent subsystems.
    //
    DashboardDataFormat      *m_dashboardDataFormat;
    
#ifdef _USE_VISION
    VisionTarget   *m_visionTarget;
#endif
    Event          *m_notifyEvent;
    PanTilt        *m_panTilt;
    //
    // Shooter components.
    //
    CanJag          m_frontMotor;
    CanJag          m_rearMotor;
    TrcPIDCtrl      m_pidCtrl;
    TrcPIDMotor     m_pidMotor;
    TrcSol          m_launcher;
    //
    // Status lights.
    //
    SolLight        m_PanLights;
    SolLight		m_TiltLights;
    //
    // Shooter states.
    //
    Event           m_panTiltEvent;
    TARGETINFO      m_targetInfo;
    double          m_setSpeed;
    double          m_currSpeed;
    double          m_rawSpeed;
    bool            m_fContinuousMode;
    bool            m_speedWithinTolerance;
    double          m_vAngleAdj;

    IIRFilter       m_speedFilter;
    
    bool			m_targetAcquired;

    static UINT32   m_speedLightChannels[4];
    static SOL_STATE m_speedLightStates[2];
    SolLight        m_speedLights;

public:
    static VAR_ENTRY m_varTable[];

    /**
     * This function stops the shooter.
     *
     * @param fStopShooter Stop the shooter motor only if true.
     */
    void
    Stop()
    {
        TLevel(API);
        TEnter();

        m_fContinuousMode = false;

        TExit();
        return;
    }   //Stop

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Shooter(
#ifdef _USE_VISION
        VisionTarget        *visionTarget,
#endif
        PanTilt             *panTilt,
        DashboardDataFormat *dash
        ): m_dashboardDataFormat(dash)
#ifdef _USE_VISION
         , m_visionTarget(visionTarget)
#endif
         , m_notifyEvent(NULL)
         , m_panTilt(panTilt)
         , m_frontMotor(CANID_FRONTSHOOTER_JAG, CANJaguar::kVoltage)
         , m_rearMotor(CANID_REARSHOOTER_JAG, CANJaguar::kVoltage)
         , m_pidCtrl("Shooter",
                     SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF,
                     SHOOTER_TOLERANCE, SHOOTER_SETTLING,
                     PIDCTRLO_ABS_SETPT | PIDCTRLO_SPEED_CTRL)
         , m_pidMotor(&m_frontMotor, &m_rearMotor,
                      SHOOTER_SYNC_GROUP,
                      &m_pidCtrl,
                      this,
                      PIDMOTORO_INVERSE)
         , m_launcher(SOL_DEPLOYER_RETRACT, SOL_DEPLOYER_EXTEND)
         , m_PanLights(SOL_PANLIGHT_LEFT,
                         SOL_PANLIGHT_RIGHT)
    	 , m_TiltLights(SOL_TILTLIGHT_LOW,
                         SOL_TILTLIGHT_HIGH)
         , m_setSpeed(0.0)
         , m_currSpeed(0.0)
         , m_fContinuousMode(false)
         , m_speedWithinTolerance(false)
         , m_vAngleAdj(-3.5)
         , m_speedFilter()
    	 , m_targetAcquired(false)
         , m_speedLights(m_speedLightChannels,
                         ARRAYSIZE(m_speedLightChannels))
    {
        TLevel(INIT);
        TEnter();
        
        //
        // Initialize motor controllers.
        //
        m_frontMotor.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
        m_frontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_frontMotor.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
        m_frontMotor.ConfigEncoderCodesPerRev(SHOOTER_ENCODER_PPR);
        m_rearMotor.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
        m_frontMotor.SetSafetyEnabled(false);
        m_rearMotor.SetSafetyEnabled(false);

#ifdef _LOGDATA_SHOOTER
        DataLogger *dataLogger = DataLogger::GetInstance();
        /*dataLogger->AddDataPoint(MOD_NAME, "", "TargetID", "%d",
                                 DataInt8, &m_targetID);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetDistance", "%5.2f",
                                 DataFloat, &m_targetInfo.distance);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetHeight", "%5.2f",
                                 DataFloat, &m_targetInfo.height);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetAngle", "%5.2f",
                                 DataFloat, &m_targetInfo.angle);*/
        dataLogger->AddDataPoint(MOD_NAME, "", "Kp", "%10.9f",
                                 DataDouble, &m_Kp);
        dataLogger->AddDataPoint(MOD_NAME, "", "Ki", "%10.9f",
                                 DataDouble, &m_Ki);
        dataLogger->AddDataPoint(MOD_NAME, "", "Kd", "%10.9f",
                                 DataDouble, &m_Kd);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetSpeed", "%5.2f",
                                 DataDouble, &m_setSpeed);
        dataLogger->AddDataPoint(MOD_NAME, "", "CurrSpeed", "%5.2f",
                                 DataDouble, &m_currSpeed);
        dataLogger->AddDataPoint(MOD_NAME, "", "RawSpeed", "%5.2f",
                                 DataDouble, &m_rawSpeed);
#endif
        RegisterTask(MOD_NAME,
                     TASK_START_MODE | TASK_STOP_MODE | TASK_POST_PERIODIC);
        RegisterCmdHandler(MOD_NAME, NULL, m_varTable);

        TExit();
    }   //Shooter

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Shooter(
        void
        )
    {
        TLevel( INIT);
        TEnter();

        Stop();
        UnregisterTask();

        TExit();
    }   //~Shooter
    
    /**
     * This function is called by the TaskMgr to start the task.
     *
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStartMode(
        UINT32 mode
        
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            m_frontMotor.EnableControl();
            m_rearMotor.EnableControl();
            //m_pidMotor.SetPower(-1.0);
        }

        TExit();
    }   //TaskStartMode
    
    void SetPower(
        float power
        )
    {
        // Shooter motor is in Voltage mode, so the SetPower unit is voltage.
        m_pidMotor.SetPower(power, -12.0, 12.0);
        LCDPrintf((LCD_LINE2, "power=%5.2f", power));
    }
    
    void SetSpeed(
        float speed
        )
    {
        m_pidMotor.SetTarget(speed);
    }

    /**
     * This function is called by the TaskMgr to stop the task.
     * 
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            Stop();
            m_frontMotor.DisableControl();
            m_rearMotor.DisableControl();
        }

        TExit();
    }   //TaskStopMode

    /**
     * This function enables or disables Continuous mode.
     *
     * @param fEnable If true, enables continuous mode, false otherwise.
     */
    void
    SetContinuousMode(
        bool fEnable,
        Event *notifyEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("fEnable=%d", fEnable));

        m_fContinuousMode = fEnable;
        m_notifyEvent = notifyEvent;

        TExit();
        return;
    }   //SetContinuousMode

    /**
     * This function gets the shooter speed.
     *
     * @return Returns the shooter speed in RPM.
     */
    float
    GetSpeed(
        void
        )
    {
        float speed;

        TLevel(HIFREQ);
        TEnter();

        m_rawSpeed = -m_frontMotor.GetAverageSpeed();
        //speed = m_speedFilter.FilterData(m_rawSpeed, 0.2);
        speed = m_rawSpeed;

        TExitMsg(("=%f", speed));
        return speed;
    }   //GetSpeed

    /**
     * This function is called from the PID controllers to get the PID input
     * value.
     *
     * @param pidCtrl Specifies the PID controller that requires the input
     *        value.
     */
    float
    GetInput(
        TrcPIDCtrl *pidCtrl
        )
    {
        float input;

        TLevel(HIFREQ);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        input = GetSpeed();

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput
    
    /**
     * Gets the verticle angle offset of the target with respect to 
     * the center of the screen (the up-down aiming error) 
     */
    double
    GetVAngle(
        void
        )
    {
        return -m_targetInfo.vAngle;
    }
    
    /**
     * Gets the adjustment factor (which is in degrees) for the vertical targeting angle
     */
    double
    GetAngleAdj(
         void)
    {
        return m_vAngleAdj;
    }
    
    void
    SetAngleAdj(
         double newAngleAdj)
    {
        m_vAngleAdj = newAngleAdj;
    }

    /**
     * This function will use the target angle readings from the vision
     * module, compare it with the current heading from the gyro, and 
     * update the status lights based on the error.
     * IT also will blink if you're too close or too far from the targets. 
     */
    void
    UpdateGuideLights(
        float speed,
        float tolerance
        )
    {
        TLevel(TASK);
        TEnter();

        if (!m_targetAcquired)
        {
            m_speedLights.Set(false, SPEED_LIGHT_ALL);
            m_PanLights.Set(false, GUIDE_LIGHT_PAN_ALL);
	    	m_TiltLights.Set(false, GUIDE_LIGHT_TILT_ALL);
        }
        else
        {
            UINT8 PanLights;
			UINT8 TiltLights;

			if (m_targetInfo.hAngle < -TARGET_PANANGLE_TOLERANCE)
			{
				PanLights = GUIDE_LIGHT_PAN_LEFT;
			}
			else if (m_targetInfo.hAngle > TARGET_PANANGLE_TOLERANCE)
			{
				PanLights = GUIDE_LIGHT_PAN_RIGHT;
			}
			else
			{
				PanLights = GUIDE_LIGHT_PAN_ALL;
			}

			if (m_targetInfo.vAngle + m_vAngleAdj < -TARGET_TILTANGLE_TOLERANCE)
			{
				TiltLights = GUIDE_LIGHT_TILT_LOW;
			}
			else if (m_targetInfo.vAngle + m_vAngleAdj > TARGET_TILTANGLE_TOLERANCE)
			{
				TiltLights = GUIDE_LIGHT_TILT_HIGH;
			}
			else
			{
				TiltLights = GUIDE_LIGHT_TILT_ALL;
			}

            if ((PanLights != GUIDE_LIGHT_PAN_ALL) ||
                (TiltLights != GUIDE_LIGHT_TILT_ALL) ||
                (speed == 0.0) ||
                (fabs(GetSpeed() - speed) > tolerance))
            {
                m_speedLights.Set(false, SPEED_LIGHT_ALL);
        		m_PanLights.Set(true, PanLights);
                m_TiltLights.Set(true, TiltLights);
            }
            else
            {
                m_speedLights.Set(m_speedLightStates,
                                  ARRAYSIZE(m_speedLightStates),
                                  SOLO_REPEAT);
            }

            if ((m_notifyEvent != NULL) &&
                (PanLights == GUIDE_LIGHT_PAN_ALL) &&
                (TiltLights == GUIDE_LIGHT_TILT_ALL))
            {
                m_notifyEvent->SetEvent();
            }
        }

        TExit();
    }   //UpdateGuideLight

    /**
     * This function pushes the frisbee into the shooter wheels
     *
     * @param notifyEvent which will trigger when the frisbee shoots (optional)
     *
     * @return Return true if success, otherwise the state machine is already
     *         busy working on a previous command.
     */
    bool
    LaunchFrisbee(
        Event *notifyEvent = NULL
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnter();

        m_launcher.Set(SolID(SOL_DEPLOYER_RETRACT), SHOOTER_FIRE_TIME,
                       SolID(SOL_DEPLOYER_EXTEND), SHOOTER_FIRE_TIME,
                       0, notifyEvent);

        TExit();
        return fSuccess;
    }   //LaunchFrisbee

    /**
     * This function is called by the TaskMgr to update the Shooter state
     * machine.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        //
        // Update target information.
        //
#ifdef _USE_VISION
        m_targetAcquired = m_visionTarget->GetTargetInfo(&m_targetInfo);
        if (m_targetAcquired)
        {
            if (m_fContinuousMode && (mode!= MODE_DISABLED))
            {
                //m_panTilt->SetPanTiltAngle(m_targetInfo.hAngle,
                //                           m_targetInfo.vAngle,
                //                           &m_panTiltEvent);
                m_panTilt->SetPanAngle(m_targetInfo.hAngle);
                m_panTilt->SetTiltAngle(m_targetInfo.vAngle + m_vAngleAdj);// + (m_targetInfo.distance - 100) * TILT_ADJUST_PER_DIST, NULL);
                //LCDPrintf((LCD_LINE4, "Setpoint=%5.2f", m_targetInfo.vAngle + (m_targetInfo.distance - 100) * TILT_ADJUST_PER_DIST));
            }
            LCDPrintf((LCD_LINE5, "vAngle=%5.1f dist=%5.2f", m_targetInfo.vAngle, m_targetInfo.distance));
        }
#endif
        LCDPrintf((LCD_LINE4, "speed=%5.2f", GetSpeed()));
        m_dashboardDataFormat->SendLowPriorityData(GetSpeed());
        
        //
        // Green light indicates the shooter speed is within tolerance.
        //
        
        UpdateGuideLights(0.0, 0.0);

#ifdef _DEBUG_SHOOTER
        LCDPrintf((LCD_LINE3, "A=%5.1f",
                   m_targetInfo.hAngle));
        LCDPrintf((LCD_LINE4, "D=%5.1f,H=%5.1f",
                   m_targetInfo.distance, m_targetInfo.height));
        LCDPrintf((LCD_LINE6, "S=%4.0fd=%2.0fa=%0.2f",
        		   GetSpeed(), (m_speedAdj-1.0)*100.0, m_angleAdj));
#endif
        /*LCDPrintf((LCD_LINE4, "D=%5.1f,H=%5.1f",
                   m_targetInfo.distance, m_targetInfo.height));
        LCDPrintf((LCD_LINE6, "S=%4.0f",
                  GetSpeed()));*/

        TExit();
        return;
    }   //TaskPostPeriodic

    /**
     * This function prints the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    GetVariable(
        PVAR_ENTRY varEntry
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd, Kf;

        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));

        m_pidCtrl.GetPID(&Kp, &Ki, &Kd, &Kf);
        switch (varEntry->varID)
        {
            case VARID_PID:
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f, Kf=%10.8f\n",
                       Kp, Ki, Kd, Kf);
                break;

            case VARID_KP:
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_KI:
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_KD:
                printf("Kd=%10.8f\n", Kd);
                break;

            case VARID_KF:
                printf("Kd=%10.8f\n", Kf);
                break;

            default:
                TErr(("Invalid variable ID (var=%s,ID-%d).",
                      varEntry->varName, varEntry->varID));
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //GetVariable

    /**
     * This function sets the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     * @param varData Points to the data to be set to the variable.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    SetVariable(
        PVAR_ENTRY varEntry,
        void *varData
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd, Kf;

        TLevel(CALLBK);
        TEnterMsg(("var=%s,varData=%p", varEntry->varName, varData));

        m_pidCtrl.GetPID(&Kp, &Ki, &Kd, &Kf);
        switch (varEntry->varID)
        {
            case VARID_KP:
                Kp = *(double *)varData;
                break;

            case VARID_KI:
                Ki = *(double *)varData;
                break;

            case VARID_KD:
                Kd = *(double *)varData;
                break;

            case VARID_KF:
                Kf = *(double *)varData;
                break;

            default:
                TErr(("Invalid variable ID (var=%s,ID=%d).\n",
                      varEntry->varName, varEntry->varID));
                rc = ERR_ASSERT;
        }
        m_pidCtrl.SetPID(Kp, Ki, Kd, Kf);

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetVariable

#ifdef _CANJAG_PERF
    /**
     * This function sets up the various perfdata objects.
     *
     * @param setMotorPerfData Points to the perfdata object to collect
     *        Set() performance.
     * @param getPosPerfData Points to the perfdata object to collect
     *        GetPosition() performance.
     * @param getSpeedPerfData Points to the perfdata object to collect
     *        GetSpeed() performance.
     */
    void
    SetPerfData(
        PerfData *setMotorPerfData,
        PerfData *getPosPerfData,
        PerfData *getSpeedPerfData
        )
    {
        TLevel(API);
        TEnter();

        m_frontMotor.SetPerfData(setMotorPerfData,
                                 getPosPerfData,
                                 getSpeedPerfData);
        m_rearMotor.SetPerfData(setMotorPerfData,
                                getPosPerfData,
                                getSpeedPerfData);

        TExit();
        return;
    }   //SetPerfData
#endif

};  //class Shooter

VAR_ENTRY Shooter::m_varTable[] =
{
    {"PID",  VARID_PID, VarDouble, NULL, 0, "%10.8f",
     "Get PID constants"},
    {"Kp",   VARID_KP,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Kp constant"},
    {"Ki",   VARID_KI,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Ki constant"},
    {"Kd",   VARID_KD,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Kd constant"},
    {"Kf",   VARID_KF,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Kf constant"},
    {NULL,      0,      VarNone,   NULL, 0, NULL, NULL}
};

UINT32 Shooter::m_speedLightChannels[4] =
{
    SOL_PANLIGHT_LEFT,
    SOL_PANLIGHT_RIGHT,
    SOL_TILTLIGHT_LOW,
    SOL_TILTLIGHT_HIGH
};

SOL_STATE Shooter::m_speedLightStates[2] =
{
    {GUIDE_LIGHT_PAN_ALL,   1.0},
    {GUIDE_LIGHT_TILT_ALL,  1.0}
};

#endif  //ifndef _SHOOTER_H
