#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="PanTilt.h" />
///
/// <summary>
///     This module controls the pan and tilt action of the robot.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_PANTILT
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "PanTilt"


#define VARID_PPID              (VARID_DATAPTR + 1)
#define VARID_PKP               (VARID_DATAPTR + 2)
#define VARID_PKI               (VARID_DATAPTR + 3)
#define VARID_PKD               (VARID_DATAPTR + 4)
#define VARID_PKF               (VARID_DATAPTR + 5)
#define VARID_UTPID             (VARID_DATAPTR + 6)
#define VARID_UTKP              (VARID_DATAPTR + 7)
#define VARID_UTKI              (VARID_DATAPTR + 8)
#define VARID_UTKD              (VARID_DATAPTR + 9)
#define VARID_UTKF              (VARID_DATAPTR + 10)
#define VARID_DTPID             (VARID_DATAPTR + 11)
#define VARID_DTKP              (VARID_DATAPTR + 12)
#define VARID_DTKI              (VARID_DATAPTR + 13)
#define VARID_DTKD              (VARID_DATAPTR + 14)
#define VARID_DTKF              (VARID_DATAPTR + 15)

#define PANCAL_STATE_DONE       71

class PanTilt: public CoopTask, PIDInput, CmdHandler
{
private:
    CanJag          m_panMotor;
    CanJag          m_tiltMotor;
    TrcPIDCtrl      m_panPIDCtrl;
    TrcPIDCtrl      m_tiltPIDCtrl;
    TrcPIDMotor     m_panPIDMotor;
    TrcPIDMotor     m_tiltPIDMotor;
    KalmanFilter    m_tiltFilter;
    
    double          m_pKp, m_pKi, m_pKd, m_pKf;
    double          m_utKp, m_utKi, m_utKd, m_utKf;
    double          m_dtKp, m_dtKi, m_dtKd, m_dtKf;

    double          m_panAngleAdj;
    double          m_tiltReadingAtZero;
    bool            m_tiltLowLimitOk;
    
    double          m_tiltError;

public:
    static VAR_ENTRY m_varTable[];
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    PanTilt(
        ): m_panMotor(CANID_PAN_JAG, CANJaguar::kPercentVbus)
         , m_tiltMotor(CANID_TILT_JAG, CANJaguar::kPercentVbus)
         , m_panPIDCtrl("Pan", PAN_KP, PAN_KI, PAN_KD, PAN_KF)
         , m_tiltPIDCtrl("Tilt", TILT_UP_KP, TILT_UP_KI, TILT_UP_KD, TILT_UP_KF,
                         0.0, 0, PIDCTRLO_INVERSE)
         , m_panPIDMotor(&m_panMotor, &m_panPIDCtrl, this)
         , m_tiltPIDMotor(&m_tiltMotor, &m_tiltPIDCtrl, this)
         , m_tiltFilter()
         , m_pKp(PAN_KP)
         , m_pKi(PAN_KI)
         , m_pKd(PAN_KD)
         , m_pKf(PAN_KF)
         , m_utKp(TILT_UP_KP)
         , m_utKi(TILT_UP_KI)
         , m_utKd(TILT_UP_KD)
         , m_utKf(TILT_UP_KF)
         , m_dtKp(TILT_DOWN_KP)
         , m_dtKi(TILT_DOWN_KI)
         , m_dtKd(TILT_DOWN_KD)
         , m_dtKf(TILT_DOWN_KF)
         , m_panAngleAdj(0.0)
         , m_tiltReadingAtZero(0.0)
         , m_tiltLowLimitOk(m_tiltMotor.GetReverseLimitOK())
         , m_tiltError(0.0)
    {
        TLevel(INIT);
        TEnter();
        //
        // Initialize motor controllers.
        //
        m_panMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_panMotor.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
        m_panMotor.ConfigEncoderCodesPerRev(PAN_ENCODER_PPR);
        m_panMotor.SetSafetyEnabled(false);
        m_tiltMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_tiltMotor.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
        m_tiltMotor.ConfigEncoderCodesPerRev(TILT_ENCODER_PPR);
        m_tiltMotor.SetSafetyEnabled(false);
        
        m_panPIDCtrl.SetOutputRange(-PAN_SLOWDOWN * 0.75, PAN_SLOWDOWN * 0.75);
        m_tiltPIDCtrl.SetOutputRange(-TILT_SLOWDOWN * 0.6, TILT_SLOWDOWN * 0.6);

        RegisterTask(MOD_NAME, TASK_POST_PERIODIC);
        
        RegisterCmdHandler(MOD_NAME, NULL, m_varTable);
        
#ifdef _LOGDATA_TILT
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, "", "error", "%5.2f",
                                 DataDouble, &m_tiltError);
#endif

        TExit();
    }   //PanTilt

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~PanTilt(
        void
        )
    {
        TLevel( INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~PanTilt

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

        if (pidCtrl == &m_panPIDCtrl)
        {
            input = (m_panMotor.GetPosition() + m_panAngleAdj)*
                    PAN_DEGREE_PER_ENC_REVOLUTION;
#ifdef _DEBUG_PANTILT
            LCDPrintf((LCD_LINE5, "encoder pos=%7.4f", m_panMotor.GetPosition()));
#endif
        }
        else if(pidCtrl == &m_tiltPIDCtrl)
        {
            //input = (m_tiltMotor.GetPosition() - m_tiltReadingAtZero) * TILT_DEGREES_PER_CLICK + TILT_ANGLE_AT_ZERO;
            input = 2 * asin((TILT_STRING_LENGTH_AT_ZERO 
                                + ((m_tiltMotor.GetPosition() - m_tiltReadingAtZero) * TILT_SPOOL_CIRCUMFRENCE)) 
                            / (2 * TILT_SPOOL_RADIUS)) * DEGREES_PER_RADIAN;
#ifdef _DEBUG_PANTILT
            //LCDPrintf((LCD_LINE4, "pos=%10.8f", 
            //        TILT_STRING_LENGTH_AT_ZERO
            //            + (m_tiltMotor.GetPosition() - m_tiltReadingAtZero) * TILT_SPOOL_CIRCUMFRENCE));
#endif
            input = input - TILT_MAGIC_NUMBER;
            input = m_tiltFilter.FilterData(input);
            LCDPrintf((LCD_LINE3, "tilt=%10.8f", input));
            
            m_tiltError = pidCtrl->GetError();
            if(m_tiltError < 0.0)
            {
                m_tiltPIDCtrl.SetPID(m_dtKp, m_dtKi, m_dtKd, m_dtKf);
            }
            else
            {
                m_tiltPIDCtrl.SetPID(m_utKp, m_utKi, m_utKd, m_utKf);
            }
        }

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput

    void
    SetPanPower(
        float power
        )
    {
        m_panPIDMotor.SetPower(power);
    }

    void
    SetTiltPower(
        float power
        )
    {
        m_tiltPIDMotor.SetPower(power);
    }

    void
    SetPanTiltPower(
        float panPower,
        float tiltPower)
    {
        SetPanPower(panPower);
        SetTiltPower(tiltPower);
    }

    void 
    SetPanAngle(
        float angle)
    {
        m_panPIDMotor.SetTarget(angle, true);
    }

    void
    SetTiltAngle(
        float angle)
    {
        if(angle < 0.0)
        {
            m_tiltPIDCtrl.SetPID(m_dtKp, m_dtKi, m_dtKd, m_dtKf);
        }
        else
        {
            m_tiltPIDCtrl.SetPID(m_utKp, m_utKi, m_utKd, m_utKf);
        }
        m_tiltPIDMotor.SetTarget(angle, true);
    }

    void 
    SetPanTiltAngle(
        float panAngle, 
        float tiltAngle)
    {
        SetPanAngle(panAngle);
        SetTiltAngle(tiltAngle);
    }

    void
    StopPanTiltAngle(
        void
        )
    {
        m_tiltPIDMotor.Reset();
        m_panPIDMotor.Reset();
    }

    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        bool currTiltLowLimitOk = m_tiltMotor.GetReverseLimitOK();

        if (currTiltLowLimitOk != m_tiltLowLimitOk)
        {
            //
            // Tilt low limit switch just changed state.
            //
            if (!currTiltLowLimitOk)
            {
                //
                // Zero tilt position when the tilt low limit switch is hit.
                //
                m_tiltReadingAtZero = m_tiltMotor.GetPosition();
            }
            m_tiltLowLimitOk = currTiltLowLimitOk;
        }
        //LCDPrintf((LCD_LINE5, "pan=%5f tilt=%5f", GetInput(&m_panPIDCtrl), GetInput(&m_tiltPIDCtrl)));
#ifdef _DEBUG_PANTILT
        LCDPrintf((LCD_LINE3, "pan=%10.8f", GetInput(&m_panPIDCtrl)));
        LCDPrintf((LCD_LINE4, "tilt=%10.8f", GetInput(&m_tiltPIDCtrl)));
        
        LCDPrintf((LCD_LINE2, "tiltError=%10.8f", m_tiltPIDCtrl.GetError()));
#endif
    }
    //needs periodic task to check if both tiltEvent and panEvent are signaled
    //if both are signaled, signal aimCompletionEvent and clear pan and tilt events
    

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

        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));
        
        switch (varEntry->varID)
        {
            case VARID_PPID:
                printf("pKp=%10.8f, pKi=%10.8f, pKd=%10.8f, pKf=%10.8f\n",
                       m_pKp, m_pKi, m_pKd, m_pKf);
                break;
                
            case VARID_UTPID:
                printf("tKp=%10.8f, tKi=%10.8f, tKd=%10.8f, tKf=%10.8f\n",
                       m_utKp, m_utKi, m_utKd, m_utKf);
                break;
                
            case VARID_DTPID:
                printf("tKp=%10.8f, tKi=%10.8f, tKd=%10.8f, tKf=%10.8f\n",
                       m_dtKp, m_dtKi, m_dtKd, m_dtKf);
                break;

            case VARID_PKP:
                printf("pKp=%10.8f\n", m_pKp);
                break;

            case VARID_PKI:
                printf("pKi=%10.8f\n", m_pKi);
                break;

            case VARID_PKD:
                printf("pKd=%10.8f\n", m_pKd);
                break;

            case VARID_PKF:
                printf("pKf=%10.8f\n", m_pKf);
                break;

            case VARID_UTKP:
                printf("tKp=%10.8f\n", m_utKp);
                break;

            case VARID_UTKI:
                printf("tKi=%10.8f\n", m_utKi);
                break;

            case VARID_UTKD:
                printf("tKd=%10.8f\n", m_utKd);
                break;

            case VARID_UTKF:
                printf("tKf=%10.8f\n", m_utKf);
                break;

            case VARID_DTKP:
                printf("tKp=%10.8f\n", m_dtKp);
                break;

            case VARID_DTKI:
                printf("tKi=%10.8f\n", m_dtKi);
                break;

            case VARID_DTKD:
                printf("tKd=%10.8f\n", m_dtKd);
                break;

            case VARID_DTKF:
                printf("tKf=%10.8f\n", m_dtKf);
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

        TLevel(CALLBK);
        TEnterMsg(("var=%s,varData=%p", varEntry->varName, varData));
        
        switch (varEntry->varID)
        {
            case VARID_PKP:
                m_pKp = *(double *)varData;
                break;
    
            case VARID_PKI:
                m_pKi = *(double *)varData;
                break;
    
            case VARID_PKD:
                m_pKd = *(double *)varData;
                break;
                
            case VARID_PKF:
                m_pKf = *(double *)varData;
                break;
                
            case VARID_UTKP:
                m_utKp = *(double *)varData;
                break;
    
            case VARID_UTKI:
                m_utKi = *(double *)varData;
                break;
    
            case VARID_UTKD:
                m_utKd = *(double *)varData;
                break;

            case VARID_UTKF:
                m_utKf = *(double *)varData;
                break;

            case VARID_DTKP:
                m_dtKp = *(double *)varData;
                break;
    
            case VARID_DTKI:
                m_dtKi = *(double *)varData;
                break;
    
            case VARID_DTKD:
                m_dtKd = *(double *)varData;
                break;

            case VARID_DTKF:
                m_dtKf = *(double *)varData;
                break;

            default:
                TErr(("Invalid variable ID (var=%s,ID=%d).\n",
                      varEntry->varName, varEntry->varID));
                rc = ERR_ASSERT;
        }
        m_panPIDCtrl.SetPID(m_pKp, m_pKi, m_pKd, m_pKf);
        //tilt PID controller values get set in the SetTiltAngle function
        //	(because it depends on whether we are going up or down on which vars to use) 

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetVariable
};

VAR_ENTRY PanTilt::m_varTable[] =
{
    {"PPID",  VARID_PPID, VarDouble, NULL, 0, "%10.8f",
     "Get pan PID constants"},
    {"pKp",   VARID_PKP,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set pan Kp constant"},
    {"pKi",   VARID_PKI,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set pan Ki constant"},
    {"pKd",   VARID_PKD,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set pan Kd constant"},
    {"pKf",   VARID_PKF,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set pan Kf constant"},
    {"uTPID",  VARID_UTPID, VarDouble, NULL, 0, "%10.8f",
     "Get tilt PID constants"},
    {"utKp",   VARID_UTKP,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kp constant"},
    {"utKi",   VARID_UTKI,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Ki constant"},
    {"utKd",   VARID_UTKD,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kd constant"},
    {"utKf",   VARID_UTKF,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kf constant"},
    {"dTPID",  VARID_DTPID, VarDouble, NULL, 0, "%10.8f",
     "Get tilt PID constants"},
    {"dtKp",   VARID_DTKP,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kp constant"},
    {"dtKi",   VARID_DTKI,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Ki constant"},
    {"dtKd",   VARID_DTKD,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kd constant"},
    {"dtKf",   VARID_DTKF,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set tilt Kf constant"},
    {NULL,      0,      VarNone,   NULL, 0, NULL, NULL}
};
