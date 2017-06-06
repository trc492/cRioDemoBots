class PanTilt
    : public CoopTask
    , public PIDInput
{
private:
    TrcServo    m_panServo;
    TrcServo    m_tiltServo;
    TrcPIDCtrl  m_panPidCtrl;
    TrcPIDCtrl  m_tiltPidCtrl;
    TrcPIDServo m_panPidServo;
    TrcPIDServo m_tiltPidServo;

    float GetInput(TrcPIDCtrl *pidCtrl)
    {
        float input = 0.0;

        if (pidCtrl == &m_panPidCtrl)
        {
            input = m_panServo.GetAngle();
        }
        else if (pidCtrl == &m_tiltPidCtrl)
        {
            input = m_tiltServo.GetAngle();
        }

        return input;
    }   //GetInput

public:
    void Stop(void)
    {
        m_panPidServo.Reset();
        m_tiltPidServo.Reset();
    }   //Stop

    PanTilt(void)
        : m_panServo(PWM_PAN_SERVO,
                     INIT_PAN_ANGLE,
                     PANANGLE_LOW_LIMIT,
                     PANANGLE_HIGH_LIMIT,
                     MAX_PAN_STEPRATE,
                     SERVOO_REVERSE)
        , m_tiltServo(PWM_TILT_SERVO,
                      INIT_TILT_ANGLE,
                      TILTANGLE_LOW_LIMIT,
                      TILTANGLE_HIGH_LIMIT,
                      MAX_TILT_STEPRATE,
                      SERVOO_REVERSE)
        , m_panPidCtrl("Pan", PAN_KP, PAN_KI, PAN_KD, PAN_KF)
        , m_tiltPidCtrl("Tilt", TILT_KP, TILT_KI, TILT_KD, TILT_KF)
        , m_panPidServo(&m_panServo, &m_panPidCtrl, this)
        , m_tiltPidServo(&m_tiltServo, &m_tiltPidCtrl, this)
    {
        Stop();
        RegisterTask("PanTilt", TASK_START_MODE | TASK_STOP_MODE);
    }   //PanTilt

    virtual ~PanTilt(void)
    {
        UnregisterTask();
        Stop();
    }   //~PanTilt

    void SetPanPower(float power)
    {
        m_panServo.SetPower(power);
    }   //SetPanPower

    void SetTiltPower(float power)
    {
        m_tiltServo.SetPower(power);
    }   //SetTiltPower

    void SetPanTiltPower(float panPower, float tiltPower)
    {
        SetPanPower(panPower);
        SetTiltPower(tiltPower);
#ifdef _DEBUG_PANTILT
        LCDPrintf((LCD_LINE2, "PA=%5.1f,TA=%5.1f",
                   m_panServo.GetAngle(),
                   m_tiltServo.GetAngle()));
#endif
    }   //SetPanTiltPower

    float GetPanAngle(void)
    {
        return m_panServo.GetAngle()/PANANGLE_GEAR_RATIO;
    }   //GetPanAngle

    float GetTiltAngle(void)
    {
        return m_tiltServo.GetAngle()/TILTANGLE_GEAR_RATIO;
    }   //GetTiltAngle

    void SetPanAngle(float gunAngle, bool fRelative = false)
    {
        float servoAngle = gunAngle*PANANGLE_GEAR_RATIO;

        if (fRelative)
        {
            servoAngle += m_panServo.GetAngle();
        }
        m_panServo.SetAngle(servoAngle);
    }   //SetPanAngle

    void SetTiltAngle(float gunAngle, bool fRelative = false)
    {
        float servoAngle = gunAngle*TILTANGLE_GEAR_RATIO;

        if (fRelative)
        {
            servoAngle += m_tiltServo.GetAngle();
        }
        m_tiltServo.SetAngle(servoAngle);
    }   //SetTiltAngle

    void SetPanTiltAngle(float panAngle, float tiltAngle, bool fRelative = false)
    {
        SetPanAngle(panAngle, fRelative);
        SetTiltAngle(tiltAngle, fRelative);
    }   //SetPanTiltAngle

    void SetPanTarget(float gunAngle, bool fHoldTarget = false)
    {
        m_panPidServo.SetTarget(gunAngle*PANANGLE_GEAR_RATIO, fHoldTarget);
    }   //SetPanTarget

    void SetTiltTarget(float gunAngle, bool fHoldTarget = false)
    {
        m_tiltPidServo.SetTarget(gunAngle*TILTANGLE_GEAR_RATIO, fHoldTarget);
    }   //SetTiltTarget

    void SetPanTiltTarget(float panAngle, float tiltAngle, bool fHoldTarget = false)
    {
        SetPanTarget(panAngle, fHoldTarget);
        SetTiltTarget(tiltAngle, fHoldTarget);
    }   //SetPanTiltTarget

    void TaskStartMode(UINT32 mode)
    {
        Stop();
    }   //TaskStartMode

    void TaskStopMode(UINT32 mode)
    {
        Stop();
    }   //TaskStopMode
};  //class PanTilt
