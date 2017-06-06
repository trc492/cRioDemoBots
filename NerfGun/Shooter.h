class Shooter
    : public CoopTask
    , public DigitalInNotify
{
private:
    Victor          m_nerfGunTrigger;
    StateMachine    m_triggerSM;
    TrcTimer        m_triggerTimer;
    Event           m_triggerEvent;

    DigitalIn       m_digitalIn;
    bool            m_fJamDoorClosed;
    bool            m_fClipPresent;

public:
    bool SetNerfGunTrigger(bool fOn)
    {
        bool fSuccess = false;

        if (!fOn)
        {
            if (!m_triggerSM.IsEnabled())
            {
                //
                // Only do this if there is no FireOneShot in progress.
                //
                m_nerfGunTrigger.Set(NERFGUN_TRIGGER_OFF);
                fSuccess = true;
            }
        }
        else if (m_fJamDoorClosed && m_fClipPresent &&
                 !m_triggerSM.IsEnabled())
        {
            //
            // Only allow trigger on if the there is no FireOneShot in
            // progress, the jam door is closed and the clip is present.
            //
            m_nerfGunTrigger.Set(NERFGUN_TRIGGER_ON);
            fSuccess = true;
        }

        return fSuccess;
    }   //SetNerfGunTrigger

    bool FireOneShot(void)
    {
        bool fSuccess = false;

        if (m_fJamDoorClosed && m_fClipPresent && !m_triggerSM.IsEnabled())
        {
            //
            // Allow it only if there isn't another FireOneShot in progress.
            //
            m_triggerSM.Start();
            fSuccess = true;
        }

        return fSuccess;
    }   //FireOneShot

    Shooter(void)
        : m_nerfGunTrigger(PWM_NERFGUN_TRIGGER)
        , m_triggerSM()
        , m_triggerTimer()
        , m_triggerEvent()
        , m_digitalIn()
        , m_fJamDoorClosed(!m_digitalIn.GetChannelState(DIO_JAMDOOR_CLOSED))
        , m_fClipPresent(!m_digitalIn.GetChannelState(DIO_CLIP_PRESENT))
    {
        //
        // Make sure the trigger is off.
        //
        SetNerfGunTrigger(false);
        m_digitalIn.RegisterNotification(this,
                                         (DInMask(DIO_JAMDOOR_CLOSED) |
                                          DInMask(DIO_CLIP_PRESENT)));
        RegisterTask("Shooter", TASK_STOP_MODE | TASK_POST_PERIODIC);
    }   //Shooter

    virtual ~Shooter(void)
    {
        UnregisterTask();
        m_digitalIn.UnregisterNotification(this);
        SetNerfGunTrigger(false);
        m_nerfGunTrigger.Disable();
    }   //~Shooter

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            SetNerfGunTrigger(false);
            m_nerfGunTrigger.Disable();
        }
    }   //TaskStopMode

    void TaskPostPeriodic(UINT32 mode)
    {
        if (mode != MODE_DISABLED && m_triggerSM.IsReady())
        {
            UINT32 currState = m_triggerSM.GetCurrentState();

            switch (currState)
            {
                case SMSTATE_STARTED:
                    if (m_fJamDoorClosed && m_fClipPresent)
                    {
                        m_nerfGunTrigger.Set(NERFGUN_TRIGGER_ON);
                        m_triggerTimer.SetTimer(TRIGGER_TIME, &m_triggerEvent);
                        m_triggerSM.WaitForSingleEvent(&m_triggerEvent,
                                                       currState + 1);
                    }
                    break;

                default:
                    m_nerfGunTrigger.Set(NERFGUN_TRIGGER_OFF);
                    m_triggerSM.Stop();
                    break;
            }
        }
        LCDPrintf((LCD_LINE5, "JamDoor: %s",
                   m_fJamDoorClosed? "Closed": "Opened"));
        LCDPrintf((LCD_LINE6, "Clip: %s",
                   m_fClipPresent? "Present": "Absent"));
    }   //TaskPostPeriodic

    void NotifyDIn(UINT8  module, UINT32 channel, bool fActive)
    {
        switch (channel)
        {
            case DIO_JAMDOOR_CLOSED:
                m_fJamDoorClosed = fActive;
                break;

            case DIO_CLIP_PRESENT:
                m_fClipPresent = fActive;
                break;
        }
    }   //NotifyDIn
};  //class Shooter
