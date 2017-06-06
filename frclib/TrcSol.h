#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcSol.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the TrcSol
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCSOL_H
#define _TRCSOL_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_SOLENOID
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Solenoid"

#define SolID(c)                ((UINT8)(1 << ((c) - 1)))

#define MAX_SOL_CHANNELS        8
#define SOLO_REPEAT             0x00000001

typedef struct _SolState
{
    UINT8   onSolMask;
    double  period;
} SOL_STATE, *PSOL_STATE;

/**
 * This module defines and implements the TrcSol object.
 */
class TrcSol: public CoopTask
{
private:
    Solenoid       *m_solenoids[MAX_SOL_CHANNELS];
    UINT8           m_solIDs[MAX_SOL_CHANNELS];
    int             m_numSols;
    StateMachine    m_solSM;
    TrcTimer        m_timer;
    Event           m_timerEvent;
    UINT8           m_onSolMask;
    PSOL_STATE      m_solStates;
    int             m_numStates;
    int             m_currSolState;
    UINT32          m_solOptions;
    Event          *m_notifyEvent;
    SOL_STATE       m_timedOnOff[3];

    /**
     * This function is called by all variants of the constructor to do
     * common initialization.
     *
     * @param channels Points to an array specifying the solenoid channels.
     * @param numSols Specifies the number of channels in the array.
     * @param module Specifies the module number.
     *
     * @return Success: Returns true.
     * @return Failure: Returns false.
     */
    bool
    CommonInit(
        UINT32 *channels,
        int numSols,
        UINT8 module
        )
    {
        bool fSuccess = true;

        TLevel(INIT);
        TEnterMsg(("channels=%p,numSols=%d,module=%d",
                   channels, numSols, module));

        m_onSolMask = 0;
        m_solStates = NULL;
        m_numStates = 0;
        m_currSolState = 0;
        m_solOptions = 0;
        m_notifyEvent = NULL;
        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_CONTINUOUS);

        m_numSols = 0;
        for (int i = 0; i < numSols; i++)
        {
            m_solenoids[i] = new Solenoid(module, channels[i]);
            if (m_solenoids[i] == NULL)
            {
                TErr(("Failed to create solenoid channel %d.", channels[i]));
                fSuccess = false;
                break;
            }
            else
            {
                m_solIDs[i] = SolID(channels[i]);
                m_numSols++;
            }
        }

        if (!fSuccess)
        {
            for (int i = 0; i < m_numSols; i++)
            {
                delete m_solenoids[i];
                m_solenoids[i] = NULL;
                m_solIDs[i] = 0;
            }
            m_numSols = 0;
        }

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //CommonInit

public:
    /**
     * This function sets the solenoid states.
     *
     * @param fOn Specifies the solenoid states to be ON or OFF.
     * @param solMask Specifies the solenoid mask.
     */
    void
    Set(
        bool  fOn,
        UINT8 solMask
        )
    {
        TLevel(API);
        TEnterMsg(("fOn=%d,masks=%x", fOn, solMask));

        if (solMask != 0)
        {
            if (m_solSM.IsEnabled())
            {
                m_timer.CancelTimer();
                m_timerEvent.ClearEvent();
                m_solSM.Stop();
            }

            if (fOn)
            {
                m_onSolMask |= solMask;
            }
            else
            {
                m_onSolMask &= ~solMask;
            }

            for (int i = 0; i < m_numSols; i++)
            {
                if (m_solIDs[i] & solMask)
                {
                    m_solenoids[i]->Set(fOn);
                }
            }
        }

        TExit();
        return;
    }   //Set

    /**
     * This function sets the states of a solenoid pair. This is usually for
     * pneumatics.
     *
     * @param onMask Specifies the ON solenoids.
     * @param offMask Specifies the OFF solenoids.
     */
    void
    Set(
        UINT8 onMask,
        UINT8 offMask
        )
    {
        TLevel(API);
        TEnterMsg(("onMask=%x,offMask=%x", onMask, offMask));

        if (m_solSM.IsEnabled())
        {
            m_timer.CancelTimer();
            m_timerEvent.ClearEvent();
            m_solSM.Stop();
        }

        Set(false, offMask);
        Set(true, onMask);

        TExit();
        return;
    }   //Set

    /**
     * This function sets the solenoid on/off states pattern.
     *
     * @param solStates Points to an array specifying the sequence of
     *        solenoid states and periods.
     * @param numStates Specifies the number of elements in the array.
     * @param options Specifies the option flags.
     * @param notifyEvent Specifies the event to notify when done.
     */
    void
    Set(
        PSOL_STATE solStates,
        int numStates,
        UINT32 options = 0,
        Event *notifyEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("states=%p,numStates=%d,options=%x,event=%p",
                   solStates, numStates, options, notifyEvent));

        if (m_solSM.IsEnabled())
        {
            m_timer.CancelTimer();
            m_timerEvent.ClearEvent();
            m_solSM.Stop();
        }

        m_solStates = solStates;
        m_numStates = numStates;
        m_solOptions = options;
        m_notifyEvent = notifyEvent;
        m_currSolState = 0;
        m_solSM.Start();

        TExit();
        return;
    }   //Set

    /**
     * This function turns a solenoid ON with given ON period.
     *
     * @param solMask Specifies the solenoid mask.
     * @param period Specifies the ON period.
     * @param notifyEvent Specifies the event to notify when done.
     */
    void
    Set(
        UINT8   solMask,
        double  period,
        Event  *notifyEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("solMask=%x,Period=%f, event=%p",
                   solMask, period, notifyEvent));

        m_timedOnOff[0].onSolMask = solMask;
        m_timedOnOff[0].period = period;
        m_timedOnOff[1].onSolMask = 0;
        m_timedOnOff[1].period = 0.0;
        Set(m_timedOnOff, 2, 0, notifyEvent);

        TExit();
        return;
    }   //Set

    /**
     * This function turns a pair of solenoids ON/OFF with given ON/OFF period.
     *
     * @param onMask Specifies the ON solenoid mask.
     * @param onPeriod Specifies the ON period.
     * @param offMask Specifies the OFF solenoid mask.
     * @param offPeriod Specifies the OFF period.
     * @param options Specifies the option flags.
     * @param notifyEvent Specifies the event to notify when done.
     */
    void
    Set(
        UINT8   onMask,
        double  onPeriod,
        UINT8   offMask,
        double  offPeriod,
        UINT32  options = 0,
        Event  *notifyEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("onMask=%x,onTime=%f,offMask=%x,offTime=%f,flags=%x,"
                   "event=%p",
                   onMask, onPeriod, offMask, offPeriod, options, notifyEvent));

        m_timedOnOff[0].onSolMask = onMask;
        m_timedOnOff[0].period = onPeriod;
        m_timedOnOff[1].onSolMask = offMask;
        m_timedOnOff[1].period = offPeriod;

        if (options & SOLO_REPEAT)
        {
            Set(m_timedOnOff, 2, options, notifyEvent);
        }
        else
        {
            m_timedOnOff[2].onSolMask = 0;
            m_timedOnOff[2].period = 0.0;
            Set(m_timedOnOff, 3, options, notifyEvent);
        }

        TExit();
        return;
    }   //Set

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channels Points to an array specifying the solenoid channels.
     * @param numSols Specifies the number of channels in the array.
     * @param module Specifies the module number.
     */
    TrcSol(
        UINT32 *channels,
        int numSols,
        UINT8 module = 1
        ): m_solSM()
         , m_timer()
         , m_timerEvent()
    {
        TLevel(INIT);
        TEnterMsg(("channels=%p,numSols=%d,module=%d",
                   channels, numSols, module));

        CommonInit(channels, numSols, module);

        TExit();
    }   //TrcSol

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel Specifies the solenoid channel.
     * @param module Specifies the module number.
     */
    TrcSol(
        UINT32 channel,
        UINT8 module = 1
        ): m_solSM()
         , m_timer()
         , m_timerEvent()
    {
        TLevel(INIT);
        TEnterMsg(("chan=%d,module=%d", channel, module));

        CommonInit(&channel, 1, module);

        TExit();
    }   //TrcSol

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel1 Specifies the first solenoid channel.
     * @param channel2 Specifies the second solenoid channel.
     * @param module Specifies the module number.
     */
    TrcSol(
        UINT32 channel1,
        UINT32 channel2,
        UINT8 module = 1
        ): m_solSM()
         , m_timer()
         , m_timerEvent()
    {
        UINT32 channels[2];

        TLevel(INIT);
        TEnterMsg(("chan1=%d,chan2=%d,module=%d", channel1, channel2, module));

        channels[0] = channel1;
        channels[1] = channel2;
        CommonInit(channels, 2, module);

        TExit();
    }   //TrcSol

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel1 Specifies the first solenoid channel.
     * @param channel2 Specifies the second solenoid channel.
     * @param channel3 Specifies the third solenoid channel.
     * @param module Specifies the module number.
     */
    TrcSol(
        UINT32 channel1,
        UINT32 channel2,
        UINT32 channel3,
        UINT8 module = 1
        ): m_solSM()
         , m_timer()
         , m_timerEvent()
    {
        UINT32 channels[3];

        TLevel(INIT);
        TEnterMsg(("chan1=%d,chan2=%d,chan3=%d,module=%d",
                   channel1, channel2, channel3, module));

        channels[0] = channel1;
        channels[1] = channel2;
        channels[2] = channel3;
        CommonInit(channels, 3, module);

        TExit();
    }   //TrcSol

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~TrcSol(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        for (int i = 0; i < m_numSols; i++)
        {
            m_solenoids[i]->Set(false);
            SAFE_DELETE(m_solenoids[i]);
        }
        UnregisterTask();

        TExit();
    }   //~TrcSol

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

        //
        // Turn off all solenoids.
        //
        Set(false, (UINT8)0xff);

        TExit();
    }   //TaskStopMode

    /**
     * This function is called by the TaskMgr periodically to run the state
     * machine.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskPostContinuous(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if (m_solSM.IsReady())
        {
            UINT32 currState = m_solSM.GetCurrentState();

#ifdef _DEBUG_TRCSOL
            LCDPrintf((LCD_LINE3, "[%d] Solenoids %08x",
                       currState - SMSTATE_STARTED, m_onSolMask));
#endif
            switch (currState)
            {
            case SMSTATE_STARTED:
                if (m_currSolState < m_numStates)
                {
                    //
                    // Turn the corresponding solenoid ON/OFF.
                    //
                    for (int i = 0; i < m_numSols; i++)
                    {
                        if (m_solIDs[i] &
                            m_solStates[m_currSolState].onSolMask)
                        {
                            m_solenoids[i]->Set(true);
                        }
                        else
                        {
                            m_solenoids[i]->Set(false);
                        }
                    }
                    m_onSolMask = m_solStates[m_currSolState].onSolMask;

                    //
                    // Set timer and wait for it if necessary.
                    //
                    if (m_solStates[m_currSolState].period > 0.0)
                    {
                        m_timer.SetTimer(
                                m_solStates[m_currSolState].period,
                                &m_timerEvent);
                        m_solSM.WaitForSingleEvent(&m_timerEvent, currState);
                    }

                    //
                    // Move to the next element of the state array.
                    //
                    m_currSolState++;
                    if ((m_solOptions & SOLO_REPEAT) &&
                        (m_currSolState >= m_numStates))
                    {
                        m_currSolState = 0;
                    }
                }
                else
                {
                    //
                    // There is no more states, we are done.
                    //
                    m_solSM.SetCurrentState(currState + 1);
                }
                break;

            default:
                //
                // We are done.
                //
                m_solStates = NULL;
                m_numStates = 0;
                m_solOptions = 0;
                m_currSolState = 0;
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                    m_notifyEvent = NULL;
                }
                m_solSM.Stop();
                break;
            }
        }

        TExit();
    }   //TaskPostContinuous

};  //class TrcSol

#endif  //ifndef _TRCSOL_H
