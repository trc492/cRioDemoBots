#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="StateMachine.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     StateMachine class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SM
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "StateMachine"

//
// Constants.
//
#define SMF_READY               0x00000001
#define SMF_TIMEOUT             0x00000002

#define SMO_WAIT_ALL            0x00000001

#define SMSTATE_DISABLED        0
#define SMSTATE_STARTED         1

/**
 * This class defines and implements the StateMachine object.
 */
class StateMachine
{
private:
    Event **m_pEvents;
    Event  *m_singleEvent;
    UINT32  m_cEvents;
    UINT32  m_nextState;
    UINT32  m_smFlags;
    UINT32  m_smOptions;
    UINT32  m_currState;
    UINT32  m_timeout;

public:
    /**
     * Constructor: Create an instance of the StateMachine object.
     * It initializes the state machine to disabled mode.
     */
    StateMachine(
        void
        ): m_pEvents(NULL)
         , m_singleEvent(NULL)
         , m_cEvents(0)
         , m_nextState(SMSTATE_DISABLED)
         , m_smFlags(0)
         , m_smOptions(0)
         , m_currState(SMSTATE_DISABLED)
         , m_timeout(0)
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //StateMachine

    /**
     * Destructor: Destroy an instance of the StateMachine object.
     */
    ~StateMachine(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~StateMachine

    /**
     * This function is called to start the state machine.
     *
     * @param state Optionally specifies the initial state to start in.
     */
    void
    Start(
        UINT32 state = SMSTATE_STARTED
        )
    {
        TLevel(API);
        TEnter();

        m_pEvents = NULL;
        m_cEvents = 0;
        m_currState = state;
        m_nextState = state;
        m_timeout = 0;
        m_smFlags = SMF_READY;

        TExit();
    }   //Start

    /**
     * This function is called to stop the state machine.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_pEvents = NULL;
        m_cEvents = 0;
        m_currState = SMSTATE_DISABLED;
        m_nextState = SMSTATE_DISABLED;
        m_timeout = 0;
        m_smFlags = 0;

        TExit();
    }   //Stop

    /**
     * This function is called to check if the state machine is enabled.
     *
     * @return Returns true if the state machine is enabled, false otherwise.
     */
    bool
    IsEnabled(
        void
        )
    {
        TLevel(API);
        TEnter();
        bool rc = m_currState != SMSTATE_DISABLED;
        TExitMsg(("=%d", rc));
        return rc;
    }   //IsEnabled

    /**
     * This function is called periodically when processing the state machine.
     * It checks if the state machine is enabled and ready. If the state
     * machine is enabled but not ready, it will check if all the events it
     * is waiting for are signaled. If so, the state machine is advanced to the
     * next state and transitioned to ready state. If the state machine is
     * enabled and in ready state, the caller will perform the task according
     * to the current state. The task usually involves initiating the next
     * operation and waits for it to complete. While it is waiting, the state
     * machine will be put back in non-ready state.
     *
     * @return Returns true if the state machine is ready, false otherwise.
     */
    bool
    IsReady(
        void
        )
    {
        bool fReady = false;
        TLevel(API);
        TEnter();

        if (m_currState != SMSTATE_DISABLED)
        {
            if (m_smFlags & SMF_READY)
            {
                fReady = true;
            }
            else if ((m_timeout > 0) && (GetMsecTime() >= m_timeout))
            {
                m_smFlags |= SMF_TIMEOUT;
                m_timeout = 0;
                fReady = true;
            }
            else
            {
                UINT cSignaledEvents = 0;

                for (UINT32 i = 0; i < m_cEvents; i++)
                {
                    if (m_pEvents[i]->IsSignaled())
                    {
                        cSignaledEvents++;
                    }
                }

                if (!(m_smOptions & SMO_WAIT_ALL))
                {
                    if (cSignaledEvents > 0)
                    {
                        fReady = true;
                    }
                }
                else if (cSignaledEvents == m_cEvents)
                {
                    fReady = true;
                }
            }

            if (fReady && !(m_smFlags & SMF_READY))
            {
                ClearAllEvents();
                m_pEvents = NULL;
                m_cEvents = 0;
                m_currState = m_nextState;
                m_smFlags |= SMF_READY;
            }
        }

        TExitMsg(("=%d", fReady));
        return fReady;
    }   //IsReady

    /**
     * This function is called to get the current state of the state machine.
     *
     * @return Returns the current state of the state machine.
     */
    UINT32 GetCurrentState(
        void
        )
    {
        TLevel(UTIL);
        TEnter();
        TExitMsg(("=%d", m_currState));
        return m_currState;
    }   //GetCurrentState

    /**
     * This function is called to set the current state of the state machine.
     *
     * @param currState Specifies the current state to set the state machine
     *        to.
     */
    void SetCurrentState(
        UINT32 currState
        )
    {
        TLevel(UTIL);
        TEnterMsg(("currState=%d", currState));

        m_currState = currState;

        TExit();
        return;
    }   //SetCurrentState

    /**
     * This function is called to clear all events.
     */
    void
    ClearAllEvents(
        void
        )
    {
        TLevel(API);
        TEnter();

        for (UINT32 i = 0; i < m_cEvents; i++)
        {
            m_pEvents[i]->ClearEvent();
        }

        TExit();
        return;
    }   //ClearAllEvents

    /**
     * This function specfies an array of events to wait for and sets the
     * state machine to waiting state.
     *
     * @param pEvents Points to the array of event object pointers to wait for.
     * @param cEvents Specifies the number of event pointers in the array.
     * @param nextState Specifies the next state to advance to once the
     *        event(s) occurred.
     * @param timeout Optionally specifies the timeout value.
     * @param options Specifies the option flags of the wait. If the
     *        SMF_WAIT_ALL option is set, all the events in the array must be
     *        in signaled state before the state machine will move on to the
     *        next state. Otherwise, the state machine will advance to the next
     *        state if any one of the events is signaled.
     *
     * @return Success: returns true. Failure: returns false.
     */
    bool
    WaitForMultipleEvents(
        Event **pEvents,
        UINT32  cEvents,
        UINT32  nextState,
        UINT32  timeout = 0,
        UINT32  options = 0
        )
    {
        bool rc = false;

        TLevel(API);
        TEnterMsg(("pEvents=%p,cEvents=%d,NextState=%d,timeout=%d,options=%x",
                   pEvents, cEvents, nextState, timeout, options));

        if ((pEvents == NULL) || (cEvents == 0))
        {
            TErr(("Must have at least one event."));
        }
        else
        {
            m_pEvents = pEvents;
            m_cEvents = cEvents;
            ClearAllEvents();
            m_nextState = nextState;
            m_timeout = timeout;
            if (m_timeout > 0)
            {
                m_timeout += GetMsecTime();
            }
            m_smOptions = options;
            m_smFlags &= ~SMF_READY;
            rc = true;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //WaitForMultipleEvents

    /**
     * This function specfies a single events to wait for and sets the state
     * machine to waiting state.
     *
     * @param event Points to the event object to wait for.
     * @param nextState Specifies the next state to advance to once the
     *        event(s) occurred.
     * @param timeout Optionally specifies the timeout value in milliseconds.
     * @param options Specifies the option flags of the wait. If the
     *        SMF_WAIT_ALL option is set, all the events in the array must be
     *        in signaled state before the state machine will move on to the
     *        next state. Otherwise, the state machine will advance to the next
     *        state if any one of the events is signaled.
     *
     * @return Success: returns true.
     * @return Failure: returns false.
     */
    bool
    WaitForSingleEvent(
        Event *event,
        UINT32 nextState,
        UINT32 timeout = 0,
        UINT32 options = 0
        )
    {
        bool rc = false;

        TLevel(API);
        TEnterMsg(("event=%p,NextState=%d,timeout=%d,options=%x",
                   event, nextState, timeout, options));

        if (event == NULL)
        {
            TErr(("Event must not be NULL."));
        }
        else
        {
            m_singleEvent = event;
            m_pEvents = &m_singleEvent;
            m_cEvents = 1;
            ClearAllEvents();
            m_nextState = nextState;
            m_timeout = timeout;
            if (m_timeout > 0)
            {
                m_timeout += GetMsecTime();
            }
            m_smOptions = options;
            m_smFlags &= ~SMF_READY;
            rc = true;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //WaitForSingleEvent

};  //class StateMachine

#endif  //ifndef _STATEMACHINE_H
