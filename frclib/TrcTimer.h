#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcTimer.h" />
///
/// <summary>
///     This module contains the definition and implementation of the TrcTimer
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCTIMER_H
#define _TRCTIMER_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TIMER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcTimer"

#define TIMERF_SET              0x00000001

/**
 * This class defines the timer object. It allows the caller to set a timer
 * and notifies the caller when the timer expires.
 */
class TrcTimer
{
private:
    Event    *m_notifyEvent;
    SEM_ID    m_semaphore;
    Notifier *m_notifier;
    UINT32    m_timerFlags;

    /**
     * This function processes the expired timer and notfies the caller.
     */
    void
    TimerExpired(
        void
        )
    {
        TLevel(EVENT);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            m_timerFlags = 0;
            if (m_notifyEvent != NULL)
            {
                m_notifyEvent->SetEvent();
                m_notifyEvent = NULL;
            }
        }
        END_REGION;

        TExit();
    }   //TimerExpired

    /**
     * This function is called when the timer expired. It will call the
     * non-static worker function.
     *
     * @param timer Points to the TrcTimer object to call the expiration
     *        handler.
     */
    static
    void
    CallTimerExpired(
        void *timer
        )
    {
        TLevel(EVENT);
        TEnterMsg(("timer=%p", timer));

        ((TrcTimer*)timer)->TimerExpired();

        TExit();
    }   //CallTimerExpired

public:
    /**
     * Constructor: Create an instance of the TrcTimer object.
     */
    TrcTimer(
        void
        ): m_notifyEvent(NULL)
         , m_timerFlags(0)
    {
        TLevel(INIT);
        TEnter();

        m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
        m_notifier = new Notifier(TrcTimer::CallTimerExpired, this);

        TExit();
    }   //TrcTimer

    /**
     * Destructor: Destroy an instance of the TrcTimer object.
     */
    ~TrcTimer(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        semFlush(m_semaphore);
        SAFE_DELETE(m_notifier);

        TExit();
    }   //~TrcTimer

    /**
     * This function sets a timer to be expired with a given delay.
     *
     * @param delay Specifies the amount of time in seconds the timer will
     *        expire.
     * @param notifyEvent Points to the Event object to signal when the timer
     *        expires.
     *
     * @return Returns true if success, false otherwise.
     */
    bool
    SetTimer(
        double delay,
        Event *notifyEvent = NULL
        )
    {
        bool rc = true;
        TLevel(API);
        TEnterMsg(("delay=%f,event=%p", delay, notifyEvent));

        CRITICAL_REGION(m_semaphore)
        {
            if (m_notifyEvent != NULL)
            {
                TErr(("There is already a pending timer."));
                rc = false;
            }
            else
            {
                m_notifyEvent = notifyEvent;
                m_notifyEvent->ClearEvent();
                m_notifier->StartSingle(delay);
                m_timerFlags |= TIMERF_SET;
            }
        }
        END_REGION;

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetTimer

    /**
     * This function cancels a pending timer.
     */
    void
    CancelTimer(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_timerFlags & TIMERF_SET)
        {
            m_notifier->Stop();
            m_notifyEvent = NULL;
        }

        TExit();
    }   //CancelTimer

};  //class TrcTimer

#endif  //ifndef _TRCTIMER_H
