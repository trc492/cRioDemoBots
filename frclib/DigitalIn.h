#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DigitalIn.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DigitalIn class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DIGITALIN_H
#define _DIGITALIN_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DIGITALIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DigitalIn"

//
// Macros.
//
#define MAX_DIGITAL_CHANNELS    14
#define DInMask(n)              (1 << (16 - (n)))

/**
 * This abstract class defines the DigitalInNotify object. The object
 * is a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the digital input events.
 */
class DigitalInNotify
{
public:
    /**
     * This function is provided by the subclass to handle a DigitalIn event
     * notification.
     *
     * @param module Specifies the Digital module number.
     * @param channel Specifies the DigitalIn channel that changed state.
     * @param fActive If true, specifies the DigitalIn channel is active,
     *        false otherwise.
     */
    virtual
    void
    NotifyDIn(
        UINT8  module,
        UINT32 channel,
        bool   fActive
        ) = 0;
};  //class DigitalInNotify

/**
 * This class defines and implements the DigitalIn object. It inherited the
 * DigitalModule object from the WPI library. It also added the capability of
 * detecting the changes of a digital input channel and call the notification
 * object for the change.
 */
class DigitalIn: public CoopTask,
                 public DigitalModule
{
private:
    UINT8            m_module;
    UINT16           m_notifyMask[MAX_DIGITAL_CHANNELS];
    UINT16           m_inverseMask[MAX_DIGITAL_CHANNELS];
    DigitalInNotify *m_notify[MAX_DIGITAL_CHANNELS];
    int              m_numNotifies;
    UINT16           m_channelMask;
    UINT16           m_prevDIn;

    /**
     * This function does the common initialization for the constructors.
     */
    void
    CommonInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        for (int idx = 0; idx < MAX_DIGITAL_CHANNELS; idx++)
        {
            m_notifyMask[idx] = 0;
            m_inverseMask[idx] = 0;
            m_notify[idx] = NULL;
        }
        m_numNotifies = 0;
        m_channelMask = 0;
        m_prevDIn = GetDIO();

#ifdef _LOGDATA_DIGITALIN
        DataLogger *dataLogger = DataLogger::GetInstance();
        char szID[3];
        snprintf(szID, sizeof(szID), "%02d", m_module);
        dataLogger->AddDataPoint(MOD_NAME, szID, "DIn", "0x%04x",
                                 DataInt16, &m_prevDIn);
#endif
        RegisterTask(MOD_NAME, TASK_PRE_PERIODIC);

        TExit();
        return;
    }   //CommonInit

public:
    /**
     * Constructor: Create an instance of the DigitalIn object.
     */
    DigitalIn(
        void
        ): DigitalModule(GetDefaultDigitalModule())
         , m_module(GetDefaultDigitalModule())
    {
        TLevel(INIT);
        TEnter();

        CommonInit();

        TExit();
    }   //DigitalIn

    /**
     * Constructor: Create an instance of the DigitalIn object.
     * 
     * @param module Specifies the Digital module number.
     */
    DigitalIn(
        UINT8 module
        ): DigitalModule(module)
         , m_module(module)
    {
        TLevel(INIT);
        TEnterMsg(("module=%d", module));

        CommonInit();

        TExit();
    }   //DigitalIn

    /**
     * Destructor: Destroy an instance of the DigitalIn object.
     */
    ~DigitalIn(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~DigitalIn

    /**
     * This function registers a notification handler for the specified
     * digital input channels.
     *
     * @param notify Points to the DigitalInNotify object for the notification
     *        callback.
     * @param notifyMask Specifies the channel mask for the notification.
     * @param inverseMask Specifies the channel mask for inverse inputs.
     */
    bool
    RegisterNotification(
        DigitalInNotify *notify,
        UINT16           notifyMask,
        UINT16           inverseMask = 0
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnterMsg(("notify=%p,notifyMask=%x,inverseMask=%x",
                   notify, notifyMask, inverseMask));

        if (m_numNotifies < MAX_DIGITAL_CHANNELS)
        {
            m_channelMask |= notifyMask;
            m_notifyMask[m_numNotifies] = notifyMask;
            m_inverseMask[m_numNotifies] = inverseMask & notifyMask;
            m_notify[m_numNotifies] = notify;
            m_numNotifies++;
            fSuccess = true;
        }

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //RegisterNotification

    /**
     * This function unregisters a notification handler.
     *
     * @param notify Identify the DitialInNotify object to unregister its
     *        notification handler.
     */
    bool
    UnregisterNotification(
        DigitalInNotify *notify
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnterMsg(("notify=%p", notify));

        for (int i = 0; i < m_numNotifies; i++)
        {
            if (m_notify[i] == notify)
            {
                //
                // move all the subsequent handlers up one slot.
                //
                for (int j = i + 1; j < m_numNotifies; j++)
                {
                    m_notifyMask[j - 1] = m_notifyMask[j];
                    m_inverseMask[j - 1] = m_inverseMask[j];
                    m_notify[j - 1] = m_notify[j];
                }
                m_notify[m_numNotifies - 1] = NULL;
                m_notifyMask[m_numNotifies - 1] = 0;
                m_inverseMask[m_numNotifies - 1] = 0;
                m_numNotifies--;
                fSuccess = true;
            }
        }

        if (fSuccess)
        {
            //
            // Update global channel mask.
            //
            m_channelMask = 0;
            for (int i = 0; i < m_numNotifies; i++)
            {
                m_channelMask |= m_notifyMask[i];
            }
        }

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //UnregisterNotification

    /**
     * This function returns the state of the a digital input channel.
     *
     * @param channel Specifies the digital channel to get its state.
     *
     * @return Returns the state of the digital input channel.
     */
    bool
    GetChannelState(
        UINT32 channel
        )
    {
        bool state;

        TLevel(API);
        TEnterMsg(("channel=%d", channel));

        state = GetDIO(channel);

        TExitMsg(("=%x", state));
        return state;
    }   //GetChannelState

    /**
     * This function is called by the TaskMgr to check and process DigitalIn
     * events. It reads all digital input channels and compares them to their
     * previous states. If any channel has changed state, the notification
     * object is called.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPrePeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        UINT16 currDIn = GetDIO();
        if (m_numNotifies > 0)
        {
            UINT16 changedDIn = (m_prevDIn^currDIn) & m_channelMask;
            UINT16 maskDIn;
            UINT32 channel;

            while (changedDIn != 0)
            {
                //
                // maskButton contains the least significant set bit.
                //
                maskDIn = changedDIn & ~(changedDIn^-changedDIn);

                for (channel = 1; channel <= MAX_DIGITAL_CHANNELS; channel++)
                {
                    if (maskDIn == DInMask(channel))
                    {
                        break;
                    }
                }

                for (int idx = 0; idx < m_numNotifies; idx++)
                {
                    if (maskDIn & m_notifyMask[idx])
                    {
                        //
                        // Majority of the digital circuits use the convention
                        // that an input is normally pulled up to a high
                        // voltage. When an input is active, it pulls the
                        // input line to ground. Therefore, the active state
                        // is actually 0, not 1.
                        //
                        bool fActive = (currDIn & maskDIn) == 0;

                        if (m_inverseMask[idx] & maskDIn)
                        {
                            fActive = !fActive;
                        }

                        m_notify[idx]->NotifyDIn(m_module, channel, fActive);
                    }
                }

                //
                // Clear the least significant set bit.
                //
                changedDIn &= ~maskDIn;
            }
        }
        m_prevDIn = currDIn;

        TExit();
    }   //TaskPrePeriodic

};  //class DigitalIn

#endif  //ifndef _DIGITALIN_H
