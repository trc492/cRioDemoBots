#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DSEnhDin.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DSEnhDin class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DSENHDIN_H
#define _DSENHDIN_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DSENHDIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DSEnhDin"

#define MAX_EIODIN_CHANNELS     16
#define EIODinMask(n)           (1 << ((n) - 1))

#define DSCHN_BCD_LOW_DIGIT     1
#define DSCHN_BCD_HIGH_DIGIT    5

#define DSCHN_BTN1              9
#define DSCHN_BTN2              10
#define DSCHN_BTN3              11
#define DSCHN_BTN4              12
#define DSCHN_SW1               13
#define DSCHN_SW2               14
#define DSCHN_SW3               15
#define DSCHN_SW4               16

/**
 * This abstract class defines the EIODinNotify object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the Driver Station Enhanced IO digital input events.
 */
class EIODinNotify
{
public:
    /**
     * This function is provided by the subclass to handle a Driver Station
     * Enhanced IO Digital Input Event notification.
     *
     * @param channel Specifies the Enhanced IO Digital Input channel (0-15)
     *        that generated the event.
     * @param state Specifies the state of the digital input channel (0 or 1).
     */
    virtual
    void
    NotifyEIODin(
        UINT32 channel,
        UINT32 state
        ) = 0;
};  //class EIODinNotify

/**
 * This class defines and implements the DSEnhDin object. This object provides
 * the support of detecting EnhancedIO Digital Input events and calling the
 * notification object. It also supports using 4 digital channels for a BCD
 * digital switch.
 */
class DSEnhDin: public CoopTask
{
private:
    DriverStationEnhancedIO& m_enhancedIO;
    UINT16          m_notifyMask[MAX_EIODIN_CHANNELS];
    EIODinNotify   *m_notify[MAX_EIODIN_CHANNELS];
    int             m_numNotifies;
    UINT16          m_channelMask;
    UINT16          m_prevEIODin;

public:
    /**
     * Constructor: Create an instance of the object.
     */
    DSEnhDin(
        void
        ): m_enhancedIO(DriverStation::GetInstance()->GetEnhancedIO())
    {
        TLevel(INIT);
        TEnter();

        //
        // Must call GetFirmwareVersion to enable API usage and forced
        // controller to Enhanced IO mode.
        //
        m_enhancedIO.GetFirmwareVersion();

        for (int i = 0; i < MAX_EIODIN_CHANNELS; i++)
        {
            //
            // Configure all digital I/O channels to input pull-up mode.
            //
            m_enhancedIO.SetDigitalConfig(
                i + 1,
                DriverStationEnhancedIO::kInputPullUp);
            m_notifyMask[i] = 0;
            m_notify[i] = NULL;
        }
        m_numNotifies = 0;
        m_channelMask = 0;
        m_prevEIODin = m_enhancedIO.GetDigitals();

#ifdef _LOGDATA_DSENHDIN
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, "", "EIODin", "0x%04x",
                                 DataInt16, &m_prevEIODin);

#endif

        RegisterTask(MOD_NAME, TASK_PRE_PERIODIC);

        TExit();
    }   //DSEnhDin

    /**
     * Destructor: Destroy an instance of the object.
     */
    virtual
    ~DSEnhDin(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~DSEnhDin

    /**
     * This function registers a notification handler for the specified
     * digital input channels.
     *
     * @param notify Points to the EIODinNotify object for the notification
     *        callback.
     * @param notifyMask Specifies the channel mask for the notification.
     */
    bool
    RegisterNotification(
        EIODinNotify *notify,
        UINT16        notifyMask
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnterMsg(("notify=%p,notifyMask=%x", notify, notifyMask));

        if (m_numNotifies < MAX_EIODIN_CHANNELS)
        {
            m_channelMask |= notifyMask;
            m_notifyMask[m_numNotifies] = notifyMask;
            m_notify[m_numNotifies] = notify;
            m_numNotifies++;
            //
            // Send the initial notifications so the robot state is in sync
            // with the states of the switches.
            //
            for (UINT32 channel = 1; channel <= MAX_EIODIN_CHANNELS; channel++)
            {
                UINT32 channelMask = EIODinMask(channel);

                if (notifyMask & channelMask)
                {
                    notify->NotifyEIODin(channel, GetChannelState(channel));
                }
            }
            
            fSuccess = true;
        }

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //RegisterNotification

    /**
     * This function unregisters a notification handler.
     *
     * @param notify Identify the EIODinNotify object to unregister its
     *        notification handler.
     */
    bool
    UnregisterNotification(
        EIODinNotify *notify
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
                    m_notify[j - 1] = m_notify[j];
                }
                m_notify[m_numNotifies - 1] = NULL;
                m_notifyMask[m_numNotifies - 1] = 0;
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
     * This function returns the state of the a EIODin channel.
     *
     * @param channel Specifies the EIODin channel to get its state.
     *
     * @return Returns the state of the digital input channel.
     */
    UINT32
    GetChannelState(
        UINT32 channel
        )
    {
        UINT32 state;

        TLevel(API);
        TEnterMsg(("channel=%d", channel));

        state = (m_enhancedIO.GetDigitals() >> (channel - 1)) & 0x01;

        TExitMsg(("=%x", state));
        return state;
    }   //GetChannelState

    /**
     * This function returns the BCD value represented by the BCD switch
     * state for one digit. A BCD digit consists of four digital input
     * channels. It is assumed that the the BCD digit is connected to
     * four consecutive digital input channels.
     *
     * @param lowChannel Specifies the lowest digital input channel that the
     *        BCD switch is connected to.
     *
     * @return Returns the state of the BCD switch.
     */
    UINT8
    GetBCDDigit(
        UINT32 lowChannel
        )
    {
        UINT8 bcdSwitch;

        TLevel(API);
        TEnterMsg(("lowChannel=%d", lowChannel));

        bcdSwitch = (~m_enhancedIO.GetDigitals() >> (lowChannel - 1)) & 0x0f;

        TExitMsg(("=%x", bcdSwitch));
        return bcdSwitch;
    }   //GetBCDDigit

    /**
     * This function is called by the TaskMgr to check and process the
     * Digital Input events.
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

        UINT16 currEIODin = m_enhancedIO.GetDigitals();
        if (m_numNotifies > 0)
        {
            UINT16 changedDin = (m_prevEIODin^currEIODin) & m_channelMask;
            UINT16 maskDin;
            UINT32 channel;

            while (changedDin != 0)
            {
                //
                // maskDin contains the least significant set bit.
                //
                maskDin = changedDin & ~(changedDin^-changedDin);

                for (channel = 1; channel <= MAX_EIODIN_CHANNELS; channel++)
                {
                    if (maskDin == EIODinMask(channel))
                    {
                        break;
                    }
                }

                for (int idx = 0; idx < m_numNotifies; idx++)
                {
                    if (maskDin & m_notifyMask[idx])
                    {
                        m_notify[idx]->NotifyEIODin(
                            channel,
                            (currEIODin & maskDin)? 1: 0);
                    }
                }

                //
                // Clear the least significant set bit.
                //
                changedDin &= ~maskDin;
            }
        }
        m_prevEIODin = currEIODin;

        TExit();
    }   //TaskPrePeriodic

};  //class DSEnhDin

#endif  //ifndef _DSENHDIN_H
