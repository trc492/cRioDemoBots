#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="AnalogIn.h" />
///
/// <summary>
///     This module contains the definition and implementation of the AnalogIn
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _ANALOGIN_H
#define _ANALOGIN_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ANALOGIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "AnalogIn"

#define ANALOGINO_INVERSE       0x00000001
#ifdef _KALMANFILTER_H
#define ANALOGINO_FILTER        0x00000002
#endif

#define ANALOGINF_CALIBRATING   0x00000001

#define ANALOGIN_ZONE_LO        0
#define ANALOGIN_ZONE_MID       1
#define ANALOGIN_ZONE_HI        2

/**
 * This abstract class defines the AnalogInNotify object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the analog input events.
 */
class AnalogInNotify
{
public:
    /**
     * This function is provided by the subclass to handle a AnalogIn event.
     *
     * @param module Specifies the Analog module number.
     * @param channel Specifies the Analog channel.
     * @param zone Specifies the analog value zone.
     * @param value Specifies the analog value.
     */
    virtual
    void
    NotifyAIn(
        UINT8  module,
        UINT32 channel,
        UINT32 zone,
        INT16  value
        ) = 0;
};  //class AnalogInNotify

/**
 * This class defines and implements the AnalogIn object. It inherits the
 * AnalogChannel object from the WPI library. It added the capability of
 * detecting if the output value of an analog sensor is in one of the three
 * zones (below the low threshold, between the low and high thresholds or
 * above the high threshold). When the sensor value crosses into another zone,
 * the notification object is called. The threshold values can be set in the
 * constructor of the object or it can also be automatically determined
 * during a calibration procedure.
 */
class AnalogIn: public CoopTask,
                public AnalogChannel
{
private:
    UINT8           m_module;
    UINT32          m_channel;
    AnalogInNotify *m_notify;
    INT16           m_thresholdLo;
    INT16           m_thresholdHi;
    UINT32          m_analogInOptions;
    UINT32          m_analogInFlags;
    INT16           m_minRaw;
    INT16           m_maxRaw;
    UINT32          m_zone;
#ifdef _KALMANFILTER_H
    KalmanFilter    m_kalman;
#endif

public:
    /**
     * Constructor: Create an instance of the AnalogIn object.
     * 
     * @param module Specifies the Analog module number.
     * @param channel Specifies the Analog channel.
     * @param thresholdLo Specifies the low threshold value of the 3 zones.
     * @param thresholdHi Specifies the high threshold value of the 3 zones.
     * @param options Specifies the option flags. For example, set the INVERSE
     *        option will reverse the sense of high and low zones.
     * @param notify Points to the Notify object for AnalogIn event
     *        notification callback.
     */
    AnalogIn(
        UINT8           module,
        UINT32          channel,
        INT16           thresholdLo,
        INT16           thresholdHi,
        UINT32          options,
        AnalogInNotify *notify = NULL
        ): AnalogChannel(module, channel)
         , m_module(module)
         , m_channel(channel)
         , m_notify(notify)
         , m_thresholdLo(thresholdLo)
         , m_thresholdHi(thresholdHi)
         , m_analogInOptions(options)
         , m_analogInFlags(0)
         , m_zone(ANALOGIN_ZONE_LO)
#ifdef _KALMANFILTER_H
         , m_kalman()
#endif
    {
        TLevel(INIT);
        TEnterMsg(("Module=%d,Channel=%d,ThresholdLo=%d,ThresholdHi=%d,"
                   "options=%x,notify=%p",
                   module, channel, thresholdLo, thresholdHi, options, notify));

        RegisterTask(MOD_NAME, TASK_PRE_PERIODIC);

        TExit();
    }   //AnalogIn

    /**
     * Destructor: Destroy an instance of the AnalogIn object.
     */
    ~AnalogIn(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~AnalogIn

    /**
     * This function checks whether the AnalogIn channel is in calibration
     * mode.
     *
     * @return Returns true if the AnalogIn channel is in calibration mode,
     *         false otherwise.
     */
    bool
    IsCalibrating(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", (m_analogInFlags & ANALOGINF_CALIBRATING) != 0));
        return (m_analogInFlags & ANALOGINF_CALIBRATING) != 0;
    }   //IsCalibrating

    /**
     * This function calibrates the AnalogIn channel to determine the low and
     * high thresholds.
     *
     * @param fStart If true, starts the calibration, stop the calibration
     *        otherwise.
     */
    void
    Calibrate(
        bool fStart
        )
    {
        TLevel(API);
        TEnterMsg(("fStart=%d", fStart));

        if (fStart)
        {
           m_analogInFlags |= ANALOGINF_CALIBRATING;
           m_minRaw = 4095;     //Assuming 12-bit A/D
           m_maxRaw = 0;
        }
        else
        {
            m_analogInFlags &= ~ANALOGINF_CALIBRATING;
            INT16 zoneRange = (m_maxRaw - m_minRaw)/3;
            m_thresholdLo = m_minRaw + zoneRange;
            m_thresholdHi = m_maxRaw - zoneRange;
            TInfo(("ThresholdLo=%d,ThresholdHi=%d",
                   m_thresholdLo, m_thresholdHi));
        }

        TExit();
    }   //Calibrate

    /**
     * This function is called by the TaskMgr to check and process AnalogIn
     * channel triggers. It reads the value of an analog channels and compares
     * it against the low and high thresholds. If it crosses to a different
     * zone, the notification object is called.
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

        INT16 value = GetValue();

#ifdef _KALMANFILTER_H
        if (m_analogInOptions & ANALOGINO_FILTER)
        {
            value = (INT16)m_kalman.FilterData((double)value);
        }
#endif
        if (IsCalibrating())
        {
            if (value < m_minRaw)
            {
                m_minRaw = value;
            }
            else if (value > m_maxRaw)
            {
                m_maxRaw = value;
            }
        }
        else
        {
            UINT32 zone;
            if (value <= m_thresholdLo)
            {
                zone = (m_analogInOptions & ANALOGINO_INVERSE)?
                       ANALOGIN_ZONE_HI: ANALOGIN_ZONE_LO;
            }
            else if (value <= m_thresholdHi)
            {
                zone = ANALOGIN_ZONE_MID;
            }
            else
            {
                zone = (m_analogInOptions & ANALOGINO_INVERSE)?
                       ANALOGIN_ZONE_LO: ANALOGIN_ZONE_HI;
            }

            if (zone != m_zone)
            {
                //
                // We have crossed to another zone, let's notify somebody.
                //
                m_zone = zone;
                if (m_notify != NULL)
                {
                    m_notify->NotifyAIn(m_module, m_channel, m_zone, value);
                }
            }
        }

        TExit();
    }   //TaskPrePeriodic

};  //class AnalogIn

#endif  //ifndef _ANALOGIN_H
