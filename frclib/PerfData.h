#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="PerfData.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     PerfData class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _PERFDATA_H
#define _PERFDATA_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PERFDATA
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "PerfData"

/**
 * This class defines and implements the PerfData object. The PerfData object
 * represents a performance data point. The data point tracks the minimum,
 * maximum and total time an operation takes in a perf period.
 */
class PerfData
{
private:
    bool    m_fPeriodStarted;
    UINT32  m_periodStartTime;
    UINT32  m_perfStartTime;

    UINT32  m_periodTotalTime;
    UINT32  m_perfTotalTime;
    UINT32  m_perfMinTime;
    UINT32  m_perfMaxTime;
    UINT32  m_perfCount;

public:
    /**
     * Constructor: Create an instance of the PerfData object.
     */
    PerfData(
        void
        ): m_fPeriodStarted(false)
         , m_periodStartTime(0)
         , m_perfStartTime(0)
         , m_periodTotalTime(0)
         , m_perfTotalTime(0)
         , m_perfMinTime(0)
         , m_perfMaxTime(0)
         , m_perfCount(0)
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //PerfData

    /**
     * Destructor: Destroy an instance of the PerfData object.
     */
    virtual
    ~PerfData(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~PerfData

    /**
     * This function starts the perfdata collection period.
     */
    void
    StartPerfPeriod(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_periodTotalTime = 0;
        m_perfTotalTime = 0;
        m_perfMinTime = 0;
        m_perfMaxTime = 0;
        m_perfCount = 0;
        m_periodStartTime = GetUsecTime();
        m_fPeriodStarted = true;

        TExit();
        return;
    }   //StartPerfPeriod

    /**
     * This function ends the perfdata collection period.
     */
    void
    EndPerfPeriod(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_fPeriodStarted = false;
        m_periodTotalTime = GetUsecTime() - m_periodStartTime;

        TExit();
        return;
    }   //EndPerfPeriod

    /**
     * This function records the start time of the operation.
     */
    void
    StartPerf(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_fPeriodStarted)
        {
            m_perfStartTime = GetUsecTime();
        }

        TExit();
        return;
    }   //StartPerf

    /**
     * This function records the end time of the operation.
     */
    void
    EndPerf(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_fPeriodStarted)
        {
            UINT32 perfTime = GetUsecTime() - m_perfStartTime;

            m_perfTotalTime += perfTime;
            m_perfCount++;
            if ((m_perfMinTime == 0) && (m_perfMaxTime == 0))
            {
                m_perfMinTime = perfTime;
                m_perfMaxTime = perfTime;
            }
            else if (perfTime < m_perfMinTime)
            {
                m_perfMinTime = perfTime;
            }
            else if (perfTime > m_perfMaxTime)
            {
                m_perfMaxTime = perfTime;
            }
        }

        TExit();
        return;
    }   //EndPerf

    /**
     * This function returns the min perf time of the operation.
     *
     * @return Returns the minimum perf time of the operation in usec.
     */
    UINT32
    GetPerfMinTime(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", m_perfMinTime));
        return m_perfMinTime;
    }   //GetPerfMinTime

    /**
     * This function returns the max perf time of the operation.
     *
     * @return Returns the maximum perf time of the operation in usec.
     */
    UINT32
    GetPerfMaxTime(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", m_perfMaxTime));
        return m_perfMaxTime;
    }   //GetPerfMaxTime

    /**
     * This function returns the total perf time of the operation for
     * the period.
     *
     * @return Returns the total perf time of the operation for the period.
     */
    UINT32
    GetPerfTotalTime(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", m_perfTotalTime));
        return m_perfTotalTime;
    }   //GetPerfTotalTime

    /**
     * This function returns the total perf count of the operation for
     * the period.
     *
     * @return Returns the total perf count of the operation for the period.
     */
    UINT32
    GetPerfCount(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", m_perfCount));
        return m_perfCount;
    }   //GetPerfCount

    /**
     * This function returns the total period time.
     *
     * @return Returns the total period time in usec.
     */
    UINT32
    GetPerfPeriodTime(
        void
        )
    {
        TLevel(API);
        TEnter();
        TExitMsg(("=%d", m_periodTotalTime));
        return m_periodTotalTime;
    }   //GetPerfPeriodTime

};  //class PerfData

#endif  //ifndef _PERFDATA_H
