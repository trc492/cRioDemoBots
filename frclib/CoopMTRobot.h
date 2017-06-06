#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CoopMTRobot.h" />
///
/// <summary>
///     This module contains the definitions of the CoopMTRobot class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _COOPMTROBOT_H
#define _COOPMTROBOT_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_COOPMTROBOT
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "CoopMTRobot"

#define GETLOOPPERIOD()         ((m_loopPeriod == 0.0)?             \
                                 ((double)m_periodPacket)/1000.0:   \
                                 m_loopPeriod)
/**
 * This class defines and implements the CoopMTRobot object. The CoopMTRobot
 * object implements a cooperative multitasking robot. Different subsystems
 * register themselves as CoopTasks. CoopMTRobot uses the TaskMgr to task
 * switch between different subsystem tasks at various points in the robot
 * loop. This basically simulates a cooperative multitasking scheduler that
 * task switches between them in different modes.
 */
class CoopMTRobot: public RobotBase
{
private:
    DriverStationLCD   *m_dsLCD;
    TaskMgr            *m_taskMgr;
#ifdef _ENABLE_DATALOGGER
    DataLogger         *m_dataLogger;
#endif
    double              m_loopPeriod;
    Timer               m_loopTimer;
    UINT32              m_periodPacket;
    UINT32              m_prevTime;

    /**
     * This function is called to determine if the next period has
     * began so that the periodic functions should be called.
     * If m_loopPeriod > 0.0, call the periodic function every
     * m_loopPeriod as compared to Timer.Get(). If m_loopPeriod == 0.0,
     * call the periodic functions whenever a packet is received
     * from the Driver Station, or about every 20 ms (50 Hz).
     *
     * @return Returns true if the next period is ready, false otherwise.
     */
    bool
    NextPeriodReady(
        void
        )
    {
        bool rc;

        TLevel(HIFREQ);
        TEnter();

        if (m_loopPeriod > 0.0)
        {
            rc = m_loopTimer.HasPeriodPassed(m_loopPeriod);
        }
        else
        {
            rc = m_ds->IsNewControlData();
            if (rc)
            {
                //
                // Determine the packet interval.
                //
                UINT32 currTime = GetMsecTime();
                m_periodPacket = currTime - m_prevTime;
                m_prevTime = currTime;
            }
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //NextPeriodReady

public:
    /*
     * The default period for the periodic function calls (seconds)
     * Setting the period to 0.0 will cause the periodic functions to
     * follow the Driver Station packet rate of about 50Hz.
     */
    static const double kDefaultPeriod = 0.0;

    /**
     * This function is called one time to do robot-wide initialization.
     */
    virtual
    void
    RobotInit(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //RobotInit

#ifdef _PERFDATA_LOOP
    /**
     * This function is called at the beginning of each robot loop.
     */
    virtual
    void
    StartPerfDataLoop(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExit();
    }   //StartPerfDataLoop

    /**
     * This function is called at the end of each robot loop.
     */
    virtual
    void
    EndPerfDataLoop(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExit();
    }   //EndPerfDataLoop
#endif

    /**
     * This function is called before entering disabled mode.
     */
    virtual
    void
    DisabledStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //DisabledStart

    /**
     * This function is called before exiting disabled mode.
     */
    virtual
    void
    DisabledStop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //DisabledStop

    /**
     * This function is called periodically at fixed intervals in disabled
     * mode.
     */
    virtual
    void
    DisabledPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //DisabledPeriodic

    /**
     * This function is called continuously in disabled mode.
     */
    virtual
    void
    DisabledContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //DisabledContinuous

    /**
     * This function is called before entering autonomous mode.
     */
    virtual
    void
    AutonomousStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //AutonomousStart

    /**
     * This function is called before exiting autonomous mode.
     */
    virtual
    void
    AutonomousStop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //AutonomousStop

    /**
     * This function is called periodically at fixed intervals in autonomous
     * mode.
     */
    virtual
    void
    AutonomousPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //AutonomousPeriodic

    /**
     * This function is called continuously in autonomous mode.
     */
    virtual
    void
    AutonomousContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //AutonomousContinuous

    /**
     * This function is called before entering teleop mode.
     */
    virtual
    void
    TeleOpStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TeleOpStart

    /**
     * This function is called before exiting teleop mode.
     */
    virtual
    void
    TeleOpStop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TeleOpStop

    /**
     * This function is called periodically at fixed intervals in teleop mode.
     */
    virtual
    void
    TeleOpPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TeleOpPeriodic

    /**
     * This function is called continuously in teleop mode.
     */
    virtual
    void
    TeleOpContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TeleOpContinuous

    /**
     * This function is called before entering test mode.
     */
    virtual
    void
    TestStart(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TestStart

    /**
     * This function is called before exiting test mode.
     */
    virtual
    void
    TestStop(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //TestStop

    /**
     * This function is called periodically at fixed intervals in test mode.
     */
    virtual
    void
    TestPeriodic(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TestPeriodic

    /**
     * This function is called continuously in test mode.
     */
    virtual
    void
    TestContinuous(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        taskDelay(1);

        TExit();
    }   //TestContinuous

    /**
     * This function sets the period for the periodic functions.
     *
     * @param period The period of the periodic function calls.
     *        0.0 means sync to driver station control data.
     */
    void
    SetPeriod(
        double period
        )
    {
        TLevel(API);
        TEnterMsg(("period=%f", period));

        if (period != 0.0)
        {
            //
            // Not sync with DS, so start the timer for the main loop.
            //
            m_loopTimer.Reset();
            m_loopTimer.Start();
        }
        else
        {
            //
            // Sync with DS, don't need timer.
            //
            m_loopTimer.Stop();
        }
        m_loopPeriod = period;

        TExit();
    }   //SetPeriod
    
    /**
     * This function gets the period for the periodic functions.
     *
     * @return Returns period of the periodic function calls.
     */
    double
    GetPeriod(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double period = GETLOOPPERIOD();
        TExitMsg(("=%f", period));
        return period;
    }   //GetPeriod

    /**
     * This function gets the number of loops per second for the
     * main loop.
     *
     * @return Returns frequency of the periodic function calls.
     */
    double
    GetLoopsPerSec(
        void
        )
    {
        double freq;

        TLevel(API);
        TEnter();

        freq = 1.0/GETLOOPPERIOD();

        TExitMsg(("=%f", freq));
        return freq;
    }   //GetLoopsPerSec

    /**
     * Start a competition.
     * This specific StartCompetition() implements "main loop" behavior like
     * that of the FRC control system in 2008 and earlier, with a primary
     * (slow) loop that is called periodically, and a "fast loop" (a.k.a.
     * "spin loop") that is called as fast as possible with no delay between
     * calls. This code needs to track the order of the field starting to
     * ensure that everything happens in the right order. Repeatedly run the
     * correct method, either Autonomous or OperatorControl when the robot is
     * enabled. After running the correct method, wait for some state to
     * change, either the other mode starts or the robot is disabled. Then go
     * back and wait for the robot to be enabled again.
     */
    void
    StartCompetition(
        void
        )
    {
        TLevel(API);
        TEnter();

        const char *runModes[] = {"Disabled", "Auto", "TeleOp", "Test"};
        UINT32 mode = MODE_DISABLED;
        UINT32 cntLoops = 0;
        UINT32 timeSliceStart = 0;
        UINT32 timeSliceUsed = 0;
        UINT32 periodStartTime = GetMsecTime();
        
        //
        // Disable the watchdog while doing initialization.
        //
        GetWatchdog().SetEnabled(false);

        TEnable(true);

        //
        // One-time robot initialization.
        //
#ifdef PROGRAM_NAME
        char szHostName[MAXHOSTNAMELEN];
        gethostname(szHostName, ARRAYSIZE(szHostName));

        printf(ESC_PREFIX SGR_FG_BLACK
               ESC_SEP SGR_BG_WHITE
               ESC_SUFFIX
               "\n****************************************\n"
               "    Name: %s\n"
               " Program: %s\n"
               "Compiled: %s, %s"
               "\n****************************************\n"
               ESC_NORMAL,
               szHostName, PROGRAM_NAME, __DATE__, __TIME__);
#endif
        RobotInit();

        //TEnable(false);

        //
        // Set normal watchdog timeout.
        //
        GetWatchdog().SetExpiration(0.5);
        GetWatchdog().SetEnabled(true);

        //
        // Loop forever, calling the appropriate mode-dependent functions.
        //
        while (true)
        {
            GetWatchdog().Feed();
            TPeriodStart();

            m_dsLCD->PrintfLine(
                DriverStationLCD::kUser_Line1, "[%s: %7.3f]",
                runModes[mode],
                (float)(GetMsecTime() - periodStartTime)/1000.0);
#ifdef _PERFDATA_LOOP
            StartPerfDataLoop();
#endif
            switch (mode)
            {
                case MODE_DISABLED:
                    if (IsEnabled())
                    {
                        if (IsAutonomous())
                        {
                            mode = MODE_AUTONOMOUS;
                            periodStartTime = GetMsecTime();
                            AutonomousStart();
                        }
                        else if (IsOperatorControl())
                        {
                            mode = MODE_TELEOP;
                            periodStartTime = GetMsecTime();
                            TeleOpStart();
                        }
                        else if (IsTest())
                        {
                            mode = MODE_TEST;
                            periodStartTime = GetMsecTime();
                            TestStart();
                        }
                        m_taskMgr->TaskStartModeAll(mode);
                    }
                    else
                    {
                        if (NextPeriodReady())
                        {
                            FRC_NetworkCommunication_observeUserProgramDisabled();
                            timeSliceStart = GetMsecTime();
                            m_taskMgr->TaskPrePeriodicAll(mode);
                            DisabledPeriodic();
                            m_taskMgr->TaskPostPeriodicAll(mode);
                            timeSliceUsed = GetMsecTime() - timeSliceStart;
                            cntLoops++;
                            if ((float)timeSliceUsed/1000.0 > GETLOOPPERIOD())
                            {
                                //
                                // Execution time exceeds the loop period.
                                //
                                TWarn(("Disabled execution takes too long (%d/%d ms)",
                                       timeSliceUsed,
                                       (int)(GETLOOPPERIOD()*1000)));
                            }
                        }
                    }
                    break;

                case MODE_AUTONOMOUS:
                    if (IsEnabled() && IsAutonomous())
                    {
                        if (NextPeriodReady())
                        {
                            FRC_NetworkCommunication_observeUserProgramAutonomous();
                            timeSliceStart = GetMsecTime();
                            m_taskMgr->TaskPrePeriodicAll(mode);
                            AutonomousPeriodic();
                            m_taskMgr->TaskPostPeriodicAll(mode);
                            timeSliceUsed = GetMsecTime() - timeSliceStart;
                            cntLoops++;
                            if ((float)timeSliceUsed/1000.0 > GETLOOPPERIOD())
                            {
                                //
                                // Execution time exceeds the loop period.
                                //
                                TWarn(("Autonomous execution takes too long (%d/%d ms)",
                                       timeSliceUsed,
                                       (int)(GETLOOPPERIOD()*1000)));
                            }
                        }
                        m_taskMgr->TaskPreContinuousAll(mode);
                        AutonomousContinuous();
                        m_taskMgr->TaskPostContinuousAll(mode);
                    }
                    else
                    {
                        AutonomousStop();
                        m_taskMgr->TaskStopModeAll(mode);
                        mode = MODE_DISABLED;
                        periodStartTime = GetMsecTime();
                    }
                    break;

                case MODE_TELEOP:
                    if (IsEnabled() && IsOperatorControl())
                    {
                        if (NextPeriodReady())
                        {
                            FRC_NetworkCommunication_observeUserProgramTeleop();
                            timeSliceStart = GetMsecTime();
                            m_taskMgr->TaskPrePeriodicAll(mode);
                            TeleOpPeriodic();
                            m_taskMgr->TaskPostPeriodicAll(mode);
                            timeSliceUsed = GetMsecTime() - timeSliceStart;
                            cntLoops++;
                            if ((float)timeSliceUsed/1000.0 > GETLOOPPERIOD())
                            {
                                //
                                // Execution time exceeds the loop period.
                                //
                                TWarn(("TeleOp execution takes too long (%d/%d ms)",
                                       timeSliceUsed,
                                       (int)(GETLOOPPERIOD()*1000)));
                            }
                        }
                        m_taskMgr->TaskPreContinuousAll(mode);
                        TeleOpContinuous();
                        m_taskMgr->TaskPostContinuousAll(mode);
                    }
                    else
                    {
                        TeleOpStop();
                        m_taskMgr->TaskStopModeAll(mode);
                        mode = MODE_DISABLED;
                        periodStartTime = GetMsecTime();
                    }
                    break;

                case MODE_TEST:
                    if (IsEnabled() && IsTest())
                    {
                        if (NextPeriodReady())
                        {
                            FRC_NetworkCommunication_observeUserProgramTest();
                            timeSliceStart = GetMsecTime();
                            m_taskMgr->TaskPrePeriodicAll(mode);
                            TestPeriodic();
                            m_taskMgr->TaskPostPeriodicAll(mode);
                            timeSliceUsed = GetMsecTime() - timeSliceStart;
                            cntLoops++;
                            if ((float)timeSliceUsed/1000.0 > GETLOOPPERIOD())
                            {
                                //
                                // Execution time exceeds the loop period.
                                //
                                TWarn(("Test execution takes too long (%d/%d ms)",
                                       timeSliceUsed,
                                       (int)(GETLOOPPERIOD()*1000)));
                            }
                        }
                        m_taskMgr->TaskPreContinuousAll(mode);
                        TestContinuous();
                        m_taskMgr->TaskPostContinuousAll(mode);
                    }
                    else
                    {
                        TestStop();
                        m_taskMgr->TaskStopModeAll(mode);
                        mode = MODE_DISABLED;
                        periodStartTime = GetMsecTime();
                    }
                    break;
            }
#ifdef _ENABLE_DATALOGGER
            m_dataLogger->LoggerTask();
#endif
#ifdef _PERFDATA_LOOP
            EndPerfDataLoop();
#endif
            TPeriodEnd();
            m_dsLCD->UpdateLCD();
        }

        TExit();
    }   //StartCompetition

protected:
    /**
     * Constructor for the TrcRobot class.
     */
    CoopMTRobot(
        void
        ): m_loopPeriod(kDefaultPeriod)
         , m_periodPacket(0)
         , m_prevTime(0)
    {
        TLevel(INIT);
        TEnter();

        m_dsLCD = DriverStationLCD::GetInstance();
        m_taskMgr = TaskMgr::GetInstance();
#ifdef _ENABLE_DATALOGGER
        m_dataLogger = DataLogger::GetInstance();
#endif
        m_watchdog.SetEnabled(false);

        TExit();
    }   //CoopMTRobot

    /**
     * Destructor for the TrcRobot class.
     */
    ~CoopMTRobot(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        TaskMgr::DeleteInstance();
#ifdef _ENABLE_DATALOGGER
        m_dataLogger->DeleteInstance();
        m_dataLogger = NULL;
#endif

        TExit();
    }   //~CoopMTRobot

};  //class CoopMTRobot

#endif  //ifndef _COOPMTROBOT_H
