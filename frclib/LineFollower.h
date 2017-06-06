#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="LineFollower.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     LineFollower class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _LINEFOLLOWER_H
#define _LINEFOLLOWER_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_LNFOLLOWER
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "LnFollower"

//
// LnFollow options.
//
#define LNFOLLOWO_MECANUM_DRIVE         0x00000001

#define LNFOLLOW_DEF_MAX_DRIVE_POWER    1.0
#define LNFOLLOW_DEF_FIND_DRIVE_POWER   0.5
#define LNFOLLOW_DEF_FIND_TURN_POWER    0.5

#define LNFOLLOW_INVALID_INPUT_VALUE    999.0

/**
 * This class defines and implements the LineFollower object. It consists
 * of a number of light sensors, a drive object and a PID controller that
 * drives the robot using the feedback of the light sensors.
 */
class LineFollower: public CoopTask
{
private:
    #define LNFOLLOWF_STARTED           0x00000001

    RobotDrive *m_drive;
    TrcPIDCtrl *m_pidCtrl;
    PIDInput   *m_pidInput;
    UINT32      m_lnFollowOptions;
    UINT32      m_lnFollowFlags;
    float       m_maxDrivePower;
    float       m_findDrivePower;
    float       m_findTurnPower;
    Event      *m_notifyEvent;
    UINT32      m_expiredTime;

public:
    /**
     * This function stops the line follower.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
        {
            m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
        }
        else
        {
            m_drive->Drive(0.0, 0.0);
        }
        m_pidCtrl->Reset();
        m_lnFollowFlags = 0;
        m_expiredTime = 0;

        TExit();
        return;
    }   //Stop

    /**
     * This function is called by TaskMgr to stop the LineFollower.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        Stop();

        TExit();
        return;
    }   //TaskStopMode

    /**
     * Constructor: Create an instance of the LineFollower object that consists
     * of a RobotDrive object, a TrcPIDCtrl object for PID turn and optionally
     * a notification object for the callback.
     *
     * @param drive Points to the RobotDrive object.
     * @param pidCtrl Points to the TrcPIDCtrl object for line following.
     * @param lnFollowOptions Specifies option flags
     */
    LineFollower(
        RobotDrive    *drive,
        TrcPIDCtrl    *pidCtrl,
        PIDInput      *pidInput,
        UINT32         lnFollowOptions = 0
        ): m_drive(drive)
         , m_pidCtrl(pidCtrl)
         , m_pidInput(pidInput)
         , m_lnFollowOptions(lnFollowOptions)
         , m_lnFollowFlags(0)
         , m_maxDrivePower(LNFOLLOW_DEF_MAX_DRIVE_POWER)
         , m_findDrivePower(LNFOLLOW_DEF_FIND_DRIVE_POWER)
         , m_findTurnPower(LNFOLLOW_DEF_FIND_TURN_POWER)
         , m_notifyEvent(NULL)
         , m_expiredTime(0)
    {
        TLevel(INIT);
        TEnterMsg(("drive=%p,pidCtrl=%p,options=%x",
                   drive, pidCtrl, lnFollowOptions));

        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_PERIODIC);

        TExit();
    }   //LineFollower

    /**
     * Destructor: Destroy an instance of the LineFollower object.
     */
    virtual
    ~LineFollower(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        Stop();
        UnregisterTask();

        TExit();
    }   //~LineFollower

    /**
     * This function starts the line follower.
     *
     * @param targetValue Specifies the center sensor value.
     * @param maxDrivePower Specifies the maximum drive power.
     * @param findDrivePower Specifies the maximum drive power when finding
     *        the target.
     * @param findTurnPower Specifies the maximum turn power when finding
     *        the target.
     * @param notifyEvent Specifies the optional notification event.
     * @param timeout Specific the timeout in msec. No timeout if zero.
     */
    void
    LineFollowStart(
        float  targetValue,
        float  maxDrivePower = LNFOLLOW_DEF_MAX_DRIVE_POWER,
        float  findDrivePower = LNFOLLOW_DEF_FIND_DRIVE_POWER,
        float  findTurnPower = LNFOLLOW_DEF_FIND_TURN_POWER,
        Event *notifyEvent = NULL,
        UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("target=%f,maxDrive=%f,findDrive=%f,findTurn=%f,"
                   "notifyEvent=%p,timeout=%d",
                   targetValue, maxDrivePower, findDrivePower, findTurnPower,
                   notifyEvent, timeout));

        m_maxDrivePower = maxDrivePower;
        m_findDrivePower = findDrivePower;
        m_findTurnPower = findTurnPower;
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

        m_pidCtrl->SetTarget(targetValue, 0.0);
        m_lnFollowFlags |= LNFOLLOWF_STARTED;

        TExit();
        return;
    }   //LineFollowStart

    /**
     * This function is called by the TaskMgr to update the LineFollower state
     * and check for completion.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        float currValue = m_pidInput->GetInput(m_pidCtrl);

        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if (m_lnFollowFlags & LNFOLLOWF_STARTED)
        {
            if ((m_expiredTime != 0) && (GetMsecTime() >= m_expiredTime))
            {
                //
                // Detected stop condition.
                //
                Stop();
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                }
            }
            else if (currValue != LNFOLLOW_INVALID_INPUT_VALUE)
            {
                float turnPower = m_pidCtrl->CalcPIDOutput(currValue);
                float drivePower = m_maxDrivePower*(1.0 - fabs(turnPower));

                if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(drivePower, 0.0, turnPower);
                }
                else
                {
                    m_drive->ArcadeDrive(drivePower, turnPower);
                }
            }
            else
            {
                //
                // We lost the line. Find it again.
                //
                if (m_lnFollowOptions & LNFOLLOWO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(m_findDrivePower,
                                                0.0,
                                                m_findTurnPower);
                }
                else
                {
                    m_drive->ArcadeDrive(m_findDrivePower,
                                         m_findTurnPower);
                }
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodic

};  //class LineFollower

#endif  //ifndef _LINEFOLLOWER_H
