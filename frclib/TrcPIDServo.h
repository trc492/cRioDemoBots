#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDServo.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDServo class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDSERVO_H
#define _TRCPIDSERVO_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDSERVO
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDServo"

/**
 * This class defines and implements the TrcPIDServo object. It supports the
 * capability of PID controlled movement by a servo. Servo motors have some
 * kind of PID control built-in. When you set the servo to a given angle,
 * it will go there and slow down when approaching target angle. However,
 * you can't control the PID constants precisely. So, the movement may cause
 * oscillation. This module provides external PID control simulating a
 * regular motor. There are two issues we need to solve. First, servo motor
 * doesn't really have power control. Fortunately, the TrcServo module
 * simulates power control by doing step rate. Secondly, servo motor does
 * not provide feedback on its current location which is crucial for PID
 * control. We may simulate this by approximating its location by the amount
 * of time it has past and the step rate it was moving at.
 */
class TrcPIDServo
    : public CoopTask
{
private:
    //
    // Flags
    //
    #define PIDSERVOF_PIDMODE_ON        0x00000001
    #define PIDSERVOF_HOLD_TARGET       0x00000002

    TrcServo   *m_servo;
    TrcPIDCtrl *m_pidCtrl;
    PIDInput   *m_pidInput;
    UINT32      m_pidServoFlags;
    Event      *m_notifyEvent;

public:
    /**
     * This function resets the PID Servo object.
     */
    void
    Reset(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_servo->Stop();
        if (m_pidCtrl != NULL)
        {
            m_pidCtrl->Reset();
        }

        m_pidServoFlags = 0;
        m_notifyEvent = NULL;

        TExit();
        return;
    }   //Reset

    /**
     * This function is called by TaskMgr to stop the PID servo.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        Reset();

        TExit();
        return;
    }   //TaskStopMode

    /**
     * Constructor: Create an instance of the TrcPIDServo object that consists
     * of a TrcServo object, a TrcPIDCtrl object and optionally a
     * notification event for signaling completion.
     *
     * @param servo Points to the TrcServo object.
     * @param pidCtrl Points to the TrcPIDCtrl object.
     * @param pidInput Specifies the PIDInput object.
     */
    TrcPIDServo(
        TrcServo   *servo,
        TrcPIDCtrl *pidCtrl,
        PIDInput   *pidInput
        ): m_servo(servo)
         , m_pidCtrl(pidCtrl)
         , m_pidInput(pidInput)
         , m_pidServoFlags(0)
         , m_notifyEvent(NULL)
    {
        TLevel(INIT);
        TEnterMsg(("servo=%p,pidCtrl=%p,pidInput=%p",
                   servo, pidCtrl, pidInput));

        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_PERIODIC);

        TExit();
    }   //TrcPIDServo

    /**
     * Destructor: Destroy an instance of the TrcPIDServo object.
     */
    virtual
    ~TrcPIDServo(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        Reset();
        UnregisterTask();

        TExit();
    }   //~TrcPIDServo

    /**
     * This function sets servo motor power.
     *
     *  @param power Specifies the power level to set the motor.
     */
    void
    SetPower(
        float power
        )
    {
        TLevel(API);
        TEnterMsg(("power=%f", power));

        if (m_pidServoFlags & PIDSERVOF_PIDMODE_ON)
        {
            //
            // There was a previous unfinished PID operation, cancel it.
            // Don't stop the motor to prevent jerkiness.
            //
            Reset();
        }
        m_servo->SetPower(power);

        TExit()
        return;
    }   //SetPower

    /**
     * This function sets PID servo target with the given setpoint.
     *
     * @param setPoint Specifies the target setPoint.
     * @param notifyEvent Specifies the event to notifying for completion.
     */
    void
    SetTarget(
        float  setPoint,
        bool   fHoldTarget = false,
        Event *notifyEvent = NULL
        )
    {
        TLevel(API);
        TEnterMsg(("setPoint=%f,event=%p", setPoint, notifyEvent));

        if (m_pidServoFlags & PIDSERVOF_PIDMODE_ON)
        {
            //
            // Previous SetTarget has not been completed, cancel it.
            //
            Reset();
        }

        m_notifyEvent = notifyEvent;
        m_pidCtrl->SetTarget(setPoint, m_pidInput->GetInput(m_pidCtrl));
        m_pidServoFlags = PIDSERVOF_PIDMODE_ON;
        if (fHoldTarget)
        {
            m_pidServoFlags |= PIDSERVOF_HOLD_TARGET;
        }

        TExit();
        return;
    }   //SetTarget

    /**
     * This function is called by the TaskMgr to update the PIDServo state
     * and check for completion.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if (m_pidServoFlags & PIDSERVOF_PIDMODE_ON)
        {
            if (!(m_pidServoFlags & PIDSERVOF_HOLD_TARGET) &&
                m_pidCtrl->OnTarget())
            {
                Reset();
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                }
            }
            else
            {
                float power = m_pidCtrl->CalcPIDOutput(
                                        m_pidInput->GetInput(m_pidCtrl));

                m_servo->SetPower(power);
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodic

};  //class TrcPIDServo

#endif  //ifndef _TRCPIDSERVO_H
