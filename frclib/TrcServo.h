#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcServo.h" />
///
/// <summary>
///     This module contains the definition and implementation of the TrcServo
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCSERVO_H
#define _TRCSERVO_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SERVO
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcServo"

#define SERVOO_REVERSE          0x00000001
#define SERVOF_STEP_ENABLED     0x00000001

#define SERVO_MIN_ANGLE         0.0
#define SERVO_MAX_ANGLE         170.0
#define SERVO_ANGLE_RANGE       (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)

/**
 * This class defines the TrcServo object. It inherits the WPILib Servo object.
 * In addition to the Servo functionality, it provides a way to set the servo
 * angle with a step rate. It is useful for controlling a servo motor with
 * a joystick so that the joystick value controls the step rate of the
 * servo motor.
 */
class TrcServo
    : public CoopTask
    , public Servo
{
private:
    float   m_angleLowLimit;
    float   m_angleHighLimit;
    float   m_maxStepRate;
    UINT32  m_options;
    float   m_currAngle;
    float   m_targetAngle;
    float   m_currStepRate;
    UINT32  m_prevTime;
    UINT32  m_servoFlags;

public:
    /**
     * This function overrides the function in Servo class and will translate
     * the servo position value according to the SERVOO_REVERSE option flag.
     *
     * @return Position from 0.0 to 1.0.
     */
    float
    Get(
        void
        )
    {
        float value = GetPosition();

        TLevel(API);
        TEnter();

        if (m_options & SERVOO_REVERSE)
        {
            value = 1.0 - value;
        }

        TExitMsg(("=%f", value));
        return value;
    }   //Get

    /**
     * This function overrides the function in Servo class and will translate
     * the servo position value according to the SERVOO_REVERSE option flag.
     *
     * @param value Specifies position value from 0.0 to 1.0.
     */
    void
    Set(
        float value
        )
    {
        TLevel(API);
        TEnterMsg(("value=%f", value));

        if (m_options & SERVOO_REVERSE)
        {
            value = 1.0 - value;
        }
        SetPosition(value);

        TExit();
        return;
    }   //Set

    /**
     * This function overrides the function in Servo class and will translate
     * the servo angle value according to the SERVOO_REVERSE option flag.
     *
     * @return Angle in degrees to which the servo is set.
     */
    float
    GetAngle(
        void
        )
    {
        TLevel(API);
        TEnter();

        float angle = Get()*SERVO_ANGLE_RANGE + SERVO_MIN_ANGLE;

        TExitMsg(("=%f", angle));
        return angle;
    }   //GetAngle

    /**
     * This function overrides the function in Servo class and will translate
     * the servo angle value according to the SERVOO_REVERSE option flag.
     *
     * @param value Specifies position value from 0.0 to 1.0.
     */
    void
    SetAngle(
        float angle
        )
    {
        TLevel(API);
        TEnterMsg(("angle=%f", angle));

        if (angle < m_angleLowLimit)
        {
            angle = m_angleLowLimit;
        }
        else if (angle > m_angleHighLimit)
        {
            angle = m_angleHighLimit;
        }
        Set(((float)(angle - SERVO_MIN_ANGLE))/SERVO_ANGLE_RANGE);

        TExit();
        return;
    }   //SetAngle

private:
    /**
     * This function is called by all the TrcServo constructors to do common
     * initialization.
     *
     * @param initAngle Specifies the initial angle to set the servo to.
     *        If the value is negative, it means no need to set initial
     *        angle.
     * @param angleLowLimit Specifies the low angle limit.
     * @param angleHighLimit Specifies the high angle limit.
     * @param maxStepRate Optionally specifies the maximum step rate for
     *        SetPower.
     * @param options Specifies option flags:
     *          SERVOO_REVERSE - Specifies whether the servo direction is
     *                           reversed.
     */
    void
    InitTrcServo(
        float   initAngle,
        float   angleLowLimit,
        float   angleHighLimit,
        float   maxStepRate,
        UINT32  options
        )
    {
        TLevel(INIT);
        TEnterMsg(("initAngle=%f,lowLimit=%f,hiLimit=%f,stepRate=%f,options=%x",
                   initAngle, angleLowLimit, angleHighLimit, maxStepRate,
                   options));

        m_angleLowLimit = angleLowLimit;
        m_angleHighLimit = angleHighLimit;
        m_maxStepRate = maxStepRate;
        m_options = options;
        m_currAngle = GetAngle();
        m_targetAngle = 0.0;
        m_currStepRate = 0.0;
        m_prevTime = 0;
        m_servoFlags = 0;
        if (initAngle >= 0.0)
        {
            SetAngle(initAngle);
        }
        RegisterTask(MOD_NAME, TASK_POST_PERIODIC);

        TExit();
        return;
    }   //InitTrcServo

public:
    /**
     * This function releases the control of the servo motor.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_servoFlags &= ~SERVOF_STEP_ENABLED;
        m_prevTime = 0;

        TExit();
        return;
    }   //Stop

    /**
     * Constructor: Create an instance of the TrcServo object.
     *
     * @param module Specifies the digital module the servo is connected to.
     * @param channel Specifies the PWM channel the servo is connected to.
     * @param initAngle Optionally specifies the initial angle to set the
     *        servo to.
     * @param angleLowLimit Optionally specifies the low angle limit.
     * @param angleHighLimit Optionally specifies the high angle limit.
     * @param maxStepRate Optionally specifies the maximum step rate for
     *        SetPower.
     * @param options Optionally specifies option flags:
     *          SERVOO_REVERSE - Specifies whether the servo direction is
     *                           reversed.
     */
    TrcServo(
        UINT8   module,
        UINT32  channel,
        float   initAngle = -1.0,
        float   angleLowLimit = SERVO_MIN_ANGLE,
        float   angleHighLimit = SERVO_MAX_ANGLE,
        float   maxStepRate = 180.0,
        UINT32  options = 0
        ): Servo(module, channel)
    {
        TLevel(INIT);
        TEnterMsg(("module=%d,channel=%d,initAngle=%f,lowLimt=%f,hiLimit=%f,stepRate=%f,options=%x",
                   module, channel, initAngle, angleLowLimit, angleHighLimit,
                   maxStepRate, options));

        InitTrcServo(initAngle, angleLowLimit, angleHighLimit, maxStepRate,
                     options);

        TExit();
    }   //TrcServo

    /**
     * Constructor: Create an instance of the TrcServo object.
     *
     * @param channel Specifies the PWM channel the servo is connected to.
     * @param initAngle Optionally specifies the initial angle to set the
     *        servo to.
     * @param angleLowLimit Optionally specifies the low angle limit.
     * @param angleHighLimit Optionally specifies the high angle limit.
     * @param maxStepRate Optionally specifies the maximum step rate for
     *        SetPower.
     * @param options Optionally specifies option flags:
     *          SERVOO_REVERSE - Specifies whether the servo direction is
     *                           reversed.
     */
    TrcServo(
        UINT32  channel,
        float   initAngle = -1.0,
        float   angleLowLimit = SERVO_MIN_ANGLE,
        float   angleHighLimit = SERVO_MAX_ANGLE,
        float   maxStepRate = 180.0,
        UINT32  options = 0
        ): Servo(channel)
    {
        TLevel(INIT);
        TEnterMsg(("channel=%d,initAngle=%f,lowLimit=%f,hiLimit=%f,stepRate=%f,options=%x",
                   channel, initAngle, angleLowLimit, angleHighLimit,
                   maxStepRate, options));

        InitTrcServo(initAngle, angleLowLimit, angleHighLimit, maxStepRate,
                     options);

        TExit();
    }   //TrcServo

    /**
     * Destructor: Destroy an instance of the TrcServo object.
     */
    ~TrcServo(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();
        SetOffline();

        TExit();
    }   //~TrcServo

    /**
     * This function is called to set the speed of the servo motor. It is
     * typically used to drive the servo motor with the joystick value as
     * the power value.
     *
     * @param power Specifies the speed of the servo motor.
     */
    void
    SetPower(
        float power
        )
    {
        TLevel(API);
        TEnterMsg(("power=%f", power));

        if (power == 0.0)
        {
            Stop();
        }
        else
        {
            m_currAngle = GetAngle();
            m_targetAngle = (power >= 0.0)? m_angleHighLimit: m_angleLowLimit;
            m_currStepRate = fabs(power)*m_maxStepRate;
            m_servoFlags = SERVOF_STEP_ENABLED;
        }

        TExit();
        return;
    }   //SetPower

    /**
     * This function is periodically called by TaskMgr to check the current
     * servo angle against the target angle. It then calculate the next
     * angle to set the servo to according to the step rate.
     *
     * @param mode Specifies the CoopTask callback mode.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if ((mode != MODE_DISABLED) && (m_servoFlags & SERVOF_STEP_ENABLED))
        {
            UINT32 currTime = GetMsecTime();
            float stepAngle;

            if (m_prevTime == 0)
            {
                m_prevTime = currTime;
            }

            stepAngle = m_currStepRate*(currTime - m_prevTime)/1000.0;
            if (m_currAngle < m_targetAngle)
            {
                m_currAngle += stepAngle;
                if (m_currAngle > m_targetAngle)
                {
                    m_currAngle = m_targetAngle;
                }
                SetAngle(m_currAngle);
            }
            else if (m_currAngle > m_targetAngle)
            {
                m_currAngle -= stepAngle;
                if (m_currAngle < m_targetAngle)
                {
                    m_currAngle = m_targetAngle;
                }
                SetAngle(m_currAngle);
            }
            m_prevTime = currTime;
        }

        TExit();
        return;
    }   //TaskPostPeriodic

};  //class TrcServo

#endif  //ifndef _TRCSERVO_H
