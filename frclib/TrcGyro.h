#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcGyro.h" />
///
/// <summary>
///     This module contains the definition and implementation of the TrcGyro
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCGYRO_H
#define _TRCGYRO_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_GYRO
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcGyro"

#ifdef _KALMANFILTER_H
#define GYROO_FILTER            0x00000001
#endif

#define PIDMODE_ANGLE           0
#define PIDMODE_VELOCITY        1
#define PIDMODE_ACCELERATION    2

/**
 * This class defines and implements the TrcGyro object. This object inherits
 * the Gyro object from the WPI library. It added the support of providing
 * angular velocity as well as angular acceleration information which is
 * missing from the Gyro class in the WPI library.
 */
class TrcGyro: public Gyro
{
private:
    float       m_period;
    SEM_ID      m_semaphore;
    Notifier   *m_notifier;
    float       m_angle;
    float       m_angularVelocity;
    float       m_angularAcceleration;
    UINT32      m_options;
#ifdef _KALMANFILTER_H
    KalmanFilter m_kalman;
#endif
#if 0
    UINT32      m_modePID;
#endif

    /**
     * This function is called periodically by the a timer callback to process
     * the gyro data. It differentiate the angle with time to calculate the
     * angular velocity, differentiates it again to calculate angular
     * acceleration.
     */
    void
    Differentiator(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            float prevAngle = m_angle;
            float prevVelocity = m_angularVelocity;
            m_angle = GetAngle();
#ifdef _KALMANFILTER_H
            if (m_options & GYROO_FILTER)
            {
                m_angle = m_kalman.FilterData(m_angle);
            }
#endif
            m_angularVelocity = (m_angle - prevAngle)/m_period;
            m_angularAcceleration = (m_angularVelocity - prevVelocity)/
                                    m_period;
            TSampling(("angle=%f,anglevel=%f,angleaccel=%f",
                       m_angle, m_angularVelocity, m_angularAcceleration));
        }
        END_REGION;

        TExit();
    }   //Differentiator

    /**
     * This function is called when the timer expired. It will call the
     * non-static worker function.
     *
     * @param timer Points to the TrcGyro object to call the worker function.
     */
    static
    void
    CallDifferentiator(
        void *gyro
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("gyro=%p", gyro));
        ((TrcGyro*)gyro)->Differentiator();
        TExit();
    }   //CallDifferentiator

    /**
     * This function does the common initialization of the TrcGyro object.
     *
     * @param options Specifies the option flags.
     */
    void
    GyroInit(
        UINT32 options
        )
    {
        TLevel(INIT);
        TEnterMsg(("options=%x", options));

        m_options = options;
        m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
        m_notifier = new Notifier(TrcGyro::CallDifferentiator, this);
        m_angle = GetAngle();
#ifdef _KALMANFILTER_H
        if (m_options & GYROO_FILTER)
        {
            m_angle = m_kalman.FilterData(m_angle);
        }
        m_angularVelocity = 0.0;
        m_angularAcceleration = 0.0;
        m_modePID = PIDMODE_ANGLE;
        m_notifier->StartPeriodic(m_period);

        TExit();
    }   //GyroInit

public:
    /**
     * Constructor: Create an instance of the TrcGyro object. It initializes
     * the object and starts the periodic timer.
     *
     * @param module Specifies the analog module number.
     * @param channel Specifies the analog channel.
     * @param options Specifies the option flags.
     * @param period Specifies the sampling time for doing calculations. This
     *        period is used to differentiate angle into velocity and
     *        acceleration.
     */
    TrcGyro(
        UINT8  module,
        UINT32 channel,
        UINT32 options = 0,
        float  period = 0.05
        ): Gyro(module, channel)
         , m_period(period)
#ifdef _KALMANFILTER_H
         , m_kalman()
#endif
    {
        TLevel(INIT);
        TEnterMsg(("module=%d,channel=%d,options=%x,period=%f",
                   module, channel, options, period));
        GyroInit(options);
        TExit();
    }   //TrcGyro

    /**
     * Constructor: Create an instance of the TrcGyro object. It initializes
     * the object and starts the periodic timer.
     *
     * @param channel Specifies the analog channel.
     * @param options Specifies the option flags.
     * @param period Specifies the sampling time for doing calculations. This
     *        period is used to differentiate angle into velocity and
     *        acceleration.
     */
    TrcGyro(
        UINT32 channel,
        UINT32 options = 0,
        float  period = 0.05
        ): Gyro(channel)
         , m_period(period)
    {
        TLevel(INIT);
        TEnterMsg(("channel=%d,options=%x,period=%f",
                   channel, options, period));
        GyroInit(options);
        TExit();
    }   //TrcGyro

    /**
     * Destructor: Destroy an instance of the TrcGyro object.
     */
    ~TrcGyro(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        semFlush(m_semaphore);
        SAFE_DELETE(m_notifier);

        TExit();
    }   //~TrcGyro

    /**
     * This function gets the current angle in degrees.
     */
    float
    GetAngle(
        void
        )
    {
        float value;

        TLevel(API);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            value = m_angle;
        }
        END_REGION;

        TExitMsg(("=%f", value));
        return value;
    }   //GetAngle

    /**
     * This function gets the current angular velocity in degrees per second.
     */
    float
    GetAngularVelocity(
        void
        )
    {
        float value;

        TLevel(API);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            value = m_angularVelocity;
        }
        END_REGION;

        TExitMsg(("=%f", value));
        return value;
    }   //GetAngularVelocity

    /**
     * This function gets the current angular acceleration in degrees per
     * second square.
     */
    float
    GetAngularAcceleration(
        void
        )
    {
        float value;

        TLevel(API);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            value = m_angularAcceleration;
        }
        END_REGION;

        TExitMsg(("=%f", value));
        return value;
    }   //GetAngularAcceleration

#if 0
    /**
     * This function sets the PID mode so that PIDGet will return
     * the desired value.
     */
    void
    SetPIDMode(
        UINT32 mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        CRITICAL_REGION(m_semaphore)
        {
            m_modePID = mode;
        }
        END_REGION;

        TExit();
    }   //SetPIDMode

    /**
     * This function is called by the PID controller to get the sensor
     * data.
     */
    double
    PIDGet(
        void
        )
    {
        double value = 0.0;
        TLevel(HIFREQ);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            switch (m_modePID)
            {
            case PIDMODE_ANGLE:
                value = m_angle;
                break;

            case PIDMODE_VELOCITY:
                value = m_angularVelocity;
                break;

            case PIDMODE_ACCELERATION:
                value = m_angularAcceleration;
                break;
            }
        }
        END_REGION;

        TExitMsg(("=%f", value));
        return value;
    }   //PIDGet
#endif

};  //class TrcGyro

#endif  //ifndef _TRCGYRO_H
