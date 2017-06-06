#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcAccel.h" />
///
/// <summary>
///     This module contains the definition and implementation of the TrcAccel
///     class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCACCEL_H
#define _TRCACCEL_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ACCEL
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcAccel"

#ifdef _KALMANFILTER_H
#define ACCELO_FILTER           0x00000001
#endif

//
// 50 calibration points at 20ms each will cost us a total of 1 second start
// up time.
//
#ifndef ACCEL_NUM_CAL_PTS
    #define ACCEL_NUM_CAL_PTS   50
#endif
#ifndef ACCEL_CAL_INTERVAL
    #define ACCEL_CAL_INTERVAL  0.01    //10ms
#endif

/**
 * This class defines and implements the TrcAccel object. The TrcAccel object
 * inherits the ADXL345_I2C accelerometer object from the WPI library. This
 * object periodically sample the accelerometer value for the acceleration
 * value. It also integrates the acceleration value to calculate the velocity
 * and then integrates the velocity to calculate the distance value.
 */
class TrcAccel: public ADXL345_I2C
{
private:
    SEM_ID          m_semaphore;
    Notifier       *m_notifier;
    AllAxes         m_accelData;
    AllAxes         m_zeroOffset;
    AllAxes         m_deadBand;
    double          m_xVel;
    double          m_yVel;
    double          m_zVel;
    double          m_xDist;
    double          m_yDist;
    double          m_zDist;
    bool            m_fEnabled;
    UINT32          m_timestamp;
    UINT32          m_options;
#ifdef _KALMANFILTER_H
    KalmanFilter    m_kalmanX;
    KalmanFilter    m_kalmanY;
    KalmanFilter    m_kalmanZ;
#endif
    
    /**
     * This function is called periodically by the a timer callback to
     * process the accelerometer data. It integrates the data with time
     * to calculate the velocity, integrates it again to calculate
     * distance.
     */
    void
    Integrator(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            if (m_fEnabled)
            {
                UINT32 timeCurr = GetMsecTime();
                float period = (float)(timeCurr - m_timestamp)/1000.0;
                m_timestamp = timeCurr;
                m_accelData = GetAccelerations();
#ifdef _KALMANFILTER_H
                if (m_options & ACCELO_FILTER)
                {
                    m_accelData.XAxis = m_kalmanX.FilterData(m_accelData.XAxis);
                    m_accelData.YAxis = m_kalmanY.FilterData(m_accelData.YAxis);
                    m_accelData.ZAxis = m_kalmanZ.FilterData(m_accelData.ZAxis);
                }
#endif
                m_accelData.XAxis -= m_zeroOffset.XAxis;
                m_accelData.YAxis -= m_zeroOffset.YAxis;
                m_accelData.ZAxis -= m_zeroOffset.ZAxis;
                m_accelData.XAxis = GRAVITY_CONSTANT*
                                    DEADBAND(m_accelData.XAxis,
                                             m_deadBand.XAxis);
                m_accelData.YAxis = GRAVITY_CONSTANT*
                                    DEADBAND(m_accelData.YAxis,
                                             m_deadBand.YAxis);
                m_accelData.ZAxis = GRAVITY_CONSTANT*
                                    DEADBAND(m_accelData.ZAxis,
                                             m_deadBand.ZAxis);
                m_xVel += m_accelData.XAxis*period;
                m_yVel += m_accelData.YAxis*period;
                m_zVel += m_accelData.ZAxis*period;
                m_xDist += m_xVel*period;
                m_yDist += m_yVel*period;
                m_zDist += m_zVel*period;
                TSampling(("X(%f,%f,%f), Y(%f,%f,%f), Z(%f,%f,%f)",
                           m_accelData.XAxis*FEET_PER_METER,
                           m_xVel*FEET_PER_METER,
                           m_xDist*FEET_PER_METER,
                           m_accelData.YAxis*FEET_PER_METER,
                           m_yVel*FEET_PER_METER,
                           m_yDist*FEET_PER_METER,
                           m_accelData.ZAxis*FEET_PER_METER,
                           m_zVel*FEET_PER_METER,
                           m_zDist*FEET_PER_METER));
            }
        }
        END_REGION;

        TExit();
    }   //Integrator

    /**
     * This function is called when the timer expired. It will call the
     * non-static worker function.
     *
     * @param timer Points to the TrcAccel object to call the worker function.
     */
    static
    void
    CallIntegrator(
        void *accel
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("accel=%p", accel));
        ((TrcAccel*)accel)->Integrator();
        TExit();
    }   //CallIntegrator

public:
    /**
     * This function is called to calibrate the zero G point and deadband.
     * It assumes the accelerometer is sitting still at level ground during
     * the calibration. It samples a number of points on all axes and averages
     * them to be the zero G point for the axes. Note that the Z-axis is not
     * really at zero G when it is still. Z-axis should be 1G at level ground.
     * But for the purpose of measuring relative G's on all axes, we calibrate
     * zero G for all axes. During calibration, we also determine the floor
     * and ceiling noise level to form the deadband zone.
     *
     * @param numCalPts Specifies the number of calibration points.
     * @param calInterval Specifies the calibration interval in seconds.
     */
    void
    Calibrate(
        UINT32 numCalPts,
        float calInterval
        )
    {
        AllAxes data;
        AllAxes zeroG;
        AllAxes min;
        AllAxes max;

        TLevel(INIT);
        TEnter();

        zeroG.XAxis = 0.0;
        zeroG.YAxis = 0.0;
        zeroG.YAxis = 0.0;
        min.XAxis = 1000.0;
        min.YAxis = 1000.0;
        min.ZAxis = 1000.0;
        max.XAxis = 0.0;
        max.YAxis = 0.0;
        max.ZAxis = 0.0;
        for (UINT32 i = 0; i < numCalPts; i++)
        {
            data = GetAccelerations();

            zeroG.XAxis += data.XAxis;
            zeroG.YAxis += data.YAxis;
            zeroG.ZAxis += data.ZAxis;

            if (data.XAxis < min.XAxis)
            {
                min.XAxis = data.XAxis;
            }
            else if (data.XAxis > max.XAxis)
            {
                max.XAxis = data.XAxis;
            }

            if (data.YAxis < min.YAxis)
            {
                min.YAxis = data.YAxis;
            }
            else if (data.YAxis > max.YAxis)
            {
                max.YAxis = data.YAxis;
            }

            if (data.ZAxis < min.ZAxis)
            {
                min.ZAxis = data.ZAxis;
            }
            else if (data.ZAxis > max.ZAxis)
            {
                max.ZAxis = data.ZAxis;
            }

            Wait(calInterval);
        }

        CRITICAL_REGION(m_semaphore)
        {
            m_zeroOffset.XAxis = zeroG.XAxis/numCalPts;
            m_zeroOffset.YAxis = zeroG.YAxis/numCalPts;
            m_zeroOffset.ZAxis = zeroG.ZAxis/numCalPts;

            m_deadBand.XAxis = max.XAxis - min.XAxis;
            m_deadBand.YAxis = max.YAxis - min.YAxis;
            m_deadBand.ZAxis = max.ZAxis - min.ZAxis;

            TInfo(("AccelZeroG:x=%f,y=%f,z=%f, AccelDeadBand:x=%f,y=%f,z=%f",
                   m_zeroOffset.XAxis, m_zeroOffset.YAxis, m_zeroOffset.ZAxis,
                   m_deadBand.XAxis, m_deadBand.YAxis, m_deadBand.ZAxis));
        }
        END_REGION;

        Reset();

        TExit();
    }   //Calibrate

    /**
     * Constructor: Create an instance of the TrcAccel object. It initializes
     * the object and starts the periodic timer.
     *
     * @param module Specifies the digital module number on which the I2C
     *        port is used for the accelerometer.
     * @param options Specifies the option flags.
     * @param period Specifies the sampling time for doing calculations. This
     *        period is used to integrate acceleration into speed and distance
     *        travelled.
     */
    TrcAccel(
        UINT8 module,
        UINT32 options = 0,
        float period = 0.02
        ): ADXL345_I2C(module)
         , m_options(options)
#ifdef _KALMANFILTER_H
         , m_kalmanX()
         , m_kalmanY()
         , m_kalmanZ()
#endif
    {
        TLevel(INIT);
        TEnterMsg(("module=%d,period=%f", module, period));

        m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
        m_notifier = new Notifier(TrcAccel::CallIntegrator, this);
        m_accelData.XAxis = 0.0;
        m_accelData.YAxis = 0.0;
        m_accelData.ZAxis = 0.0;
        m_zeroOffset.XAxis = 0.0;
        m_zeroOffset.YAxis = 0.0;
        m_zeroOffset.ZAxis = 0.0;
        m_deadBand.XAxis = 0.0;
        m_deadBand.YAxis = 0.0;
        m_deadBand.ZAxis = 0.0;
        m_xVel = 0.0;
        m_yVel = 0.0;
        m_zVel = 0.0;
        m_xDist = 0.0;
        m_yDist = 0.0;
        m_zDist = 0.0;
        m_fEnabled = false;
        Calibrate(ACCEL_NUM_CAL_PTS, ACCEL_CAL_INTERVAL);
        m_notifier->StartPeriodic(period);
#ifdef _LOGDATA_ACCEL
        DataLogger *dataLogger = DataLogger::GetInstance();
        char szID[3];
        sprintf(szID, sizeof(szID), "%02d", module);
        dataLogger->AddDataPoint(MOD_NAME, szID, "xAccel", "%f",
                                 DataDouble, &m_accelData.XAxis);
        dataLogger->AddDataPoint(MOD_NAME, szID, "yAccel", "%f",
                                 DataDouble, &m_accelData.YAxis);
        dataLogger->AddDataPoint(MOD_NAME, szID, "zAccel", "%f",
                                 DataDouble, &m_accelData.ZAxis);
        dataLogger->AddDataPoint(MOD_NAME, szID, "xVel", "%f",
                                 DataDouble, &m_xVel);
        dataLogger->AddDataPoint(MOD_NAME, szID, "yVel", "%f",
                                 DataDouble, &m_yVel);
        dataLogger->AddDataPoint(MOD_NAME, szID, "zVel", "%f",
                                 DataDouble, &m_zVel);
        dataLogger->AddDataPoint(MOD_NAME, szID, "xPos", "%f",
                                 DataDouble, &m_xDist);
        dataLogger->AddDataPoint(MOD_NAME, szID, "yPos", "%f",
                                 DataDouble, &m_yDist);
        dataLogger->AddDataPoint(MOD_NAME, szID, "zPos", "%f",
                                 DataDouble, &m_zDist);
#endif

        TExit();
    }   //TrcAccel

    /**
     * Destructor: Destroy an instance of the TrcAccel object.
     */
    ~TrcAccel(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        semFlush(m_semaphore);
        SAFE_DELETE(m_notifier);

        TExit();
    }   //~TrcAccel

    /**
     * This function returns the current acceleration value of the X axis in
     * the unit of meters per second square.
     *
     * @return Returns the X acceleration value.
     */
    double
    GetMetricAccelX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_accelData.XAxis));
        return m_accelData.XAxis;
    }   //GetMetricAccelX

    /**
     * This function returns the current acceleration value of the X axis in
     * the unit of feet per second square.
     *
     * @return Returns the X acceleration value.
     */
    double
    GetAccelX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_accelData.XAxis*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetAccelX

    /**
     * This function returns the current acceleration value of the Y axis in
     * the unit of meters per second square.
     *
     * @return Returns the Y acceleration value.
     */
    double
    GetMetricAccelY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_accelData.YAxis));
        return m_accelData.YAxis;
    }   //GetMetricAccelY

    /**
     * This function returns the current acceleration value of the Y axis in
     * the unit of feet per second square.
     *
     * @return Returns the Y acceleration value.
     */
    double
    GetAccelY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_accelData.YAxis*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetAccelY

    /**
     * This function returns the current acceleration value of the Z axis in
     * the unit of meters per second square.
     *
     * @return Returns the Z acceleration value.
     */
    double
    GetMetricAccelZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_accelData.ZAxis));
        return m_accelData.ZAxis;
    }   //GetMetricAccelZ

    /**
     * This function returns the current acceleration value of the Z axis in
     * the unit of feet per second square.
     *
     * @return Returns the Z acceleration value.
     */
    double
    GetAccelZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_accelData.ZAxis*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetAccelZ

    /**
     * This function returns the current velocity value of the X axis in
     * the unit of meters per second.
     *
     * @return Returns the X velocity value.
     */
    double
    GetMetricVelX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_xVel));
        return m_xVel;
    }   //GetMetricVelX

    /**
     * This function returns the current velocity value of the X axis in
     * the unit of feet per second.
     *
     * @return Returns the X velocity value.
     */
    double
    GetVelX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_xVel*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetVelX

    /**
     * This function returns the current velocity value of the Y axis in
     * the unit of meters per second.
     *
     * @return Returns the Y velocity value.
     */
    double
    GetMetricVelY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_yVel));
        return m_yVel;
    }   //GetMetricVelY

    /**
     * This function returns the current velocity value of the Y axis in
     * the unit of feet per second.
     *
     * @return Returns the Y velocity value.
     */
    double
    GetVelY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_yVel*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetVelY

    /**
     * This function returns the current velocity value of the Z axis in
     * the unit of meters per second.
     *
     * @return Returns the Z velocity value.
     */
    double
    GetMetricVelZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_zVel));
        return m_zVel;
    }   //GetMetricVelZ

    /**
     * This function returns the current velocity value of the Z axis in
     * the unit of feet per second.
     *
     * @return Returns the Z velocity value.
     */
    double
    GetVelZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_zVel*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetVelZ

    /**
     * This function returns the current distance value of the X axis in
     * the unit of meters.
     *
     * @return Returns the X distance value.
     */
    double
    GetMetricDistX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_xDist));
        return m_xDist;
    }   //GetMetricDistX

    /**
     * This function returns the current distance value of the X axis in
     * the unit of feet.
     *
     * @return Returns the X distance value.
     */
    double
    GetDistX(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_xDist*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetDistX

    /**
     * This function returns the current distance value of the Y axis in
     * the unit of meters.
     *
     * @return Returns the Y distance value.
     */
    double
    GetMetricDistY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_yDist));
        return m_yDist;
    }   //GetMetricDistY

    /**
     * This function returns the current distance value of the Y axis in
     * the unit of feet.
     *
     * @return Returns the Y distance value.
     */
    double
    GetDistY(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_yDist*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetDistY

    /**
     * This function returns the current distance value of the Z axis in
     * the unit of meters.
     *
     * @return Returns the Z distance value.
     */
    double
    GetMetricDistZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        TExitMsg(("=%f", m_zDist));
        return m_zDist;
    }   //GetMetricDistZ

    /**
     * This function returns the current distance value of the Z axis in
     * the unit of feet.
     *
     * @return Returns the Z distance value.
     */
    double
    GetDistZ(
        void
        )
    {
        TLevel(HIFREQ);
        TEnter();
        double value = m_zDist*FEET_PER_METER;
        TExitMsg(("=%f", value));
        return value;
    }   //GetDistZ

    /**
     * This function resets the acceleration, velocity and distance values.
     */
    void
    Reset(
        void
        )
    {
        TLevel(API);
        TEnter();

        CRITICAL_REGION(m_semaphore)
        {
            m_accelData.XAxis = 0.0;
            m_accelData.YAxis = 0.0;
            m_accelData.ZAxis = 0.0;
            m_xVel = 0.0;
            m_yVel = 0.0;
            m_zVel = 0.0;
            m_xDist = 0.0;
            m_yDist = 0.0;
            m_zDist = 0.0;
        }
        END_REGION;

        TExit();
    }   //Reset

    /**
     * This function sets the accelerometer to enable or disable state.
     *
     * @param fEnabled If true, enables the accelerometer, false otherwise.
     */
    void
    SetEnabled(
        bool fEnabled
        )
    {
        TLevel(API);
        TEnter();

        m_fEnabled = fEnabled;
        if (fEnabled)
        {
            Reset();
            m_timestamp = GetMsecTime();
        }

        TExit();
    }   //SetEnabled

};  //class TrcAccel

#endif  //ifndef _TRCACCEL_H
