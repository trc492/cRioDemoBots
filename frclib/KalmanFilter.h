#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="KalmanFilter.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     KalmanFilter class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_FILTER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Kalman"

/**
 * This module defines and implements the KalmanFilter object.
 */
class KalmanFilter
{
private:
    double  m_Q;
    double  m_R;
    double  m_prevP;
    double  m_prevXEst;
    bool    m_fInitialized;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    KalmanFilter(
        void
        ): m_Q(0.022)
         , m_R(0.617)
         , m_prevP(0.0)
         , m_prevXEst(0.0)
         , m_fInitialized(false)
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //KalmanFilter

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~KalmanFilter(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~KalmanFilter

    /**
     * This function applies the Kalman filter algorithm to the data point.
     * It returns the best estimate of the data according to the previous
     * estimate and correction.
     * 
     * @param data Specifies the current data point.
     *
     * @return Returns the estimated value.
     */
    double
    FilterData(
        double data
        )
    {
        TLevel(API);
        TEnterMsg(("data=%f", data));

        if (!m_fInitialized)
        {
            m_prevXEst = data;
            m_fInitialized = true;
        }

        double tempP = m_prevP + m_Q;
        double K = tempP/(tempP + m_R);
        double xEst = m_prevXEst + K*(data - m_prevXEst);
        double P = (1 - K)*tempP;
        
        m_prevP = P;
        m_prevXEst = xEst;

        TExitMsg(("=%f", m_prevXEst));
        return m_prevXEst;
    }   //FilterData

};  //class KalmanFilter

#endif  //ifndef _KALMANFILTER_H
