#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CanJag.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     CanJag class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _CANJAG_H
#define _CANJAG_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_CANJAG
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "CanJag"

#ifndef MINIMUM_AVERAGE_SPEED_SAMPLING_TIME
    #define MINIMUM_AVERAGE_SPEED_SAMPLING_TIME 0.02
#endif

/** \brief Wrapper over WPI's CANJaguar class. In case of brown out, can 
 *             restore JAG settings
 * 
 * This class defines and implements the CanJag object. The CanJag object
 * inherits from the CANJaguar object in the WPI library. It basically wraps
 * the CANJaguar class so it can shadow all the volatile Jaguar configuration
 * parameters. If the Jaguar ever browns out, we will be able to restore the
 * Jaguar configurations.
 */
class CanJag: public CANJaguar
{
private:
    UINT16              m_encoderLines;
    UINT16              m_potTurns;
    float               m_faultTime;
    NeutralMode         m_neutralMode;
    double              m_fwdLimitPos;
    double              m_revLimitPos;
    double              m_voltRampRate;

    SpeedReference      m_speedRef;
    PositionReference   m_posRef;
    double              m_Kp;
    double              m_Ki;
    double              m_Kd;

    bool                m_fPowerCycled;
    float               m_motorValue;
    double              m_position;
    double              m_speed;
    
    double              m_prevPosition; //for use with the GetAverageSpeed
    UINT32              m_prevTime;
    double              m_avgSpeed;

#ifdef _CANJAG_PERF
    PerfData           *m_setMotorPerfData;
    PerfData           *m_getPosPerfData;
    PerfData           *m_getSpeedPerfData;
#endif

    /**
     * This funcion checks if a power cycle has occurred. If so, it will
     * reconfigure the Jaguar with the shadowed information.
     *
     * @return Returns true if the Jaguar has been power cycled since we
     *         check last.
     */
    bool
    CheckPowerCycled(
        void
        )
    {
        TLevel(FUNC);
        TEnter();

        m_fPowerCycled = GetPowerCycled();
        if (m_fPowerCycled)
        {
            //
            // Jaguar has lost power, restore configuration appropriately.
            //
            TWarn(("Detected brownout on Jag %d.", m_deviceNumber));
            CANJaguar::ChangeControlMode(m_controlMode);
            if (m_controlMode == kPosition)
            {
                CANJaguar::SetPID(m_Kp, m_Ki, m_Kd);
                CANJaguar::SetPositionReference(m_posRef);
                if (m_posRef == kPosRef_QuadEncoder)
                {
                    CANJaguar::ConfigEncoderCodesPerRev(m_encoderLines);
                }
                else if (m_posRef == kPosRef_Potentiometer)
                {
                    CANJaguar::ConfigPotentiometerTurns(m_potTurns);
                }
            }
            else if (m_controlMode == kSpeed)
            {
                CANJaguar::SetPID(m_Kp, m_Ki, m_Kd);
                CANJaguar::SetSpeedReference(m_speedRef);
                if (m_speedRef != kSpeedRef_None)
                {
                    CANJaguar::ConfigEncoderCodesPerRev(m_encoderLines);
                }
            }
            else if (m_controlMode == kCurrent)
            {
                CANJaguar::SetPID(m_Kp, m_Ki, m_Kd);
            }

            if (m_maxOutputVoltage > 0.0)
            {
                CANJaguar::ConfigMaxOutputVoltage(m_maxOutputVoltage);
            }

            if (m_faultTime > 0.0)
            {
                CANJaguar::ConfigFaultTime(m_faultTime);
            }

            if (m_neutralMode != kNeutralMode_Jumper)
            {
                CANJaguar::ConfigNeutralMode(m_neutralMode);
            }

            if (m_fwdLimitPos == 0.0 && m_revLimitPos == 0.0)
            {
                CANJaguar::DisableSoftPositionLimits();
            }
            else
            {
                CANJaguar::ConfigSoftPositionLimits(m_fwdLimitPos,
                                                    m_revLimitPos);
            }

            if (m_voltRampRate > 0.0)
            {
                CANJaguar::SetVoltageRampRate(m_voltRampRate);
            }

            CANJaguar::EnableControl();
        }

        TExitMsg(("=%d", m_fPowerCycled));
        return m_fPowerCycled;
    }   //CheckPowerCycled

public:
    /**
     * Constructor: Create an instance of the CanJag object that inherits
     * the CANJaguar class.
     *
     * @param deviceNumber Specifies the CAN ID for the device.
     * @param controlMode Specifies the control mode to set the device to.
     */
    CanJag(
        UINT8 deviceNumber,
        ControlMode controlMode = kPercentVbus
        ): CANJaguar(deviceNumber, controlMode)
         , m_encoderLines(0)
         , m_potTurns(0)
         , m_faultTime(0.0)
         , m_neutralMode(kNeutralMode_Jumper)
         , m_fwdLimitPos(0.0)
         , m_revLimitPos(0.0)
         , m_voltRampRate(0.0)
         , m_speedRef(kSpeedRef_None)
         , m_posRef(kPosRef_None)
         , m_Kp(0.0)
         , m_Ki(0.0)
         , m_Kd(0.0)
         , m_motorValue(0.0)
         , m_position(0.0)
         , m_speed(0.0)
         , m_prevPosition(0.0)
         , m_prevTime(0)
         , m_avgSpeed(0.0)
    {
        TLevel(INIT);
        TEnterMsg(("CanID=%d,mode=%d", deviceNumber, controlMode));

        if (controlMode == kSpeed)
        {
            m_speedRef = GetSpeedReference();
            m_Kp = GetP();
            m_Ki = GetI();
            m_Kd = GetD();
        }
        else if (controlMode == kPosition)
        {
            m_posRef = GetPositionReference();
            m_Kp = GetP();
            m_Ki = GetI();
            m_Kd = GetD();
        }
        else if (controlMode == kCurrent)
        {
            m_Kp = GetP();
            m_Ki = GetI();
            m_Kd = GetD();
        }

        m_motorValue = CANJaguar::Get();
        m_position = CANJaguar::GetPosition();
        m_speed = CANJaguar::GetSpeed();
        //
        // Clear the power cycled flag.
        //
        m_fPowerCycled = GetPowerCycled();

#ifdef _LOGDATA_CANJAG
        DataLogger *dataLogger = DataLogger::GetInstance();
        char szID[3];
        snprintf(szID, sizeof(szID), "%02d", deviceNumber);
        dataLogger->AddDataPoint(MOD_NAME, szID, "PowerCycled", "%d",
                                 DataInt8, &m_fPowerCycled);
        dataLogger->AddDataPoint(MOD_NAME, szID, "MotorValue", "%f",
                                 DataFloat, &m_motorValue);
        dataLogger->AddDataPoint(MOD_NAME, szID, "MotorPos", "%f",
                                 DataDouble, &m_position);
        dataLogger->AddDataPoint(MOD_NAME, szID, "MotorSpeed", "%f",
                                 DataDouble, &m_speed);
#endif

#ifdef _CANJAG_PERF
        m_setMotorPerfData = NULL;
        m_getPosPerfData = NULL;
        m_getSpeedPerfData = NULL;
#endif


        TExit();
    }   //CanJag

    /**
     * Destructor: Destroy an instance of the CanJag object.
     */
    virtual
    ~CanJag(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~CanJag

    /**
     * This function sets the motor power.
     *
     * @param value Specifies the motor power.
     * @param syncGroup Optionally specifies the syncgroup of the motor.
     */
    void
    Set(
        float value,
        UINT8 syncGroup = 0
        )
    {
        TLevel(API);
        TEnterMsg(("value=%f,group=%d", value, syncGroup));

        CheckPowerCycled();
        if (value != m_motorValue)
        {
            //
            // Only setting the motor value if it has changed.
            //
            m_motorValue = value;
#ifdef _CANJAG_PERF
            if (m_setMotorPerfData != NULL)
            {
                m_setMotorPerfData->StartPerf();
            }
#endif
            CANJaguar::Set(value, syncGroup);
#ifdef _CANJAG_PERF
            if (m_setMotorPerfData != NULL)
            {
                m_setMotorPerfData->EndPerf();
            }
#endif
        }
        else if (m_safetyHelper != NULL)
        {
            //
            // Motor value did not change but we still need to feed the
            // watchdog.
            //
            m_safetyHelper->Feed();
        }

        TExit();
        return;
    }   //Set

    /**
     * This function sets the reference source device for speed control mode.
     *
     * @param reference Specifies the reference device.
     */
    void
    SetSpeedReference(
        SpeedReference reference
        )
    {
        TLevel(API);
        TEnterMsg(("ref=%d", reference));

        m_speedRef = reference;
        CANJaguar::SetSpeedReference(reference);

        TExit();
        return;
    }   //SetSpeedReference

    /**
     * This function sets the reference source device for position control mode.
     *
     * @param reference Specifies the reference device.
     */
    void
    SetPositionReference(
        PositionReference reference
        )
    {
        TLevel(API);
        TEnterMsg(("ref=%d", reference));

        m_posRef = reference;
        CANJaguar::SetPositionReference(reference);

        TExit();
        return;
    }   //SetPositionReference

    /**
     * This function sets the PID constants for the closed loop modes.
     *
     * @param Kp Specifies the P constant.
     * @param Ki SPecifies the I constant.
     * @param Kd Specifies the D constant.
     */
    void
    SetPID(
        double Kp,
        double Ki,
        double Kd
        )
    {
        TLevel(API);
        TEnterMsg(("Kp=%f,Ki=%f,Kd=%f", Kp, Ki, Kd));

        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
        CANJaguar::SetPID(Kp, Ki, Kd);

        TExit();
        return;
    }   //SetPID

    /**
     * This function sets the maximum voltage change rate.
     *
     * @param rampRate Specifies the max voltage ramp rate.
     */
    void
    SetVoltageRampRate(
        double rampRate
        )
    {
        TLevel(API);
        TEnterMsg(("rampRate=%f", rampRate));

        m_voltRampRate = rampRate;
        CANJaguar::SetVoltageRampRate(rampRate);

        TExit();
        return;
    }   //SetVoltageRampRate

    /**
     * This function configures the neutral mode.
     *
     * @param mode Specifies the neutral mode.
     */
    void
    ConfigNeutralMode(
        NeutralMode mode
        )
    {
        TLevel(API);
        TEnterMsg(("mode=%d", mode));

        m_neutralMode = mode;
        CANJaguar::ConfigNeutralMode(mode);

        TExit();
        return;
    }   //ConfigNeutralMode

    /**
     * This function configures the number of encoder lines per revolution.
     *
     * @param encoderLines Specifies the number of encoder lines per rev.
     */
    void
    ConfigEncoderCodesPerRev(
        UINT16 encoderLines
        )
    {
        TLevel(API);
        TEnterMsg(("lines=%d", encoderLines));

        m_encoderLines = encoderLines;
        CANJaguar::ConfigEncoderCodesPerRev(encoderLines);

        TExit();
        return;
    }   //ConfigEncoderCodesPerRev

    /**
     * This function configures the number of turns of the potentiometer.
     *
     * @param turns Specifies the number of turns of the potentiometer.
     */
    void
    ConfigPotentiometerTurns(
        UINT16 turns
        )
    {
        TLevel(API);
        TEnterMsg(("turns=%d", turns));

        m_potTurns = turns;
        CANJaguar::ConfigPotentiometerTurns(turns);

        TExit();
        return;
    }   //ConfigPotentiometerTurns

    /**
     * This function configures the forward and reverse position limits.
     *
     * @param fwdLimitPos Specifies the forward limit position.
     * @param revLimitPos Specifies the reverse limit position.
     */
    void
    ConfigSoftPositionLimits(
        double fwdLimitPos,
        double revLimitPos
        )
    {
        TLevel(API);
        TEnterMsg(("fwdLimit=%f,revLimit=%f", fwdLimitPos, revLimitPos));

        m_fwdLimitPos = fwdLimitPos;
        m_revLimitPos = revLimitPos;
        CANJaguar::ConfigSoftPositionLimits(fwdLimitPos, revLimitPos);

        TExit();
        return;
    }   //ConfigSoftPositionLimits

    /**
     * This function disables soft position limits.
     */
    void
    DisableSoftPositionLimits(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_fwdLimitPos = 0.0;
        m_revLimitPos = 0.0;
        CANJaguar::DisableSoftPositionLimits();

        TExit();
        return;
    }   //DisableSoftPositionLimits

    /**
     * This function configures how long the Jaguar waits in the case of a
     * fault before resuming operation.
     *
     * @param faultTime Specifies the fault time.
     */
    void
    ConfigFaultTime(
        float faultTime
        )
    {
        TLevel(API);
        TEnterMsg(("faultTime=%f", faultTime));

        m_faultTime = faultTime;
        CANJaguar::ConfigFaultTime(faultTime);

        TExit();
        return;
    }   //ConfigFaultTime

    /**
     * This function gets the motor position from the Jaguar controller.
     *
     * @return Returns the motor position.
     */
    double
    GetPosition(
        void
        )
    {
        TLevel(API);
        TEnter();

#ifdef _CANJAG_PERF
        if (m_getPosPerfData != NULL)
        {
            m_getPosPerfData->StartPerf();
        }
#endif
        m_position = CANJaguar::GetPosition();
#ifdef _CANJAG_PERF
        if (m_getPosPerfData != NULL)
        {
            m_getPosPerfData->EndPerf();
        }
#endif

        TExitMsg(("=%f", m_position));
        return m_position;
    }   //GetPosition

    /**
     * This function gets the motor speed from the Jaguar controller.
     * It will only return accurate results for speeds under around 1500RPM.
     * This function takes the measures the time between the two most recent 
     * encoder clicks and extrapolates speed from that.  
     *
     * @return Returns the motor speed.
     */
    double
    GetSpeed(
        void
        )
    {
        TLevel(API);
        TEnter();

#ifdef _CANJAG_PERF
        if (m_getSpeedPerfData != NULL)
        {
            m_getSpeedPerfData->StartPerf();
        }
#endif
        m_speed = CANJaguar::GetSpeed();
#ifdef _CANJAG_PERF
        if (m_getSpeedPerfData != NULL)
        {
            m_getSpeedPerfData->EndPerf();
        }
#endif

        TExitMsg(("=%f", m_speed));
        return m_speed;
    }   //GetSpeed

    /**
     * This function calculates the average motor speed since
     * the last time this function was called.
     * Instead of calculating the instantaneous speed based on the 
     * time between the latest two encoder clicks, this method uses
     * the position functionality of the jag and the timer in the 
     * cRio to make an average.  
     * 
     * This function is only accurate if called frequently.    
     *
     * @return Returns the motor speed.
     */
    double
    GetAverageSpeed(
        void
        )
    {
        TLevel(API);
        TEnter();

#ifdef _CANJAG_PERF
        if (m_getPosPerfData != NULL)
        {
            m_getPosPerfData->StartPerf();
        }
#endif
        //
        // If this is the first time running this function, call the Jag to
        // get the instantaneous speed and set up for next time the user
        // calls it. Otherwise, do a standard delta_pos/delta_time = avg_speed.
        //
        if (m_prevTime == 0)
        {
            m_prevTime = GetMsecTime();
            m_prevPosition = GetPosition();
            m_avgSpeed = GetSpeed();
        }
        else
        {
            UINT32 currTime = GetMsecTime();
            double timeDiff = (currTime - m_prevTime)/1000.0;
            
            if(timeDiff >= MINIMUM_AVERAGE_SPEED_SAMPLING_TIME)
            {
                double currPosition = GetPosition();
                double posDiff = currPosition - m_prevPosition;
    
                m_prevTime = currTime;
                m_prevPosition = currPosition;
                m_avgSpeed = posDiff/timeDiff;
            }
        }

#ifdef _CANJAG_PERF
        if (m_getPosPerfData != NULL)
        {
            m_getPosPerfData->EndPerf();
        }
#endif  
        
        TExitMsg(("=%f", m_avgSpeed));
        return m_avgSpeed;
    }   //GetAverageSpeed

#ifdef _CANJAG_PERF
    /**
     * This function sets up the various perfdata objects.
     *
     * @param setMotorPerfData Points to the perfdata object to collect
     *        Set() performance.
     * @param getPosPerfData Points to the perfdata object to collect
     *        GetPosition() performance.
     * @param getSpeedPerfData Points to the perfdata object to collect
     *        GetSpeed() performance.
     */
    void
    SetPerfData(
        PerfData *setMotorPerfData,
        PerfData *getPosPerfData,
        PerfData *getSpeedPerfData
        )
    {
        TLevel(API);
        TEnter();
 
        m_setMotorPerfData = setMotorPerfData;
        m_getPosPerfData = getPosPerfData;
        m_getSpeedPerfData = getSpeedPerfData;

        TExit();
        return;
    }   //SetPerfData
#endif

};  //class CanJag

#endif  //ifndef _CANJAG_H
