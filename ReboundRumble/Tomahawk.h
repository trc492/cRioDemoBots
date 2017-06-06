#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Tomahawk.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Tomahawk class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_TOMAHAWK
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Tomahawk"

/**
 * This module defines and implements the Tomahawk subsytem. The Tomahawk
 * subsystem consists of a motor driven by a Jaguar used to deploy a hammer
 * like actuator to push the bridge down so it can cross over it. It also
 * consists of two limit switches directly connected to the Jaguar to limit
 * the movement range of the actuator.
 */
class Tomahawk: public CoopTask
{
private:
    CanJag  m_tomahawkMotor;

public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Tomahawk(
        void
        ): m_tomahawkMotor(CANID_TOMAHAWK_JAG, CANJaguar::kPercentVbus)
    {
        TLevel(INIT);
        TEnter();
        
        m_tomahawkMotor.SetSafetyEnabled(false);

        RegisterTask(MOD_NAME, TASK_START_MODE | TASK_STOP_MODE);

        TExit();
    }   //Tomahawk

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Tomahawk(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~Tomahawk

    /**
     * This function is called by the TaskMgr to start the task.
     * 
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStartMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            m_tomahawkMotor.EnableControl();
        }

        TExit();
    }   //TaskStartMode

    /**
     * This function is called by the TaskMgr to stop the task.
     * 
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            m_tomahawkMotor.DisableControl();
        }

        TExit();
    }   //TaskStopMode
    
    /**
     * This function sets the Tomahawk motor speed.
     * 
     * @param speed Specifies the motor speed.
     */
    void
    SetSpeed(
        float speed
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("speed=%5.2f", speed));

        m_tomahawkMotor.Set(-speed);

        TExit();
    }   //SetSpeed

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

        m_tomahawkMotor.SetPerfData(setMotorPerfData,
                                    getPosPerfData,
                                    getSpeedPerfData);

        TExit();
        return;
    }   //SetPerfData
#endif

};  //class Tomahawk
