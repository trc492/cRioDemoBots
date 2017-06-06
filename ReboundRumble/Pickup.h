#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Pickup.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Pickup class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_PICKUP
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Pickup"

/**
 * This module defines and implements the Pickup subsytem. The Pickup
 * subsystem consists of a conveyor and a pickup roller.
 */
class Pickup: public CoopTask
{
private:
    Victor  m_conveyorMotor;
    Victor  m_rollerMotor;

public:
    /**
     * This function sets the conveyor speed.
     *
     * @param speed Specifies the conveyor speed.
     */
    void
    SetConveyorSpeed(
        float speed
        )
    {
        TLevel(API);
        TEnterMsg(("speed=%f", speed));

        m_conveyorMotor.Set(speed);

        TExit();
    }   //SetConveyorSpeed

    /**
     * This function sets the roller speed.
     *
     * @param speed Specifies the roller speed.
     */
    void
    SetRollerSpeed(
        float speed
        )
    {
        TLevel(API);
        TEnterMsg(("speed=%f", speed));

        m_rollerMotor.Set(speed);

        TExit();
    }   //SetRollerSpeed
    
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Pickup(
        void
        ): m_conveyorMotor(PWM_CONVEYOR_MOTOR)
         , m_rollerMotor(PWM_ROLLER_MOTOR)
    {
        TLevel(INIT);
        TEnter();
        //
        // Initialize motor controller.
        //
        
        m_conveyorMotor.SetSafetyEnabled(false);
        m_rollerMotor.SetSafetyEnabled(false);
        
        RegisterTask(MOD_NAME, TASK_STOP_MODE);

        TExit();
    }   //Pickup
        
    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Pickup(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SetConveyorSpeed(0.0);
        SetRollerSpeed(0.0);
        UnregisterTask();

        TExit();
    }   //~Ladder

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
            SetConveyorSpeed(0.0);
            SetRollerSpeed(0.0);
        }

        TExit();
    }   //TaskStopMode

};  //class Pickup
