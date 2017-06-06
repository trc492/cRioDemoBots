#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Hanger.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Hanger class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_HANGER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Hanger"

/**
 * This module defines and implements the Hanger subsytem. The Hanger
 * subsystem consists of a ???
 */
class Hanger: public CoopTask
{
private:
    //
    //declaring the pneumatics 
    //
    TrcSol m_Piston;
    
public:
    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Hanger(
        void
        ): m_Piston(SOL_WHEELIE_EXTEND, SOL_WHEELIE_RETRACT)
    {
        TLevel(INIT);
        TEnter();
        
        m_Piston.Set(WHEELIE_RETRACT, WHEELIE_EXTEND);
        
        RegisterTask(MOD_NAME, 0);

        TExit();
    }   //Pickup
        
    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Hanger(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~Ladder

    /**
     * This function POPS A WHEELIE
     */
    void
    Extend(
        void
        )
    {
        TLevel(API);
        TEnter();

        //pop a wheelie...
        //  first close the retract channel,
        //  then open the extend channel
        m_Piston.Set(WHEELIE_EXTEND, WHEELIE_RETRACT);

        TExit();
        return;
    }   //Extend

    /**
     * This function contracts the hanger.
     */
    void
    Retract(
        void
        )
    {
        TLevel(API);
        TEnter();

        //RETRACTS HANGER
        m_Piston.Set(WHEELIE_RETRACT, WHEELIE_EXTEND);

        TExit();
        return;
    }   //Retracts
};  //class Hanger
