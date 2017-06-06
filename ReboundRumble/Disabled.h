#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Disabled.h" />
///
/// <summary>
///     This main module contains the disabled mode code.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

/**
 * This function is called before starting the disabled mode.
 */
void
TrcRobot::DisabledStart(
    void
    )
{
    TLevel(INIT);
    TEnter();
    //
    // Do other initializations for disabled mode here.
    //
    TExit();
}   //DisabledStart

/**
 * This function is called before exiting the disabled mode.
 */
void
TrcRobot::DisabledStop(
    void
    )
{
    TLevel(INIT);
    TEnter();
    //
    // Do clean up before exiting disabled mode here.
    //
    TExit();
}   //DisabledStop

/**
 * This function is called periodically at fixed intervals in disabled mode.
 */
void
TrcRobot::DisabledPeriodic(
    void
    )
{
	
    TLevel(HIFREQ);
    TEnter();

    //
    // Process input subsystems here.
    // (e.g. Reading joysticks, buttons, switches and sensors etc and
    //       computing the actions).
    //

    //
    // Perform output functions here.
    // (e.g. Programming motors, pneumatics and solenoids etc)
    //

    //
    // send the dashboard data associated with the I/O ports
    //
    m_dashboardDataFormat.SendIOPortData();

    TExit();
}   //DisabledPeriodic

