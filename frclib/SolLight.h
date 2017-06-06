#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="SolLight.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     SolLight class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _SOLLIGHT_H
#define _SOLLIGHT_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_SOLLIGHT
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "SolLight"

/**
 * This module defines and implements the SolLight object.
 */
class SolLight: public TrcSol
{
public:
    /**                                                                        
     * This function sets the light state.                                     
     *                                                                         
     * @param fOn Specifies the light state to be ON or OFF.                   
     * @param lights Specifies the light mask.                                 
     */                                                                        
    void                                                                       
    Set(                                                                       
        bool fOn,                                                              
        UINT8 lights
        )                                                                      
    {                                                                          
        TLevel(API);                                                           
        TEnterMsg(("fOn=%d,masks=%x", fOn, lights));                           
                                                                               
        TrcSol::Set(fOn, lights);
                                                                               
        TExit();                                                               
        return;                                                                
    }   //Set                                                                  
                                                                               
    /**
     * This function sets the blinking pattern.
     *
     * @param lightStates Points to an array specifying the sequence of
     *        light states and periods.
     * @param numStates Specifies the number of elements in the array.
     * @param options Specifies the option flags.
     */
    void
    Set(
        PSOL_STATE lightStates,
        int numStates,
        UINT32 options = 0
        )
    {
        TLevel(API);
        TEnterMsg(("states=%p,numStates=%d,options=%x",
                   lightStates, numStates, options));

        TrcSol::Set(lightStates, numStates, options);

        TExit();
        return;
    }   //Set

    /**
     * This function sets the blinking pattern.
     *
     * @param onTime Specifies the ON period in seconds.
     * @param offTime Specifies the OFF period in seconds.
     * @param lights Specifies the light mask.
     */
    void
    Set(
        double onTime,
        double offTime,
        UINT8 lights
        )
    {
        TLevel(API);
        TEnterMsg(("onTime=%f,offTime=%f,mask=%x", onTime, offTime, lights));

        if ((onTime == 0.0) && (offTime == 0.0))
        {
            //
            // No onTime or offTime means turn them off.
            //
            Set(false, lights);
        }
        else
        {
            TrcSol::Set(lights, onTime, 0, offTime, SOLO_REPEAT);
        }

        TExit();
        return;
    }   //Set

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channels Points to an array specifying the solenoid channels.
     * @param numLights Specifies the number of channels in the array.
     * @param module Specifies the module number.
     */
    SolLight(
        UINT32 *channels,
        int numLights,
        UINT8 module = 1
        ): TrcSol(channels, numLights, module)
    {
        TLevel(INIT);
        TEnterMsg(("channels=%p,numLights=%d,module=%d",
                   channels, numLights, module));
        TExit();
    }   //SolLight

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel Specifies the solenoid channel of the light.
     * @param module Specifies the module number.
     */
    SolLight(
        UINT32 channel,
        UINT8 module = 1
        ): TrcSol(channel, module)
    {
        TLevel(INIT);
        TEnterMsg(("chan=%d,module=%d", channel, module));
        TExit();
    }   //SolLight

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel1 Specifies the solenoid channel of the light 1.
     * @param channel2 Specifies the solenoid channel of the light 2.
     * @param module Specifies the module number.
     */
    SolLight(
        UINT32 channel1,
        UINT32 channel2,
        UINT8 module = 1
        ): TrcSol(channel1, channel2, module)
    {
        TLevel(INIT);
        TEnterMsg(("chan1=%d,chan2=%d,module=%d", channel1, channel2, module));
        TExit();
    }   //SolLight

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     *
     * @param channel1 Specifies the solenoid channel of the light 1.
     * @param channel2 Specifies the solenoid channel of the light 2.
     * @param channel3 Specifies the solenoid channel of the light 3.
     * @param module Specifies the module number.
     */
    SolLight(
        UINT32 channel1,
        UINT32 channel2,
        UINT32 channel3,
        UINT8 module = 1
        ): TrcSol(channel1, channel2, channel3, module)
    {
        TLevel(INIT);
        TEnterMsg(("chan1=%d,chan2=%d,chan3=%d,module=%d",
                   channel1, channel2, channel3, module));
        TExit();
    }   //SolLight

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~SolLight(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~SolLight

};  //class SolLight

#endif  //ifndef _SOLLIGHT_H
