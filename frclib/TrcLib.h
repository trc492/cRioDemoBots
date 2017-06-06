#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcLib.h" />
///
/// <summary>
///     This module includes all the TRC library modules and also contains the
///     implementation of the forward referenced functions.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCLIB_H
#define _TRCLIB_H

/** \mainpage 2013 Titan Robotics Club Library Reference
 *  
 *  \section intro_sec Why does this exist?
 *  
 *  There's a lot of functionality that we end up using often.  Not only from 
 *  year to year, but also multiple times within the same year.  Keeping 
 *  common logic in the library means you don't have to keep re-writing the same
 *  sections of code over and over again.  This reduces time spent debugging 
 *  because once something in the library is fixed, the same logic is fixed
 *  everywhere it's used.  
 *  
 *  You'll find that after you take the time to learn to use the library 
 *  effectively, you'll be saving yourself a lot of time. 
 *  
 *  \section easyP This looks complicated...
 *  
 *  It might look complicated at first, but that's because it's generic.  The 
 *  library was written so it could be used in as many different scenarios as
 *  possible.  Once you learn to use it, you will find yourself writing fewer
 *  lines of code, spending less time programming, writing easier-to-read code,
 *  and spending less time debugging.  
 *  
 *  \section necessaryP Do I have to use the library?
 *  
 *  If you're on team 492, Yes.  It's for your own good.    
 *  
 *  \section whyP Why?
 *  
 *  Besides the reasons outlined in the first section, using the library is an
 *  important skill.  It teaches you how to read other peoples' source code, 
 *  how to use a framework, and how to read documentation to figure stuff out.
 *  
 *  \section whine But I don't want to!
 *  
 *  Don't you start with me!
 *  
 *  \section acceptance You've convinced me, the TRC lib is great!  How can I contribute?
 *  
 *  There are a couple ways to contribute:
 *   - Use it!  Using it is the only way to find bugs
 *   - Report or Fix bugs you find
 *   - Help write documentation - We use doxygen.  Google it.
 *   - Write test cases
 *   - Figure out how to automate test cases  
 */

#include <math.h>
#include <hostlib.h>
//
// Common and Debugging modules.
//
#include "TrcDefs.h"
#include "Ansi.h"
#include "DbgTrace.h"
#include "Console.h"
#include "DataLogger.h"
#include "PerfData.h"
//
// Tasks, Events and State Machines.
//
#include "Task.h"
#include "CoopMTRobot.h"
#include "Event.h"
#include "TrcTimer.h"
#include "StateMachine.h"
//
// Inputs.
//
#include "KalmanFilter.h"
#include "IIRFilter.h"
#include "TrcJoystick.h"
#include "DigitalIn.h"
#include "AnalogIn.h"
#include "DSEnhDin.h"
#include "TrcAccel.h"
#include "VisionTask.h"
//
// Outputs.
//
#include "TrcSol.h"
#include "SolLight.h"
#include "CanJag.h"
#include "TrcServo.h"
#include "TrcPIDCtrl.h"
#include "TrcPIDMotor.h"
#include "TrcPIDDrive.h"
#include "TrcPIDServo.h"
#include "DriveBase.h"
#include "LineFollower.h"

#endif  //#ifndef _TRCLIB_H
