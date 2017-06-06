#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Shooter.h" />
///
/// <summary>
///     This module contains the definitions and implementation of the
///     Shooter class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _SHOOTER_H
#define _SHOOTER_H

#ifdef MOD_ID
#undef MOD_ID
#endif
#define MOD_ID                  MOD_SHOOTER
#ifdef MOD_NAME
#undef MOD_NAME
#endif
#define MOD_NAME                "Shooter"

#define VARID_PID               (VARID_DATAPTR + 1)
#define VARID_KP                (VARID_DATAPTR + 2)
#define VARID_KI                (VARID_DATAPTR + 3)
#define VARID_KD                (VARID_DATAPTR + 4)

#define SMSTATE_TURN_TO_TARGET  (SMSTATE_STARTED + 0)
#define SMSTATE_SHOOT_BALL      (SMSTATE_STARTED + 100)
#define SMSTATE_DONE            (SMSTATE_STARTED + 1000)

#define GUIDE_LIGHT_LEFT        SolID(SOL_LEFT_LIGHT)
#define GUIDE_LIGHT_MIDDLE      SolID(SOL_MIDDLE_LIGHT)
#define GUIDE_LIGHT_RIGHT       SolID(SOL_RIGHT_LIGHT)
#define GUIDE_LIGHT_ALL         ((UINT8)(GUIDE_LIGHT_LEFT |     \
                                         GUIDE_LIGHT_MIDDLE |   \
                                         GUIDE_LIGHT_RIGHT))

/**
 * This module defines and implements the Shooter subsytem. The Shooter
 * subsystem consists of two shooter motors that can run at a given constant
 * speed.
 */

class Shooter: public CoopTask,
               public PIDInput,
               public CmdHandler
{
private:
    //
    // Dependent subsystems.
    //
    VisionTarget   *m_visionTarget;
    DriveBase      *m_driveBase;
    Pickup         *m_pickup;
    TrcSol         *m_ballgate;
    Event          *m_notifyEvent;
    //
    // Shooter components.
    //
#ifndef _NO_SHOOTER_JAGS
    CanJag          m_shooterMotor1;
    CanJag          m_shooterMotor2;
    TrcPIDCtrl      m_shooterPIDCtrl;
    TrcPIDMotor     m_shooterPIDMotor;
#endif
    //
    // Status lights.
    //
    SolLight        m_guideLights;
    SolLight        m_speedLight;
    //
    // Shooter states.
    //
    UINT8           m_targetID;
    TARGETINFO      m_targetInfo;
    double          m_setSpeed;
    double          m_currSpeed;
    double          m_rawSpeed;
    bool            m_fContinuousMode;
    bool            m_prevErrStatus;
    bool            m_currErrStatus;
    double          m_speedAdj;
    double          m_angleAdj;
    bool            m_speedWithinTolerance;
    //
    // State machine.
    //
    StateMachine    m_shooterSM;
    Event           m_ballgateEvent;
    Event           m_turnEvent;

    KalmanFilter    m_kalmanFilter;
    double          m_Kp, m_Ki, m_Kd;

public:
    static VAR_ENTRY m_varTable[];

    /**
     * This function stops the shooter.
     *
     * @param fStopMotor If true, stop the shooter motors.
     * @param fNotify If true, notify event.
     */
    void
    StopPID(
        void
        )
    {
        TLevel( API);
        TEnter();

#ifndef _NO_SHOOTER_JAGS
        m_shooterPIDMotor.Reset(false);
#endif

        TExit();
        return;
    }   //StopPID
    
    /**
     * This function stops the shooter.
     *
     * @param fStopShooter Stop the shooter motor only if true.
     */
    void
    Stop(
        bool fStopShooter = true
        )
    {
        TLevel( API);
        TEnter();
        //
        // Stop vision targetting from monitoring speed.
        //
        if (fStopShooter)
        {
            m_fContinuousMode = false;
            //
            // Disable the state machine.
            //
            if (m_shooterSM.IsEnabled())
            {
                m_shooterSM.Stop();
                m_ballgateEvent.ClearEvent();
                m_turnEvent.ClearEvent();
            }
        }
        //
        // Disable related subsystems.
        //
        m_pickup->SetConveyorSpeed(0.0);
        m_pickup->SetRollerSpeed(0.0);
        m_driveBase->Stop();

#ifndef _NO_SHOOTER_JAGS
        if(fStopShooter)
        {
            m_shooterPIDMotor.Reset(true);
            m_shooterMotor1.DisableControl();
            m_shooterMotor2.DisableControl();
        }
#endif

        TExit();
        return;
    }   //Stop

    /**
     * Constructor for the class object.
     * Create instances of all the components.
     */
    Shooter(
           VisionTarget *visionTarget
         , DriveBase    *driveBase
         , Pickup       *pickup
         , TrcSol       *ballgate
        ): m_visionTarget(visionTarget)
         , m_driveBase(driveBase)
         , m_pickup(pickup)
         , m_ballgate(ballgate)
         , m_notifyEvent(NULL)
#ifndef _NO_SHOOTER_JAGS
         , m_shooterMotor1(CANID_SHOOTER1_JAG, CANJaguar::kPercentVbus)
         , m_shooterMotor2(CANID_SHOOTER2_JAG, CANJaguar::kPercentVbus)
         , m_shooterPIDCtrl("Shooter",
                            SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF,
                            SHOOTER_TOLERANCE, SHOOTER_SETTLING,
                            PIDCTRLO_ABS_SETPT | PIDCTRLO_SPEED_CTRL)
//                            SHOOTER_INITIAL_OUTPUT)
         , m_shooterPIDMotor(&m_shooterMotor1, &m_shooterMotor2,
                             SHOOTER_SYNC_GROUP,
                             &m_shooterPIDCtrl,
                             this,
                             PIDCTRLO_INVERSE)
#endif
         , m_guideLights(SOL_RIGHT_LIGHT, SOL_MIDDLE_LIGHT, SOL_LEFT_LIGHT)
         , m_speedLight(SOL_GREEN_LIGHT)
         , m_targetID(TARGET_TOP)
         , m_setSpeed(0.0)
         , m_currSpeed(0.0)
         , m_fContinuousMode(false)
         , m_prevErrStatus(false)
         , m_currErrStatus(false)
         , m_speedAdj(DEFAULT_SPEED_ADJ)
         , m_angleAdj(DEFAULT_ANGLE_ADJ)
         , m_speedWithinTolerance(false)
         , m_shooterSM()
         , m_ballgateEvent()
         , m_turnEvent()
         , m_kalmanFilter()
         , m_Kp(0.0)
         , m_Ki(0.0)
         , m_Kd(0.0)
    {
        TLevel(INIT);
        TEnter();
        //
        // Initialize motor controllers.
        //
#ifndef _NO_SHOOTER_JAGS
        m_shooterMotor1.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_shooterMotor1.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
        m_shooterMotor1.ConfigEncoderCodesPerRev(SHOOTER_ENCODER_PPR);
        m_shooterMotor2.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_shooterMotor2.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
        m_shooterMotor2.ConfigEncoderCodesPerRev(SHOOTER_ENCODER_PPR);
        m_shooterMotor1.SetSafetyEnabled(false);
        m_shooterMotor2.SetSafetyEnabled(false);
#endif

#ifdef _LOGDATA_SHOOTER
        DataLogger *dataLogger = DataLogger::GetInstance();
        /*dataLogger->AddDataPoint(MOD_NAME, "", "TargetID", "%d",
                                 DataInt8, &m_targetID);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetDistance", "%5.2f",
                                 DataFloat, &m_targetInfo.distance);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetHeight", "%5.2f",
                                 DataFloat, &m_targetInfo.height);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetAngle", "%5.2f",
                                 DataFloat, &m_targetInfo.angle);*/
        dataLogger->AddDataPoint(MOD_NAME, "", "Kp", "%10.9f",
                                 DataDouble, &m_Kp);
        dataLogger->AddDataPoint(MOD_NAME, "", "Ki", "%10.9f",
                                 DataDouble, &m_Ki);
        dataLogger->AddDataPoint(MOD_NAME, "", "Kd", "%10.9f",
                                 DataDouble, &m_Kd);
        dataLogger->AddDataPoint(MOD_NAME, "", "TargetSpeed", "%5.2f",
                                 DataDouble, &m_setSpeed);
        dataLogger->AddDataPoint(MOD_NAME, "", "CurrSpeed", "%5.2f",
                                 DataDouble, &m_currSpeed);
        dataLogger->AddDataPoint(MOD_NAME, "", "RawSpeed", "%5.2f",
                                 DataDouble, &m_rawSpeed);
#endif
        RegisterTask(MOD_NAME,
                     TASK_START_MODE | TASK_STOP_MODE | TASK_POST_PERIODIC);
        RegisterCmdHandler(MOD_NAME, NULL, m_varTable);

        TExit();
    }   //Shooter

    /**
     * Destructor for the class object.
     * Destroy instances of components that were created in the constructor.
     */
    virtual
    ~Shooter(
        void
        )
    {
        TLevel( INIT);
        TEnter();

        Stop();
        UnregisterTask();

        TExit();
    }   //~Shooter
    
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
#ifndef _NO_SHOOTER_JAGS
            m_shooterMotor1.EnableControl();
            m_shooterMotor2.EnableControl();
#endif
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
            Stop();
#ifndef _NO_SHOOTER_JAGS
            m_shooterMotor1.DisableControl();
            m_shooterMotor2.DisableControl();
#endif
        }

        TExit();
    }   //TaskStopMode

    /**
     * This function enables or disables Continuous mode.
     *
     * @param fEnable If true, enables continuous mode, false otherwise.
     */
    void
    SetContinuousMode(
        bool fEnable
        )
    {
        TLevel(API);
        TEnterMsg(("fEnable=%d", fEnable));

        m_fContinuousMode = fEnable;

        TExit();
        return;
    }   //SetContinuousMode

    /**
     * This function sets the PID constants for the shooter.
     *
     * @param Kp Specifies the proportional constant.
     * @param Ki Specifies the integral constant.
     * @param Kd Specifies the differential constant.
     *
     * @return Returns the new shooter speed adjustment factor.
     */
    void
    SetPID(
        float Kp,
        float Ki,
        float Kd
        )
    {
        TLevel(API);
        TEnterMsg(("Kp=%5.2f,Ki=%5.2f,Kd=%5.2f", Kp, Ki, Kd));

#ifndef _NO_SHOOTER_JAGS
        m_shooterPIDCtrl.SetPID(Kp, Ki, Kd);
#endif

        TExit();
        return;
    }   //SetPID

    /**
     * This function returns the Proportional constant of the shooter PID
     * control.
     *
     * @return Returns Kp.
     */
    float
    GetKp(
        void
        )
    {
        TLevel(API);
        TEnter();
        float value = 0.0;
        
#ifndef _NO_SHOOTER_JAGS
        value = m_shooterPIDCtrl.GetKp();
#endif

        TExitMsg(("=%f", value));
        return value;
    }   //GetKp

    /**
     * This function returns the Integral constant of the shooter PID
     * control.
     *
     * @return Returns Ki.
     */
    float
    GetKi(
        void
        )
    {
        TLevel(API);
        TEnter();
        float value = 0.0;

#ifndef _NO_SHOOTER_JAGS
        value = m_shooterPIDCtrl.GetKi();
#endif

        TExitMsg(("=%f", value));
        return value;
    }   //GetKi

    /**
     * This function returns the Differential constant of the shooter PID
     * control.
     *
     * @return Returns Kd.
     */
    float
    GetKd(
        void
        )
    {
        TLevel(API);
        TEnter();
        float value = 0.0;

#ifndef _NO_SHOOTER_JAGS
        value = m_shooterPIDCtrl.GetKd();
#endif

        TExitMsg(("=%f", value));
        return value;
    }   //GetKd

    /**
     * This function gets the angle adjustment factor.
     *
     * @return Returns the shooter speed adjustment factor.
     */
    double
    GetAngleAdj(
        void
        )
    {
        TLevel( API);
        TEnter();
        TExitMsg(("=%f", m_angleAdj));
        return m_angleAdj;
    }   //GetSpeedAdj

    /**
     * This function adds to the angle adjustment factor.
     *
     * @param adjInc Specifies the adjustment to be added to the angle
     *        adjustment factor. It can be negative.
     *
     * @return Returns the new shooter angle adjustment factor.
     */
    double
    AddAngleAdj(
        double adjInc
        )
    {
        TLevel(API);
        TEnterMsg(("adjInc=%f", adjInc));

        m_angleAdj += adjInc;

        TExitMsg(("=%f", m_angleAdj));
        return m_angleAdj;
    }   //AddAngleAdj

    /**
     * This function gets the speed adjustment factor.
     *
     * @return Returns the shooter speed adjustment factor.
     */
    double
    GetSpeedAdj(
        void
        )
    {
        TLevel( API);
        TEnter();
        TExitMsg(("=%f", m_speedAdj));
        return m_speedAdj;
    }   //GetSpeedAdj

    /**
     * This function adds to the speed adjustment factor.
     *
     * @param adjInc Specifies the adjustment to be added to the speed
     *        adjustment factor. It can be negative.
     *
     * @return Returns the new shooter speed adjustment factor.
     */
    double
    AddSpeedAdj(
        double adjInc
        )
    {
        TLevel(API);
        TEnterMsg(("adjInc=%f", adjInc));

        m_speedAdj += adjInc;

        TExitMsg(("=%f", m_speedAdj));
        return m_speedAdj;
    }   //AddSpeedAdj

    /**
     * This function sets the speed of the shooter.
     *
     * @param speed Specifies the speed of the shooter.
     * @param notifyEvent Specifies the event to notify when speed reaches
     *        target.
     */
    void
    SetSpeed(
        float speed, 
        Event *notifyEvent = NULL
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("speed=%f", speed));
        //
        // Somebody is setting specific speed, so don't let vision targetting
        // changes the speed.
        //
        m_fContinuousMode = false;
        m_setSpeed = speed;
#ifndef _NO_SHOOTER_JAGS
        m_shooterPIDMotor.SetTarget(m_setSpeed, false, notifyEvent);
#endif

        TExit();
    }   //SetSpeed

    /**
     * This function gets the shooter speed.
     *
     * @return Returns the shooter speed in RPM.
     */
    float
    GetSpeed(
        void
        )
    {
        float speed;

        TLevel( HIFREQ);
        TEnter();
        

#ifndef _NO_SHOOTER_JAGS
        m_rawSpeed = fabs(m_shooterMotor2.GetSpeed());
        speed = m_kalmanFilter.FilterData(m_rawSpeed);
#endif

        TExitMsg(("=%f", speed));
        return speed;
    }   //GetSpeed

    /**
     * This function is called from the PID controllers to get the PID input
     * value.
     *
     * @param pidCtrl Specifies the PID controller that requires the input
     *        value.
     */
    float
    GetInput(
        TrcPIDCtrl *pidCtrl
        )
    {
        float input;

        TLevel(HIFREQ);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        input = GetSpeed();

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput

    /**
     * This function sets the power of the shooter.
     *
     * @param power Specifies the power of the shooter motors.
     */
    void
    SetPower(
        float power
        )
    {
        TLevel(HIFREQ);
        TEnterMsg(("power=%f", power));

        m_fContinuousMode = false;
#ifndef _NO_SHOOTER_JAGS
        m_shooterPIDMotor.Reset(false);
        m_shooterMotor1.Set(power);
        m_shooterMotor2.Set(power);
        m_shooterMotor2.UpdateSyncGroup(SHOOTER_SYNC_GROUP);
#endif

        TExit();
    }   //SetPower

    /**
     * This function calculates and sets the new shooter speed according to
     * the given target info.
     *
     * @param distance Specifies the target distance.
     * @param height Specifies the target height.
     */
    void
    UpdateShooterSpeed(
        float distance,
        float height
        )
    {
        TLevel(FUNC);
        TEnterMsg(("dist=%f,height=%f", distance, height));
        //
        // Calculate the shooter velocity in inches per
        // second.
        //
        //    d = V*cos(theta)*t
        // => t = d/(V*cos(theta))
        //
        //    h = V*sin(theta)*t - 0.5*g*t^2
        // => h = V*sin(theta)*d/(V*cos(theta)) - 0.5*g*t^2
        // => h = d*tan(theta) - 0.5*g*t^2
        // => 0.5*g*t^2 = d*tan(theta) - h
        // => t^2 = (2/g)*(d*tan(theta) - h)
        // => t = sqrt((2/g)*(d*tan(theta) - h))
        // => d/(V*cos(theta)) = sqrt((2/g)*(d*tan(theta) - h))
        // => V = d/cos(theta)/sqrt((2/g)*(d*tan(theta) - h))
        //
        m_setSpeed = distance/
                     cos(SHOOTER_ANGLE_IN_RADIANS);
        double term = (distance*tan(SHOOTER_ANGLE_IN_RADIANS) - height)*
                      2.0/G_INCHES_PER_SEC_SQUARE;
        if (term > 0.0)
        {
            m_setSpeed /= sqrt(term);
            m_currErrStatus = false;
        }
        else
        {
            TErr(("Impossible target: either too close or too high!!!"));
            m_currErrStatus = true;
        }
        //
        // Convert it to RPM.
        //
        m_setSpeed /= SHOOTER_WHEEL_CIRCUMFERENCE;
        m_setSpeed *= 2.0;  //double speed to account for back spin.
        m_setSpeed *= 60.0;
        //
        // Adjust the speed to account friction and other
        // factors.
        //
        m_setSpeed *= m_speedAdj;

        TExit();
        return;
    }   //UpdateShooterSpeed

    /**
     * This function will use the target angle readings from the vision
     * module, compare it with the current heading from the gyro, and 
     * update the status lights based on the error.
     * IT also will blink if you're too close or too far from the targets. 
     */
    void
    UpdateGuideLights(
        void
        )
    {
        TLevel(TASK);
        TEnter();

        if (m_currErrStatus == false)
        {
            UINT8 lights = 0;

            if (m_targetInfo.angle < -TARGET_ANGLE_TOLERANCE)
            {
                lights |= GUIDE_LIGHT_LEFT;
            }
            else if (m_targetInfo.angle > TARGET_ANGLE_TOLERANCE)
            {
                lights |= GUIDE_LIGHT_RIGHT;
            }

            if (fabs(m_targetInfo.angle) <= 2*TARGET_ANGLE_TOLERANCE)
            {
                lights  |= GUIDE_LIGHT_MIDDLE;
            }

            m_guideLights.Set(false, GUIDE_LIGHT_ALL);
            m_guideLights.Set(true, lights);
            //printf("lights=%x\n", lights);
            //printf("angle=%5.2f", m_targetInfo.angle);
            m_prevErrStatus = false;
        }
        else if (m_currErrStatus != m_prevErrStatus)
        {
            m_guideLights.Set(0.5, 0.5, GUIDE_LIGHT_ALL);
            m_prevErrStatus = m_currErrStatus;
        }

        TExit();
    }   //UpdateGuideLight
    
    void
    SelectTarget(
        int targetID
        )
    {
        m_targetID = targetID;
    }   //SelectTarget

    /**
     * This function turns the robot towards the selected target.
     *
     * @param targetID Specifies the target to turn to.
     * @param notifyEvent which will trigger when the ball shoots (optional)
     *
     * @return Return true if success, otherwise the state machine is already
     *         busy working on a previous command.
     */
    bool
    TurnToTarget(
        int targetID = TARGET_SELECTED,
        Event *notifyEvent = NULL
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnter();
        
        if (targetID != TARGET_SELECTED)
        {
            m_targetID = targetID;
        }

        if (m_shooterSM.IsEnabled())
        {
            Stop(false);   
        }
        m_notifyEvent = notifyEvent;
        m_shooterSM.Start(SMSTATE_TURN_TO_TARGET);
        fSuccess = true;

        TExit();
        return fSuccess;
    }   //TurnToTarget
    
    void
    SetAlignEnabled(
        bool fEnabled,
        Event *turnEvent = NULL
        )
    {
        TLevel(API);
        TEnter();
        
        if(fEnabled)
        {
#ifdef _USE_MECANUM
            m_driveBase->DriveSetTarget(
                0.0,
                0.0,
                m_targetInfo.angle,
                true,
                turnEvent);
#else
            m_driveBase->DriveSetTarget(
                0.0,
                m_targetInfo.angle,
                true,
                turnEvent);
#endif
        }
        else
        {
            m_driveBase->Stop();
        }
        
        TExit();
    }  //SetAlignEnabled

    /**
     * This function opens the ballgate and shoots a basketball.
     *
     * @param notifyEvent which will trigger when the ball shoots (optional)
     *
     * @return Return true if success, otherwise the state machine is already
     *         busy working on a previous command.
     */
    bool
    ShootBall(
        Event *notifyEvent = NULL
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnter();

        if (!m_shooterSM.IsEnabled())
        {
            m_notifyEvent = notifyEvent;
            m_shooterSM.Start(SMSTATE_SHOOT_BALL);
            fSuccess = true;
        }

        TExit();
        return fSuccess;
    }   //ShootBall

    /**
     * This function is called by the TaskMgr to update the Shooter state
     * machine.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));
        
        m_shooterPIDCtrl.GetPID(&m_Kp, &m_Ki, &m_Kd);
        m_Kp *= 100000000.0;
        m_Ki *= 100000000.0;
        m_Kd *= 100000000.0;
        //
        // Update target information.
        //
        if (m_visionTarget->GetTargetInfo(m_targetID, &m_targetInfo))
        {
            //printf("Updating Target: %d\n", GetMsecTime());
            m_targetInfo.angle += m_angleAdj;
#ifndef _ENABLE_COMPETITION
           //TInfo(("ErrStatus=%x,mode=%x\n", m_currErrStatus, mode));
#endif
            if (m_fContinuousMode && (mode != MODE_DISABLED))
            {
                UpdateShooterSpeed(m_targetInfo.distance, m_targetInfo.height);
#ifndef _NO_SHOOTER_JAGS
                m_shooterPIDMotor.SetTarget(m_setSpeed, false);
#endif
                LCDPrintf((LCD_LINE2, "ShooterSpeed=%5.2f", m_setSpeed));
            }
        }
        //
        // Green light indicates the shooter speed is within tolerance.
        //
        m_currSpeed = GetSpeed();
        m_speedWithinTolerance = fabs(m_setSpeed - m_currSpeed) <
                                      SHOOTER_SPEED_TOLERANCE * m_setSpeed;
        m_speedLight.Set(m_speedWithinTolerance, SolID(SOL_GREEN_LIGHT));
        UpdateGuideLights();

#ifdef _DEBUG_SHOOTER
        LCDPrintf((LCD_LINE3, "ID=%d,A=%5.1f",
                   m_targetID, m_targetInfo.angle));
        LCDPrintf((LCD_LINE4, "D=%5.1f,H=%5.1f",
                   m_targetInfo.distance, m_targetInfo.height));
#endif
        LCDPrintf((LCD_LINE6, "S=%4.0fd=%2.0fa=%0.2f",
                   m_currSpeed, (m_speedAdj-1.0)*100.0, m_angleAdj));

        //don't run the statemachine during disabled
        if (mode != MODE_DISABLED && m_shooterSM.IsReady())
        {
            UINT32 currState = m_shooterSM.GetCurrentState();

#ifdef _DEBUG_SHOOTER
            LCDPrintf((LCD_LINE6, "ShooterState=%d",
                       currState - SMSTATE_STARTED));
#endif
            switch (currState)
            {
            case SMSTATE_TURN_TO_TARGET:
                m_fContinuousMode = true;
                if (fabs(m_targetInfo.angle) <= TARGET_ANGLE_TOLERANCE)
                {
                    m_shooterSM.SetCurrentState(currState + 1);
                }
                else
                {
                    SetAlignEnabled(true, &m_turnEvent);
                    m_shooterSM.WaitForSingleEvent(&m_turnEvent, currState + 1);
                }
                break;

            case SMSTATE_TURN_TO_TARGET + 1:
                if (fabs(m_setSpeed - m_currSpeed) <=
                    SHOOTER_SPEED_TOLERANCE * m_setSpeed)
                {
                    m_shooterSM.SetCurrentState(SMSTATE_DONE);
                }
                break;

            case SMSTATE_SHOOT_BALL:
                m_pickup->SetConveyorSpeed(CONVEYOR_SPEED);
                m_ballgate->Set(BALLGATE_OPEN, BALLGATE_OPEN_PERIOD,
                                BALLGATE_CLOSE, BALLGATE_HOLD_TIME,
                                0, &m_ballgateEvent);
                m_shooterSM.WaitForSingleEvent(&m_ballgateEvent, currState + 2);
                break;

            case SMSTATE_SHOOT_BALL + 1:
                m_pickup->SetConveyorSpeed(CONVEYOR_SPEED);
                break;

            case SMSTATE_SHOOT_BALL + 2:
                m_pickup->SetConveyorSpeed(0.0);
                m_shooterSM.SetCurrentState(SMSTATE_DONE);
                break;

            case SMSTATE_DONE:
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                    m_notifyEvent = NULL;
                }
                //
                // Let it fall through to stop the state machine.
                //
            default:
                m_shooterSM.Stop();
                break;
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodic

    /**
     * This function prints the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    GetVariable(
        PVAR_ENTRY varEntry
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd;

        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));

        m_shooterPIDCtrl.GetPID(&Kp, &Ki, &Kd);
        switch (varEntry->varID)
        {
            case VARID_PID:
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f\n", Kp, Ki, Kd);
                break;

            case VARID_KP:
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_KI:
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_KD:
                printf("Kd=%10.8f\n", Kd);
                break;

            default:
                printf("Error: invalid variable ID (var=%s,ID=%d).\n",
                       varEntry->varName, varEntry->varID);
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //GetVariable

    /**
     * This function sets the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     * @param varData Points to the data to be set to the variable.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    SetVariable(
        PVAR_ENTRY varEntry,
        void *varData
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd;

        TLevel(CALLBK);
        TEnterMsg(("var=%s,varData=%p", varEntry->varName, varData));

        m_shooterPIDCtrl.GetPID(&Kp, &Ki, &Kd);
        switch (varEntry->varID)
        {
            case VARID_KP:
                Kp = *(double *)varData;
                break;

            case VARID_KI:
                Ki = *(double *)varData;
                break;

            case VARID_KD:
                Kd = *(double *)varData;
                break;

            default:
                printf("Error: invalid variable ID (var=%s,ID=%d).\n",
                       varEntry->varName, varEntry->varID);
                rc = ERR_ASSERT;
        }
        m_shooterPIDCtrl.SetPID(Kp, Ki, Kd);

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetVariable

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

#ifndef _NO_SHOOTER_JAGS
        m_shooterMotor1.SetPerfData(setMotorPerfData,
                                    getPosPerfData,
                                    getSpeedPerfData);
        m_shooterMotor2.SetPerfData(setMotorPerfData,
                                    getPosPerfData,
                                    getSpeedPerfData);
#endif

        TExit();
        return;
    }   //SetPerfData
#endif

};  //class Shooter

VAR_ENTRY Shooter::m_varTable[] =
{
    {"PID",  VARID_PID, VarDouble, NULL, 0, "%10.8f",
     "Get PID constants"},
    {"Kp",   VARID_KP,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Kp constant"},
    {"Ki",   VARID_KI,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Ki constant"},
    {"Kd",   VARID_KD,  VarDouble, NULL, 0, "%10.8f",
     "Get/Set Kd constant"},
    {NULL,      0,      VarNone,   NULL, 0, NULL, NULL}
};

#endif  //ifndef _SHOOTER_H
